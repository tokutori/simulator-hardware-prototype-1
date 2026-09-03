# Architecture remediation plan

## 1. 目的

この計画は、現在のCAD-as-code基盤を維持しながら、巨大moduleへ集中した責務とprototype固有依存を段階的に分離する。機械設計の修正とsoftware refactorを同じcommitへ混在させず、各段階で振舞いを固定する。

目標は次の依存方向である。

```text
gimbal-cli
  -> geared-gimbal-design
  -> gimbal-kernel-manifold
  -> gimbal-export

geared-gimbal-design -> gimbal-core
gimbal-kernel-manifold -> gimbal-core
gimbal-export -> gimbal-core
```

`gimbal-core`は`no_std + alloc`のgenericなgeometry、mechanism、assembly、datum、relationおよびgear domainに限定する。現在のpitch/roll prototype、部品role、配置規則およびparameter schemaは`geared-gimbal-design`へ移す。

## 2. 不変条件

1. refactor commitでは生成geometry、instance pose、relation、manifest schemaおよびartifact hashを意図せず変更しない。
2. 機械形状を変更するcommitでは、対応するrelation、validatorおよびtestを同時に変更する。
3. crate依存はDAGにし、kernel、filesystem、TOML、Blenderおよび具体的export形式をcoreへ入れない。
4. module分割は行数ではなく変更理由とdomain boundaryに基づく。
5. generic validatorは`ComponentRole`等のprototype固有taxonomyから検査対象を推測しない。
6. relation variant追加時に、未対応validatorをcompilerまたはcoverage reportが必ず検出する。
7. public APIのfilesystem writerには、可能な形式からin-memory encoderを分離する。
8. outputやanimationの更新は形状・pose・render metadataが変わるcheckpointで行い、純粋なファイル移動だけでは再生成しない。

## 3. 問題一覧

| ID | 重要度 | 問題 | 影響 |
| --- | --- | --- | --- |
| R-01 | High | `prototype.rs`がparameter、全subsystem、geometry、placement、relationを保持する | 局所変更が巨大fileへ集中する |
| R-02 | High | `gimbal-cli/main.rs`がcommand、生成、manifest、reportおよびintegration testを保持する | composition rootが薄い境界になっていない |
| R-03 | High | `validation.rs`がreport、broad phase、interferenceおよび全relation validatorを保持する | Phase 5–6の追加で網羅性と可読性が低下する |
| R-04 | High | generic validatorが`ComponentRole::has_high_detail_gear_geometry()`へ依存する | 別prototypeで再利用しにくい |
| R-05 | High | generic coreとgeared-gimbal固有のparameter、role、kinematicsが同じcrateにある | 別mechanismで不要なdomainを引き込む |
| R-06 | Medium | `Definitions`が平坦でdefinitionとdatum groupの対応が弱い | 部品追加時の編集箇所が増える |
| R-07 | Medium | `ComponentLocation::ordinal`の意味がroleごとに変わる | 不正なidentityを型で排除できない |
| R-08 | Medium | `assembly.rs`と`relation.rs`がID型を介して意味上相互依存する | module依存方向が不明確になる |
| R-09 | Medium | generic `Point2`がgear moduleに定義される | geometryからgearへの逆向き依存になる |
| R-10 | Medium | `CoordinateExpr`がpitch/rollへ固定される | generic mechanism coreとして再利用しにくい |
| R-11 | Medium | exporter public APIがfilesystem pathへ直結する | WASM、VirtualFS、in-memory testで再利用しにくい |
| R-12 | Medium | Blender detail選択がobject名prefixに依存する | Rust側semantic identityと乖離する |
| R-13 | Medium | public contractのrustdocが不足する | 単位、owner、検証範囲およびpanic条件を追跡しにくい |
| R-14 | Low | command parse、helpおよびdispatchが文字列で分散する | command追加時に同期漏れが起きる |
| R-15 | Low | remediation planが長期履歴を同時に保持する | 現在状態を探しにくい |
| R-16 | High | generationが既存`output`へ直接書き、旧artifactを存在確認だけでmanifestへ再収録し得る | source、validation report、画像・動画の世代が混在する |
| R-17 | High | 検査対象definitionとgeometry fidelityを一つの`scope`として扱うと、高精細gear除外とproxy/exactの意味が結合する | 「歯形を省くが構造はexact」という中間gateを表現できない |
| R-18 | High | AABB proxy候補をerrorとして列挙するだけではfalse positiveが多く、exact structural検査は現状数分を要する | 日常編集で大域的誤りを速く発見するrouteと、確定判定routeの間が空く |
| R-19 | Medium | front/rearで上部接続形状が異なるcarriageを一つのdefinitionとして180度回転再利用していた | rearの取付足だけが下へ反転し、平面位置は一致しても接触面積0になる |
| R-20 | Medium | definition/instance単位の存在理由監査だけでは、一つのsolid内に残る旧boss、rib、穴、逃げ形状を見落とす | 廃止済み機構のfeatureが干渉・造形時間・荷重集中を残す |

## 3.1 Validation gate matrix

検査はdefinition coverage、geometry fidelity、motion coverageの独立した三軸として扱う。command名や`full`という一語から保証範囲を推測しない。

| Route | Definition coverage | Geometry | Motion | 用途 | 正式加工gate |
| --- | --- | --- | --- | --- | --- |
| `validate-proxy` | 高精細gearを除外 | conservative structural proxy | static pose | 1秒級の大域候補抽出 | しない |
| `validate` | 高精細gearを除外 | exact solid | static pose | gear歯形以外の確定干渉とrelation検査 | 中間gate |
| `validate-full` | 全definition | exact solid | static pose | 高精細gearを含む静止姿勢検査 | motion gateと併用時のみ |
| 将来のmotion route | profileで明示 | proxyまたはexact | sampled/adaptive | ±pitch/roll包絡と床clearance | coverageをreportへ記録した場合のみ |

`validate-proxy`のAABB overlap件数は実干渉数ではない。`validate`もstatic poseであり、全可動域を保証しない。現状のexact structural routeはrelease buildでも数分を要するため、AABBとfull Manifold Booleanの間に、topologyを保った低polygon structural solidによるsecond stageを追加する。性能改善のために干渉閾値を緩めたり、relation participantをallow-listで除外したりしない。

## 4. Phase計画

状態は`未着手`、`進行中`、`検証中`、`完了`、`保留`のいずれかとする。

| Phase | 内容 | 状態 | Assembly phaseとの関係 |
| ---: | --- | --- | --- |
| A0 | baseline、境界、振舞い固定方法の記録 | 完了 | Phase 4と並行 |
| A1 | relation validationの網羅性とdispatcher分離 | 完了 | Phase 4完了前 |
| A2 | CLI integration testの外部化 | 完了 | Phase 5前 |
| A3 | CLIをcommand/generate/validate/manifestへ分割 | 完了 | Phase 5前 |
| A4 | kernel validationをreport/interference/relationsへ分割 | 完了 | Phase 5前 |
| A4P | validation coverage/fidelity分離と日常route高速化 | 進行中 | Phase 5–7と並行 |
| A5 | prototypeをsubsystem別moduleへ分割しDefinitionsをgroup化 | 完了 | Phase 5前 |
| A6 | `geared-gimbal-design` crateを追加し固有設計を移す | 進行中 | Phase 6前 |
| A7 | generic identity、coordinateおよびmodule依存方向を整理 | 進行中 | Phase 6と並行 |
| A8 | export/renderer境界をsemantic metadataとencoderへ変更 | 進行中 | Assembly Phase 8–9前 |
| A9 | rustdoc、最小README、CIおよび公開前gateを同期 | 進行中 | Assembly Phase 10 |

### A0: baselineと移行規則

- 現在のcrate DAG、workspace test、`no_std` checkおよび主要artifactをbaselineにする。
- 機械設計commitと移動・分割だけのcommitを分ける。
- `prototype.rs`、`main.rs`、`validation.rs`の変更理由を分類する。
- この計画をassembly remediation planから参照する。

Exit criteria:

- 移行順と各Phaseの非目標が明記される。
- 機械Phase 4–6を無期限に止める全面rewrite計画になっていない。

### A1: relation validationの網羅性

- relationごとに`Validated`、`Failed`、`SkippedByScope`、`Unsupported`を記録する。
- `complete`を全relation statusから導出し、literalを撤去する。
- `match AssemblyRelation`をwildcardなしのdispatcherにする。
- `SurfaceContact`と`FastenedJoint`を個別validator関数へ分離する。
- Phase 5–6で追加する`CylindricalFit`と`GearMesh`は、実装前には`Unsupported`としてreportを不完全扱いにする。
- `GeometryFidelity`と`MotionCoverage`の分離はこのPhaseで型だけ確定し、adaptive motion実装はassembly Phase 7で行う。

Exit criteria:

- 未対応relationを含むreportは`complete=false`になる。
- relation variant追加時にdispatcherがcompile errorになる。
- CLI JSONとunit testがrelation別coverageを出力する。

### A2: CLI test外部化

- repository既定設計のintegration testを`crates/gimbal-cli/tests/default_design.rs`へ移す。
- production helperへtestだけのvisibilityを与えず、必要な処理はlibrary moduleへ移す。
- 移動前後でtest名、件数および検査内容を維持する。

### A3: CLI module分割

```text
gimbal-cli/src/
  main.rs
  command.rs
  config.rs
  generate.rs
  validate.rs
  manifest.rs
  output.rs
```

- `Command` enumへ境界でparseする。
- `main()`はparseとdispatchだけにする。
- SHA-256とartifact refreshをmanifest/output責務へ集約する。
- command helpとdispatchのsource of truthを一つにする。

### A4: kernel validation分割

```text
validation/
  mod.rs
  report.rs
  plan.rs
  interference.rs
  proximity.rs
  relations/
    mod.rs
    surface_contact.rs
    fastened.rs
    cylindrical_fit.rs
    gear_mesh.rs
```

- public DTOとManifold計算helperを分離する。
- validatorへprototype固有roleではなく`ValidationPlan`を渡す。
- 高精細gear除外はdesign側がinstance/definition集合として明示する。
- fast structural routeとexact routeの保証範囲を型にする。

### A4P: validationを止めずに使える速度へする

- `DefinitionCoverage::{Selected, All}`を`GeometryFidelity`および`MotionCoverage`から分離する。
- high-detail gear除外はCLI/design composition rootがdefinition集合として決め、kernelはrole名を知らない。
- AABBだけのrouteは候補抽出として保持し、candidate countをerror countまたは完成までの残件数と呼ばない。
- structural-exact routeで確定したpairだけを形状修正へ用いる。
- low-detail structural solidをdefinitionごとに一度評価・cacheし、同じdefinitionの多数instanceで再利用しない。
- report filenameとerror messageをprofile固有pathへ一致させる。

Exit criteria:

- 高精細gearを除外したstatic exact検査が独立commandとstructured reportを持つ。
- proxy、structural-exact、full-exactのcoverageがJSONで区別される。
- 日常second-stage検査が通常開発機で30秒以内を目標とし、確定干渉0を証明するexact gateは節目で実行できる。
- front/rear等の非同型部品を無理に同一definitionとして扱わず、製造artifactのquantityとvariantがmanifestに残る。

### A5: geared-gimbal designのmodule分割

最初はcrateを増やさず、`gimbal-core/src/prototype/`配下で振舞いを変えない分割を行う。

```text
prototype/
  mod.rs
  parameters.rs
  identity.rs
  definitions.rs
  fixed_frame.rs
  pitch_unit.rs
  pitch_gearbox.rs
  roll.rs
  cockpit.rs
  hardware.rs
  tests.rs
```

- `Defined<D>`でdefinition IDとdatum bundleを一体にする。
- `Definitions`をfixed frame、pitch、roll、cockpit、hardwareへgroup化する。
- 各subsystemが自分のdefinition、instance、relationを構築する。
- feature/instance existence auditの分類結果を対応subsystemに残す。
- definition/instanceだけでなく、solid builder内の各boss、rib、hole、reliefおよびmounting padに現在の機能を対応付ける。対応先がないfeatureは削除し、接続のためのoverlapへ転用しない。

Exit criteria:

- ファイル移動前後で既定parameterから得るdefinition/instance/relation数が一致する。
- 主要poseとmesh metricのsnapshotが一致する。
- workspace test、fmt、Clippy、native/wasm `no_std`が成功する。

### A6: generic coreと固有designのcrate分離

- `geared-gimbal-design`を`no_std + alloc`で追加する。
- prototype parameter、component key、role mapping、pitch/roll command adapterおよび`build_prototype`を移す。
- `gimbal-core`にはgenericなFeature DAG、datum、relation、assembly、constraint、gear、transform、meshだけを残す。
- CLIはdesign crateをcomposition rootで選択する。

非目標:

- plugin systemや動的loadを導入しない。
- 一度に複数prototype schemaを一般化しない。
- generic coreから現prototypeの全意味を消すための互換性のない全面rewriteはしない。

### A7: identity、coordinate、module DAG

- `ComponentDefinitionId`、`ComponentInstanceId`、`AssemblyRelationId`等を下位の`ids`または`component` moduleへ移す。
- `Point2`をgeneric geometry/mathへ移す。
- generic `CoordinateId`とconstraint expressionをcoreへ置き、`PitchRollCommand`はdesign adapterにする。
- `PrototypeComponentKey`でroleごとの不正なlocation/ordinal組合せを表現不能にする。

### A8: exporterとrenderer境界

- 可能なformatで`encode_* -> Vec<u8>`とfilesystem wrapperを分ける。
- hash/manifest処理をexport crateからCLIへ移す。
- glTF node extrasまたはrender manifestへrole、side、end、ordinal、subsystemを出す。
- Blender adapterの旧prefixと撤去済み部品名を削除し、semantic metadataで選択する。
- generation runごとのstaging directoryへ全artifactを生成し、成功時だけ`output`へatomicに置換する。
- manifestへcommit SHA、design/process hash、producer、generation run IDおよびvalidation coverageを記録し、旧世代artifactを存在確認だけで再収録しない。

### A9: public contractとgate

- 座標単位、quaternion、ID owner、relation追加時検証、validation coverage、panic/Result条件をrustdocにする。
- READMEは目的、build、主要command、parameter入口、preview-onlyとvalidated artifactの違い、設計文書への導線だけに限定する。
- command一覧はCLI helpをauthorityとし、文書へ手書きで重複させない。
- GitHub Actionsでfmt、Clippy、workspace test、native/wasm `no_std`およびdependency auditを実行する。
- MIT公開前にlicense header、dependency license、secret、大容量生成物およびremote SHAを検査する。

## 5. 現在の判断

外側のcrate DAG、`no_std` core、Feature DAG、kernel adapterおよびexport形式別moduleは維持する。現時点で新しい抽象frameworkを作る必要はない。

先に直すのは、未対応relationを成功扱いし得る検証上の欠陥である。その後、振舞いを変えないmodule分割をPhase 5のshaft/bearing設計前に行う。`geared-gimbal-design` crateへの分離は、module境界が実装で確認できてから行い、巨大fileを巨大crateへ移すだけの変更にはしない。

## 6. 進捗記録

### 2026-09-03

- A0として、現行の良いcrate DAGと、巨大module・prototype固有依存・renderer prefix依存を分けて記録した。
- A1の最初のcheckpointとして、全relationへ`Validated`、`Failed`、`SkippedByScope`、`Unsupported`のstatusを必ず割り当てるcoverage reportを追加した。
- `ValidationReport::is_complete()`をrelation statusから導出し、CLI JSONのhard-coded `complete: true`を撤去した。未実装の`CylindricalFit`または`GearMesh`を含むreportは`complete=false`かつ`valid=false`になる。
- relation kindとstatusをCLI JSONへ列挙するようにし、未対応`CylindricalFit` fixture、成功する`FastenedJoint` fixtureおよび失敗する`FastenedJoint` fixtureでcoverage statusを回帰検査した。
- `AssemblyRelation`をwildcardなしで処理する単一dispatcherを実装し、`SurfaceContact`と`FastenedJoint`を個別validator関数へ分離した。variant追加時はdispatcherの網羅性検査が働く。
- 旧`ValidationScope`を、独立した`GeometryFidelity`と`MotionCoverage`を持つ`ValidationProfile`へ置換した。現在のvalidatorが保証するmotion範囲は`StaticPose`だけであり、exact geometryと全可動域検査を同じ`full`という語で混同しない。
- CLI reportは`structural-proxy`または`exact`のgeometry fidelityと、`static-pose`のmotion coverageを別fieldで出力する。A1のexit criteriaを満たしたためA1を完了した。
- A2として、repository既定設計の21 integration testsを`crates/gimbal-cli/tests/default_design.rs`へ移した。CLI packageへlibrary targetを追加し、binary `main.rs`は`gimbal_cli::run()`の結果をprocess exitへ変換する8行だけにした。
- test移動後もtest名と検査内容を維持し、`cargo test -p gimbal-cli --no-run`とwarning-as-error Clippyでbinary、library、外部integration testの3 targetが成功した。A2を完了し、A3を進行中へ変更した。
- A3として、CLI libraryを`command`、`generate`、`validate`、`manifest`、`output`へ分割した。`lib.rs`はconfiguration読込みとtyped command dispatchだけを担い、`main.rs`はprocess boundaryだけを担う。
- command文字列を境界で`Command` enumへ一度だけ変換し、default command、全subcommand、helpおよびunknown commandをunit testで固定した。helpは各validation commandが`structural-proxy/static-pose`または`exact/static-pose`であることを明示する。
- `generate.rs`はartifact生成、`validate.rs`はvalidator orchestrationとreport、`manifest.rs`はhash/refresh、`output.rs`は出力directoryの削除だけを担当する。A3を完了し、A4を進行中へ変更した。
- A4のmodule分割checkpointとして、kernel validatorを`report.rs`、`instance.rs`、`interference.rs`、`proximity.rs`、`relations.rs`、`tests.rs`へ分離した。`mod.rs`はpublic error、validator contextおよびmodule wiringだけを保持する。
- `ValidationPlan`を`plan.rs`へ追加し、検査対象definition集合をcallerが明示する構造へ変更した。kernel validatorから`ComponentRole::has_high_detail_gear_geometry()`への依存を撤去し、CLI composition rootだけが既定prototypeの高精細gear除外policyを組み立てる。
- report、plan、instance world geometry、interference、proximity、relation検証およびtestの変更理由をmodule境界へ反映し、A4のexit criteriaを満たしたため完了とした。
- A5の最初のcheckpointとして、単一の`prototype.rs`を`parameters`、`validation`、`definitions`、`fixed_frame`、`pitch_unit`、`pitch_geometry`、`roll`、`component_geometry`および`tests`へ振舞いを変えず分割した。`mod.rs`は設計構築順序を示すcomposition rootだけを主に保持する。
- 平坦だった`Definitions`をfixed frame、pitch unit、rollおよびhardwareへgroup化し、datumを持つdefinitionは`Defined<D>`でdefinition IDとdatum bundleを一体にした。
- definition IDの決定順を一箇所で明示するためdefinition生成は`definitions.rs`に集約し、instanceとrelationは`fixed_frame.rs`、`pitch_unit.rs`、`roll.rs`が所有する構成を採用した。分散したbuilderから暗黙のID順序を復元するより変更経路が明確であるため、当初案の「各subsystemがdefinitionも構築する」は採用しない。
- 分割後も既定設計のdefinition/instance/relation数、主要pose、床clearance、exact solid回帰を含むworkspace 54 testsが成功した。fmt、warning-as-error Clippy、native/wasm `no_std` checkも成功したためA5を完了とした。
- A6の最初のcheckpointとして、既定prototypeのparameter、validation、geometry、definition、instance、relationおよび`build_prototype`を`no_std + alloc`の`geared-gimbal-design` crateへ移した。CLIはcomposition rootでdesign crateを選択し、`gimbal-core`は具体設計をexportしない。
- `ComponentRole`とpitch/roll固定のcoordinate adapterはまだgeneric core側に残るため、A6は進行中とする。
- A8の独立checkpointとして、任意fileのSHA-256計算を`gimbal-export`から`gimbal-cli::manifest`へ移した。export crateはmesh/CAD形式のserializationだけを担当し、artifact provenanceはCLI shellが担当する。
- A9のCI checkpointをAssembly Phase 10より前倒しし、push/PRでformat、workspace check、warning-as-error Clippy、workspace test、generic coreと固有designのnative/WASM `no_std` checkを行うworkflowを追加した。依存監査は`Cargo.lock`に対してversion固定した`cargo-audit`を実行する。公開前artifact gate、rustdocおよびREADME同期が残るためA9は進行中とする。
- glTF nodeの`extras`へrole、side、longitudinal end、vertical endおよびordinalを出力し、Blender adapterのdetail対象選択をobject名prefixからsemantic custom propertyへ移行した。撤去済み部品名がadapterに残って別物を選ぶ経路を廃止した。全233 nodeへのmetadata出力、Rust unit testおよびBlender 5.1.2によるcustom property選択・静止画8枚・動画3本の再生成を確認した。
- 最初のLinux CIでRust 1.98の新しいClippy lintを検出し、MSRV 1.88でstableな`slice::as_chunks`へ修正した。Node.js 20非推奨のcheckout v4も公式v7.0.1のcommit SHA固定へ更新した。再run `33681538072`ではRust quality gatesとRustSec auditの両jobが成功した。
- relation dispatcherの次の実装として`CylindricalFit`を追加し、中心線間距離、軸方向およびtarget radial clearanceをengineering toleranceで検証するようにした。正常、軸方向に離れた同軸datum、0.2 mm偏心、軸傾斜およびclearance不一致fixtureを追加し、未実装relationのcoverage testは`GearMesh`へ移した。
- A7の最初のbehavior-preserving checkpointとして、汎用2D座標型`Point2`を`gear`から`geometry`へ移した。gear profileは下位のgeometry primitiveを利用する依存方向となり、generic geometryがgear domainへ意味上依存する逆転を解消した。ID moduleとgeneric coordinateへの整理は残るためA7は進行中とする。
- 追加監査で指摘されたrepository構造上の課題をA6–A9へ対応付けた。次のbehavior-preserving順序は、(1) datum/instance IDへowner provenanceを持たせる、(2) generic `CoordinateId`へpitch/roll固定式を移す、(3) `ComponentRole`と不正な`ordinal`組合せを固有designのtyped keyへ移す、(4) exporterへin-memory encoderを追加する、(5) rustdocと公開前artifact gateを同期する、である。機械Phase 5–6の形状・relation変更と同一commitへ混ぜない。
- datum owner provenanceは既存commit `35340c6`で既に実装済みであることを現HEADへ再照合した。`DatumId<T>`と`DatumSet`は発行元`ComponentDefinitionId`を保持し、relation追加時にinstanceのdefinitionと一致しなければ`DatumOwnerMismatch`で拒否する。同kind・同indexの別definition datumを渡す回帰testも存在するため、A-31を重複実装せず完了扱いにする。Assembly/Frame等の別arena間ID provenanceはA7の残作業として区別する。
- R-16への最初の実装として、Rust artifact生成をworkspace直下の専用staging treeへ隔離し、全出力とmanifestが完成した場合だけ既存`output`と置換するtransactionを追加した。manifest上は物理staging pathでなく論理`output/...`を記録する。生成開始時に旧outputを破壊せず、成功時に旧世代PNG/MP4を持ち越さないことをunit testで固定した。commit SHA、design/process hash、run IDおよびexternal Blender artifactを含む一括transactionはA8に残る。
- A8のencoder checkpointとして、Wavefront OBJ/MTL、binary STL、canonical FDM形式の3MF、glTF animationおよびDXFへfilesystemへ触れない`encode_*` APIを追加し、既存`write_*` APIを同じencoderへ委譲した。in-memory bytesとfilesystem wrapperの出力が同一であることをunit testで固定した。利用中の`dxf 0.6.1`が公式API [`Drawing::save(&mut impl Write)`](https://docs.rs/dxf/0.6.1/dxf/struct.Drawing.html#method.save)を提供することを確認し、DXFも一時fileへ依存せず検査する。`Drawing::new()`が生成する時刻とUUIDは固定metadataへ正規化し、同じprofileからbyte-identicalなDXFを得る。
- A8のprovenance checkpointとして、manifest schema 4へproducer/version、Git commit、dirty state、parameter/process file hash、generation modeおよびこれらから導出するcontent-addressed generation IDを追加した。同じsource/input/modeからは同じIDを得て、preview-onlyとvalidated outputは異なるIDになる。Git管理外のsource archiveでもinput hashは必ず残し、repository情報だけを`null`にする。
- compliant partの加工形状と組付け形状を混同しないため、generic coreの`Body`へ`Compliant { manufacturing_solid, assembly_solid }`を追加した。assembly表示とkernel validationは組付け形状、validated FDM exporterは無負荷の加工形状を選ぶ。現在のpitch retention flexureを最初の利用例とし、二つのshape IDが異なることと、FDM側の選択APIを回帰testへ固定した。これはA6/A8の境界を利用した機械Phase 5のcheckpointであり、coreへpitch固有のspring semanticsは持ち込まない。
- `CylindricalFit`はdatum originの三次元一致でなく、共通軸に直交する中心線間距離を検証するよう修正した。同一shaft上で軸方向位置が異なる複数bearingを正しく表現でき、軸に直交する0.2 mm偏心は従来どおり失敗する。issue名も`origin separation`から`axis separation`へ変更し、CLI JSON contractとtestを同期した。
