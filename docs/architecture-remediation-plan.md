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

## 4. Phase計画

状態は`未着手`、`進行中`、`検証中`、`完了`、`保留`のいずれかとする。

| Phase | 内容 | 状態 | Assembly phaseとの関係 |
| ---: | --- | --- | --- |
| A0 | baseline、境界、振舞い固定方法の記録 | 完了 | Phase 4と並行 |
| A1 | relation validationの網羅性とdispatcher分離 | 完了 | Phase 4完了前 |
| A2 | CLI integration testの外部化 | 完了 | Phase 5前 |
| A3 | CLIをcommand/generate/validate/manifestへ分割 | 完了 | Phase 5前 |
| A4 | kernel validationをreport/interference/relationsへ分割 | 完了 | Phase 5前 |
| A5 | prototypeをsubsystem別moduleへ分割しDefinitionsをgroup化 | 未着手 | Phase 5前 |
| A6 | `geared-gimbal-design` crateを追加し固有設計を移す | 未着手 | Phase 6前 |
| A7 | generic identity、coordinateおよびmodule依存方向を整理 | 未着手 | Phase 6と並行 |
| A8 | export/renderer境界をsemantic metadataとencoderへ変更 | 未着手 | Assembly Phase 8–9前 |
| A9 | rustdoc、最小README、CIおよび公開前gateを同期 | 未着手 | Assembly Phase 10 |

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
- SHA-256、artifact refreshおよびstagingをmanifest/output責務へ集約する。
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
- 平坦だった`Definitions`をfixed frame、pitch unit、rollおよびhardwareへgroup化し、datumを持つdefinitionは`Defined<D>`でdefinition IDとdatum bundleを一体にした。各subsystemへdefinition構築自体を移す作業が残るため、A5は引き続き進行中とする。
