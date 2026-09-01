# Assembly・機械接続 再設計計画

## 1. 文書の目的

この文書は、`prototype/geared-gimbal` の `5298feebc9eac83335083f842d63bdc506947e89` までに判明したassembly・締結・干渉・動力伝達上の問題を記録し、修正をPhase単位で管理するためのものである。

現在のモデルは、外観、概略配置、Feature DAGおよびpitch/rollの運動表示までは作成できるが、機械的な接続関係を十分に表現していない。したがって、現時点の3MF、Blender model、DXF、静止画および動画を、組立可能性や機械的成立性の証明として扱ってはならない。

本計画では、個々の部品を少しずつ移動して干渉を隠すのではなく、assembly relation、実形状および検証器を同じ設計へ揃える。

## 2. 根本原因

現在のdomain modelは、主に次を表現する。

```text
ComponentDefinition = local geometry
ComponentInstance   = frame上の配置
Frame / Joint       = kinematic motion
```

一方、次の機械的関係を表現する型が存在しない。

- 面接触
- bolt締結
- 軸と穴の嵌合
- 軸受支持
- key、clamp等によるトルク伝達
- gear meshと歯の位相
- 意図的なinterference fit

この欠落を補うため、実装では一部の接続を正の交差体積や同一frameへの所属で代用してしまった。

```text
形状が重なる
    ≠ 接触している
    ≠ 締結されている
    ≠ 荷重を伝えられる
    ≠ トルクを伝えられる
    ≠ 同じ運動をする
```

したがって、個別のoverlapを除くだけでは根本解決にならない。

## 3. 正しい設計上の不変条件

以後は、次をrepository全体の不変条件とする。

1. 同一ComponentDefinition内のBoolean Unionは、1部品を構成するための形状操作として許可する。
2. 別ComponentInstanceの内部は原則としてdisjointであり、正の体積交差を干渉として扱う。
3. 接続は体積交差ではなく、型付けした`AssemblyRelation`で明示する。
4. press fit等の意図的な交差は、専用relationと許容値が存在する場合だけ許可する。
5. face contactは、接触面の距離、向き、重なり面積および必要なclearanceで検証する。
6. bolt締結は、部品側の実穴、同軸性、bolt stack、head/nut座面および工具空間を含めて成立させる。
7. kinematics上で同じ角度になる部品には、実際のトルク伝達relationを必須とする。
8. relationが参照するdatumはkernelのface番号ではなく、core側のstable semantic datumとして保持する。
9. engineering toleranceとkernelのnumerical toleranceを別の型・設定として扱う。
10. gear meshはmodule、圧力角、中心距離、軸方向重なり、歯の位相および回転比を一つのrelationとして扱う。
11. kinematic constraint graphに閉路がある場合は、閉路全体のratioとphaseが矛盾しないことを検証する。
12. animation、collision、floor clearanceおよびmanufacturing outputは、同じassemblyとkinematicsをsource of truthとする。
13. manifestのsemantic claimはliteralで書かず、validatorが確認した事実から導出する。
14. `validate`が成功しない設計から正式な加工データや完成扱いのpreviewを生成しない。

一般的なallow-listによって干渉を隠すことは禁止する。例外は、意味と許容範囲を持つrelationとして表現する。

## 4. 判明している問題

| ID | 重要度 | 問題 | 影響 | 対応Phase |
| --- | --- | --- | --- | --- |
| A-01 | Blocker | `AssemblyRelation`が存在しない | 接触、締結、嵌合、干渉を区別できない | 1 |
| A-02 | Blocker | 正の交差体積を接続判定に使用した | 干渉を正常接続として扱う | 0, 2 |
| A-03 | Blocker | M3 boltに対応する実穴がない | 組立不能、DXFも加工不能 | 4 |
| A-04 | Blocker | clamp、gusset、key、hub、hangerの一部が別solidへ食い込む | 組立不能、干渉判定不能 | 3–5 |
| A-05 | Blocker | shaftとgear等を同じframeで動かすだけでトルク伝達構造がない | 物理的に運動が成立しない | 5 |
| A-06 | High | pitch sectorを局所的な肉盛りとoverlapで補強した | 干渉を増やし、正しい荷重経路を作らない | 3 |
| A-07 | High | `pitch_crossmember_*`が細い円筒だった | 固定frameの接合と荷重経路が不明確 | 3 |
| A-08 | High | gear meshのreference phaseを表現していない | 複数pinionが同時に噛み合わない可能性 | 6 |
| A-09 | High | 干渉検査が手書きpairと9姿勢だけ | 未列挙部品と中間姿勢を保証しない | 2, 7 |
| A-10 | High | `minimum_moving_z()`が実形状と別のmagic numberを持つ | clearanceのsource of truthが二重化 | 7 |
| A-11 | High | `gimbal validate`が部品単体meshしか検証しない | CLI成功がassembly成立を意味しない | 2 |
| A-12 | High | `Body::Sheet`が加工穴を保持できない | laser部品のnominal DXFが不完全 | 4, 8 |
| A-13 | Medium | `moving_crossbar_clamp`に締付機構がない | clampという名称と実機能が一致しない | 5 |
| A-14 | Medium | side/end等を`String`で分岐している | typoを静的に排除できない | 1 |
| A-15 | Medium | instanceの意味を名前prefixで判定するtestが多い | renameで検証が壊れる | 1, 2 |
| A-16 | Medium | inspection用3MFと製造用出力の責務が混在する | purchased/laser/FDM部品の扱いが曖昧 | 8 |
| A-17 | Medium | quality gateがCIで強制されていない | 公開branchで退行を防げない | 10 |
| A-18 | Low | laser bed dimensionがNaN/Infinityを拒否しない | 不正設定を通す | 8 |
| A-19 | Low | 一部previewがmanifestへ含まれない | artifact追跡が不完全 | 9 |
| A-20 | Blocker | constraint graphの閉路整合性を検証しない | 複数pinionとgearboxを個別には配置できても全体では拘束が矛盾する | 1, 6 |
| A-21 | High | relationが参照するstable datumの仕様がない | shape変更やBoolean後に接続先identityが壊れる | 1 |
| A-22 | High | engineering toleranceとnumerical toleranceを区別しない | 設計上のclearanceと計算誤差を混同する | 1, 2 |
| A-23 | High | backlashがgear単体とmesh relationへ重複し得る | source of truthが分散する | 6 |
| A-24 | High | `InternalGearPair`のinterference判定が不十分 | 一般APIが成立しないpairを受理し得る | 6 |
| A-25 | High | gear contact ratioを検証しない | 連続噛合いの成立を保証できない | 6 |
| A-26 | High | FDM build orientationと重要荷重方向を保持しない | 造形時に異方性を考慮した設計意図を失う | 8 |
| A-27 | High | manifestの機械的claimがhard-coded literal | 実モデルが条件を失っても`true`を出力できる | 2, 8 |
| A-28 | Low | pitch入力対carrier比の文書値が約175:1 | 現parameterから導かれる169:1と不一致 | 0 |

この一覧は完了条件ではない。Phase 2以降の全instance監査で新たに発見した問題も、この表へ追加する。

## 5. 目標domain model

### 5.1 Structured identity

表示名を機械判定に使わない。

```rust
struct ComponentInstanceId(u32);

enum Side {
    Left,
    Right,
}

enum LongitudinalEnd {
    Front,
    Rear,
}

enum VerticalEnd {
    Upper,
    Lower,
}

enum ComponentRole {
    PitchSector,
    FixedRail,
    FixedPost,
    FixedCrossmember,
    Shaft,
    Gear,
    Bearing,
    Cockpit,
    Fastener,
    // 必要な有限状態を追加する
}
```

`String`は表示・artifact名にだけ使用し、side/end/roleの分岐にはenumを使用する。

### 5.2 Stable semantic datum

relationはCAD kernelが返す一時的なface番号を参照しない。設計時に意味を持つdatumをcoreで定義し、component local coordinatesで保持する。

```rust
struct PointDatum {
    point: Point3,
}

struct AxisDatum {
    origin: Point3,
    direction: UnitVector3,
}

struct PlaneDatum {
    origin: Point3,
    normal: UnitVector3,
}

struct CylinderDatum {
    axis: AxisDatum,
    radius: PositiveLength,
}
```

少なくともbolt axis、shaft axis、bearing bore axis、mounting plane、gear mid-planeおよびgear reference directionをdatumとして表現する。datum IDはcomponent definition内でstableであり、kernel topologyの再番号付けに依存しない。

### 5.3 Tolerance model

```rust
struct EngineeringTolerance {
    linear: NonNegativeLength,
    angular: NonNegativeAngle,
}

struct NumericalTolerance {
    linear_epsilon: PositiveLength,
    volume_epsilon: PositiveVolume,
}
```

engineering toleranceは要求clearance、fit、公差および許容interferenceを表す。numerical toleranceは浮動小数点、tessellationおよびBoolean kernelの誤差を0とみなす境界であり、設計上のfitとして解釈しない。

### 5.4 Assembly relation

初期relation setは次を基本とする。実装時に用途のないvariantを先行追加しない。

```rust
enum AssemblyRelation {
    SurfaceContact(SurfaceContact),
    Fastened(FastenedJoint),
    CylindricalFit(CylindricalFit),
    Bearing(BearingJoint),
    Keyed(KeyedJoint),
    Clamped(ClampedJoint),
    GearMesh(GearMesh),
    InterferenceFit(InterferenceFit),
}
```

各relationは最低限、relation ID、関係するinstance ID、stable local datum、期待する自由度または拘束、engineering toleranceを持つ。単なるinstance名のpairや「このpairは無視する」という情報にはしない。

### 5.5 Constraint graph

kinematic relationは、自由度間の関係を概念的に次のaffine constraintとして表せるようにする。

```text
q_j ≡ a q_i + b  (mod relevant angular period)
```

`a`はgear ratioと回転方向、`b`はreference phaseである。gearのphaseは歯ピッチに対応する周期を持つため、単純な実数等式として比較しない。backlashを考慮する場合、nominal relationと実際の接触状態は許容角度区間を持つ。

constraint graphに閉路がある場合は、一周したratioの積が1で、phase残差が該当周期に合同であり、backlash由来の許容区間内にあることを検証する。個々のpairが成立しても、閉路全体が矛盾する設計は構築またはvalidationを成功させない。

### 5.6 Validation boundary

```text
Validated parameters
        ↓
Feature DAG + Assembly + Relations + Kinematics
        ↓
AssemblyValidator
        ├─ part geometry
        ├─ relation geometry
        ├─ all-pair interference
        ├─ gear phase / ratio
        ├─ motion samples or conservative bounds
        └─ fabrication readiness
        ↓
ValidationReport
        ├─ gimbal validate
        ├─ generate precondition
        └─ integration tests
```

validatorは`#[cfg(test)]`内へ閉じ込めない。CLIとtestが同じ実装を呼ぶ。

## 6. Phase計画

状態は`未着手`、`進行中`、`検証中`、`完了`、`保留`のいずれかとする。完了は、記載したexit criteriaをすべて満たした場合だけ付ける。

| Phase | 内容 | 状態 | 主な依存 |
| ---: | --- | --- | --- |
| 0 | 誤った不変条件の撤去とbaseline固定 | 完了 | なし |
| 1 | typed identityとAssemblyRelation | 進行中 | Phase 0 |
| 2 | 共通AssemblyValidator | 未着手 | Phase 1 |
| 3 | 固定pitch frameとsector荷重経路 | 未着手 | Phase 2 |
| 4 | M3締結とlaser sheet hole | 未着手 | Phase 2, 3 |
| 5 | roll/pitchの軸、軸受、clamp、hub、key | 未着手 | Phase 2, 4 |
| 6 | gear mesh、位相、gearbox伝達 | 未着手 | Phase 2, 5 |
| 7 | motion envelope、床、全pair干渉 | 未着手 | Phase 3–6 |
| 8 | fabrication outputの成立性 | 未着手 | Phase 4–7 |
| 9 | Blender model、静止画、MP4の更新 | 未着手 | Phase 7, 8 |
| 10 | CI、MIT公開前gate、最終監査 | 未着手 | Phase 9 |

### Phase 0: 誤った不変条件の撤去とbaseline固定

目的は、既知の誤設計を「一時的にtestが通る状態」として残さないことである。

作業:

- `positive intersection volume = connection`というtestと説明を撤去する。
- `docs/model-design.md`の16 mm backbone、positive-volume接続および旧clamp/gusset記述を撤回する。
- pitch入力対carrier比を現parameterから導出した169:1へ修正し、手書きの派生値を減らす。
- sectorの局所backbone、食い込むend clampおよび重なるlower gussetを撤去する。
- 現在の生成物はmechanically validatedではないことを設計文書に明示する。
- 修正前の失敗例と対象instanceを記録する。
- 現在の未コミット固定frame変更をレビューし、Phase 3へ持ち越す変更と破棄する変更を分ける。

Exit criteria:

- 正の交差体積を接続成功として要求するcode/test/documentが存在しない。
- `docs/model-design.md`とintegration testが新しい不変条件と矛盾しない。
- pitch gear ratioの文書値とcodeから導出される値が一致する。
- 既知の誤設計を完成済みとして主張する文書が存在しない。
- `cargo fmt --check`、`cargo check --workspace`、既存testの結果を記録する。

### Phase 1: typed identityとAssemblyRelation

作業:

- `ComponentInstanceId`を導入し、`Assembly::add_instance`からIDを得られるようにする。
- `AssemblyRelationId`を導入し、relationをstructured identityで参照できるようにする。
- `Side`、`LongitudinalEnd`、`VerticalEnd`、`ComponentRole`を導入または既存enumへ統合する。
- prototype builderからside/end文字列分岐を除去する。
- `PointDatum`、`AxisDatum`、`PlaneDatum`、`CylinderDatum`等のstable semantic datumを導入する。
- datumをkernel face番号ではなくcomponent local geometryへ結び付ける。
- engineering toleranceとnumerical toleranceを別の型として導入する。
- `AssemblyRelation`とrelationごとのdatum/engineering toleranceを導入する。
- kinematic constraintをratioとreference phaseを持つedgeとして表現できる基礎型を導入する。
- 表示名prefixに依存するsemantic testをstructured identityへ移行する。
- manifestへ出すsemantic metadataをhard-coded文字列ではなくstructured modelへ移行する。
- relation graphの参照先、有効性、重複およびself-relationをcoreで検査する。

Exit criteria:

- 機械的意味の判定に自由文字列を使用しない。
- relationが存在しないinstance pairと、relationで結ばれたpairを列挙できる。
- relationがstable datumを参照し、Feature DAGのnode追加やkernel評価順に依存しない。
- engineering toleranceを変えてもnumerical epsilonが暗黙に変化しない。
- constraint graphの閉路を列挙できる。
- `gimbal-core`の`no_std + alloc`を維持する。

### Phase 2: 共通AssemblyValidator

作業:

- kernel-backed `AssemblyValidator`とstructured `ValidationReport`を非test codeとして実装する。
- 全instance pairをbroad phaseで列挙し、relationに応じたpolicyを選ぶ。
- numerical tolerance以下のkernel noiseを設計上のfitから分離する。
- numerical toleranceを超える別instanceの正の交差体積を原則エラーにする。
- `SurfaceContact`の面距離、法線方向、接触面積を検証する。
- relation未定義の近接・接触をwarningまたはerrorとして報告するpolicyを定義する。
- `gimbal validate`と`gimbal generate`のpreconditionから同じvalidatorを呼ぶ。
- `#[cfg(test)]`にしかないassembly検査を共通実装へ移す。
- manifestの`pitch_sectors_ground_fixed`等のclaimをvalidator resultから導出する。

Exit criteria:

- `gimbal validate`が部品単体だけでなくassembly reportを出す。
- known-overlapを含むfixtureが失敗し、正しいface contact fixtureが成功する。
- numerical noise fixtureとengineering interference fixtureを別々に検証する。
- validation失敗時にsemantic claimを`true`としてmanifestへ書けない。
- 一般的なcollision allow-listが存在しない。

### Phase 3: 固定pitch frameとsector荷重経路

目的は、pitch sectorの歯面荷重を局所肉盛りではなく固定frameへ流すことである。

作業:

- 左右のupper/lower railを垂直材または必要に応じてtrussで接続する。
- 前後crossmemberを丸シャフトではなく矩形等の構造部材として構成する。
- crossmember、rail、postの接合位置を同一nodeへ揃える。
- 下部frameを床へ直接接地させ、追加の細脚を設けない。
- sector支持部を歯面接触領域から分離し、sectorからrail/postへ明確なload pathを作る。
- 部材同士を体積重複させず、接触面と締結位置を定義する。
- sectorの曲げ、歯元応力集中、左右frameのねじれを後続解析対象として明記する。

Exit criteria:

- 固定frameの全接続がrelationとして列挙される。
- rail/post/crossmember/sector支持部に意図しない正の体積交差がない。
- sectorから床までのstructural connectivityをrelation graphで追跡できる。
- Blender model上で浮いた部材、端面だけがずれた接続、細い丸棒crossmemberがない。

### Phase 4: M3締結とlaser sheet hole

作業:

- `Body::Sheet`または2D regionを、outer contourと複数cutoutを保持できる構造へ拡張する。
- M3 clearance hole、bolt head/nut座面、washerおよび工具accessをnominal geometryへ反映する。
- bolt、nut、washerをPurchased componentとしてdefinition/instance化する。
- `FastenedJoint`にbolt軸、締結対象layer、head/nut側およびstack長を持たせる。
- 2枚の板を積層する場合は厚み方向に別layerへ配置し、同一空間へ重ねない。
- DXFへhole contourを出力し、closed contourと単位を再読込検証する。

Exit criteria:

- すべてのM3 shankが同軸clearance hole内にあり、母材と交差しない。
- head/nut/washerの座面と必要空間が成立する。
- laser部品のDXFだけから締結穴を再現できる。
- boltを削除しても部品同士の接触関係、boltを追加すると締結関係が検証できる。

### Phase 5: 軸、軸受、clamp、hub、keyによるトルク伝達

対象には少なくとも次を含む。

- continuous roll shaft
- front/rear roll bearing
- cockpit hanger
- roll driven gear/hub
- pitch gearbox各shaft/gear
- moving crossbar clamp
- shaft keyおよびkeyway

作業:

- shaft/bore clearance、軸方向位置決めおよびaxial retentionをrelationで表す。
- bearingのinner/outer race側を区別し、固定側と回転側を明示する。
- 別体hubを採用するならface contactとfastenerを作り、一体造形なら同一definition内でUnionする。
- keyを使う箇所にはshaft/hub双方へ実keywayを作る。
- clampはslit、締付boltおよびclamping surfaceを持つ実形状にするか、別方式へ変更する。
- cockpit hangerはcockpitへ食い込ませず、接触面と締結を成立させる。
- kinematic co-motionごとに、そのトルクまたは荷重を伝えるrelationを対応付ける。

Exit criteria:

- 「同じframeだから回る」だけのshaft/gear pairが残っていない。
- roll shaftが前後で連続し、bearing supportとcockpit suspensionが矛盾しない。
- shaft、bearing、gearおよびhubの全件について軸方向に拘束される理由を追跡できる。
- key、hub、hanger、clampの別instance間に意図しない正の体積交差がない。
- cockpitがroll軸下へ吊られ、roll駆動喪失時の重力復元方向をkinematicsで確認できる。

### Phase 6: gear mesh、位相、gearbox伝達

作業:

- pitch sectorと2 drive pinion、retention pinionの各meshを`GearMesh`として定義する。
- front/rear roll gear、pinionおよび全gearbox stageも同じrelationで表す。
- module、圧力角、center distance、axial overlap、target backlash、歯数とmesh種別を検証する。
- tooth thickness modificationはgear geometry、target/actual backlashはmating pairの`GearMesh`をsource of truthとする。
- internal gearのinvolute、trochoidおよびtrimming interference条件を検証する。
- external/internal meshのtransverse contact ratioを計算し、要求下限を検証する。
- 各meshへreference phaseを与え、nominal poseで歯同士が干渉しないことを検証する。
- 1 unit内の2 drive pinionが同時に成立する位相と配置を導出する。
- GearMeshとgearbox constraintから得られる全閉路についてratio/phase consistencyを検証する。
- phaseは歯ピッチ周期に対する合同として扱い、backlash由来の許容角度区間と区別する。
- mesh relationから回転比を導出し、animation用に別の式を重複実装しない。
- gearboxの各shaft、bearing、gear固定とhousing clearanceを検証する。

Exit criteria:

- 全gear meshがtyped relationとして列挙される。
- nominal poseとmotion sampleで歯面の不正penetrationがない。
- ratioとphaseのtestに、意図的に1歯ずらした失敗fixtureを含む。
- constraint cycleに矛盾を注入したfixtureが失敗する。
- 周期分だけphaseをずらした同値fixtureと、backlash許容内外の境界fixtureを持つ。
- internal gear interferenceとcontact ratioの境界fixtureを持つ。
- backlashのsource of truthがpairごとに一つだけである。
- animationと検証が同一のgear relationから角度を得る。

### Phase 7: motion envelope、床clearance、全pair干渉

作業:

- `minimum_moving_z()`のmagic-number geometryを撤去する。
- 実solidとcore kinematicsからfloor clearanceを計算する。
- 全instance pairをrelation-aware policyで検査する。
- pitch/roll端点だけでなく中間姿勢を含むdense samplingを行う。
- sampled validationとcontinuous guaranteeを文書・report上で区別する。
- 必要なpairには保守的なswept boundまたはcontinuous collision手法を検討する。
- gear tooth contactは一般solid collisionと別のmesh-specific policyで扱う。

Exit criteria:

- 可動部と床の最小clearanceを実geometryから報告できる。
- 手書きのwatched pairだけに依存しない。
- reportに検査範囲、sampling間隔、最小clearance姿勢および未保証範囲が記録される。
- 静止画で見えない中間姿勢の回帰testを含む。

### Phase 8: fabrication outputの成立性

作業:

- inspection artifactとmanufacturing artifactを分離する。
- FDMはdefinition単位の3MF、laserはholeを含むnominal DXF、PurchasedはBOMへ出力する。
- FDM partごとにbuild orientationとcritical load directionをprocess metadataとして保持する。
- kerfとFDM compensationをnominal geometryから分離したprocess profileとして維持する。
- laser bed寸法をfiniteかつpositiveな型へ変換し、NaN/Infinityを拒否する。
- artifact manifestへ部品role、製法、数量、parameter hashおよびfile hashを記録する。
- manifestの機械的claimを`ValidationReport`から導出し、検査していない事項を`true`にしない。
- relation validatorが成功した設計だけを正式なfabrication outputとして扱う。

Exit criteria:

- FDM、laser、Purchasedの全部品が重複なく分類される。
- DXF round-trip、3MF unit/watertightnessおよびBOM quantityが通る。
- nominal designとprocess compensationのhashを区別できる。
- FDM artifactのbuild orientationとcritical load directionをmanifestから追跡できる。
- manifestのclaimごとにvalidator check IDまたは`not_validated`を追跡できる。

### Phase 9: Blender model、静止画、MP4の更新

作業:

- 新assemblyのglTF、Blender model、全方向previewを再生成する。
- pitch/roll motionを新しいrelation由来のkinematicsで再生成する。
- gimbal全体に加え、pitch gearbox、roll gearbox、固定frame jointおよびgear mesh detailを出力する。
- camera clipping、上下左右、色、白飛び、黒潰れ、環境光、床影を目視確認する。
- 生成するすべてのpreviewとMP4をmanifestへ含める。

Exit criteria:

- `gimbal-motion.mp4`、gearbox animation、PNG、`.blend`が新モデルと同一manifestから生成される。
- 動画で固定sectorが動かず、pinion/gearbox/roll assemblyがpitchで移動する。
- roll shaftとコクピが正しい親子関係で動く。
- detail renderで接続、穴、軸受、歯車位相を視認できる。

### Phase 10: CI、MIT公開前gate、最終監査

作業:

- GitHub Actionsでformat、clippy、workspace test、`no_std`、WASMおよびdependency auditを実行する。
- MIT公開とdependency licenseの互換性を検査する。
- secrets、large files、生成物、第三者assetおよびGit履歴を監査する。
- `validate`、`generate`、artifact再読込およびrender smokeをrelease候補commitで実行する。
- structured validator reportをCI artifactとして保存し、commit SHA、parameter hashおよびprocess profile hashと結び付ける。
- prototypeの安全境界と未検証事項をmanifest/reportへ残す。

Exit criteria:

- CIのrequired checksが成功する。
- repositoryにMIT Licenseがあり、同梱物の権利関係が説明できる。
- remote branchのSHAと検証対象SHAが一致する。
- CI artifactのvalidator reportが検証対象SHAと一致する。
- 2 m・50–80 kg実機への強度保証を行っていないことが明示される。

## 7. Phaseごとのcommit方針

- 各Phaseは、原則として専用commitへ分ける。
- Phase途中で一時的にtestを緩めてmainline相当のbranchへ残さない。
- geometry変更、relation追加、validator追加およびartifact再生成を無関係な1 commitへ混在させない。
- 各Phaseのcommit前にdiffを確認し、対応するexit criteriaをtest結果とともにこの文書へ記録する。
- Gitignoredなpreviewはcommitしないが、生成に使ったmanifest hashと確認結果を記録する。

## 8. 進捗記録

### 2026-09-01

- `5298fee`を監査baselineとして問題を記録した。
- 正の交差体積を接続とみなす方針を誤りと判定した。
- sector局所backbone、end clamp、lower gussetを撤去する未コミット変更がある。
- upper/lower railを結ぶvertical postと矩形crossmemberへ変更する未コミット試作がある。
- 上記変更はPhase 3の最終成果ではなく、relationとvalidator導入前の暫定作業である。
- `cargo check --workspace`と固定frame face-contactの限定testは通過しているが、assembly全体の成立性は未検証である。

### 2026-09-02

- remote HEAD `7416f26`に対する計画・実装・既存文書の再監査結果を反映した。
- Phase 1へstable semantic datum、engineering/numerical tolerance、relation IDおよびconstraint graphの基礎を追加した。
- Phase 6へconstraint cycle consistency、pair-level backlash、internal gear interferenceおよびcontact ratioを追加した。
- Phase 8へFDM build orientation、critical load directionおよびvalidator由来のmanifest claimを追加した。
- `docs/model-design.md`の16 mm backbone、positive-volume接続および旧clamp/gusset記述を撤回し、pitch比を169:1へ修正した。
- KHKのinternal gear資料でinvolute、trochoid、trimming interferenceと、組付中心距離を前提とするbacklashの扱いを確認した。
- UltiMakerのdesign guidanceでFDMが造形方向に依存する異方性を持つことを確認した。
- 旧16 mm backbone、overlap clamp、lower gusset、対応M3 fastenerおよびpositive-intersection接続testを実装から撤去した。
- Phase 3相当の矩形post/crossmember試作はPhase 0 commitへ混在させず、relation validator導入後に再実装する方針とした。
- `cargo fmt --all -- --check`、`cargo check --workspace`、`cargo clippy --workspace --all-targets --all-features -- -D warnings`および`cargo test --workspace`が成功した。workspace testは28件成功、失敗0件だった。
- `gimbal-core`のnative `no_std` checkと`wasm32-unknown-unknown` `no_std` checkが成功した。
- Phase 0のexit criteriaを満たしたため、Phase 0を完了、Phase 1を進行中へ変更した。
- Phase 1の最初のcheckpointとして、`ComponentInstanceId`、`AssemblyRelationId`、typed `DatumId<T>`およびappend-onlyなdatum setを実装した。
- stable semantic datumとしてpoint、axis、plane、cylinderを導入し、kernelのface番号や評価順に依存しないcomponent-local geometryとして保持する構造にした。
- engineering toleranceとkernelのnumerical toleranceを別型にし、正値・非負値の区別を`PositiveLength`等の型へ移した。
- `SurfaceContact`、`CylindricalFit`、`GearMesh`をtyped datum endpointで表現し、self relation、無効なinstance/datum、datum kind不一致および完全重複relationを`Assembly::add_relation`で拒否するようにした。
- 全instance pair、relationがあるpair、relationがないpairを列挙できるAPIを追加した。これはPhase 2の全pair validatorの入力とする。
- ratio、reference phaseおよびphase backlashを持つangular constraint graphを追加し、閉路を方向付きedge列として列挙できるようにした。閉路整合性の判定はPhase 6で実装する。
- side、longitudinal end、vertical endおよびcomponent roleをenum化し、現prototypeの繰返しinstanceへstructured locationを付与した。重複identityはtestで拒否する。
- manifestのdefinition/instance metadataへroleとstructured locationを出力するようにした。既存のhard-coded semantic claimはPhase 2でvalidator由来へ置換するまで未解決である。
- 実形状が未成立なjointへ先にrelationを付けることはしない。prototypeへのdatum/relation付与はPhase 3–6で各jointの形状を成立させるのと同じcommit系列で行う。
- `cargo fmt --all -- --check`、`cargo check --workspace`、warning-as-errorのClippyおよび`cargo test --workspace`が成功した。workspace testは35件成功、失敗0件だった。
- `gimbal-core`のnativeおよび`wasm32-unknown-unknown`の`no_std` checkが成功した。

Phase 1は引き続き進行中である。次は残存する表示名依存のsemantic testをstructured identityへ移行し、Phase 2で使用するrelation/identity traversalをfixtureで固める。実joint relationの登録は未成立形状を正当化しないよう、Phase 3–6の再設計と同時に行う。

## 9. 完成の定義

この再設計は、見た目が整った時点では完了しない。次のすべてを満たした時点で完了とする。

- すべての別instance間overlapが、干渉エラーまたは明示的なintentional relationとして説明される。
- すべての必要な接続がtyped relationとして存在し、実形状で検証される。
- すべてのkinematic couplingに、実際の荷重・トルク伝達構造が対応する。
- pitch/roll可動域、床clearanceおよびgear meshの検査範囲が機械可読reportとして残る。
- laser/FDM/Purchasedの加工・調達データが同じvalidated designから生成される。
- Blender model、静止画および動画がvalidated assemblyと一致する。
- CIと公開前監査が成功する。

## 10. 調査根拠

- [KHK Internal Gears — Technical Information](https://khkgears.net/pdf/2025/internal-gears.pdf)
- [KHK Spur Gears — Technical Information](https://khkgears.net/pdf/2023/spur-gears.pdf)
- [UltiMaker: Design for FDM 3D printing](https://ultimaker.com/learn/design-for-fff-3d-printing-maximize-your-success/)
- [開発方針](https://zenn.dev/bem130/articles/1b352797de94e7)
