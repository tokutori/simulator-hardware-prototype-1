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
| A-17 | Medium | 基本quality gateはCI化したが、validator artifactとrelease gateが未統合 | 検証対象SHAとartifactの一致をまだ自動保証しない | 10 |
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
| A-29 | High | 旧構成のinstanceまたはfeatureが、現在の接続相手・荷重経路・保持機能を失った後も残り得る | 不要部品、干渉、誤った組立意図および加工点数を増やす | 4–6 |
| A-30 | High | floor clearance testが手書きした一部instanceだけを対象とし、`RollGearboxMount`を漏らしていた | pitch端での床干渉を成功扱いする | 4, 7 |
| A-31 | High | `DatumId<T>`が発行元definitionを保持せず、同kind・同indexの別definition datumを誤受理し得る | relationが意図しない接続面・軸を参照する | 1 |
| A-32 | High | relation validatorが未対応relationを黙って通過し、reportが`complete: true`になり得る | 未検証jointを検証済みと誤表示する | 2, 5, 6 |
| A-33 | High | non-locating側608の内輪をshaft上でfloatさせる誤った配置を一時採用した | 回転嵌合面の摺動・frettingと軸位置不安定を招く | 5 |

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
    PlaneClearance(PlaneClearance),
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

巨大moduleとprototype固有依存の分離は[`architecture-remediation-plan.md`](architecture-remediation-plan.md)で別trackとして管理する。機械形状変更とsoftware refactorは同一commitへ混在させず、relation validationの網羅化をPhase 4完了前、振舞いを変えない主要module分割をPhase 5着手前に行う。

| Phase | 内容 | 状態 | 主な依存 |
| ---: | --- | --- | --- |
| 0 | 誤った不変条件の撤去とbaseline固定 | 完了 | なし |
| 1 | typed identityとAssemblyRelation | 完了 | Phase 0 |
| 2 | 共通AssemblyValidator | 完了 | Phase 1 |
| 3 | 固定pitch frameとsector荷重経路 | 完了 | Phase 2 |
| 4 | M3締結とlaser sheet hole | 進行中 | Phase 2, 3 |
| 5 | roll/pitchの軸、軸受、clamp、hub、key | 進行中 | Phase 2, 4 |
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
- `DatumId<T>`へ発行元`ComponentDefinitionId`を保持し、別definitionから借用したdatumをrelation endpointとして拒否する。
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
- relation endpointのdatumが、そのinstanceのcomponent definitionから発行されたことを検査できる。
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
- コクピ直下のmoving crossbarを撤去し、roll軸周辺またはコクピ上側へmoving carrierの前後・左右接続を移す。コクピ下側はfloor clearance用keep-out volumeとする。

Exit criteria:

- 固定frameの全接続がrelationとして列挙される。
- rail/post/crossmember/sector支持部に意図しない正の体積交差がない。
- sectorから床までのstructural connectivityをrelation graphで追跡できる。
- Blender model上で浮いた部材、端面だけがずれた接続、細い丸棒crossmemberがない。

### Phase 4: M3締結とlaser sheet hole

Phase 4の先頭でcomponent/feature existence auditを行う。締結を追加して旧形状を延命する前に、各instanceと各definition内featureを次のいずれかへ分類する。

1. 現在の荷重経路を構成する。
2. 現在の運動またはトルク伝達を構成する。
3. 現在の位置決め、軸方向保持または脱落防止を構成する。
4. 床支持、安全または工具accessを構成する。
5. 明示されたPurchased/reference geometryである。

どれにも該当しないinstanceは削除する。一つのdefinition内でも、廃止した相手部品のためのboss、rib、tab、逃げ、仮Union等はfeature単位で削除する。名前、色またはpreview上の見栄えは存在理由にしない。

作業:

- `Body::Sheet`または2D regionを、outer contourと複数cutoutを保持できる構造へ拡張する。
- M3 clearance hole、bolt head/nut座面、washerおよび工具accessをnominal geometryへ反映する。
- bolt、nut、washerをPurchased componentとしてdefinition/instance化する。
- `FastenedJoint`にbolt軸、締結対象layer、head/nut側およびstack長を持たせる。
- 2枚の板を積層する場合は厚み方向に別layerへ配置し、同一空間へ重ねない。
- DXFへhole contourを出力し、closed contourと単位を再読込検証する。
- 全instanceと主要なUnion/Difference featureについて、現在のrelation、load pathまたはkeep-out requirementへ到達する存在理由を監査する。
- 高精細gearを除外した高速な全可動instance対floor検査を常時gateとし、歯形固有検査は独立した低頻度routeへ分ける。

Exit criteria:

- すべてのM3 shankが同軸clearance hole内にあり、母材と交差しない。
- head/nut/washerの座面と必要空間が成立する。
- laser部品のDXFだけから締結穴を再現できる。
- boltを削除しても部品同士の接触関係、boltを追加すると締結関係が検証できる。
- 旧構成だけを根拠とするinstanceおよび内部featureが残っていない。
- 手書きの対象部品列挙なしで、全可動structural instanceが全sample姿勢で床上5 mm以上を保つ。

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
- 全pitch/roll gearbox input shaftへ低荷重手回し用のPH2-compatible cross recessを設け、工具accessと周囲clearanceを検証する。

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
- 1 unit内の2 drive pinionと1 retention pinionをsector上で離して配置し、支持スパン、sector端margin、carriage曲げおよびdistribution伝達経路を同時に成立させる。
- pitch gearboxとdistribution部をsectorの外側から左右frame間の内側へ移し、コクピ/roll機構keep-out、左右unit間隔、工具accessおよびpitch可動包絡を検証する。
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
- 現prototypeのcustom partは一旦すべてFDMとし、PLA/ABSをprocess profileで選択可能にする。LaserCutのdomain型、DXF exporterおよびtestは次prototype用に維持する。
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

- GitHub Actionsでformat、clippy、workspace test、`no_std`、WASMおよびdependency auditを実行する。基本workflowはPhase 4中に前倒し導入し、Phase 10ではrequired check化とvalidator artifact統合を完了する。
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

### 7.1 Component / feature existence audit

存在理由はcomponent名から推測せず、現行relation、荷重経路、運動拘束またはkeep-out requirementで判定する。2026-09-03時点の優先監査結果は次の通りである。

| 対象 | 判定 | 対応 |
| --- | --- | --- |
| `RollGearboxMount` 4個 | 削除 | 上側armとplateの隙間を埋めるだけの旧中継blockだったためrole/definition/instanceを撤去し、armをplateへ直接接続した |
| sector上下support | 維持 | 歯面荷重をpostへ渡し、中央のpinion通過域を空ける現在の荷重経路である |
| sector両側clevis cheek | 維持 | sector/postのface contactをM3で締結する現在のjoint形状である |
| `PitchGearboxTieRod` | 削除・置換済み | head/nut/washerのない仮円柱role/definition/12 instanceを撤去し、各gearbox 3本のM3x25 bolt、nut、両washer、実穴および`FastenedJoint`へ置換した |
| outboard plateのretention軸boss/boreと別`RetentionBearingBlock` | 削除・置換済み | rigid bossと別blockによる重複拘束を撤去し、inner/outboard支持板にbearing island、平行flexure、bridge、anchor ribを一体化した |
| `RetentionLeafSpring` | 削除・置換済み | 固定端・可動端のない8個のbox形状を撤去し、各支持板solid内の2本のradial flexure beamへ置換した。ばね定数、予圧および疲労はPhase 5で検証する |
| cockpit hangerのcockpit内2 mm延長 | 削除 | 接続をpenetrationで代用した旧featureを撤去し、cockpit上面とhanger下面を型付き`SurfaceContact`へ置換した。cockpit本体の実締結方式はPhase 5で確定する |
| `RollDrivenHub`、`RollDrivenKey`、`CockpitShaftKey` | 削除・置換済み | 実keywayを持たず別solidへ食い込んでいた6 instanceと3 definitionを撤去した。driven gear/hubを同一definition内でUnionし、連続shaftのgear/hanger stationだけをD-flat、相手側をclearance付きD-boreとした。軸方向保持とfit公差はPhase 5で引き続き検証する |
| drive/retention flange | 維持・再検証 | sectorからの軸方向脱落防止という現行機能を持つ。Phase 5でshaft retentionと工具accessを含めて再検証する |
| front/rear roll driven gearとgearbox | 維持・topology確定待ち | コクピ前後に置く要求に対応する。Phase 6でactive/passive分類と閉路整合性を確定する |

この表は削除対象だけでなく、維持する理由も記録する。featureを変更したcommitでは該当行を更新し、未監査の複合solidを完成扱いしない。

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

### 2026-09-03

- retention軸を同時に剛体拘束していた別`RetentionBearingBlock` 4 instanceと、固定端・可動端を持たない`RetentionLeafSpring` 8 instanceを撤去した。
- inner carriageとoutboard supportの双方に、bearing island、2本の平行flexure beam、moving/fixed bridgeおよびanchor ribを同一solidとして構成した。旧inboard encoder anchor boss/rib/holeも不要形状として撤去した。
- outboard support位置がpitch gearbox plateの寸法へ偶然依存していたため、`outboard_support_plate_offset`へ分離した。drive/retention flange外端との最小0.25 mm隙間をparameter validationで要求した。
- outboard supportとdrive/retention pinion、shaft、全flangeの高精細Boolean regressionが成功した。軽量structural routeは高精細gearを除外したまま維持する。
- 変更前の40 definitions・251 instancesから、38 definitions・239 instancesへ削減した。別部品を増やさず、必要機能を既存支持板の意味ある形状へ統合した。
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
- integration testからinstance名、prefixおよび`starts_with`による機械的対象選択を撤去し、`ComponentRole + ComponentLocation`による一意なselectorへ移行した。
- structured selectorへ移行後も、可動域、床clearanceおよびgearbox plate干渉を含むworkspace test 35件が成功し、失敗0件だった。
- Phase 1のexit criteriaを満たしたためPhase 1を完了し、Phase 2を進行中へ変更した。
- Phase 2の最初のcheckpointとして、definition metrics、全instance pair broad phase、kernel-backed exact intersectionおよびstructured issueを持つ`AssemblyValidator`/`ValidationReport`を実装した。
- 別instanceの面接触は干渉とせず、正の交差体積だけをerrorとするcube fixtureを追加した。
- `gimbal validate`と通常の`gimbal generate`を同じvalidatorへ接続した。validationに失敗したassemblyから通常artifactは生成できない。
- inspection用途に限り、fabrication outputを更新せず未検証であることをmanifestへ明記する`generate-preview`を追加した。これは通常のvalidation gateを緩和するものではない。
- 現行prototypeをzero poseで全pair検査した結果、全24,753 pair中637 pairがbroad-phase候補となり、410 pairでnumerical toleranceを超える正の交差体積を確認した。reportは`output/validation-report.json`へ出力した。
- 大きな既知干渉にはpitch unit lower/upper frame arm間約8,298 mm³、pitch end upper tie/roll bearing pedestal間約2,818 mm³およびroll driven gear/hub間約1,558 mm³が含まれる。M3 fastener、flange、plate、gear mesh等にも多数の干渉があり、局所修正ではなくPhase 3–6の再設計が必要であることを確認した。
- Manifoldの既定Booleanでnon-manifold edge panicが発生したため、pair検査をrobust Booleanへ統一し、panic時にinstance pairを含むerrorへ変換する境界を追加した。
- 637候補の逐次exact checkが日常gateとして遅すぎることを確認し、kernel adapter内だけでManifold parallel featureと決定的な32-pair chunk評価を有効化した。高速fixture gateとfull assembly gateは実行頻度を分ける。
- `generate-preview`とBlender adapterを用いて、現在の中間モデルから静止画、`.blend`、`gimbal-motion.mp4`、pitch gearbox動画およびroll gearbox動画を再生成した。manifestは`preview_only: true`かつmechanically invalid/unvalidatedと明記する。
- `SurfaceContact`について、stableな`PlaneDatum`をworld座標へ変換し、engineering linear/angular toleranceに基づく面間距離と対向法線を検証するrelation validatorを追加した。接触面積検証は引き続きPhase 2の未完了項目である。
- Phase 2 checkpoint時点でworkspace test 37件、format、workspace check、warning-as-errorのClippy、native/wasm32の`gimbal-core no_std` checkが成功した。
- 再生成後のpreview-only manifestに`.blend`、静止画8枚、MP4 3本、inspection model、animationおよびvalidation reportを含む19 artifactを記録し、全SHA-256が実ファイルと一致することを確認した。
- `SurfaceContact`にminimum contact areaを追加し、実solidを接触面座標へ変換して得た2断面の共通面積を検証するようにした。linear、area、volumeのnumerical toleranceはそれぞれ別の型付き値として扱う。
- relation未定義の面接触または指定距離以内の近接を`Ignore/Warning/Error`から選ぶtyped policyを追加した。一般的なcollision allow-listは導入していない。
- 通常の`gimbal validate`を`StructuralFast` scopeとし、高精細gear 9 definition/56 instanceを明示的に除外して、残る35 definition/13,861 pairをconservative proxy AABBで検査する経路を追加した。warm実行は約0.2秒、clean build込みで約5.2秒だった。
- `StructuralFast`は471件のpotential structural interferenceと14件の未定義近接を報告した。これは確定solid interferenceではなく、大域的修正を優先順位付けする保守的候補である。高精細gearとexact solid Booleanを含む検査は`gimbal validate-full`へ分離した。
- manifestから未検証の`pitch_sectors_ground_fixed: true`等のhard-coded boolean claimを撤去し、設計意図とvalidatorが確認した事実を分けた。
- positive-volume、face contact、numerical noise、engineering interference、contact area不足およびstructural scope除外のfixtureを追加した。Phase 2のexit criteriaを満たしたためPhase 2を完了し、Phase 3を進行中へ変更した。
- Phase 3の最初の配置修正として、4基のpitch gearboxをsector外側から左右sector間へ移した。sector mid-planeから内側へ近側支持板6.5 mm、第一gear layer 10.5 mm、遠側支持板24.0 mmとし、各寸法をvalidated parameterへ移した。
- 接触pinionの外側支持板とretention preload部だけをsector外側へ残し、減速gear、近側gearbox plate、遠側plateおよびtie rodを内側へ反転した。gearbox shaftは固定値でずらさず、2枚の支持板の中点から位置と長さを導出するようにした。
- 左右の遠側支持板間には52 mmの中央通路が残り、現在のコクピ幅45 mmを上回ることを自動テストした。これはX/Z方向を含む可動包絡の成立を意味せず、Phase 7までpreview-onlyである。
- 内側移設後のpreview model、静止画8枚、`gimbal-motion.mp4`、pitch/roll gearbox動画および`.blend`を再生成した。軽量構造検査では507件のproxy候補を得ており、中央側の既存構造との整理をPhase 3で継続する。
- コクピ直下の2本のmoving crossbar、実機能のなかったannular clampおよび実穴のない仮M3 fastenerをmodelから撤去した。上側moving carrierをroll軸上24 mmの左右長手材と前後矩形tieへ置換した。
- contact carriageから上側長手材へ伸びる2本のtruss webは、別instance同士のoverlapを接続扱いせず、FDM carriage plate definition内でUnionした。roll bearing pedestalは、roll軸のbearing bossから上側carrier rail内面までを2本のribとbridgeで結ぶ一体のhangerへ置換した。roll gearbox支持はコクピ前後で上側tieから下ろす斜材へ変更した。
- 固定pitch frameの前後crossmemberは円形shaftから幅12 mm・高さ8 mmの矩形構造部材へ変更し、左右railの内面間へ配置した。下側railと同じ8 mm高さを共通parameterにして床上面とのface contactへ揃えた結果、床への2 mm penetrationを解消した。軽量構造検査のproxy候補は507件から259件へ減少したが、これは機械的成立を意味せず、残候補を引き続きrelationと実joint形状へ置換する。
- 固定frameへ前後4本の垂直postを追加し、upper/lower rail、矩形crossmember、floorおよびsector supportの24箇所をstable plane datum付き`SurfaceContact`として登録した。旧`SectorToRailLink`は撤去し、sector自身へpinion通過域を避けた上下分離supportをUnionした。
- relation graph上で4つすべてのsectorからpost、lower railを経てfloorへ到達できることを自動検査した。moving carrier側もrail、carriage mounting pad、front/rear carrier end、L形roll gearbox supportおよびmount plateの20箇所を`SurfaceContact`として登録し、全44 contact relationをsemantic identityで検査する。
- pitch contact unitを径方向の内2 drive・外1 retentionから、外2 drive・内1 retentionへ反転した。drive pinion偏角を±7.5度として中心間隔を約18 mmから約40.7 mmへ広げ、pitch限界±20度でもsector端へ2.5度を残す。gearbox本体のY方向配置は引き続き左右sector間の内側とした。
- 離した2 branchへ共通54T distribution gearを配置し、既存54T driven gearと同じFeature DAG solidを再利用した。pitch inputの相対回転比は現構成から62:1へ更新した。複数meshの位相閉路と荷重均等化はPhase 6まで未検証である。
- 新しい38 definition・187 instanceのpreview model、静止画8枚、`.blend`、`gimbal-motion.mp4`、pitch gearbox動画およびroll gearbox動画を再生成した。manifestは20 artifactを記録し、全SHA-256一致を確認した。動画はH.264、720×540、12 fps、6秒である。
- 高速検証は高精細gear 10 definitionを除外し、201件のconservative proxy候補と12件の未定義近接を報告している。これは確定干渉ではない。±20度の全pair motion sweepはPhase 7まで未完了であり、可動域成立をまだ宣言しない。
- Phase 3 checkpointでworkspace test 44件、format、warning-as-error Clippy、native/wasm32の`gimbal-core no_std` checkが成功した。主要moving structure対floor、短縮コクピ対roll support、upper carrier対roll mountおよびgearbox plate対gear/shaftの列挙済みexact干渉testは成功したが、全pair・全中間姿勢の証明ではない。
- 固定構造17部品の全136 pairと、固定・可動構造の全44 `SurfaceContact` pairを高精細solid Booleanで検査するtestを追加した。この検査でsector端と4本のpostに各約19.915 mm³、sector歯先と4本のupper railに各約3.196 mm³の正体積交差を検出した。
- post接合面をsector端の内歯歯先包絡より2.39 mm内側へ移し、upper rail下面を外歯歯先包絡より1.20 mm上へ移した。両clearanceをparameter検証へ追加し、sector一体supportとpost/railのface contactは維持した。修正後は全136 fixed pairと全44 contact pairで正体積交差0を確認した。
- 修正後の38 definition・187 instanceからinspection model、Blender model、静止画8枚およびMP4 3本を再生成した。manifestの20 artifactは全SHA-256一致、動画はすべてH.264、720×540、12 fps、6秒である。
- Phase 3最終gateとしてworkspace test 46件、format、warning-as-error Clippy、native/wasm32の`gimbal-core no_std` checkが成功した。Blender modelで固定frame、sector一体support、post、upper/lower railおよび矩形crossmemberの端面接続を確認し、Phase 3のexit criteriaを満たしたためPhase 3を完了、Phase 4を進行中へ変更した。
- Phase 4の基盤として、`Body::Sheet`をouter contourと複数cutoutを保持する構造へ拡張し、DXFへ各contourを閉じた`CUT` polylineとして出力・再読込検証する経路を追加した。現prototypeをFDM前提とする方針は変えず、次prototypeのlaser部品で同じnominal hole geometryを使用できるようにする。
- M3 bolt/nut/washerを個別のPurchased component roleとして型付けし、2部材のhole cylinder datum、座面plane datum、head/nut側、radial clearanceおよびgrip長を持つ`FastenedJoint`を追加した。relation挿入時にdatum所属、全participant IDの存在・一意性とhardware roleを検査し、kernel validatorで穴軸、穴径、座面法線およびgrip長を検査する。これはrelation/DXF基盤のcheckpointであり、prototype実部品への実穴・hardware配置・工具空間適用は未完了である。
- このcheckpointでworkspace test 49件、format、warning-as-error Clippy、native/wasm32の`gimbal-core no_std` checkが成功した。hardware role検査追加後の対象unit testも再実行して成功した。
- sector–post接続を、別solidのoverlapではなくsector一体の両側clevis cheek、postの実M3 clearance hole、M3x20 bolt、nut、両washerおよび8件の`FastenedJoint`へ置換した。8 jointの全participant pairについて高精細Boolean intersection volumeが0であることを確認した。
- 2026-09-03の追加監査を現HEADへ照合した。±4度branch、sector端margin式、固定post不足およびcarriage/rail overlapの指摘は既に現設計で解消済みだったため再実装しない。一方、全可動部品対床、drive participation、shaft/bearing/spring、relation validation coverage、artifact staging、FDM orientationおよびCIは未完了として各Phaseへ維持する。
- floor testの手書き`watched`配列を撤去し、高精細gearを除く全可動instanceをframe poseから自動抽出する0.05秒級のstructural routeへ置換した。これにより旧`roll_gearbox_*_carrier_mount_*`がpitch端で床へ3.28 mm干渉することを検出した。
- `RollGearboxMount`は上側armとgearbox plateの間を埋めるだけで独立部品としての機械的役割がなかったため、role、definitionおよび4 instanceを削除した。armをplateの実接触面まで延長し、plate側support tabを8.2 mm上げた。変更後は自動列挙された全可動structural instanceが9 sample姿勢で床上5 mm以上を満たした。
- 必要な`CockpitHanger`内部に残っていたcockpit内への2 mm延長をfeature単位で削除した。hanger下面とcockpit上面へstable plane datumを付け、2箇所の`SurfaceContact`を登録した。全42件のstructural contactについてexact solid overlapが0であり、高速floor sweepも維持されることを確認した。
- `PitchGearboxTieRod`という名称だけM3だった12本の円柱を削除し、既存の両plate実穴をstable cylinder datumとして公開した。4 unitそれぞれをM3x25 bolt 3本、nut、両washerおよび計12件の`FastenedJoint`で締結する構成へ置換し、全joint participant pairのexact intersection volumeが0であることを確認した。
- commit `5e98bc1`から旧outputを全消去してpreviewを再生成した。40 definition、251 instanceのassembly、静止画8枚、`.blend`、`gimbal-motion.mp4`およびpitch/roll gearbox動画を同じglTFから生成し、manifest記載18 artifactのSHA-256が全件一致した。正式加工可否は引き続き`preview_only=true`、`validation.valid=false`である。
- 2026-09-03の追加監査で、`DatumId<T>`が発行元definitionを保持しないため、同kind・同indexのforeign datumを誤受理できることを確認した。`DatumSet`をdefinition IDに所有させ、relation endpoint検証でinstance definitionとの一致を必須にした。異なるdefinitionの`PlaneDatum[0]`を借用する回帰testで`DatumOwnerMismatch`を確認した。空のdatum setだけはdatumを発行できないためunownedを許容する。
- `FastenerHardware`をinstance IDだけでなく、boltの軸・頭下面・shank先端、nutの軸・bearing面・外面、washerの軸・両面をtyped datumで参照する構造へ変更した。共通validatorは全hardware軸の同軸度、member–washer–bolt/nut間の座面接触、M3 nutの最小full-thread engagement 2.4 mmおよびbolt先端の1 pitch以上の突出を検査する。意図的にhardware軸を0.2 mmずらしたfixtureと短いbolt fixtureが失敗し、現prototypeのsector–post 8 jointとpitch gearbox 12 jointが新検査を通過した。
- 全relationへ`Validated`、`Failed`、`SkippedByScope`、`Unsupported`のcoverage statusを割り当て、CLI reportの`complete`をstatusから導出するようにした。未実装の`CylindricalFit`または`GearMesh`を含むreportは、error issueが0件でも`complete=false`かつ`valid=false`になる。これによりA-32の誤成功経路を塞いだ。個別relation validatorへのmodule分割はarchitecture A1で継続する。
- 実keywayを持たず別solidへ食い込んでいた`RollDrivenHub`、`RollDrivenKey`および`CockpitShaftKey`の6 instance・3 definitionを撤去した。driven gearとhubを一つのFDM definition内でUnionし、前後を切断しないroll shaftのgear/hanger stationだけへD-flat、相手側へnominal 0.15 mm clearanceのD-boreを設けた。shaft、前後driven gearおよび2個のhangerの全組合せについて高精細Boolean intersection volumeが0である回帰testを追加して成功した。これは旧overlapの撤去であり、軸方向保持、typed fit relationおよび製造公差の成立を意味しないためPhase 5は未完了のままとする。
- 上記変更後に旧`output`を全消去し、35 definition・233 instanceのpreview model、静止画8枚、`.blend`、`gimbal-motion.mp4`およびpitch/roll gearbox動画を再生成した。manifest記載20 artifactのSHA-256は全件一致し、3動画はいずれもH.264、720×540、12 fps、6秒である。static structural-proxy validationは高精細gear 10 definitionを除外した25 definitionから221件のconservative candidateを報告して失敗しており、正式加工可能とは扱わない。
- このcheckpointでworkspace test 55件、format、warning-as-error Clippy、`gimbal-core`と`geared-gimbal-design`のnative/wasm32 `no_std` checkが成功した。
- Phase 10を待たず、push/PRごとにformat、workspace check、warning-as-error Clippy、workspace test、両`no_std` crateのnative/WASM checkおよび`cargo-audit 0.22.2`を実行するGitHub Actions workflowを追加した。これはA-17の基本gate部分だけを前倒しするものであり、structured validator reportのartifact保存、検証対象SHAとの照合、branch protectionおよび公開前監査はPhase 10に残す。
- GitHub Actions再run `33681538072`でRust quality gatesは7分8秒、dependency auditは3分1秒で成功した。Linux Rust 1.98でのみ先行して有効になったClippy lintも修正済みであり、基本CIを実行可能なgateとして確認した。
- `CylindricalFit`の共通validatorを実装し、datum originの一致、軸方向、shaft/bore半径差とtarget radial clearanceをengineering toleranceで検証するようにした。これでrelation variantのうち`SurfaceContact`、`Fastened`、`CylindricalFit`が検証可能となり、未実装は`GearMesh`だけになった。実prototypeのroll shaft/bearing datumとrelation登録はPhase 5の次checkpointとする。
- Phase 5の最初のrelation checkpointとして、連続roll shaftの前後journal、各bearingのinner bore/outer surfaceおよびcarrierのbearing boreをstable cylinder datumで表した。shaft–bearing内径2件とbearing外径–carrier bore 2件、合計4件の`CylindricalFit`を登録し、当初のreference clearance 0.15 mm/0.20 mm、datum origin、軸方向および半径差が共通validatorを通ることを回帰testで確認した。このreference clearanceは次の608寸法checkpointでnominal 0 mmへ置換した。
- 8 mm連続roll shaftに対する購入軸受の寸法基準として、NTN公式product dataに基づく608 seriesの8 × 22 × 7 mm envelopeをparameterへ追加した。旧18 mm外径の無銘reference形状と、nominal geometryへ焼き付けていた0.15/0.20 mm radial clearanceは撤去し、nominal fitは0 mm、実shaft公差とFDM穴補正はprocess layerで扱う。seal形式、carrierへの挿入方法、outer/inner raceの軸方向保持は未確定である。
- 608 envelopeへの形状変更後、旧`output`を全消去して35 definition・233 instanceのinspection model、Blender model、静止画8枚およびMP4 3本を再生成した。manifest記載18 artifactのSHA-256は全件一致し、3動画はいずれもH.264、720 × 540、12 fps、6秒である。isometric、左側面およびroll gearbox detailを目視し、軸受外径変更による明白なframe/cockpit干渉やcamera見切れがないことを確認した。正式加工可否は引き続き`preview_only=true`である。
- 608外輪の軸方向保持として、carrier一体の1 mm内側shoulderと3 mm FDM retainer plateを追加した。各端3本、合計6本のM3x20 bolt、nutおよび両washerでretainerをcarrierへ締結し、外輪両面とshoulder/retainer、retainerとcarrierの面接触をtyped `SurfaceContact`として登録した。6件の`FastenedJoint`はhardware軸・座面・thread engagementを共通validatorで検証し、bearing/carrier/retainerの全接触pairはexact Booleanで正の体積交差が0であることを回帰testへ固定した。これはouter raceのnominal axial retentionだけを確定するcheckpointであり、inner raceとshaftの軸方向保持および実FDM fit公差は未完了である。
- 新しい軸受保持締結によって全M3数と全SurfaceContact数が増えるため、旧固定件数だけに依存していたintegration testを撤去した。pitch gearbox testは対象12 jointごとにbolt、nutおよび2枚のwasherのroleを検査し、surface contact testは宣言された全relationを走査する。別subsystemの正当な追加が無関係なtestを壊す構造を残さない。
- commit `3edd35f`から旧`output`を全消去し、36 definition・259 instanceのinspection model、Blender model、静止画8枚およびMP4 3本を再生成した。roll gearbox detailにはfront側の608 bearing、carrier end、retainer plateおよびそのM3 hardwareもsemantic roleから選択して表示する。manifest記載18 artifactのSHA-256は全件一致し、3動画はいずれもH.264、720 × 540、12 fps、6秒である。isometric、左側面およびroll gearbox detailを目視し、外輪保持追加によるcamera見切れ、白飛びまたは明白なcockpit/frame干渉がないことを確認した。正式加工可否は引き続き`preview_only=true`、`validation.valid=false`である。
- roll shaftの軸方向位置決めを前側608だけで行い、後側608をaxially floatingとする構成を採用した。前側内輪の両面へ、NBK公式の608ZZ用`NSCS-8-8-SB1`寸法に基づく8 mm clamp collarを2個配置した。購入部品のnominal body modelは外径20 mm、幅8.5 mm、内輪当接boss径11.7 mmであり、outboard側retainerには1 mmの外輪保持lipを残した段付きcounterboreを追加してcollar本体との干渉を避ける。2件のcollar–shaft `CylindricalFit`、2件のcollar–inner-race `SurfaceContact`および全関連pairのexact non-intersectionを回帰testにした。後側へcollarを置かないこともtestし、両bearingを軸方向に剛固定する過拘束を防ぐ。付属M3 clamp screwの突出・工具envelope、collar保持力、shaft公差およびFDM bore補正は未検証なのでPhase 5は継続する。
- commit `bc5c7dc`から旧`output`を全消去し、37 definition・261 instanceのinspection model、Blender model、静止画8枚およびMP4 3本を再生成した。roll gearbox detailへ前側608の内外2個のshaft collarを追加し、isometric、frontおよびdetailを目視してcamera見切れ、白飛びまたは新規部品の明白な干渉がないことを確認した。manifest記載18 artifactのSHA-256は全件一致し、3動画はいずれもH.264、720 × 540、12 fps、6秒である。正式加工可否は引き続き`preview_only=true`、`validation.valid=false`である。

- 上記の「後側にcollarを置かず内輪をshaft上でfloatさせる」判断は撤回する。SKFのlocating/non-locating bearing arrangementでは、回転軸へinterference fitする内輪を両側で軸方向に固定し、非分離型軸受の反対側ringをhousing seat上で移動させる。両608内輪をそれぞれ2個のcollarでshaftへ固定し、後側外輪だけへcarrier boreのnominal 0.15 mm radial slide clearanceとoutboard stopまで1.0 mmのaxial travelを与える構成へ修正した。後側retainerはbearing zoneを1.0 mm recessした専用definitionとし、`PlaneClearance` relationで外輪端面とstopの距離、法線および投影重なり面積を検証する。前側外輪は従来どおりshoulder/retainerで位置決めする。この訂正により、inner ringが回転shaft上で摺動・frettingする誤った荷重経路を除いた。

次の作業はPhase 4とPhase 5を依存順に進める。残る全instanceとdefinition内featureの存在理由監査を続け、不要形状を削除した上で、FDM前提の固定frame接合をM3通しbolt、実穴、washer/nut座面および工具空間を持つ実jointへ置換する。同時にroll軸系はcollar clamp screwの工具空間、保持力、購入shaft公差、後側外輪slide fitおよびFDM bore補正を確定する。relation coverage reportは実装済みであり、今後追加するrelationも未対応なら`Unsupported`として正式生成を失敗させる。LaserCutの`Body::Sheet` hole表現とDXF経路は次prototype向けに維持するが、現prototypeのcustom partはFDMを前提とする。

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
- [NBK NSCS-SB: Set Collars for Securing Bearing](https://www.nbk1560.com/images/en/product/setcollar/NSCS-SB/NSCS-SB_1.pdf)
- [SKF Super-precision bearings catalogue: locating and non-locating bearing arrangements](https://cdn.skfmediahub.skf.com/api/public/0901d19680495562/pdf_preview_medium/Super-precision_bearings_catalogue_-_13383_2_EN_pdf_preview_medium.pdf)
- [SKF High-speed spherical roller bearings: non-locating outer-ring displacement](https://cdn.skfmediahub.skf.com/api/public/0901d1968080459c/pdf_preview_medium/17857_EN_VA991_High_speed_spherical_roller_bearings_pdf_preview_medium.pdf)
- [開発方針](https://zenn.dev/bem130/articles/1b352797de94e7)
