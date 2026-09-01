# Double-helical rack-and-pinion experiment

FDM造形を前提とした、手回し式ラック・ピニオン試験機の CAD-as-code モデルです。

- 平歯車2段: module 1.8、pressure angle 25°、ハンドル軸12T → D大径31T、D小径12T → 左右B/C 28T（約6.03:1、各段変更可能）
- 最終段: normal module 2.0、20T、helix angle 15°のダブルヘリカルピニオン
- ラック: 両面歯、全歯幅20 mm、中央ギャップなし、24歯で約156 mm
- B/C: ラック片側に置く2個の駆動ピニオン
- A: B/Cの中間かつラック反対側に置く、無動力の受動ピニオン
- 支持: M6×40ボルト、下プレート一体中空軸／支柱、独立スラストスペーサー、上プレート嵌合穴

このexperimentでは、中央ギャップのないherringbone形状そのものがラックの軸方向変位をどの程度抑えるかを観察します。このため、結果を隠すフランジやケース側の軸方向ガイドは意図的に設けません。

金属シャフトや軸受はモデルに含めません。D/B/C/Aでは、下プレートから立つ外径10 mmの印刷中空軸上で回転部品が回ります。中空軸と四隅支柱は下プレート一体で、上プレート下面の深さ2 mmの嵌合穴へ1.5 mm入ります。直径方向と奥行き方向のクリアランスは各0.5 mmです。別体のスラストスペーサーがギヤの軸方向移動を上下合計1 mmに抑えます。Dは大小平歯車を段間ギャップなしで接合した複合部品です。B/Cは下側ヘリカル歯を2 mm延長して28T平歯車へ直接接続し、テーパーと空中へ張り出す面をなくしています。Aは今回はセンサーを付けません。

ハンドル12Tギヤは印刷回転軸と角ドライブを一体化し、上プレートを貫通します。下側軸から平歯車歯底までは高さ4.5 mmの支持テーパーで接続し、下プレートにも回転テーパーに沿う逃げ穴を設けます。ギヤ上側には軸方向拘束スペーサーを置きます。半径40 mmの着脱式クランクはM6ボルト頭との干渉を避ける高さに置き、別体ノブをM6×40で取り付けます。

## 生成

```powershell
cargo run -p double-helical-cli
```

`output/` に次を生成します。

- `prototype-assembly.obj` / `.mtl`: Blenderなどで開く色付き組立モデル
- `prototype-assembly.blend`: 部品別オブジェクト、mm単位、カメラ、照明を設定したBlenderモデル
- `prototype-blender.png`: Blenderで検証レンダーした画像
- `prototype-compounds.png`: D/B/C複合ギヤを積層方向の側面から見た印刷性確認画像
- `prototype-case-fit.png`: 下プレート一体軸／支柱と、反転した上プレート嵌合穴の確認画像
- `prototype-handle.png`: ハンドル歯車一体軸と下側支持テーパーの側面確認画像
- `prototype-assembly.3mf`: 部品を個別オブジェクトとして保持する組立モデル
- `prototype-assembly.stl`: 組立確認用の単一STL
- `prototype-preview.scad`: OpenSCAD確認シーン
- `handle-shaft-spur.stl`, `handle-crank.stl`, `handle-knob.stl`: ハンドル部品
- `reduction-d-compound.stl`, `driven-b-compound.stl`, `driven-c-compound.stl`, `idler-pinion.stl`, `double-helical-rack.stl`: 回転・摺動部品
- `top-frame-plate.stl`, `bottom-frame-plate.stl`: 上ケースと、固定軸／四隅支柱一体の下ケース
- `handle-upper-thrust-spacer.stl`, `reduction-d-upper-thrust-spacer.stl`, `driven-lower-thrust-spacer.stl`（2個印刷）, `idler-lower-thrust-spacer.stl`: 軸方向拘束部品
- `report.txt`: 寸法、三角形数、噛み合い体積干渉

Blenderでは `prototype-assembly.obj` をインポートします。色を読むため、同じディレクトリの `.mtl` を移動しないでください。

Steam版Blenderから `.blend` と検証レンダーを再生成する場合:

```powershell
& 'C:\Program Files (x86)\Steam\steamapps\common\Blender\blender.exe' `
  --background --python .\scripts\export_blend.py -- `
  .\output\prototype-assembly.obj `
  .\output\prototype-assembly.blend `
  .\output\prototype-blender.png `
  .\output\prototype-compounds.png `
  .\output\prototype-case-fit.png `
  .\output\prototype-handle.png
```

OpenSCADがPATH上にある場合、確認画像は次のように生成できます。

```powershell
openscad.com -o .\output\prototype-preview.png --imgsize=1600,1100 --projection=p --colorscheme=Tomorrow --camera=0,0,0,55,0,25,520 .\output\prototype-preview.scad
```

## パラメータ

寸法とメッシュ分割数は UTF-8 の `parameters.toml` で変更できます。既定値では最終ピニオンのピッチ径は約41.41 mm、ラック長は約156.12 mm、ケース平面寸法は130 × 130 mm、外寸厚38 mmです。

## 検証

```powershell
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo run -p double-helical-cli
```

設計上の定義、積層方向、未検証範囲は [docs/design.md](docs/design.md) を参照してください。
