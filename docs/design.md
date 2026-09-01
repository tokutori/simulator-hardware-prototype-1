# ダブルヘリカルラック・ピニオン試験設計

## 試験構成

中央Dを大小平歯車の複合部品とし、独立したハンドル軸からD大径側へ入力する。平歯車moduleは1.25、初期値はハンドル軸18TからD大径36Tを2:1、D小径18Tから左右B/C 40Tを約2.22:1とする。全体は約4.44:1だが、この比自体は要件ではなく、各歯数を設定から変更できる。

```text
                 A: passive 20T
                        O
                        │
             double-sided rack
        ═════════════════════════
              │             │
         B: driven 20T  C: driven 20T
              O             O
              │             │
         B: 40T spur    C: 40T spur
                \       /
              D-small: 18T
                    │ same shaft
              D-large: 36T
                    │
          handle-shaft spur: 18T
```

B/Cはそれぞれ40T平歯車と駆動20Tピニオンを接続した一体部品とする。Dも18T小径歯車と36T大径歯車を接合した一体部品とする。D小径がB/Cの両方へ同時に噛むため、B/Cは同方向へ回転してラックを送る。AはB/Cの中間かつラック反対側へ置き、3接触点でラック姿勢を安定させる。Aは動力系へ接続せず、今回はセンサーも付けない。

B/C間隔は平歯車中心距離36.25 mmで決まるため、各最終ピニオンのラックに対する歯位相を複合部品内で個別に合わせる。これによりDとの噛み合いと、B/C両方のラック噛み合いを同時に成立させる。

D大径とD小径は軸方向の隙間を設けず、大径側を下にして小径側を直接積層できる形状とする。B/Cでは下側ヘリカル歯をラック有効歯幅の外へ2 mm延長し、40T平歯車上面へ直接つなぐ。40T平歯車のroot radiusはヘリカルピニオンのtip radiusより大きいため、延長歯の最下層を全周で支持できる。円錐台テーパーは設けない。

## ダブルヘリカル形状

このモデルは一体FDM造形を前提とし、中央逃げ溝を設けず左右のねじれ歯を中央で連続させる。厳密には herringbone geometry に相当する。

- normal module: 2.0 mm
- normal pressure angle: 20°
- helix angle: 15°
- pinion teeth: 20
- 左右歯幅: 各10 mm
- center relief: 0 mm
- 全歯幅: 20 mm

normal systemから軸直角断面へ変換する。

```text
transverse module = normal module / cos(helix angle)
transverse pressure angle = atan(tan(normal pressure angle) / cos(helix angle))
```

既定値でtransverse moduleは約2.071 mm、20Tピニオンのピッチ径は約41.411 mmとなる。中央の歯先まで連続させ、歯底より内側の円板またはラック本体も連続させる。

ラックは上下両面に同じ歯形を持つ。Z方向へ進むごとにX方向へ歯形をshearし、左右halfではshear方向を反転する。ピニオンはインボリュート断面をZ方向へtwist extrusionする。ラックは偶数24歯、約156.12 mmとし、130 mm角のケースを貫通して移動する。

## 印刷部品と締結

金属シャフトは使わない。D/B/C/Aと四隅ではM6×40ボルトを下プレート一体中空軸／支柱へ通して締結する。ハンドル軸だけは印刷回転軸とする。

- plate hole / bolt clearance: 6.4 mm
- printed journal outside diameter: 10.0 mm
- rotating part bore: 10.4 mm
- thrust spacer outside diameter: 15.0 mm
- top/bottom plate thickness: 4.0 mm
- plate plan size: 130 × 130 mm
- plate inner spacing: 30.0 mm
- frame outside thickness: 38.0 mm
- M6 nut pocket: across flats 10.4 mm, depth 3.0 mm
- bolt engagement in 5 mm nut: 5.0 mm
- integrated post length: 31.5 mm
- top socket: depth 2.0 mm, diameter clearance 0.5 mm, axial clearance 0.5 mm

外径10 mmのD/B/C/A中空軸と外径15 mmの四隅支柱は下プレートへ一体化する。各先端は上プレート下面の対応穴へ1.5 mm入り、深さ方向に0.5 mm残す。受け穴直径は各支柱より0.5 mm大きい。別体の外径15 mmスラストスペーサーをハンドル上側、D上側、B/C下側、A下側へ配置し、各回転部品の上下合計軸方向遊びを1.0 mmとする。下プレートにはD/B/C/Aと四隅について六角ナットポケットを設ける。標準厚5 mmのナットはプレート下面へ約2 mm出るが、M6×40のねじが5 mm全体に掛かる。

ハンドル18Tギヤには外径10 mmの印刷回転軸と10 mm角ドライブを一体化する。上下プレートの10.4 mm穴で支持し、上側へ着脱式40 mmクランクを差し込む。クランクはM6ボルト頭を越える高さに置く。ノブは外径15 mm、長さ25 mm、M6通し穴を持つ別部品とする。

これは初期クリアランスであり、プリンタ、材料、ノズル径、収縮、穴補正に合わせたcoupon試験が必要である。ボルトのねじ部を摺動面へ直接当てる構成ではない。

## FDM積層方向

- 円形ギヤと上下プレートは、軸をZ方向にして平置きする。
- ラックは、長手方向Xと歯高方向Yをbuild plate面内へ置き、ラック幅をZ方向にする。
- 中央部を薄いneckにせず、左右halfの内部axial loadを連続bodyへ流す。
- hub、歯底、穴周囲は急な断面変化を避ける。初版にはfilletをまだ実装していないため、負荷試験前に追加検討する。

## ソフトウェア構成

- `double-helical-core`: 単位、normal/transverse変換、インボリュート断面、ラック断面、部品配置を保持する。I/OやCADカーネルには依存しない。
- `double-helical-kernel-manifold`: twist/shear extrusion、中央body、軸穴、プレート、スペーサーを閉じたメッシュへ変換する。
- `double-helical-export`: STL、3MF、OBJ/MTLを直列化する。
- `double-helical-cli`: TOML入力、干渉検査、全出力の生成を行う。

## 現在の検証範囲

自動検証は次を確認する。

- normal/transverse寸法変換
- インボリュート断面範囲
- 2段の設定減速比、B–D–C対称配置と各中心距離
- 閉じた非空のダブルヘリカルメッシュ
- handle/D-large、D-small/B、D-small/C、B/rack、C/rack、A/rackの配置時体積干渉
- STL triangle countと決定的な3MF出力

未検証事項:

- 実際のFDM寸法精度、摺動クリアランス、収縮
- 歯当たり、接触率、負荷容量、疲労、層間剝離
- ラック横方向centering forceと軽負荷時の挙動
- ボルト締結トルク、摩耗、ワッシャー、ナット緩み止め
- ハンドル角ドライブの摩耗・抜け止め、ラックガイド、idler preload調整機構
