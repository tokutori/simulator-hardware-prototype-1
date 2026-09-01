# ダブルヘリカルラック・ピニオン試験設計

## 試験構成

中央Dを大小平歯車の複合部品とし、独立したハンドル軸からD大径側へ入力する。FDMで歯元を太くするため平歯車moduleは1.8とし、130 mmケース内で小歯数を成立させるためpressure angleは25°とする。ハンドル軸12TからD大径31T、D小径12Tから左右B/C 28Tとし、全体を約6.03:1とする。

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
         B: 28T spur    C: 28T spur
                \       /
              D-small: 12T
                    │ same shaft
              D-large: 31T
                    │
          handle-shaft spur: 12T
```

B/Cはそれぞれ28T平歯車と駆動20Tピニオンを接続した一体部品とする。Dも12T小径歯車と31T大径歯車を接合した一体部品とする。D小径がB/Cの両方へ同時に噛むため、B/Cは同方向へ回転してラックを送る。AはB/Cの中間かつラック反対側へ置き、3接触点でラック姿勢を安定させる。Aは動力系へ接続せず、今回はセンサーも付けない。

B/C間隔は平歯車中心距離36.0 mmで決まるため、各最終ピニオンのラックに対する歯位相を複合部品内で個別に合わせる。これによりDとの噛み合いと、B/C両方のラック噛み合いを同時に成立させる。

D大径とD小径は軸方向の隙間を設けず、大径側を下にして直接積層できる形状とする。各平歯車段では大径側を3.5 mmのまま、小径12Tだけを上へ2 mm延長して5.5 mmとする。したがってハンドル12T/D大径31Tは5.5/3.5 mm、D小径12T/B/C大径28Tも5.5/3.5 mmである。長い小径側が大径側を軸方向に覆い、ガタがあっても噛み合いを維持する。

B/C大径上面とラック有効歯幅下面の間には、意図的な2 mm空間を維持する。D小径だけがこの空間へ上方向に伸び、B/C大径自体は伸ばさない。B/Cの下側ヘリカル歯は同じ2 mm区間を下へ延長して28T平歯車へ直接つなぐ。28T平歯車のroot radius 22.95 mmはヘリカルピニオンのtip radius約22.71 mmより大きいため、延長歯の最下層を全周で支持できる。接続用の円錐台テーパーは設けない。

## ダブルヘリカル形状

このモデルは一体FDM造形を前提とし、中央逃げ溝を設けず左右のねじれ歯を中央で連続させる。厳密には herringbone geometry に相当する。

- normal module: 2.0 mm
- normal pressure angle: 20°
- helix angle: 15°
- pinion teeth: 20
- ピニオン左右歯幅: 各10 mm
- ラック左右歯幅: 各7.5 mm
- center relief: 0 mm
- ピニオン全歯幅: 20 mm
- ラック全歯幅: 15 mm

中央ギャップ0 mmはFDM造形性と連続した歯底を優先した確定制約である。また、このexperimentの目的はherringbone歯形だけでラックの軸方向変位をどこまで抑えられるかを観察することなので、ラックの軸方向位置を直接規制するフランジやケースガイドは意図的に設けない。通常の機械としての脱落防止を検証する試験ではない。

normal systemから軸直角断面へ変換する。

```text
transverse module = normal module / cos(helix angle)
transverse pressure angle = atan(tan(normal pressure angle) / cos(helix angle))
```

既定値でtransverse moduleは約2.071 mm、20Tピニオンのピッチ径は約41.411 mmとなる。中央の歯先まで連続させ、歯底より内側の円板またはラック本体も連続させる。

ラックは上下両面に同じ歯形を持つ。Z方向へ進むごとにX方向へ歯形をshearし、左右halfではshear方向を反転する。ピニオンはインボリュート断面をZ方向へtwist extrusionする。ラックは偶数24歯、歯部約156.12 mmとし、130 mm角のケースを貫通して移動する。

ラックの正X側先端には、物体を押すための30 mm（Y）×15 mm（Z）のフラット端面を持つ長さ8 mmの一体パッドを設ける。helical shearによる端部ずれ約2.01 mmより内側までパッドを重ね、全積層でラック本体と接続する。Z寸法はラック歯幅15 mmと同一にして垂直方向へ突出させず、横倒し造形時のbuild plate面内Y方向だけを広げる。全長は約166.13 mmとなる。幅広パッドがケースへ入る位置までラックを引き込まないことを試験時の可動範囲とする。

ラックのherringbone中央面はピニオンの中央面と一致させ、ピニオン全歯幅20 mmは強度のため変更しない。ラックだけを15 mmへ狭めることで、組立時のラック上面からtop plate下面までを3.0 mm、ラック下面からbottom plate上面までを12.0 mmとする。上下間隔は非対称だが、ラックをZ方向へ移動すると左右helixの中央がずれて試験条件が変わるため、ラック中心位置は移動しない。

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

ハンドル12Tギヤと印刷回転軸は別部品とする。5.5 mm歯幅のギヤには9.3 mm角穴を設けて完全に平置き造形する。軸は下から順に、bottom plate内のØ9→11 mm丸テーパー、ギヤ下面を支えるØ11 mm肩、ギヤ内の9.0 mm角軸、スペーサー内のØ9 mm丸軸、top plate内のØ9→8.6 mm丸テーパー、クランク用6.0 mm角軸で構成する。plate側は各テーパーへ直径0.4 mmを加えた逆向きの受け穴とし、締結時にシャフトの下抜けと上抜けを軽く拘束する。

組立時はシャフトをbottom plate内側からテーパーへ入れ、最大Ø9 mmの上側丸軸を9.3 mm角穴へ通してギヤを9 mm角軸へ嵌める。次に内径10.4 mmの上側スペーサーを丸軸へ通す。top plateは対角約8.49 mmの6 mm角軸を通過し、上側丸テーパーへ収まる。最後に6.3 mm角穴のクランクを取り付ける。Ø11 mm下肩は9.3 mm角穴より大きいためギヤを保持する。軸を下端から縦造形すると、上へ進む断面は緩い下側テーパーを除いて直下形状の投影内へ縮小するため、急なoverhangを作らない。ノブは外径15 mm、長さ25 mm、M6通し穴を持つ別部品とする。

これは初期クリアランスであり、プリンタ、材料、ノズル径、収縮、穴補正に合わせたcoupon試験が必要である。ボルトのねじ部を摺動面へ直接当てる構成ではない。

現在の`tooth_backlash_mm = 0.10`は、mesh全体の値ではなく各部材のpitch circle上の歯厚減少量として適用される。両部材へ0.10 mmずつ適用するため、一対のnominal backlashは約0.20 mmである。FDM試作でB/C閉ループの誤差を吸収する初期値として維持するが、最終値はcouponで決める。

## 実機で確認する項目

B/CはDから同期駆動され、同じラックにも噛み合うため閉じたkinematic loopを構成する。CAD上の歯位相とintersection volumeが成立していても、FDMの歯形誤差、pitch誤差、軸位置誤差および収縮差によってB/C間にinternal loadが生じる可能性がある。journalの直径クリアランス0.4 mmと歯面backlashがどこまで誤差を吸収するか、手回し時の抵抗、局所的な噛み込み、B/Cの荷重分担として確認する。

正転・逆転の両方向について、ラックの軸方向変位量と変位方向を測る。フランジやガイドを設けないため、この結果をherringbone歯形による保持挙動として直接観察できる。試験中はラック脱落を前提に低速で操作し、可動範囲外に手を置かない。

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
- 正逆転時のラック軸方向変位、centering forceと軽負荷時の挙動
- B/C二重駆動のload sharing、閉ループ誤差によるinternal loadと噛み込み
- ボルト締結トルク、摩耗、ワッシャー、ナット緩み止め
- ハンドル角ドライブの摩耗・抜け止め、idler preload調整機構
