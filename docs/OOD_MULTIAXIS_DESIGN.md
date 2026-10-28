# 多軸OODの実験設計とUbuntuでの実行

位置・向き・関節角について、単独要因、強度、組合せを比較する実験です。元の論文用環境、学習済み方策、初期状態、制御時間を揃えて目標を変更します。各物体・各条件100例が既定です。設定は[`configs/ood_multiaxis.json`](../configs/ood_multiaxis.json)です。

## 評価の範囲と比較の原則

- 空間目標は、公開評価の目標分布からの外挿として扱います。公開学習の全目標支持集合は分からないため、あらゆる学習姿勢からの厳密な分布外であるとは断定しません。
- 関節角については、公開学習コードのランダム目標範囲 `[0.5, 1.5] rad` の外かを個別に記録します。可動域内かも別に記録します。
- 同じ物体・sample_indexでは初期ラベル対と乱数を揃えます。単独条件と組合せ条件でも、同じ因子の目標値を共有します。
- 移動距離の条件では対照の方向を保持します。方向の条件では対照の移動距離を保持します。
- 世界Z軸の水平回転を机上の姿勢条件にします。世界X/Y軸の大きな傾きと任意軸の3D回転は、持ち上げ中の目標として与えます。最終的には机上の対照姿勢へ戻して開閉します。
- 空中姿勢には、同じ0.6 mの持ち上げ目標を使う`air_control`を対照にします。机上での大きな傾きが台へめり込むことによる失敗と、姿勢制御の不達を混同しにくい設計です。空中目標でも手の到達可能性までは保証しません。
- 空中対照自体がほとんど成功しない場合、全失敗を向きだけの問題と断定しません。全100例の主表に加え、対照に成功した例での達成率とその分母も表示します。対照成功0例ならこの比率は未算出です。
- 物体の可動域を超える180°などはストレス評価に分け、主な汎化評価と区別します。

## 基本セット：17条件

`--suite core`で選択します。10物体×17条件×100例＝**17,000試行**です。

| 条件 | 変更する要素 | 目標 | 対照 |
| --- | --- | --- | --- |
| `id_control` | 公開評価範囲の対照 | 元評価の位置・向き・角度範囲 | 自身 |
| `height_medium` | 持ち上げ量 | 0.35〜0.45 m | id_control |
| `height_large` | 持ち上げ量 | 0.60〜0.90 m | id_control |
| `distance_medium` | 水平移動距離 | 0.20〜0.35 m、方向は対照と同一 | id_control |
| `distance_large` | 水平移動距離 | 0.50〜0.80 m、方向は対照と同一 | id_control |
| `direction_balanced` | 水平移動方向 | 世界XY平面の0/90/180/270°、各25例、距離は対照と同一 | id_control |
| `yaw_medium` | 最終配置の水平回転 | 世界Z軸±60〜75° | id_control |
| `yaw_large` | 最終配置の水平回転 | 世界Z軸±120〜150° | id_control |
| `air_control` | 空中姿勢の専用対照 | 持ち上げ量0.60 m、向きは対照と同じ | id_control |
| `air_roll_large` | 持ち上げ中の傾き | 世界X軸±90〜120°、高さ0.60 m | air_control |
| `air_pitch_large` | 持ち上げ中の傾き | 世界Y軸±90〜120°、高さ0.60 m | air_control |
| `air_so3_large` | 持ち上げ中の任意軸回転 | 球面一様な軸、±90〜150°、高さ0.60 m | air_control |
| `angle_small` | 小さい関節目標 | 絶対角15〜23°（約0.26〜0.40 rad） | id_control |
| `angle_upper` | 可動域内の大きい関節目標 | 物体の可動域の85〜98% | id_control |
| `combo_distance_yaw` | 距離×水平回転 | distance_mediumとyaw_mediumの組合せ | id_control |
| `combo_planar` | 高さ・距離・水平回転・関節角 | height_medium＋distance_medium＋yaw_medium＋angle_upper | id_control |
| `combo_spatial` | 距離・空中3D姿勢・関節角 | distance_medium＋air_so3_large＋angle_upper | air_control |

回転角は、同じサンプルの対照目標姿勢からの増分です。`±`は各物体100例なら正負50例ずつです。方向や回転軸・符号は条件間で共有し、強度を変えても同じ方向へ動かします。

`angle_upper`は全物体で可動域内ですが、関節目標が必ず学習のランダム範囲外になるわけではありません。上限1.5 radのespressomachineでは範囲内です。この物体も100例実行して主表に残し、関節OODだけの条件付き集計では適用対象外と分かるように表示します。

`angle_small`は、多くの物体で初期の閉角に近い目標です。必ず開状態から閉じる動作を検証する条件ではありません。初期角が既に論文の0.5 rad閾値内かを記録し、後述の0.1 radの指標も併記します。

## 詳細セットとストレスセット

| セット | 内容 | 条件数 | 10物体・100例での試行数 |
| --- | --- | ---: | ---: |
| `core` | 上の基本条件 | 17 | 17,000 |
| `full` | core＋弱いyaw、空中X/Y/任意軸の中程度回転、4方向の個別条件、可動域65〜80%の関節角 | 26 | 26,000 |
| `stress` | id_control＋絶対関節角180°＋水平回転180°＋極端な複合条件 | 4 | 4,000 |
| `all` | full＋3つのストレス条件 | 29 | 29,000 |

fullではyawの30〜45/60〜75/120〜150°、空中のX/Y回転の45〜60/90〜120°、任意軸回転の45〜75/90〜150°を比較できます。

180°の正負は同じ終端姿勢になるため、ストレス条件の180°yawは符号を分けません。

ストレス条件の180°関節角は要求どおり維持します。espressomachine、ketchup、phoneでは可動域を超えます。勝手に上限へ丸めず、成功率の分母と範囲外件数に残します。可動域は関節角だけの検査で、手・物体・台との完全な幾何学的到達可能性を保証するものではありません。

## 向きの表現と与えるタイミング

向きはwxyz quaternionで保存し、軸角回転を合成します。Euler角の各成分を独立に一様抽出する方法は使いません。任意軸は球面上で一様、角度は指定区間で一様です。回転全体をHaar一様に抽出するという意味ではありません。

```json
"orientation": {
  "axis": "random",
  "angle_deg": [90, 150],
  "frame": "world",
  "stage": "lift",
  "signed": true
}
```

- `axis`: `x`、`y`、`z`、`random`。
- `frame`: 世界座標での回転は`world`（左から合成）、対照目標の物体座標での回転は`object`（右から合成）。プリセットはworldです。
- `stage`: `placement`は配置段階だけ、`lift`は持ち上げ段階だけ、`both`は両段階。指定しなかった段階の向きは対照と同じです。

旧5条件のEuler第2成分による目標は、そのまま再現します。新しいquaternion回転を使う条件では、公開判定互換の姿勢参照にも生成した最終目標の第2Euler成分を使います。公開実装の「目標quaternionを読む」判定方法は保持し、実測姿勢を使う論文AEとは分けています。

## 評価指標と結果表

主指標は従来どおり、実測最終状態による**論文Suc.T**と、同じ軌跡への**公開実装の判定互換**です。[定義と実装差](OOD_EVALUATION_UBUNTU.md)は変更していません。新しい目標分布への評価なので、論文Table 3との比較は参考比較です。

多軸版では次を追加します。

| 指標 | 定義 |
| --- | --- |
| 空中姿勢達成率 | 持ち上げ区間の同一時刻で位置誤差 < 0.05 mかつSO(3)姿勢誤差 < 0.2 rad |
| OOD目標達成率 | 論文Suc.T＋高さ目標達成＋空中姿勢達成。途中の回転失敗を最後の復帰成功で隠さない |
| 厳密目標達成率 | 上の関節角誤差閾値だけを0.5 radから0.1 radへ変更した補助指標 |
| 開閉開始時の許容内件数 | 開閉開始前から要求角との差が論文閾値0.5 rad以内だった件数 |
| 関節OODの条件付き成功率 | 可動域内かつランダム学習目標[0.5,1.5] radの外にある計画試行だけで集計 |

論文Suc.T自体に新しい閾値を混ぜません。関節角は有限可動域なので、角度差を2πで折り返しません。リセット時・開閉開始時・要求・最終の角度と開閉区間内の変化量、要求姿勢の初期からの角距離、持ち上げ終端の位置・姿勢誤差も保存します。

`report.md` / `summary.json`には次を出します。

- 各物体・各条件の成功数/100、Wilson 95%区間、論文/公開判定の無加工誤差と公開表示のクリップ値。
- 因子・強度別の成功率、同じ高さの空中対照との対応比較。
- 最終位置・最終姿勢・最終関節角・空中姿勢の各達成率。各列は独立の条件で、排他的な失敗原因分類ではありません。
- 移動4方向、回転正負の内訳。全物体の集約と物体別主表を併記。
- 可動域外件数、関節OOD対象数、初期関節角が既に許容内だった数。
- 距離×yawの4条件の対応比較。交互作用は `100×平均(S距離yaw − S距離 − Syaw ＋ S対照)` のppで報告します。検定結果とは区別した記述統計です。
- 論文8物体と追加2物体を分けた集約と論文Table 3の公開値。

全計画試行を主表の分母にし、数値異常は失敗、未完了は明示します。未完了の表は暫定下限で、全件が終わるまで信頼区間は出しません。誤差を測れない試行は0で埋めず測定数を示します。

## Ubuntuで実行

既存の論文用conda環境とライセンスを使います。新しいcheckoutの場合だけ、先にそのcheckoutで`cd raisimGymTorch; python setup.py develop; cd ..`を実行します。C++環境やURDFは今回変更していません。

まず少数で全段階を確認します。少数試行から定量的な結論は出しません。

```bash
python tools/ood_eval.py run --config configs/ood_multiaxis.json --suite core \
  --samples 2 --output results/multiaxis_smoke --device cuda
```

本番の計画を確認してから実行する場合です。計画生成にはRaiSim・GPUは不要です。

```bash
python tools/ood_eval.py plan --config configs/ood_multiaxis.json --suite core \
  --output results/multiaxis_core_seed1
python tools/ood_eval.py run --output results/multiaxis_core_seed1 --resume --device cuda
```

直接開始することもできます。

```bash
python tools/ood_eval.py run --config configs/ood_multiaxis.json --suite core \
  --output results/multiaxis_core_seed1 --device cuda
```

追加の強度・方向別条件まで評価する場合は`--suite full`、到達不能要求も含むストレス評価は`--suite stress`を使い、出力先を分けてください。`--samples`は各物体・各条件の例数です。GPUは方策推論に使い、RaiSim物理計算はCPUです。

一部だけ実行する例です。比較に使う対照も含めてください。対照がなければレポートの対応比較は未完ペアとして表示されます。

```bash
python tools/ood_eval.py run --config configs/ood_multiaxis.json \
  --profiles id_control air_control air_roll_large air_pitch_large air_so3_large \
  --output results/multiaxis_orientation_seed1 --device cuda
```

同じ実験の再開は保存済み計画を使用します。

```bash
python tools/ood_eval.py run --output results/multiaxis_core_seed1 --resume
```

## 定性的評価

既存のRaiSimUnityを起動してAuto-connectを有効にし、数値評価後に次を実行します。

```bash
python tools/ood_record.py --results results/multiaxis_core_seed1 --per-outcome 5
python tools/ood_eval.py report --results results/multiaxis_core_seed1
```

多軸設定の`video_metric`は`ood_goal_success`なので、途中の空中姿勢も含めた成功・失敗を各物体・各条件で最大5件ずつ選びます。元の評価済み操作列を再生し、全軌跡と判定が一致した実MP4だけを採用します。成功0件の条件は成功動画0件として不足を明記します。core全条件では最大1,700本になるので、必要な条件だけ`--profiles`で絞ることもできます。

論文指標や厳密目標指標による選別もできます。索引は指標ごとに分かれます。

```bash
python tools/ood_record.py --results results/multiaxis_core_seed1 \
  --profiles air_so3_large combo_spatial --metric paper_success
python tools/ood_record.py --results results/multiaxis_core_seed1 \
  --profiles air_so3_large combo_spatial --metric strict_goal_success
```

## 実行中の従来実験を続ける場合

既存の`configs/ood_eval.json`の5条件、seedから作る目標値、論文・公開判定は維持しています。ただし評価コードのハッシュは更新されるため、**現在走っている実験のフォルダではgit pullせず、動画収集までその版のまま完了・再開**してください。

先に多軸版を準備するなら、別のcheckoutを作れます。次はGitの履歴を共有するworktreeを使う例です。元のフォルダのコードは変更しません。

```bash
cd ~/goto/artigrasp
git fetch origin
git worktree add --detach ../artigrasp-multiaxis origin/main
cd ../artigrasp-multiaxis/raisimGymTorch
python setup.py develop
cd ..
```

新しいcheckoutで多軸版の確認・計画生成を行い、本番は計算資源の空きに合わせて実行してください。既存結果へ新しい目標や異なる版の結果を追記する運用は避け、新しい出力先を使います。
