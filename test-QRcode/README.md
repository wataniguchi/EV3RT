# test-QRcode

ETrobocon 2026 向けQRコード検出スクリプト。
カメラ画像からQRコードを高精度・低遅延でデコードする。

---

## ファイル構成

| ファイル | 役割 |
|---------|------|
| `QR_zxing_loaddata.py` | バッチ評価スクリプト。`data/` 内の画像を順次処理し、デコード率・処理時間を集計する |
| `QR_tuned.py` | 本番モニタリングスクリプト。カメラから40ms周期でフレームを取得し、別スレッドでQR検出を行い、結果をウィンドウ表示する |

---

## 非機能要件

| 画像の種類 | 内容 | 処理時間目標 |
|-----------|------|-------------|
| `noise_*` | QRコードを含まない画像 | ≤ 5ms |
| `part1_*` | QRv1の部分画像（デコード不可） | ≤ 5ms |
| `part2_*` | QRv6の部分画像（デコード不可） | ≤ 5ms |
| `hint1_*` | QRv1完全画像（デコード可） | ≤ 10ms |
| `hint2_32_*` | QRv6完全画像・距離32cm（デコード可） | ≤ 70ms |

- 左右クロップ: 1920px幅の中央960px（x=480〜1440）のみ処理する
  - ロボットがQRコードを左右中心に捉えるよう回転できるため
- 上下クロップは行わない
  - ロボットとQRコードの距離が変動するため

---

## デコード戦略

### 概要

3段階のパイプラインで処理する。各段階で成功したら即座に次の段階をスキップする（早期終了）。

```
Stage 1: zxingcpp (GlobalHistogram binarizer, raw)     ~0.5ms
Stage 2: zxingcpp (LocalAverage binarizer, raw)        ~1.2ms
  └─ 位置検出のみの場合: WeChat QR on tight ROI        ~30ms
Stage 3: CLAHE(cl=3, 8×8) + zxingcpp (LocalAverage)   ~1.8ms
  └─ 位置検出のみの場合: WeChat QR on tight ROI        ~30ms
```

### 各段階の詳細と採用理由

#### Stage 1 — zxingcpp GlobalHistogram binarizer（raw画像）

**採用理由:**
GlobalHistogram binarizer は LocalAverage binarizer の約2〜3倍高速（0.5ms vs 1.2ms）。
QRv1（hint1）の半数以上はコントラストが十分高く、最速の二値化でデコードできる。
まず最速の手段を試すことで、大多数の hint1 画像を最小コストで処理できる。

#### Stage 2 — zxingcpp LocalAverage binarizer（raw画像）＋ `return_errors=True`

**採用理由:**
LocalAverage binarizer は局所的なコントラスト差に基づいて二値化するため、
GlobalHistogram で失敗した残りの hint1 画像をカバーできる。

`return_errors=True` を指定することで、デコードに失敗した場合でも
QRコードのファインダーパターンを検出できた場合は **位置情報（4コーナー座標）** を返す。
これを利用してQRコード周囲の tight ROI（余白40px付き）を切り出し、
WeChat QR に渡すことで hint2_32 のデコードに対応する。

**WeChat QR を採用した理由:**
hint2_32（QRv6, 距離32cm）は px/module 密度が約7.5px/module と低く、
通常の二値化では誤り訂正が追いつかずデコード失敗する。
WeChat QR はニューラルネットによる超解像処理を内蔵しており、
低解像度のQRコードでも高い確率でデコードできる。
ただし処理時間が1回あたり約30〜60msと重いため、
**位置が検出された場合のみ**（ゲート処理）呼び出す。

WeChat QR を full crop に対して呼び出してもデコード失敗することが多い。
tight ROI に絞り込み、さらに small tile CLAHE（tileGridSize=(4,4)〜(6,6)）で
局所コントラストを強調してから渡すことで、デコード率が大幅に改善した。

#### Stage 3 — CLAHE(clipLimit=3, tileGridSize=8×8) ＋ zxingcpp LocalAverage

**採用理由:**
Stage 2 で位置が検出できなかった hint2_32 画像の一部は、
適度なコントラスト強調（CLAHE）を施すことで検出可能になる。

CLAHE のパラメータは **noise/part 画像に対して誤検出ゼロ** のものに限定している。
誤検出が発生すると WeChat QR が不必要に呼び出され、noise/part の処理時間が
5ms 目標を大きく超えてしまうためである。
検証の結果、`clipLimit=3, tileGridSize=(8,8)` は全 noise/part 画像で
ファインダーパターンの誤検出が発生しないことを確認した。

より強いCLAHE（clipLimit=10以上、tileGridSize=4×4など）は hint2_32 の検出率を
若干向上させるが、part1/part2 で誤検出が発生するため採用しない。

### 不採用となった手法とその理由

| 手法 | 不採用理由 |
|------|-----------|
| `cv2.QRCodeDetector` | 1枚あたり25〜44ms。zxingcpp より遅く、目標時間を満足しない |
| pyzbar | hint1 でも24枚中9枚しかデコードできず、精度・速度ともに劣る |
| 画像の上下クロップ（固定） | ロボットとQRコードの距離が変動するため適用不可 |
| 簡易統計量によるプレリジェクト | 現サンプルが母集団を統計的に代表していないため信頼できない |
| WeChat QR を full crop に適用 | hint2_32 に対してデコード率がほぼゼロ。tight ROI が必須 |

---

## 実績（CPython, 2026-05-30 計測）

| カテゴリ | デコード率 | 平均処理時間 | 目標 | 判定 |
|---------|-----------|-------------|------|------|
| hint1   | 24/24 (100%) | 1.3ms | 10ms | ✓ |
| hint2_32 | 29/52 (56%) | 31ms | 70ms | ✓ |
| part1   | -/16 | 3.8ms | 5ms | ✓ |
| part2   | -/18 | 3.8ms | 5ms | ✓ |
| noise   | -/13 | 3.6ms | 5ms | ✓ |

hint2_32 の残り44%（23枚）はボケ・部分遮蔽など撮影品質の問題であり、
アルゴリズムでの改善余地はない。
スマートフォンでもデコード不可であることを確認している。

---

## 実行方法

```bash
# バッチ評価
pypy3 QR_zxing_loaddata.py

# カメラモニタリング（q キーで終了）
pypy3 QR_tuned.py
```
