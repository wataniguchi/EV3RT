from collections import deque
import sys
import threading
import time
import libraspike_art_python as lib
from libraspike_art_python import pbio_port

RASPIKE_COM_NAME = '/dev/USB_SPIKE'

# 判定周期(秒)
SENSE_INTERVAL = 0.04

# ロバスト化用の直近サンプル数
WINDOW_SIZE = 5

# 色ID定数
COLOR_UNKNOWN  = -1
COLOR_BLACK    = 0
COLOR_BLUE     = 1
COLOR_GREEN    = 2
COLOR_YELLOW   = 3
COLOR_RED      = 4
COLOR_WHITE    = 5

COLOR_NAMES = {
    COLOR_UNKNOWN: 'unknown',
    COLOR_BLACK:   'black',
    COLOR_BLUE:    'blue',
    COLOR_GREEN:   'green',
    COLOR_YELLOW:  'yellow',
    COLOR_RED:     'red',
    COLOR_WHITE:   'white',
}

# --- 収集データから導いた閾値 ---
# black : V < 30, S < 25  (V avg≈26)
# white : V > 55, S < 22  (V avg≈98, S avg≈5)
# blue  : H in [205,220], S > 80  (H avg≈213, S avg≈93)
# green : H in [140,155], S > 60  (H avg≈148, S avg≈73)
# yellow: H in [35,65], V > 88    (H avg≈52, V avg≈99)
# red   : H > 345 or H < 10, S > 75  (H avg≈354, S avg≈87)

def classify_single(h: int, s: int, v: int) -> int:
    """1サンプルのHSVから色を判定する。"""
    # 純白：収集データのS最大値は19。S<20 かつ高輝度のみ白と認める
    if s < 20 and v > 75:
        return COLOR_WHITE
    # 有彩色判定（S が高い領域）
    # 青は境界でSが59〜71まで低下するため閾値を緩める
    if 205 <= h <= 220 and s > 50:
        return COLOR_BLUE
    if 140 <= h <= 155 and s > 60:
        return COLOR_GREEN
    if 35 <= h <= 65 and v > 88:
        return COLOR_YELLOW
    if (h > 345 or h < 10) and s > 75:
        return COLOR_RED
    # 上記いずれにも該当しない低彩度(S<35)はすべて黒扱い
    # 外乱光でVが上昇しても、彩度が低ければ有色ラインではない
    if s < 35:
        return COLOR_BLACK
    return COLOR_UNKNOWN


def classify_robust(window: deque) -> int:
    """
    直近 WINDOW_SIZE サンプルの多数決で色を決定する。
    走行体の左右揺れや外乱光による一時的なノイズを抑制する。
    黒判定は除外し、有色・白の多数決を優先する。
    """
    if not window:
        return COLOR_UNKNOWN

    votes: dict[int, int] = {}
    for color_id in window:
        votes[color_id] = votes.get(color_id, 0) + 1

    # 黒以外の票を集計し、過半数なら採用
    non_black = {k: v for k, v in votes.items() if k != COLOR_BLACK}
    if non_black:
        best = max(non_black, key=lambda k: non_black[k])
        if non_black[best] >= WINDOW_SIZE // 2 + 1:
            return best

    # 黒が最多なら黒
    return max(votes, key=lambda k: votes[k])


def receiver_task() -> None:
    while True:
        lib.raspike_prot_receive()


def main() -> None:
    desc = lib.raspike_open_usb_communication(RASPIKE_COM_NAME)
    if desc is None:
        print(f"Cannot open: {RASPIKE_COM_NAME}", file=sys.stderr)
        sys.exit(-1)

    lib.raspike_prot_init(desc)

    t1 = threading.Thread(target=receiver_task, daemon=True)
    t1.start()

    color_sensor = lib.pup_color_sensor_get_device(pbio_port.ID_E)

    window: deque[int] = deque(maxlen=WINDOW_SIZE)
    prev_color = COLOR_UNKNOWN

    print("Color identification started. Press Ctrl+C to stop.")
    print(f"{'Time':>8}  {'R':>5} {'G':>5} {'B':>5}  {'H':>4} {'S':>4} {'V':>4}  {'Single':<10} {'Robust':<10}")
    print("-" * 72)

    try:
        while True:
            time.sleep(SENSE_INTERVAL)

            r, g, b = lib.pup_color_sensor_rgb(color_sensor)
            h, s, v = lib.pup_color_sensor_hsv(color_sensor, True)

            single = classify_single(h, s, v)
            window.append(single)
            robust = classify_robust(window)

            ts = time.strftime("%H:%M:%S")
            print(
                f"{ts}  {r:5d} {g:5d} {b:5d}  {h:4d} {s:4d} {v:4d}"
                f"  {COLOR_NAMES[single]:<10} {COLOR_NAMES[robust]:<10}",
                flush=True,
            )

            if robust != prev_color:
                print(f">>> Color changed: {COLOR_NAMES[prev_color]} -> {COLOR_NAMES[robust]}")
                prev_color = robust

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
