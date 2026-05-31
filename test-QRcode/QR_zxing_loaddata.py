import sys
import platform
if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')
import cv2
import zxingcpp
from pathlib import Path
import time

_QR_ONLY = zxingcpp.BarcodeFormat.QRCode
_GH      = zxingcpp.Binarizer.GlobalHistogram
_WECHAT  = cv2.wechat_qrcode_WeChatQRCode()

# Pre-create CLAHE objects once to avoid repeated allocation
# Detection stage: only variants with zero false-positive rate on noise/part
_CL_DETECT = cv2.createCLAHE(clipLimit=3, tileGridSize=(8, 8))
# ROI decode stage: tight crop around detected QR position
_CL_ROI = [
    cv2.createCLAHE(clipLimit=10, tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=6,  tileGridSize=(6, 6)),
    cv2.createCLAHE(clipLimit=20, tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=40, tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=3,  tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=6,  tileGridSize=(4, 4)),
]

# Center crop: keep columns 480-1440 (half width) of 1920px images
_CROP_X1 = 480
_CROP_X2 = 1440
_ROI_MARGIN = 40


def _extract_roi(result, crop):
    pos = result.position
    pts = [
        (pos.top_left.x,     pos.top_left.y),
        (pos.top_right.x,    pos.top_right.y),
        (pos.bottom_right.x, pos.bottom_right.y),
        (pos.bottom_left.x,  pos.bottom_left.y),
    ]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    x1, x2 = min(xs), max(xs)
    y1, y2 = min(ys), max(ys)
    return crop[
        max(0, y1 - _ROI_MARGIN) : y2 + _ROI_MARGIN,
        max(0, x1 - _ROI_MARGIN) : x2 + _ROI_MARGIN,
    ]


def _wechat_decode_roi(roi):
    for cl in _CL_ROI:
        texts, _ = _WECHAT.detectAndDecode(cl.apply(roi))
        t = next((s for s in texts if s), None)
        if t:
            return t
    return ""


def decode_qr(img_gray):
    """Decode QR code from a grayscale image.
    Returns decoded text string, or empty string if not found."""
    crop = img_gray[:, _CROP_X1:_CROP_X2]

    # Stage 1 — zxingcpp fast path (GlobalHistogram, raw): catches most QRv1
    codes = zxingcpp.read_barcodes(crop, formats=_QR_ONLY, binarizer=_GH)
    if codes:
        return codes[0].text

    # Stage 2 — zxingcpp LocalAverage with return_errors to also detect QRv6 position
    results = zxingcpp.read_barcodes(crop, formats=_QR_ONLY, return_errors=True)
    if results:
        r = results[0]
        if r.valid:
            return r.text
        # QRv6 detected but not decoded: try WeChat on tight ROI
        return _wechat_decode_roi(_extract_roi(r, crop))

    # Stage 3 — one CLAHE variant (zero false-positive rate on noise/part images)
    results = zxingcpp.read_barcodes(
        _CL_DETECT.apply(crop), formats=_QR_ONLY, return_errors=True
    )
    if results:
        r = results[0]
        if r.valid:
            return r.text
        return _wechat_decode_roi(_extract_roi(r, crop))

    return ""


# kind -> (decodable, time_target_ms)
_KIND_META = {
    "hint1":    (True,  10.0),
    "hint2_32": (True,  70.0),
    "part1":    (False,  5.0),
    "part2":    (False,  5.0),
    "noise":    (False,  5.0),
}


def _kind_of(name):
    for k in _KIND_META:
        if name.startswith(k):
            return k
    return "other"


def read_qr_code():
    data_dir = Path("data")
    if not data_dir.exists() or not data_dir.is_dir():
        print("data/ directory does not exist. Please create it and add images to it.")
        return

    # kind -> [decoded, total, total_ms]
    stats = {k: [0, 0, 0.0] for k in _KIND_META}
    stats["other"] = [0, 0, 0.0]

    try:
        for img_path in sorted(data_dir.glob("*.png")):
            img_orig = cv2.imread(str(img_path))
            if img_orig is None:
                print(f"Failed to read {img_path}")
                continue

            t0 = time.perf_counter()
            img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
            text = decode_qr(img_gray)
            elapsed_ms = (time.perf_counter() - t0) * 1000

            kind = _kind_of(img_path.name)
            stats[kind][1] += 1
            stats[kind][2] += elapsed_ms

            if text:
                stats[kind][0] += 1
                print(f"QR code successfully decoded in {img_path} ({elapsed_ms:.1f}ms)")
                print(f"Decoded QR code text: {text}")
            else:
                print(f"Failed to decode {img_path} ({elapsed_ms:.1f}ms)")

    finally:
        print("\n--- Summary ---")
        print(f"  {'kind':<12} {'decoded':>10} {'mean ms':>8} {'target':>8} {'OK?':>5}")
        print(f"  {'-'*12} {'-'*10} {'-'*8} {'-'*8} {'-'*5}")
        for kind, (dec, tot, ms) in stats.items():
            if tot == 0:
                continue
            mean_ms = ms / tot
            decodable, target = _KIND_META.get(kind, (None, None))
            dec_str = f"{dec}/{tot} ({dec/tot*100:.0f}%)" if decodable else f"-/{tot}"
            ok = "OK" if target is None else ("OK" if mean_ms <= target else "NG")
            target_str = f"{target:.0f}ms" if target else "-"
            print(f"  {kind:<12} {dec_str:>10} {mean_ms:>7.1f}ms {target_str:>8} {ok:>5}")
        dec_total = sum(s[0] for k, s in stats.items() if _KIND_META.get(k, (False,))[0])
        tot_decodable = sum(s[1] for k, s in stats.items() if _KIND_META.get(k, (False,))[0])
        print(f"\n  Decodable total: {dec_total}/{tot_decodable} ({dec_total/max(1,tot_decodable)*100:.0f}%)")
        print("Finished processing all images in data/ directory.")


if __name__ == '__main__':
    read_qr_code()
