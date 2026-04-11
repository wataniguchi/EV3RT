import sys
import platform
if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')
import cv2
import numpy as np
import zxingcpp
import threading
import time

# video capture interval in seconds
CAPTURE_INTERVAL = 0.04  # 40ms = 25fps

# frame size for Raspberry Pi USB camera capture
IN_FRAME_WIDTH  = 1920
IN_FRAME_HEIGHT = 1080

# frame size for OpenCV processing (same as input)
FRAME_WIDTH  = 1920
FRAME_HEIGHT = 1080

# frame size for display window
OUT_FRAME_WIDTH  = 640
OUT_FRAME_HEIGHT = 360

# QR decode pipeline constants
_QR_ONLY = zxingcpp.BarcodeFormat.QRCode
_GH      = zxingcpp.Binarizer.GlobalHistogram
_WECHAT  = cv2.wechat_qrcode_WeChatQRCode()

_CL_DETECT = cv2.createCLAHE(clipLimit=3, tileGridSize=(8, 8))
_CL_ROI = [
    cv2.createCLAHE(clipLimit=10, tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=6,  tileGridSize=(6, 6)),
    cv2.createCLAHE(clipLimit=20, tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=40, tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=3,  tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=6,  tileGridSize=(4, 4)),
]

_CROP_X1   = 480
_CROP_X2   = 1440
_ROI_MARGIN = 40


def _result_pos_to_corners(r):
    """Return 4 corner points (x, y) in crop coordinates, or None."""
    pos = r.position
    return [
        (pos.top_left.x,     pos.top_left.y),
        (pos.top_right.x,    pos.top_right.y),
        (pos.bottom_right.x, pos.bottom_right.y),
        (pos.bottom_left.x,  pos.bottom_left.y),
    ]


def _extract_roi(result, crop):
    pts = _result_pos_to_corners(result)
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


def detect_qr(img_gray):
    """Detect and decode QR from a grayscale 1920x1080 image.

    Returns (text, corners) where:
      text    -- decoded string, or "" if not decoded
      corners -- list of 4 (x, y) in full-image coords if QR located, else None
    """
    crop = img_gray[:, _CROP_X1:_CROP_X2]

    # Stage 1: GH fast path (QRv1 mostly)
    codes = zxingcpp.read_barcodes(crop, formats=_QR_ONLY, binarizer=_GH)
    if codes:
        r = codes[0]
        corners = [(x + _CROP_X1, y) for x, y in _result_pos_to_corners(r)]
        return r.text, corners

    # Stage 2: LA + return_errors to get QRv6 position
    results = zxingcpp.read_barcodes(crop, formats=_QR_ONLY, return_errors=True)
    if results:
        r = results[0]
        corners = [(x + _CROP_X1, y) for x, y in _result_pos_to_corners(r)]
        if r.valid:
            return r.text, corners
        text = _wechat_decode_roi(_extract_roi(r, crop))
        return text, corners

    # Stage 3: CLAHE detection (zero false-positive rate on noise/part)
    results = zxingcpp.read_barcodes(
        _CL_DETECT.apply(crop), formats=_QR_ONLY, return_errors=True
    )
    if results:
        r = results[0]
        corners = [(x + _CROP_X1, y) for x, y in _result_pos_to_corners(r)]
        if r.valid:
            return r.text, corners
        text = _wechat_decode_roi(_extract_roi(r, crop))
        return text, corners

    return "", None


# ---------- shared state between capture thread and detection thread ----------

_frame_lock  = threading.Lock()
_latest_gray = None   # latest grayscale frame for detection thread to consume

_result_lock      = threading.Lock()
_detected_text    = ""    # last successfully decoded QR text
_detected_corners = None  # corners of most recently located QR (full-image coords)
_is_detecting     = False # True while detection thread is busy
_last_decode_time = 0.0   # time.time() of last successful decode

TEXT_EXPIRY_SEC = 2.0     # clear decoded text after this many seconds


def _detection_worker():
    """Background thread: continuously processes the latest frame."""
    global _latest_gray, _detected_text, _detected_corners, _is_detecting, _last_decode_time

    while True:
        # Grab latest frame
        with _frame_lock:
            gray = _latest_gray
            _latest_gray = None  # mark as consumed

        if gray is None:
            time.sleep(0.005)
            continue

        with _result_lock:
            _is_detecting = True

        text, corners = detect_qr(gray)

        with _result_lock:
            _is_detecting = False
            _detected_corners = corners
            if text:
                _detected_text = text
                _last_decode_time = time.time()


def monitor_video():
    global _latest_gray

    global _latest_gray, _detected_text, _last_decode_time

    # Start detection thread
    t = threading.Thread(target=_detection_worker, daemon=True)
    t.start()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  IN_FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IN_FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

    last_capture_time = 0
    img_prev = np.zeros((IN_FRAME_HEIGHT, IN_FRAME_WIDTH, 3), np.uint8)

    # Thumbnail scale factors
    sx = OUT_FRAME_WIDTH  / FRAME_WIDTH
    sy = OUT_FRAME_HEIGHT / FRAME_HEIGHT

    try:
        while True:
            current_time = time.time()
            if current_time - last_capture_time < CAPTURE_INTERVAL:
                cv2.waitKey(1)
                continue
            last_capture_time = current_time

            ret, frame = cap.read()
            if not ret or frame is None or len(frame) == 0:
                frame = img_prev.copy()
            else:
                img_prev = frame.copy()
                # Feed latest frame to detection thread
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                with _frame_lock:
                    _latest_gray = gray

            # Resize to processing resolution if needed
            if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
                frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            # Read detection state; expire text after TEXT_EXPIRY_SEC
            with _result_lock:
                if _detected_text and (time.time() - _last_decode_time) > TEXT_EXPIRY_SEC:
                    _detected_text = ""
                qr_text   = _detected_text
                corners   = _detected_corners
                detecting = _is_detecting

            # Gray out regions outside the crop (x < CROP_X1 and x > CROP_X2)
            vis = frame.copy()
            gray3 = cv2.cvtColor(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
            vis[:, :_CROP_X1]  = gray3[:, :_CROP_X1]
            vis[:, _CROP_X2:]  = gray3[:, _CROP_X2:]

            # Draw bounding box around located QR (in full-image coords)
            if corners is not None:
                color = (0, 255, 0) if qr_text else (0, 200, 255)
                pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(vis, [pts], isClosed=True, color=color, thickness=3)

            # Shrink to display size
            thumb = cv2.resize(vis, (OUT_FRAME_WIDTH, OUT_FRAME_HEIGHT))

            # Draw corners on thumbnail (scaled)
            if corners is not None:
                thumb_pts = np.array(
                    [(int(x * sx), int(y * sy)) for x, y in corners],
                    dtype=np.int32,
                ).reshape((-1, 1, 2))
                color = (0, 255, 0) if qr_text else (0, 200, 255)
                cv2.polylines(thumb, [thumb_pts], isClosed=True, color=color, thickness=2)

            # Build text panel
            text_h = 80
            panel = np.zeros((text_h, OUT_FRAME_WIDTH, 3), np.uint8)

            # Status line
            if detecting:
                status = "Detecting..."
                status_color = (0, 200, 255)
            elif corners is not None and not qr_text:
                status = "QR located - decoding failed"
                status_color = (0, 140, 255)
            elif qr_text:
                status = "Decoded"
                status_color = (0, 255, 0)
            else:
                status = "No QR"
                status_color = (120, 120, 120)

            cv2.putText(panel, status, (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 1, cv2.LINE_AA)

            # Decoded text (persistent until next success)
            if qr_text:
                display_text = qr_text if len(qr_text) <= 48 else qr_text[:45] + "..."
                cv2.putText(panel, display_text, (8, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)

            img_comm = cv2.vconcat([thumb, panel])
            cv2.imshow("QR Monitor", img_comm)

            if cv2.waitKey(1) == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    monitor_video()
