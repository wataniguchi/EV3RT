from pathlib import Path
import sys
import threading
import time
import libraspike_art_python as lib
from libraspike_art_python import pbio_port, pbio_color, pbio_error

RASPIKE_COM_NAME = '/dev/USB_SPIKE'

COLLECT_INTERVAL = 0.1
COLLECT_DURATION = 10.0

CASE_COLOR = {
    0: 'black',
    1: 'blue',
    2: 'green',
    3: 'yellow',
    4: 'red',
    5: 'white',
    6: 'black - white boundary',
    7: 'black - blue boundary',
    8: 'black - green boundary',
    9: 'black - yellow boundary',
    10: 'black - red boundary',
}

def receiver_task() -> None:
    while True:
        lib.raspike_prot_receive()

def wait_for_touch(force_sensor) -> bool:
    print("Waiting for touch...")
    while True:
        time.sleep(0.1)
        if lib.pup_force_sensor_touched(force_sensor):
            return True
    return False

def collect_data(color_sensor, case_id, case_name, file) -> None:
    time_start = time.time()
    while time.time() - time_start < COLLECT_DURATION:
        time.sleep(COLLECT_INTERVAL)
        r,g,b = lib.pup_color_sensor_rgb(color_sensor)
        h,s,v = lib.pup_color_sensor_hsv(color_sensor, True)
        print(f"Case {case_id}: RGB: {r:3d} {g:3d} {b:3d}  HSV: {h:3d} {s:3d} {v:3d}")
        file.write(f"{case_id},{case_name},{r},{g},{b},{h},{s},{v}\n")

if __name__ == "__main__":
    desc = lib.raspike_open_usb_communication(RASPIKE_COM_NAME)
    if desc is None:
        print(f"Cannot Open desc name={RASPIKE_COM_NAME}")
        sys.exit(-1)
    
    lib.raspike_prot_init(desc)

    t1 = threading.Thread(target=receiver_task,daemon=True)
    t1.start()

    force_sensor = lib.pup_force_sensor_get_device(pbio_port.ID_D)
    color_sensor = lib.pup_color_sensor_get_device(pbio_port.ID_E)

    # create directory for data if not exists
    data_dir = Path('data')
    data_dir.mkdir(exist_ok=True)
    # create a file in overwrite mode
    data_file = data_dir / 'color_sensor_data.txt'
    with data_file.open('w') as f:
        f.write("CaseID,CaseName,RGB_R,RGB_G,RGB_B,HSV_H,HSV_S,HSV_V\n")

        # collect data for each case
        for case_id, case_name in CASE_COLOR.items():
            print(f"Case {case_id}: {case_name}")
            if wait_for_touch(force_sensor):
                collect_data(color_sensor, case_id, case_name, f)