import asyncio
import serial
import numpy as np

RASPIKE_RX_SIZE = 11
MAX_CMD_ID = 127
DATA_AREA_SIZE = MAX_CMD_ID + 1
SENSOR_1_CONFIG = 56
MOTOR_A_CONFIG = 64
ONE_WAY_CMD = 0
WAIT_ACK_CMD = 1

ack_received = np.full(DATA_AREA_SIZE, False, dtype=np.bool)
rx_data_area = np.zeros(DATA_AREA_SIZE, dtype=np.int32)
tx_data_area = np.zeros(DATA_AREA_SIZE, dtype=np.int16)
tx_data_area_prev = np.zeros(DATA_AREA_SIZE, dtype=np.int16)

send_order = [
    [SENSOR_1_CONFIG, ONE_WAY_CMD],
    [MOTOR_A_CONFIG, WAIT_ACK_CMD]
    ]

ser = serial.Serial('/dev/ttyAMA1', 115200)

def receive_packet():
    while True:
        c = ser.read()
        if c == b'@':
            type = "status"
            break
        elif c == b'<':
            type = "ack"
            break
        else:
            print(f'@={c.hex()}')

    data = ser.read(RASPIKE_RX_SIZE)
    data = data.decode('utf-8').split(':')
    cmd_id = int(data[0])
    val = int(data[1])
    if (cmd_id < 0 or cmd_id > MAX_CMD_ID):
        print("cmd value error!")
    if type == "status":
        rx_data_area[cmd_id] = val
        if (cmd_id == 29 or cmd_id == 30):
            print(cmd_id, val)
    else:
        if cmd_id != 127:
            ack_received[cmd_id] = True

async def receiver():
    print(" -- receiver start")
    while True:
        receive_packet()
        await asyncio.sleep(0.01)

def send_packet():
    for [cmd_id,type] in send_order:
        if tx_data_area[cmd_id] != tx_data_area_prev[cmd_id]:
            value = int(abs(tx_data_area[cmd_id]))
            # Byte 1: MSB = 1
            byte1 = (0x80 | (cmd_id & 0x7f)).to_bytes(1, byteorder = "big")
            # Byte 2: MSB = 0 + 0 + sign bit + 5 higher value bits
            if tx_data_area[cmd_id] >= 0:
                byte2 = ((value >> 7) & 0x1f).to_bytes(1, byteorder = "big")
            else:
                # set sign bit to minus
                byte2 = ((value >> 7) & 0x1f | 0x20).to_bytes(1, byteorder = "big")
            # Byte 3: MSB = 0 + 7 lower value bits
            byte3 = (value & 0x7f).to_bytes(1, byteorder = "big")
            print(byte1.hex(), byte2.hex(), byte3.hex())
        
async def transmitter():
    print(" -- transmitter start")
    while True:
        send_packet()
        await asyncio.sleep(0.01)

async def main_task():
    task1 = asyncio.create_task(receiver())
    task2 = asyncio.create_task(transmitter())
    print(" -- main task logic start")
    tx_data_area[MOTOR_A_CONFIG] = -1000
    await task1
    await task2
    print(" -- main task end")
        
asyncio.run(main_task())
