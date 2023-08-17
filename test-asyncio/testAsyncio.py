import asyncio
import serial
import numpy as np

RASPIKE_RX_SIZE = 11
MAX_CMD_ID = 127
DATA_AREA_SIZE = MAX_CMD_ID + 1

# cmd_id for TX
PANEL_LED = 0
MOTOR_A_POWER = 1
MOTOR_B_POWER = 2
MOTOR_C_POWER = 3
MOTOR_D_POWER = 4
MOTOR_A_STOP = 5
MOTOR_B_STOP = 6
MOTOR_C_STOP = 7
MOTOR_D_STOP = 8
MOTOR_A_RESET = 9
MOTOR_B_RESET = 10
MOTOR_C_RESET = 11
MOTOR_D_RESET = 12
GYRO_RESET = 13
SENSOR_1_CONFIG = 56
SENSOR_2_CONFIG = 57
SENSOR_3_CONFIG = 58
SENSOR_4_CONFIG = 59
SENSOR_1_MODE = 60
SENSOR_2_MODE = 61
SENSOR_3_MODE = 62
SENSOR_4_MODE = 63
MOTOR_A_CONFIG = 64
MOTOR_B_CONFIG = 65
MOTOR_C_CONFIG = 66
MOTOR_D_CONFIG = 67
# cmd type for TX
ONE_WAY_CMD = 0
WAIT_ACK_CMD = 1

ack_received = np.full(DATA_AREA_SIZE, False, dtype=np.bool)
rx_data_area = np.zeros(DATA_AREA_SIZE, dtype=np.int32)
tx_data_area = np.zeros(DATA_AREA_SIZE, dtype=np.int16)
tx_data_area_prev = np.zeros(DATA_AREA_SIZE, dtype=np.int16)

send_order = [
    [SENSOR_1_CONFIG, ONE_WAY_CMD],
    [SENSOR_2_CONFIG, ONE_WAY_CMD],
    [SENSOR_3_CONFIG, ONE_WAY_CMD],
    [SENSOR_4_CONFIG, ONE_WAY_CMD],
    [SENSOR_1_MODE, WAIT_ACK_CMD],
    [SENSOR_2_MODE, WAIT_ACK_CMD],
    [SENSOR_3_MODE, WAIT_ACK_CMD],
    [SENSOR_4_MODE, WAIT_ACK_CMD],
    [MOTOR_A_CONFIG, ONE_WAY_CMD],
    [MOTOR_B_CONFIG, ONE_WAY_CMD],
    [MOTOR_C_CONFIG, ONE_WAY_CMD],
    [MOTOR_D_CONFIG, ONE_WAY_CMD],
    [MOTOR_A_STOP, WAIT_ACK_CMD],
    [MOTOR_B_STOP, WAIT_ACK_CMD],
    [MOTOR_C_STOP, WAIT_ACK_CMD],
    [MOTOR_D_STOP, WAIT_ACK_CMD],
    [MOTOR_A_RESET, WAIT_ACK_CMD],
    [MOTOR_B_RESET, WAIT_ACK_CMD],
    [MOTOR_C_RESET, WAIT_ACK_CMD],
    [MOTOR_D_RESET, WAIT_ACK_CMD],
    [GYRO_RESET, WAIT_ACK_CMD],
    [MOTOR_A_POWER, ONE_WAY_CMD],
    [MOTOR_B_POWER, ONE_WAY_CMD],
    [MOTOR_C_POWER, ONE_WAY_CMD],
    [MOTOR_D_POWER, ONE_WAY_CMD],
    [PANEL_LED, ONE_WAY_CMD],
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

    rawdata = ser.read(RASPIKE_RX_SIZE)
    data = rawdata.decode('utf-8').split(':')
    cmd_id = int(data[0])
    val = int(data[1])
    if (cmd_id < 0 or cmd_id > MAX_CMD_ID):
        print("cmd value error!")
    if type == "status":
        rx_data_area[cmd_id] = val
        #if cmd_id == 64:
        #    print(f" pkt = [{cmd_id}:{val}]")
    else:
        if cmd_id != 127:
            print(f" -- ack = {cmd_id}")
            ack_received[cmd_id] = True

async def receiver():
    print(" -- receiver start")
    while True:
        receive_packet()
        await asyncio.sleep(0.01)

def make_cmd(cmd_id, value) -> bytearray:
    buf = bytearray(3)
    value_abs = int(abs(value))
    # Byte 1: MSB = 1
    buf[0] = 0x80 | (cmd_id & 0x7f)
    # Byte 2: MSB = 0 + 0 + sign bit + 5 higher value bits
    buf[1] = (value_abs >> 7) & 0x1f
    if value < 0:
        # set sign bit to minus
        buf[1] |= 0x20
    # Byte 3: MSB = 0 + 7 lower value bits
    buf[2] = value_abs & 0x7f
    if cmd_id != 127:
        print(f" -- cmd = {buf.hex()}[{cmd_id}:{value}]")
    return buf
        
async def send_packet():
    print(" -- send_packet start")
    while True:
        k = 0
        for [cmd_id,type] in send_order:
            if tx_data_area[cmd_id] != tx_data_area_prev[cmd_id]:
                ack_received[cmd_id] = False
                ser.write(make_cmd(cmd_id, tx_data_area[cmd_id]))
                k += 1
                if type == WAIT_ACK_CMD:
                    while True:
                        #print(f" --- waiting for ack = {cmd_id}")
                        if ack_received[cmd_id]:
                            break
                        await asyncio.sleep(0.01)
                tx_data_area_prev[cmd_id] = tx_data_area[cmd_id]
        if k == 0:
            ser.write(make_cmd(127,0))
        await asyncio.sleep(0.01)
        
# reference: RasPike/target/raspi_gcc/drivers/motor/src/motor_dri.c
MOTOR_A = 0
MOTOR_B = 1
MOTOR_C = 2
MOTOR_D = 3
motor_power = np.zeros(4, dtype=np.int16)

def motor_get_angle(motor) -> int: # opOUTPUT_GET_COUNT
    return rx_data_area[MOTOR_A_CONFIG + motor]

def motor_clr_angle(motor): # opOUTPUT_CLR_COUNT
    tx_data_area[MOTOR_A_RESET + motor] = 1

def motor_set_power(motor, power): # opOUTPUT_POWER & opOUTPUT_SPEED
    motor_power[motor] = power

def motor_start(motor): # opOUTPUT_START
    tx_data_area[MOTOR_A_POWER + motor] = motor_power[motor]

def motor_break(motor, brake): # opOUTPUT_STOP
    tx_data_area[MOTOR_A_POWER + motor] = 0
    tx_data_area[MOTOR_A_STOP + motor] = brake

def motor_config(motor):
    tx_data_area[MOTOR_A_CONFIG + motor] = 20 # opOUTPUT_SET_TYPE

# reference: RasPike/sdk/common/library/libcpp-ev3/src/Motor.cpp
#            RasPike/sdk/common/library/libcpp-ev3/include/Motor.h
#            RasPike/sdk/common/ev3api/src/ev3api_motor.c
PWM_MAX = 100
PWM_MIN = -100
brake_mode = 0
def ev3_motor_config(motor):
    motor_config(motor)
    motor_break(motor, 0) 

def ev3_motor_reset_counts(motor):
    motor_clr_angle(motor)

def ev3_motor_set_power(motor, pwm):
    if pwm > PWM_MAX:
        pwm = PWM_MAX
    if pwm < PWM_MIN:
        pwm = PWM_MIN
    if brake_mode == 1 and pwm == 0:
        motor_break(motor, 1) # ev3_motor_stop
    else:
        motor_set_power(motor, pwm) # ev3_motor_set_power
        motor_start(motor) # ev3_motor_set_power

def ev3_motor_stop(motor, brake):
    motor_break(motor, brake)
        
async def main_task():
    print(" -- main task start")
    task1 = asyncio.create_task(receiver())
    task2 = asyncio.create_task(send_packet())
    
    print(" -- main task logic start")
    ev3_motor_config(MOTOR_A)
    ev3_motor_reset_counts(MOTOR_A)
    ev3_motor_set_power(MOTOR_A, -30)
    await asyncio.sleep(1.0)
    ev3_motor_set_power(MOTOR_A, 30)
    await asyncio.sleep(1.0)
    ev3_motor_stop(MOTOR_A, 0)
    # TODO: send_packet() to send everything before cancelled
    await asyncio.sleep(1.0)
    print(" -- main task logic end")
    
    task1.cancel()
    task2.cancel()
    print(" -- main task end")
        
asyncio.run(main_task())
