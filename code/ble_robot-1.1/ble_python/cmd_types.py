from enum import Enum

class CMD(Enum):
    PING = 0
    SEND_TWO_INTS = 1
    SEND_THREE_FLOATS = 2
    ECHO = 3
    DANCE = 4
    SET_VEL = 5
    GET_TIME_MILLIS = 6
    GET_TEMP_5s = 7
    GET_TEMP_5s_RAPID = 8
    GET_TOF_5s = 9
    GET_ACC_5s = 10
    GET_IMU_5s_rapid = 11
    GET_IMU_ToF_5s = 12