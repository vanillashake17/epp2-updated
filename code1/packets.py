PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP  = 0
# TODO (Activity 2): define your own command type for the color sensor here.
COMMAND_COLOR  = 1
COMMAND_MOVE   = 2
COMMAND_ARM    = 3

RESP_OK     = 0
RESP_STATUS = 1
# TODO (Activity 2): define your own response type for the color sensor here.
RESP_COLOR  = 2
RESP_ARM    = 3

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16