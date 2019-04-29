#ifndef RET_VALUE_H_
#define RET_VALUE_H_

typedef int RET_VALUE;

#define RET_OK                  0
#define RET_ERROR               1
#define RET_NOT_ENOUGH_MEMORY   2

#define RET_INVALID_ARGUMENT    3
#define RET_INVALID_FRAME       4
#define RET_INVALID_RESPONSE    5

#define RET_ALREADY_STARTED     6
#define RET_NOT_RUNNING         7
#define RET_NOT_SUPPORTED_FUNCTION  8
#define RET_TIMEOUT             9

#define RET_RESPONSE_TIMEOUT    10

#define RET_DATA_TOO_LONG       11
#define RET_BUFFER_TOO_SMALL    12

#define RET_SOCKET_CLOSED       13
#define RET_OUT_OF_RANGE        14
#define RET_ALREADY_EXIST       15
#define RET_IGNORE_FARME        16

#define RET_BUSY                20
#define RET_EMPTY               21
#define RET_FRAME_TOO_SHORT     22
#define RET_FRAME_INVALID       23
#define RET_NACK                24

#define RET_JOIN_FAILED         (0x1000 | 0x01)

#define RET_INVALID_COMMAND     (0x2000 | 0x01)

#endif
