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
#define RET_NOT_SUPPORTED_FUNCTION  7
#define RET_TIMEOUT             8

#define RET_RESPONSE_TIMEOUT    9

#define RET_DATA_TOO_LONG       10
#define RET_BUFFER_TOO_SMALL    11

#define RET_SOCKET_CLOSED       12
#define RET_OUT_OF_RANGE        13
#define RET_ALREADY_EXIST       14

#define RET_JOIN_FAILED         (0x1000 | 0x01)

#define RET_INVALID_COMMAND     (0x2000 | 0x01)

#endif
