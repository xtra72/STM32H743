#ifndef WIRE_GUARD_H__
#define WIRE_GUARD_H__


typedef enum
{
    WG_STATUS_STOPPED,
    WG_STATUS_INIT,
    WG_STATUS_INITIALIZING,
    WG_STATUS_INIT_FINISHED,
    WG_STATUS_WAITING_FOR_CONTRACT,
    WG_STATUS_READY,
    WG_STATUS_MOTION_DETECTION,
    WG_STATUS_MOTION_DETECTED,
    WG_STATUS_SCAN,
    WG_STATUS_SLEEP,
    WG_STATUS_MAX
}   WG_STATUS;

const   char*       WG_STATUS_getString(WG_STATUS _status);

        bool        WG_setStatus(WG_STATUS _status);
        WG_STATUS   WG_getStatus(void);

        RET_VALUE   WG_KEEPALIVE_init(bool _start);
        RET_VALUE   WG_KEEPALIVE_start();
        RET_VALUE   WG_KEEPALIVE_stop();
        uint32_t    WG_KEEPALIVE_getPeriod();
        RET_VALUE   WG_KEEPALIVE_setPeriod(uint32_t _period);

        uint32_t    WG_getTransferInterval();
        RET_VALUE   WG_setTransferInterval(uint32_t _interval);

        uint32_t    WG_getReadyTimeout();
        RET_VALUE   WG_setReadyTimeout(uint32_t _interval);

        uint32_t    WG_getTransferNOP();
        RET_VALUE   WG_setTransferNOP(uint32_t _nop);


        void        WG_main(void);
        RET_VALUE   WG_onData(uint8_t *_data);

        RET_VALUE   WG_TRANS_init(void);
        RET_VALUE   WG_TRANS_start();
        RET_VALUE   WG_TRANS_stop();
        bool        WG_TRANS_isRun();

        bool        WG_toSleep(uint32_t wakeUpTime);

#endif
