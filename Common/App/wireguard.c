#include "target.h"
#include "config.h"
#include "adc.h"
#include "wireguard.h"
#include "rf.h"
#include "scan.h"
#include "shell_rf.h"
#include "shell_scan.h"
#include "max17043.h"
#include "system.h"

#define __MODULE_NAME__ "WG"
#include "trace.h"

static  void WG_taskMain(void const * argument);
static  RET_VALUE   WG_commandProcessing(uint8_t* _data, uint32_t _length);

void    TRACE_monitorCallback(void const * argument);
void    WG_readyTimeoutCallback(void const* argument);

RET_VALUE   SHELL_monitor(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command);
RET_VALUE   SHELL_adc(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command);
RET_VALUE   SHELL_sdram(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command);
RET_VALUE   SHELL_config(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command);
RET_VALUE   SHELL_spi(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command);
RET_VALUE   SHELL_i2c(char *argv[], uint32_t argc, struct _SHELL_COMMAND  const* command);

RET_VALUE   COM_scan(char *argv[], uint32_t argc, struct _COM_COMMAND  const* command);
RET_VALUE   COM_DATA(char *argv[], uint32_t argc, struct _COM_COMMAND  const* command);

extern  uint32_t    ST_count;
CONFIG  config_;

static  osThreadId          threadId_ = NULL;
static  bool                stop_ = true;
static  QueueHandle_t       rxQueue_ = NULL;

static  uint32_t    motionDetectedNotificationCount_ = 0;
static  uint32_t    motionDetectedNotificationCountMax_ = 1;
static  uint32_t    motionDetectedNotificationInterval_ = 10000;
static  uint32_t    motionDetectedTime_ = 0;

static  uint32_t            sleepTime_ = 0;
static  uint32_t            sleepStartTime_ = 0;

static const SHELL_COMMAND   shellCommands[] =
{
    {
        .name       = "monitor",
        .admin      = true,
        .function   = SHELL_monitor,
        .shortHelp  = "Monitor"
    },
    {
        .name       = "config",
        .admin      = false,
        .function   = SHELL_config,
        .shortHelp  = "Config"
    },
    {
        .name       = "adc",
        .admin      = true,
        .function   = SHELL_adc,
        .shortHelp  = "ADC"
    },
    {
        .name       = "rf",
        .admin      = false,
        .function   = SHELL_RF,
        .shortHelp  = "RF"
    },
    {
        .name       = "spi",
        .admin      = true,
        .function   = SHELL_spi,
        .shortHelp  = "SPI"
    },
    {
        .name       = "i2c",
        .admin      = true,
        .function   = SHELL_i2c,
        .shortHelp  = "I2C"
    },
    {
        .name       = "scan",
        .admin      = false,
        .function   = SHELL_SCAN,
        .shortHelp  = "Scan"
    },
    {
        .name       = "sdram",
        .admin      = true,
        .function   = SHELL_sdram,
        .shortHelp  = "SDRAM"
    }
};

#if SUPPORT_COM
static const COM_COMMAND   comCommands[] =
{
    {
        .name       = "scan",
        .function   = COM_scan,
        .shortHelp  = "Scan"
    },
    {
        .name       = "data",
        .function   = COM_DATA,
        .shortHelp  = "Data"
    }
};
#endif

static  const char*   statusStrings_[] =
{
    "Stopped",
    "Init",
    "Init",
    "Init",
    "Waiting for contract",
    "Ready",
    "Motion Detection",
    "Motion Detected",
    "Scan",
    "Sleep"
};

static  WG_STATUS   status_ = WG_STATUS_INIT;

static  osTimerId   timerReadyTimeoutHandler = 0;

bool        WG_setStatus(WG_STATUS _status)
{
    if (status_ != _status)
    {
        if (timerReadyTimeoutHandler == 0)
        {
            osTimerDef(timerReadyTimeout, WG_readyTimeoutCallback);
            timerReadyTimeoutHandler = osTimerCreate(osTimer(timerReadyTimeout), osTimerOnce, NULL);
        }

        if (status_ == WG_STATUS_READY)
        {
            osTimerStop(timerReadyTimeoutHandler);
        }
        DEBUG("Status changed : %s -> %s", WG_STATUS_getString(status_), WG_STATUS_getString(_status));
        status_ = _status;

        if (status_ == WG_STATUS_READY)
        {
            DEBUG("Set Ready Timeout : %d s", config_.readyTimeout);
            osTimerStart(timerReadyTimeoutHandler, config_.readyTimeout * 1000);
        }

    }

    return  true;
}

WG_STATUS   WG_getStatus(void)
{
    return  status_;
}

const char*       WG_STATUS_getString(WG_STATUS status)
{
    if (status < WG_STATUS_MAX)
    {
        return  statusStrings_[status];
    }

    return  "Unknown";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Keep Alive
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static  osTimerId   WG_KEEPALIVE_timerHandler_ = 0;

uint32_t    WG_KEEPALIVE_getPeriod()
{
    return  config_.keepAlive;
}

RET_VALUE   WG_KEEPALIVE_setPeriod(uint32_t _period)
{
    config_.keepAlive = _period;

    return  RET_OK;
}

static  void    WG_KEEPALIVE_callback(void const * argument)
{
    uint16_t    cell = 0;

    MAX17043_getCell(&cell);

    RF_sendKeepAlive(0, cell, 100);
}

RET_VALUE   WG_KEEPALIVE_start()
{
    if (osTimerStart(WG_KEEPALIVE_timerHandler_, config_.keepAlive * 1000) != osOK)
    {
        DEBUG("Keep Alive timer start failed!\n");
        return  RET_ERROR;
    }

    return  RET_OK;
}

RET_VALUE   WG_KEEPALIVE_stop()
{
    if (osTimerStop(WG_KEEPALIVE_timerHandler_) != osOK)
    {
        DEBUG("Keep Alive timer stop failed!\n");
        return  RET_ERROR;
    }

    return  RET_OK;
}

RET_VALUE   WG_KEEPALIVE_init(bool _start)
{
    osTimerDef(timerKeepAlive, WG_KEEPALIVE_callback);
    WG_KEEPALIVE_timerHandler_ = osTimerCreate(osTimer(timerKeepAlive), osTimerPeriodic, NULL);

    if (_start)
    {
        return  WG_KEEPALIVE_start();
    }

    return  RET_OK;
}


uint32_t    WG_getTransferInterval()
{
    return  config_.transferInterval;
}

RET_VALUE   WG_setTransferInterval(uint32_t _interval)
{
    config_.transferInterval = _interval;

    return  RET_OK;
}

uint32_t    WG_getReadyTimeout()
{
    return  config_.readyTimeout;
}

RET_VALUE   WG_setReadyTimeout(uint32_t _timeout)
{
    config_.readyTimeout = _timeout;

    return  RET_OK;
}

uint32_t    WG_getTransferNOP()
{
    return  config_.nop;
}

RET_VALUE   WG_setTransferNOP(uint32_t _nop)
{
    if ((TARGET_TRANSFER_NOP_MIN <= _nop) && (_nop <= TARGET_TRANSFER_NOP_MAX))
    {
        config_.nop = _nop;
        return  RET_OK;
    }

    return  RET_INVALID_ARGUMENT;
}

void    WG_readyTimeout(void const * argument)
{
    RF_motionDetectionStart(0);
    WG_setStatus(WG_STATUS_MOTION_DETECTION);
}


void WG_main(void)
{
    if (CONFIG_load(&config_) != RET_OK)
    {
        CONFIG_loadDefault(&config_);
    }

    SHELL_init(shellCommands, sizeof(shellCommands) / sizeof(SHELL_COMMAND));
#if SUPPORT_COM
    COM_init(comCommands, sizeof(comCommands) / sizeof(COM_COMMAND));
#endif
    TIME2_init();

    SHELL_start();
    SDRAM_start();
#if SUPPORT_COM
    COM_start();
#endif
    ADC_start();

#if SUPPORT_DRAM
    SCAN_init();
#endif

    rxQueue_ = xQueueCreate( 16, RF_getBufferSize());

    osThreadDef(taskMain, WG_taskMain, osPriorityIdle, 0, 512);
    threadId_ = osThreadCreate(osThread(taskMain), NULL);
    if (threadId_ == NULL)
    {
        DEBUG("Main task creation failed!");
    }

    WG_KEEPALIVE_init(true);

    for(;;)
    {
        osDelay(1);
    }
}

RET_VALUE   WG_stop(void)
{
    RET_VALUE   ret = RET_OK;

    while(!stop_)
    {
        osDelay(1);
    }

    osThreadTerminate(threadId_);
    threadId_ = NULL;

    return  ret;
}


void WG_taskMain(void const * argument)
{
    DEBUG("RF task start!\n");
    static      uint32_t    initial_time;
    static      uint32_t    init_state = 0;
    uint32_t    serverRequestTime = 0;

    static  uint8_t     buffer[512];

    WG_TRANS_init();

    RF_reset();

    stop_ = false;

    RF_start();

    while(!stop_)
    {
        if (xQueueReceive(rxQueue_, buffer, 10)  == pdPASS)
        {
            WG_commandProcessing(buffer, RF_getBufferSize());
        }

        switch(WG_getStatus())
        {
        case    WG_STATUS_INIT:
            {
                init_state = 0;
                initial_time = TICK_get();
                WG_setStatus(WG_STATUS_INITIALIZING);
            }
            break;

        case    WG_STATUS_INITIALIZING:
            {
                uint32_t    current_time = TICK_get();
                uint32_t    diff_time = current_time - initial_time;

                if (current_time - initial_time >= 10000)
                {
                    WG_setStatus(WG_STATUS_INIT_FINISHED);
                }
                else if ((diff_time / 1000) != init_state)
                {
                    init_state = (diff_time / 1000);
                    switch(init_state)
                    {
                    case    1:
                        RF_motionDetectionStart(0);
                        break;

                    case    2:
                        {
                            uint16_t    cell = 0;
                            if (MAX17043_getCell(&cell) != RET_OK)
                            {
                                ERROR("Cell voltage check failed!\n");
                            }
                            else
                            {
                                ERROR("Cell voltage : %3.1f V\n", (cell / 100) / 10.0);
                            }
                        }
                        break;

                    case    9:
                        RF_motionDetectionStop(0);
                        break;
                    }
                }
            }
            break;

        case    WG_STATUS_INIT_FINISHED:
            {
                RF_sendRadioStart(0, &config_.rf, 10);
                WG_setStatus(WG_STATUS_WAITING_FOR_CONTRACT);
            }
            break;

        case    WG_STATUS_WAITING_FOR_CONTRACT:
            {
                if (TICK_remainTime(serverRequestTime, 10000) == 0)
                {
                    RF_sendContract(0, config_.deviceId, config_.adc.channelCount, 10);
                    WG_setStatus(WG_STATUS_WAITING_FOR_CONTRACT);
                    serverRequestTime =  TICK_get();;
                }
            }
            break;
        case    WG_STATUS_READY:
            {
            }
            break;

        case    WG_STATUS_MOTION_DETECTED:
            {
                if (motionDetectedNotificationCount_ < motionDetectedNotificationCountMax_)
                {
                    if (TICK_remainTime(motionDetectedTime_, motionDetectedNotificationCount_ * motionDetectedNotificationInterval_) == 0)
                    {
                        RF_sendMotionDetected(0, 1000);
                        motionDetectedNotificationCount_++;
                    }
                }
                else
                {
                    if (TICK_remainTime(motionDetectedTime_, motionDetectedNotificationCountMax_ * motionDetectedNotificationInterval_) == 0)
                    {
                        if (RF_motionDetectionStart(0) == RET_OK)
                        {
                            WG_setStatus(WG_STATUS_MOTION_DETECTION);
                        }
                    }
                }
            }
            break;

        case    WG_STATUS_SCAN:
            {
            }
            break;

        case    WG_STATUS_SLEEP:
            {
                if (TICK_remainTime(sleepStartTime_, sleepTime_) > 0)
                {
                    osDelay(1000);
                }
            }
            break;
        }
        osDelay(1);
    }

    stop_ = true;
    WG_setStatus(WG_STATUS_STOPPED);

}


RET_VALUE   WG_onData(uint8_t *_data)
{
    if (xQueueSend(rxQueue_, _data, 10)  != pdPASS)
    {
        return  RET_ERROR;
    }

    return  RET_OK;
}

RET_VALUE   WG_commandProcessing(uint8_t* _data, uint32_t _length)
{
    if (_length < sizeof(RF_FRAME_HEADER))
    {
        DEBUG("Invalid frame!\n");
        return  RET_ERROR;
    }

    RF_IO_FRAME*   io_frame = (RF_IO_FRAME*)_data;
    switch(io_frame->cmd)
    {
    case    RF_IO_REP_MOTION_DETECT_STARTED:
        {
            DEBUG("Motion detection started!\n");
            if (WG_getStatus() >= WG_STATUS_READY)
            {
    //            WG_setStatus(WG_STATUS_READY);
                RF_sendMotionDetectionStart(0, 100);
            }
        }
        break;

    case    RF_IO_REP_MOTION_DETECT_STOPPED:
        {
            DEBUG("Motion detection stopped!\n");
            if (WG_getStatus() >= WG_STATUS_READY)
            {
    //            if (WG_getStatus() == WG_STATUS_READY)
    //            {
    //                WG_setStatus(WG_STATUS_INIT);
    //            }
                RF_sendMotionDetectionStop(0, 100);
            }
        }
        break;

    case    RF_IO_NOTI_MOTION_DETECTED:
        {
            if (WG_getStatus() > WG_STATUS_READY)
            {
                DEBUG("Motion detected!\n");
                RF_motionDetectionStop(0);
                motionDetectedTime_ = TICK_get();
                motionDetectedNotificationCount_ = 0;
                WG_setStatus(WG_STATUS_MOTION_DETECTED);
            }
        }
        break;

    case    RF_IO_NOTI_RADIO_STARTED:
        {
            DEBUG("Radio started!\n");
        }
        break;

    case    RF_IO_NOTI_FROM_SERVER:
        {
            DEBUG("Receive slave command : %d\n", ((RF_IO_REQUEST_FROM_SLAVE_PARAMS*)io_frame->payload)->cmd);
            switch(((RF_IO_REQUEST_FROM_SLAVE_PARAMS*)io_frame->payload)->cmd)
            {
            case    RF_REQ_SRV_SCAN_START:
                {
                    SCAN_DATA_reset();
                    SCAN_start();
                    WG_TRANS_start();
                    WG_setStatus(WG_STATUS_SCAN);
                }
                break;

            case    RF_REQ_SRV_SCAN_STOP:
                {
                    SCAN_stop();
                }
                break;

            case    RF_REQ_SRV_TRANSFER_START:
                {
                    WG_TRANS_start();
                }
                break;

            case    RF_REQ_SRV_TRANSFER_STOP:
                {
                    WG_TRANS_stop();
                }
                break;

            case    RF_REQ_SRV_MOTION_DETECTION_START:
                {
                    RF_motionDetectionStart(0);
                    WG_setStatus(WG_STATUS_MOTION_DETECTION);}
                break;

            case    RF_REQ_SRV_MOTION_DETECTION_STOP:
                {
                    RF_motionDetectionStop(0);
                }
                break;

            case    RF_REQ_SRV_SLEEP:
                {
                    if (WG_getStatus() == WG_STATUS_SCAN)
                    {
                        SCAN_stop();
                        WG_TRANS_stop();
                    }
                    WG_setStatus(WG_STATUS_READY);
                }
                break;

            default:
                DEBUG("Invalid RF Command : %d\n", ((RF_IO_REQUEST_FROM_SLAVE_PARAMS*)io_frame->payload)->cmd);
            }
        }
        break;

    case    RF_IO_STATUS:
        {
        }
        break;

    case    RF_IO_DOWNLINK:
        {
            RF_FRAME*   rf_frame = (RF_FRAME*)io_frame->payload;
            if ((sizeof(RF_FRAME_HEADER) <= io_frame->length) && (sizeof(RF_FRAME_HEADER) + rf_frame->header.length <= io_frame->length))
            {
                switch(rf_frame->header.port)
                {
                case    RF_MSG_CONTRACT_CONFIRM:
                    {
                        if (WG_getStatus() == WG_STATUS_WAITING_FOR_CONTRACT)
                        {
                            if (rf_frame->header.length == sizeof(RF_RESPONSE_CONTRACT))
                            {
                                RF_RESPONSE_CONTRACT*   response_contract = (RF_RESPONSE_CONTRACT*)rf_frame->payload;
                                uint32_t    timeStamp = 0;

                                timeStamp |= (response_contract->timeStamp >> 24) & 0x000000FF;
                                timeStamp |= (response_contract->timeStamp >> 8) & 0x0000FF00;
                                timeStamp |= (response_contract->timeStamp << 8) & 0x00FF0000;
                                timeStamp |= (response_contract->timeStamp << 24) & 0xFF000000;
                                DEBUG("Contract Received : %d", timeStamp);
                                RF_motionDetectionStart(0);
                                WG_setStatus(WG_STATUS_MOTION_DETECTION);
                            }
                            else
                            {
                                DEBUG("Payload does not conform to Contract response format.");
                            }
                        }
                        else
                        {
                            DEBUG("Contract is not waiting.");
                        }
                    }
                    break;

                case    RF_MSG_MOTION_DETECTION_START:
                    {
                        if (WG_getStatus() == WG_STATUS_READY)
                        {
                            if (rf_frame->header.length == sizeof(RF_MOTION_DETECTION))
                            {
                                RF_MOTION_DETECTION*   params = (RF_MOTION_DETECTION*)rf_frame->payload;
                                uint32_t    mid = 0;

                                mid |= (params->mid >> 24) & 0x000000FF;
                                mid |= (params->mid >> 8) & 0x0000FF00;
                                mid |= (params->mid << 8) & 0x00FF0000;
                                mid |= (params->mid << 24) & 0xFF000000;

                                RF_motionDetectionStart(0);
                                WG_setStatus(WG_STATUS_MOTION_DETECTION);
                            }
                            else
                            {
                                DEBUG("Payload does not conform to Contract response format.");
                            }

                        }
                        else
                        {
                            DEBUG("This command is not allowed in the current status[%s].", WG_STATUS_getString(WG_getStatus()));
                        }
                    }
                    break;

                case    RF_MSG_MOTION_DETECTION_STOP:
                    {
                        if (WG_getStatus() == WG_STATUS_MOTION_DETECTION)
                        {
                            if (rf_frame->header.length == sizeof(RF_MOTION_DETECTION))
                            {
                                RF_MOTION_DETECTION*   params = (RF_MOTION_DETECTION*)rf_frame->payload;
                                uint32_t    mid = 0;

                                mid |= (params->mid >> 24) & 0x000000FF;
                                mid |= (params->mid >> 8) & 0x0000FF00;
                                mid |= (params->mid << 8) & 0x00FF0000;
                                mid |= (params->mid << 24) & 0xFF000000;

                                RF_motionDetectionStop(0);
                                WG_setStatus(WG_STATUS_READY);
                            }
                            else
                            {
                                DEBUG("Payload does not conform to Contract response format.");
                            }
                        }
                        else
                        {
                            DEBUG("This command is not allowed in the current status[%s].", WG_STATUS_getString(WG_getStatus()));
                        }
                    }
                    break;

                case    RF_MSG_READY:
                    {
                        switch(WG_getStatus())
                        {
                        case    WG_STATUS_SCAN:
                            {
                                SCAN_stop();
                                //WG_TRANS_stop();
                                WG_setStatus(WG_STATUS_READY);
                            }
                            break;

                        case    WG_STATUS_MOTION_DETECTION:
                            {
                                RF_motionDetectionStop(0);
                                WG_setStatus(WG_STATUS_READY);
                            }
                            break;

                        case    WG_STATUS_MOTION_DETECTED:
                            {
                                WG_setStatus(WG_STATUS_READY);
                            }
                            break;

                        default:
                            {
                                DEBUG("This command is not allowed in the current status[%s].", WG_STATUS_getString(WG_getStatus()));
                            }
                        }
                    }
                    break;

                case    RF_MSG_SCAN_START:
                    {
                        if (WG_getStatus() != WG_STATUS_SCAN)
                        {
                            if (rf_frame->header.length == sizeof(RF_REQ_SCAN_PARAMS))
                            {
                                RF_REQ_SCAN_PARAMS*   params = (RF_REQ_SCAN_PARAMS*)rf_frame->payload;
                                uint32_t    mid = 0;

                                mid |= (params->mid >> 24) & 0x000000FF;
                                mid |= (params->mid >> 8) & 0x0000FF00;
                                mid |= (params->mid << 8) & 0x00FF0000;
                                mid |= (params->mid << 24) & 0xFF000000;

                                RF_motionDetectionStop(0);
                                if (SCAN_start() == RET_OK)
                                {
                                    RF_sendScanStart(0, 100);
                                    if (!WG_TRANS_isRun())
                                    {
                                        SCAN_DATA_reset();
                                        WG_TRANS_start();
                                    }
                                    WG_setStatus(WG_STATUS_SCAN);
                                }
                            }
                            else
                            {
                                DEBUG("Payload does not conform to Contract response format.");
                            }
                        }
                        else
                        {
                            DEBUG("This command is not allowed in the current status[%s].", WG_STATUS_getString(WG_getStatus()));
                        }
                    }
                    break;

                case    RF_MSG_SCAN_STOP:
                    {
                        if (WG_getStatus() == WG_STATUS_SCAN)
                        {
                            if (rf_frame->header.length == sizeof(RF_REQ_SCAN_PARAMS))
                            {
                                RF_REQ_SCAN_PARAMS*   params = (RF_REQ_SCAN_PARAMS*)rf_frame->payload;
                                uint32_t    mid = 0;

                                mid |= (params->mid >> 24) & 0x000000FF;
                                mid |= (params->mid >> 8) & 0x0000FF00;
                                mid |= (params->mid << 8) & 0x00FF0000;
                                mid |= (params->mid << 24) & 0xFF000000;

                                SCAN_stop();
                                RF_sendScanStop(0, 100);
                                WG_setStatus(WG_STATUS_READY);
                            }
                            else
                            {
                                DEBUG("Payload does not conform to Contract response format.");
                            }
                        }
                        else
                        {
                            DEBUG("This command is not allowed in the current status[%s].", WG_STATUS_getString(WG_getStatus()));
                        }
                    }
                    break;

                case    RF_MSG_TRANS_START:
                    {
                        if (rf_frame->header.length == sizeof(RF_MOTION_DETECTION))
                        {
                            RF_MOTION_DETECTION*   params = (RF_MOTION_DETECTION*)rf_frame->payload;
                            uint32_t    mid = 0;

                            mid |= (params->mid >> 24) & 0x000000FF;
                            mid |= (params->mid >> 8) & 0x0000FF00;
                            mid |= (params->mid << 8) & 0x00FF0000;
                            mid |= (params->mid << 24) & 0xFF000000;

                            WG_TRANS_start();
                            RF_sendTransferStart(0, 100);
                        }
                        else
                        {
                            DEBUG("Payload does not conform to Contract response format.");
                        }
                    }
                    break;

                case    RF_MSG_TRANS_STOP:
                    {
                        if (rf_frame->header.length == sizeof(RF_MOTION_DETECTION))
                        {
                            RF_MOTION_DETECTION*   params = (RF_MOTION_DETECTION*)rf_frame->payload;
                            uint32_t    mid = 0;

                            mid |= (params->mid >> 24) & 0x000000FF;
                            mid |= (params->mid >> 8) & 0x0000FF00;
                            mid |= (params->mid << 8) & 0x00FF0000;
                            mid |= (params->mid << 24) & 0xFF000000;

                            WG_TRANS_stop();
                            RF_sendTransferStop(0, 100);
                        }
                        else
                        {
                            DEBUG("Payload does not conform to Contract response format.");
                        }
                    }
                break;

                case    RF_MSG_SLEEP:
                    {
                        if (rf_frame->header.length == sizeof(RF_SLEEP))
                        {
                            RF_SLEEP*   params = (RF_SLEEP *)rf_frame->payload;
                            uint32_t    sleepTime =0;

                            sleepTime |= (params->time >> 24) & 0x000000FF;
                            sleepTime |= (params->time >>  8) & 0x0000FF00;
                            sleepTime |= (params->time <<  8) & 0x00FF0000;
                            sleepTime |= (params->time << 24) & 0xFF000000;

                            RF_motionDetectionStop(0);
                            SCAN_stop();
                            WG_TRANS_stop();
                            sleepTime_ = sleepTime;
                            sleepStartTime_ = TICK_get();

                            if (SYS_sleep(sleepTime_) != RET_OK)
                            {
                                DEBUG("Sleep failed!");
                            }
                            WG_setStatus(WG_STATUS_SLEEP);
                        }
                    }
                break;

                case    RF_MSG_STOP:
                    {
                        if (WG_getStatus() == WG_STATUS_MOTION_DETECTION)
                        {
                            RF_motionDetectionStop(0);
                            WG_setStatus(WG_STATUS_READY);
                        }
                        else if (WG_getStatus() == WG_STATUS_SCAN)
                        {
                            SCAN_stop();
                            RF_sendScanStop(0, 100);
                            WG_setStatus(WG_STATUS_READY);
                        }
                    }
                    break;

                default:
                    DEBUG("Receive Data : %d\n", io_frame->length);
                    DEBUG_DUMP("DL", io_frame->payload, io_frame->length, 16);
                }
            }
            else
            {
                DEBUG("Invalid downlink data\n");
            }
        }
        break;


    default:
        DEBUG("Unknown CMD : %02x\n", io_frame->cmd);
    }

    return  RET_OK;
}



static  void WG_TRANS_callback(void const * argument);
static  osTimerId   timerLoopFinishedHandler = 0;
        uint32_t    transfer_index = 0;
        bool        doing_transfer = false;

RET_VALUE   WG_TRANS_init(void)
{
    osTimerDef(timerLoopFinished, WG_TRANS_callback);
    timerLoopFinishedHandler = osTimerCreate(osTimer(timerLoopFinished), osTimerPeriodic, NULL);

    return  RET_OK;
}

RET_VALUE   WG_TRANS_start()
{
    if (!doing_transfer)
    {
        transfer_index = 0;
        doing_transfer = true;
        osTimerStart(timerLoopFinishedHandler, config_.transferInterval);
    }

    return  RET_OK;
}

RET_VALUE   WG_TRANS_stop()
{
    if (doing_transfer)
    {
        doing_transfer = false;
        osTimerStop(timerLoopFinishedHandler);
    }

    return  RET_OK;
}

bool    WG_TRANS_isRun()
{
    return  doing_transfer;
}

void    WG_TRANS_callback(void const * argument)
{
    uint8_t    buffer[RF_PAYLOAD_SIZE_MAX];
    uint32_t    length = 0;

    if (transfer_index + config_.nop <= SCAN_getCurrentLoop())
    {
        for(uint32_t i =  0 ; i <  config_.nop ; i++)
        {
            SCAN_LOOP_DATA* data = SCAN_getLoopData(transfer_index + i);

            if (i == 0)
            {
                uint32_t    time = transfer_index * config_.scan.interval;
                buffer[length++] = (time >> 24) & 0xFF;
                buffer[length++] = (time >> 16) & 0xFF;
                buffer[length++] = (time >>  8) & 0xFF;
                buffer[length++] = (time      ) & 0xFF;
            }

            for(uint32_t j =  0 ; j <  ADC_CHANNEL_getCount() ; j++)
            {
                buffer[length++] = (data->data[j] >>  8) & 0xFF;
                buffer[length++] = (data->data[j]      ) & 0xFF;
            }
        }

        RET_VALUE ret = RF_sendData(0, buffer, length, 0, 1000);
        if (ret == RET_OK)
        {
            transfer_index += config_.nop;
        }
        else
        {
            DEBUG("Data send failed[%d/%d]\n", transfer_index, SCAN_getCurrentLoop());
        }
    }
    else
    {
        WG_TRANS_stop();
    }
}

void    WG_readyTimeoutCallback(void const* argument)
{
    RF_motionDetectionStart(0);
    WG_setStatus(WG_STATUS_MOTION_DETECTION);
}