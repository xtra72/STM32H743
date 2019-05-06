#include <stdbool.h>
#include <string.h>
#include "target.h"
#include "sx1231.h"

#define __MODULE_NAME__ "SX1231"

#include "trace.h"

#define RF_FRAME_TIMEOUT(BitRate) (uint32_t)(((((uint32_t)SX1231_RF_BUFFER_SIZE_MAX *  8 * 5.0) / (4 * BitRate)) * 128) + 1)

__weak  void        SX1231_SPI_select(bool select);
__weak  bool        SX1231_SPI_transmit(uint8_t* buffer, uint32_t size, uint32_t timeout);
__weak  bool        SX1231_SPI_receive(uint8_t* buffer, uint32_t size, uint32_t timeout);

static  void        SX1231_internalSetMode(uint8_t mode);
static  void        SX1231_internalWriteRegister(uint8_t address, uint16_t value);
static  uint16_t    SX1231_internalReadRegister(uint8_t address);
static  RET_VALUE   SX1231_lockAPI(uint32_t timeout);
static  RET_VALUE   SX1231_unlockAPI(void);

/*******************************************************************
** Global variables                                               **
*******************************************************************/
static  uint8_t     state_ = SX1231_RF_STOP;                // RF state machine
static  uint8_t     previousMode = SX1231_RF_STANDBY;       // Previous chip operating mode
static  uint8_t     receiveBuffer_[SX1231_RF_BUFFER_SIZE_MAX];      // Pointer to the RF frame
static  uint32_t    receivedLength_ = 0;
static  uint32_t    timeoutSPI_ = 10;
static  SemaphoreHandle_t   receiveSemaphore_ = NULL;
static  SemaphoreHandle_t   apiSemaphore_ = NULL;
//static  bool                apiLocked = false;

static  uint16_t RegistersCfg[] =
{   // SX1231 configuration registers values
	SX1231_DEF_FIFO, // Left for convenience, not to be changed
	SX1231_DEF_OPMODE | SX1231_RF_OPMODE_SEQUENCER_ON | SX1231_RF_OPMODE_LISTEN_OFF | SX1231_RF_OPMODE_STANDBY,
	SX1231_DEF_DATAMODUL | SX1231_RF_DATAMODUL_DATAMODE_PACKET | SX1231_RF_DATAMODUL_MODULATIONTYPE_FSK | SX1231_RF_DATAMODUL_MODULATIONSHAPING_00,
	SX1231_DEF_BITRATEMSB | SX1231_RF_BITRATEMSB_300000,
	SX1231_DEF_BITRATELSB | SX1231_RF_BITRATELSB_300000,
	SX1231_DEF_FDEVMSB | SX1231_RF_FDEVMSB_5000,
	SX1231_DEF_FDEVLSB | SX1231_RF_FDEVLSB_5000,
	SX1231_DEF_FRFMSB | SX1231_RF_FRFMSB_915,
	SX1231_DEF_FRFMID | SX1231_RF_FRFMID_915,
	SX1231_DEF_FRFLSB | SX1231_RF_FRFLSB_915,
	SX1231_DEF_OSC1,
	SX1231_DEF_OSC2,
	SX1231_DEF_LOWBAT | SX1231_RF_LOWBAT_OFF | SX1231_RF_LOWBAT_TRIM_1835,
	SX1231_DEF_LISTEN1 | SX1231_RF_LISTEN1_RESOL_4100 | SX1231_RF_LISTEN1_CRITERIA_RSSI | SX1231_RF_LISTEN1_END_01,
	SX1231_DEF_LISTEN2 | SX1231_RF_LISTEN2_COEFIDLE_VALUE,
	SX1231_DEF_LISTEN3 | SX1231_RF_LISTEN3_COEFRX_VALUE,
	SX1231_DEF_VERSION, 			// Read Only

	SX1231_DEF_PALEVEL | SX1231_RF_PALEVEL_PA0_ON | SX1231_RF_PALEVEL_PA1_OFF | SX1231_RF_PALEVEL_PA2_OFF | SX1231_RF_PALEVEL_OUTPUTPOWER_11111,
	SX1231_DEF_PARAMP | SX1231_RF_PARAMP_40,
	SX1231_DEF_OCP | SX1231_RF_OCP_ON | SX1231_RF_OCP_TRIM_100,

	SX1231_DEF_AGCREF | SX1231_RF_AGCREF_AUTO_ON | SX1231_RF_AGCREF_LEVEL_MINUS80,
	SX1231_DEF_AGCTHRESH1 | SX1231_RF_AGCTHRESH1_SNRMARGIN_101 | SX1231_RF_AGCTHRESH1_STEP1_16,
	SX1231_DEF_AGCTHRESH2 | SX1231_RF_AGCTHRESH2_STEP2_3 | SX1231_RF_AGCTHRESH2_STEP3_11,
	SX1231_DEF_AGCTHRESH3 | SX1231_RF_AGCTHRESH3_STEP4_9 | SX1231_RF_AGCTHRESH3_STEP5_11,
	SX1231_DEF_LNA | SX1231_RF_LNA_ZIN_200 | SX1231_RF_LNA_LOWPOWER_OFF | SX1231_RF_LNA_GAINSELECT_AUTO,
	SX1231_DEF_RXBW | SX1231_RF_RXBW_DCCFREQ_010 | SX1231_RF_RXBW_MANT_24 | SX1231_RF_RXBW_EXP_5,
	SX1231_DEF_AFCBW | SX1231_RF_AFCBW_DCCFREQAFC_100 | SX1231_RF_AFCBW_MANTAFC_20 | SX1231_RF_AFCBW_EXPAFC_3,
	SX1231_DEF_OOKPEAK | SX1231_RF_OOKPEAK_THRESHTYPE_PEAK | SX1231_RF_OOKPEAK_PEAKTHRESHSTEP_000 | SX1231_RF_OOKPEAK_PEAKTHRESHDEC_000,
	SX1231_DEF_OOKAVG | SX1231_RF_OOKAVG_AVERAGETHRESHFILT_10,
	SX1231_DEF_OOKFIX | SX1231_RF_OOKFIX_FIXEDTHRESH_VALUE,
	SX1231_DEF_AFCFEI | SX1231_RF_AFCFEI_AFCAUTOCLEAR_OFF | SX1231_RF_AFCFEI_AFCAUTO_OFF,
	SX1231_DEF_AFCMSB, 			// Read Only
	SX1231_DEF_AFCLSB, 			// Read Only
	SX1231_DEF_FEIMSB, 			// Read Only
	SX1231_DEF_FEILSB, 			// Read Only
	SX1231_DEF_RSSICONFIG | SX1231_RF_RSSI_FASTRX_OFF,
	SX1231_DEF_RSSIVALUE,  		// Read Only

	SX1231_DEF_DIOMAPPING1 | SX1231_RF_DIOMAPPING1_DIO0_00 | SX1231_RF_DIOMAPPING1_DIO1_00 | SX1231_RF_DIOMAPPING1_DIO2_00 | SX1231_RF_DIOMAPPING1_DIO3_00,
	SX1231_DEF_DIOMAPPING2 | SX1231_RF_DIOMAPPING2_DIO4_00 | SX1231_RF_DIOMAPPING2_DIO5_01 | SX1231_RF_DIOMAPPING2_CLKOUT_OFF,
	SX1231_DEF_IRQFLAGS1,
	SX1231_DEF_IRQFLAGS2,
	SX1231_DEF_RSSITHRESH | 228,	// Must be set to (-Sensitivity x 2)
	SX1231_DEF_RXTIMEOUT1 | SX1231_RF_RXTIMEOUT1_RXSTART_VALUE,
	SX1231_DEF_RXTIMEOUT2 | SX1231_RF_RXTIMEOUT2_RSSITHRESH_VALUE,

	SX1231_DEF_PREAMBLEMSB | SX1231_RF_PREAMBLESIZE_MSB_VALUE,
	SX1231_DEF_PREAMBLELSB | SX1231_RF_PREAMBLESIZE_LSB_VALUE,
	SX1231_DEF_SYNCCONFIG | SX1231_RF_SYNC_ON | SX1231_RF_SYNC_FIFOFILL_AUTO | SX1231_RF_SYNC_SIZE_4 | SX1231_RF_SYNC_TOL_0,
	SX1231_DEF_SYNCVALUE1 | 0x69,
	SX1231_DEF_SYNCVALUE2 | 0x81,
	SX1231_DEF_SYNCVALUE3 | 0x7E,
	SX1231_DEF_SYNCVALUE4 | 0x96,
	SX1231_DEF_SYNCVALUE5 | SX1231_RF_SYNC_BYTE5_VALUE,
	SX1231_DEF_SYNCVALUE6 | SX1231_RF_SYNC_BYTE6_VALUE,
	SX1231_DEF_SYNCVALUE7 | SX1231_RF_SYNC_BYTE7_VALUE,
	SX1231_DEF_SYNCVALUE8 | SX1231_RF_SYNC_BYTE8_VALUE,
	SX1231_DEF_PACKETCONFIG1 | SX1231_RF_PACKET1_FORMAT_VARIABLE | SX1231_RF_PACKET1_DCFREE_OFF | SX1231_RF_PACKET1_CRC_ON | SX1231_RF_PACKET1_CRCAUTOCLEAR_ON | SX1231_RF_PACKET1_ADRSFILTERING_OFF,
	SX1231_DEF_PAYLOADLENGTH | 255,
	SX1231_DEF_NODEADRS | SX1231_RF_NODEADDRESS_VALUE,
	SX1231_DEF_BROADCASTADRS | SX1231_RF_BROADCASTADDRESS_VALUE,
	SX1231_DEF_AUTOMODES | SX1231_RF_AUTOMODES_ENTER_OFF | SX1231_RF_AUTOMODES_EXIT_OFF | SX1231_RF_AUTOMODES_INTERMEDIATE_SLEEP,
	SX1231_DEF_FIFOTHRESH | SX1231_RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | SX1231_RF_FIFOTHRESH_VALUE,
	SX1231_DEF_PACKETCONFIG2 | SX1231_RF_PACKET2_RXRESTARTDELAY_1BIT | SX1231_RF_PACKET2_AUTORXRESTART_ON | SX1231_RF_PACKET2_AES_OFF,
	SX1231_DEF_AESKEY1 | SX1231_RF_AESKEY1_VALUE,
	SX1231_DEF_AESKEY2 | SX1231_RF_AESKEY2_VALUE,
	SX1231_DEF_AESKEY3 | SX1231_RF_AESKEY3_VALUE,
	SX1231_DEF_AESKEY4 | SX1231_RF_AESKEY4_VALUE,
	SX1231_DEF_AESKEY5 | SX1231_RF_AESKEY5_VALUE,
	SX1231_DEF_AESKEY6 | SX1231_RF_AESKEY6_VALUE,
	SX1231_DEF_AESKEY7 | SX1231_RF_AESKEY7_VALUE,
	SX1231_DEF_AESKEY8 | SX1231_RF_AESKEY8_VALUE,
	SX1231_DEF_AESKEY9 | SX1231_RF_AESKEY9_VALUE,
	SX1231_DEF_AESKEY10 | SX1231_RF_AESKEY10_VALUE,
	SX1231_DEF_AESKEY11 | SX1231_RF_AESKEY11_VALUE,
	SX1231_DEF_AESKEY12 | SX1231_RF_AESKEY12_VALUE,
	SX1231_DEF_AESKEY13 | SX1231_RF_AESKEY13_VALUE,
	SX1231_DEF_AESKEY14 | SX1231_RF_AESKEY14_VALUE,
	SX1231_DEF_AESKEY15 | SX1231_RF_AESKEY15_VALUE,
	SX1231_DEF_AESKEY16 | SX1231_RF_AESKEY16_VALUE,

	SX1231_DEF_TEMP1 | SX1231_RF_TEMP1_ADCLOWPOWER_ON,
	SX1231_DEF_TEMP2
};

/*******************************************************************
** Configuration functions                                        **
*******************************************************************/

void    SX1231_init(void)
{
    receiveSemaphore_ = xSemaphoreCreateBinary();
    xSemaphoreGive( receiveSemaphore_);

    apiSemaphore_ = xSemaphoreCreateBinary();
    xSemaphoreGive( apiSemaphore_ );

}

/*******************************************************************
** InitRFChip : This routine initializes the RFChip registers     **
**              Using Pre Initialized variables                   **
********************************************************************
** In  : -                                                        **
** Out : -                                                        **
*******************************************************************/
void SX1231_initRFChip (void)
{
    uint16_t i;

    /////// RC CALIBRATION (Once at POR) ///////
    SX1231_internalSetMode(SX1231_RF_STANDBY);
    SX1231_internalWriteRegister(0x57, 0x80);
    SX1231_internalWriteRegister(SX1231_REG_OSC1, SX1231_internalReadRegister(SX1231_REG_OSC1) | SX1231_RF_OSC1_RCCAL_START);
    while ((SX1231_internalReadRegister(SX1231_REG_OSC1) & SX1231_RF_OSC1_RCCAL_DONE) == 0x00);
    SX1231_internalWriteRegister(SX1231_REG_OSC1, SX1231_internalReadRegister(SX1231_REG_OSC1) | SX1231_RF_OSC1_RCCAL_START);
    while ((SX1231_internalReadRegister(SX1231_REG_OSC1) & SX1231_RF_OSC1_RCCAL_DONE) == 0x00);
    SX1231_internalWriteRegister(0x57, 0x00);
    ////////////////////////////////////////////

    for(i = 1; i <= SX1231_REG_TEMP2; i++)
    {
        SX1231_internalWriteRegister(i, RegistersCfg[i]);
    }

    SX1231_internalSetMode(SX1231_RF_SLEEP);
}

/*******************************************************************
** SX1231_setRFMode : Sets the SX1231 operating mode                     **
********************************************************************
** In  : mode                                                     **
** Out : -                                                        **
*******************************************************************/
void SX1231_setRFMode(uint8_t mode)
{
    SX1231_internalSetMode(mode);
}

uint8_t SX1231_getPreviousMode(void)
{
    return  previousMode;
}

/*******************************************************************
** SX1231_writeRegister : Writes the register value at the given address **
**                  on the SX1231                                 **
********************************************************************
** In  : address, value                                           **
** Out : -                                                        **
*******************************************************************/
void SX1231_writeRegister(uint8_t address, uint16_t value)
{
    SX1231_internalWriteRegister(address, value);
}

/*******************************************************************
** SX1231_readRegister : Reads the register value at the given address on**
**                the SX1231                                      **
********************************************************************
** In  : address                                                  **
** Out : value                                                    **
*******************************************************************/
uint16_t SX1231_readRegister(uint8_t address)
{
    return  SX1231_internalReadRegister(address);
}

/*******************************************************************
** Communication functions                                        **
*******************************************************************/

/*******************************************************************
** SendRfFrame : Sends a RF frame                                 **
********************************************************************
** In  : *buffer, size                                            **
** Out : *pReturnCode                                             **
*******************************************************************/
RET_VALUE   SX1231_sendFrame(uint8_t *buffer, uint8_t size, uint32_t timeout)
{
    RET_VALUE   ret;

    if((size+1) > SX1231_RF_BUFFER_SIZE_MAX)
    {
        state_ |= SX1231_RF_STOP;
        return  RET_ERROR;
    }

    ret = SX1231_lockAPI(timeout);
    if (ret != RET_OK)
    {
        DEBUG("API is locked\n");
        return  ret;
    }

    state_ |= SX1231_RF_BUSY;
    state_ &= ~SX1231_RF_STOP;

    SX1231_internalWriteRegister(SX1231_REG_DIOMAPPING1, (RegistersCfg[SX1231_REG_DIOMAPPING1] & 0x3F) | SX1231_RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
    SX1231_internalWriteRegister(SX1231_REG_FIFOTHRESH, (RegistersCfg[SX1231_REG_FIFOTHRESH] & 0x7F) | SX1231_RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY);

    SX1231_internalSetMode(SX1231_RF_SLEEP);

    SX1231_internalWriteRegister(SX1231_REG_FIFO, size);
    for(int i = 0; i < size; )
    {
        SX1231_internalWriteRegister(SX1231_REG_FIFO, buffer[i++]);
    }

    DEBUG("Transmit Start\n");

    SX1231_internalSetMode(SX1231_RF_TRANSMITTER); //   => Tx starts since FIFO is not empty
    DEBUG("Transmit Start\n");

    do{

    }while(!SX1231_getDIO0()); // Wait for Packet sent

        DEBUG("Transmit Finished\n");
    SX1231_internalSetMode(SX1231_RF_SLEEP);
        DEBUG("Transmit Finished\n");


    state_ |= SX1231_RF_STOP;
    state_ &= ~SX1231_RF_TX_DONE;

    SX1231_unlockAPI();

    return  RET_OK;
}

/*******************************************************************
** ReceiveRfFrame : Receives a RF frame                           **
********************************************************************
** In  : -                                                        **
** Out : *buffer, size, *pReturnCode                              **
*******************************************************************/
RET_VALUE SX1231_receiveFrame(uint8_t *_buffer, uint32_t _bufferSize, uint32_t *_receivedLength, uint32_t _timeout)
{
    RET_VALUE   ret;

    if((state_ & SX1231_RF_STOP) != SX1231_RF_STOP)
    {
        return  RET_ERROR;
    }

    ret = SX1231_lockAPI(_timeout);
    if (ret != RET_OK)
    {
        DEBUG("API is locked\n");
        return  ret;
    }

    if (xSemaphoreTake( receiveSemaphore_, ( TickType_t ) 0 )  != pdTRUE)
    {
        DEBUG("Take failed\n");
        ret = RET_ERROR;
    }
    else
    {
        TickType_t  tickStart  = xTaskGetTickCount();
        TickType_t  tickTimeout = _timeout / portTICK_PERIOD_MS;

        receivedLength_ = 0;

        SX1231_internalWriteRegister(SX1231_REG_DIOMAPPING1, (RegistersCfg[SX1231_REG_DIOMAPPING1] & 0x3F) | SX1231_RF_DIOMAPPING1_DIO0_01); // DIO0 is "PAYLOADREADY"
        SX1231_internalWriteRegister(SX1231_REG_SYNCCONFIG, (RegistersCfg[SX1231_REG_SYNCCONFIG] & 0xBF) | SX1231_RF_SYNC_FIFOFILL_AUTO);

        SX1231_internalSetMode(SX1231_RF_RECEIVER);

        state_ |= SX1231_RF_BUSY;
        state_ &= ~(SX1231_RF_STOP | SX1231_RF_TIMEOUT);

        if (xSemaphoreTake( receiveSemaphore_, tickTimeout)  != pdTRUE)
        {
            //DEBUG("Receive timeout\n");
            uint8_t flags1, flags2;
            flags1 = SX1231_internalReadRegister(SX1231_REG_IRQFLAGS1);
            flags2 = SX1231_internalReadRegister(SX1231_REG_IRQFLAGS2);

            if (flags2 & SX1231_RF_IRQFLAGS2_FIFONOTEMPTY)
            {
                DEBUG("Fifo not empty\n");
            }
            if (flags1 || flags2)
            {
    //            DEBUG("Receive timeout : %02x %02x\n", flags1, flags2);
            }

            ret = RET_TIMEOUT;
        }
        else
        {
            DEBUG("Received\n");
            if (receivedLength_ == 0)
            {
                DEBUG("Received length is 0\n");
                ret = RET_ERROR;
            }
            else if (_bufferSize < receivedLength_)
            {
                DEBUG("Received length too long\n");
                ret = RET_BUFFER_TOO_SMALL;
            }
            else
            {
                memcpy(_buffer, receiveBuffer_, receivedLength_);
                *_receivedLength = receivedLength_;
                ret = RET_OK;
            }
        }
    }

    SX1231_receiveCancel();

    SX1231_unlockAPI();

    return  ret;
}

RET_VALUE   SX1231_receiveCallback(void)
{
    static BaseType_t higherPriorityTaskWoken;

	higherPriorityTaskWoken = pdFALSE;

    SX1231_internalSetMode(SX1231_RF_SLEEP);

    receivedLength_ = SX1231_internalReadRegister(SX1231_REG_FIFO);
    for(uint32_t i = 0 ; i < sizeof(receiveBuffer_) ; i++)
    {
        receiveBuffer_[i] = SX1231_internalReadRegister(SX1231_REG_FIFO);
    }

    state_ |= SX1231_RF_STOP;
    state_ &= ~SX1231_RF_RX_DONE;

    xSemaphoreGiveFromISR( receiveSemaphore_, &higherPriorityTaskWoken );

    return RET_OK;
}

RET_VALUE   SX1231_getReceivedFrame(uint8_t* _buffer, uint32_t _bufferSize, uint32_t* _frameLength)
{
    if (receivedLength_ == 0)
    {
        return  RET_EMPTY;
    }

    if (_bufferSize < receivedLength_)
    {
        return  RET_BUFFER_TOO_SMALL;
    }

    memcpy(_buffer, receiveBuffer_, receivedLength_);
    *_frameLength = receivedLength_;

    return  RET_OK;
}

RET_VALUE   SX1231_receiveCancel(void)
{
 //   if (SX1231_getPreviousMode() == SX1231_RF_RECEIVER)
    {
        SX1231_internalSetMode(SX1231_RF_SLEEP);

        state_ |= SX1231_RF_STOP;

        xSemaphoreGive( receiveSemaphore_);
    }

    return RET_OK;
}

/*******************************************************************
** ReadRssi : Reads the Rssi value from the SX1231                **
********************************************************************
** In  : -                                                        **
** Out : value                                                    **
*******************************************************************/
uint16_t SX1231_ReadRssi(void)
{ // Must be called while in RX
	uint16_t value;
	SX1231_internalWriteRegister(SX1231_REG_RSSICONFIG, RegistersCfg[SX1231_REG_RSSICONFIG] | SX1231_RF_RSSI_START); // Triggers RSSI measurement
	while ((SX1231_internalReadRegister(SX1231_REG_RSSICONFIG) & SX1231_RF_RSSI_DONE) == 0x00);               // Waits for RSSI measurement to be completed
	value = SX1231_internalReadRegister(SX1231_REG_RSSIVALUE);                                         // Reads the RSSI result
	return value;
}

/*******************************************************************
** ReadFei : Triggers FEI measurement and returns its value       **
********************************************************************
** In  : -                                                        **
** Out : value                                                    **
*******************************************************************/
int16_t SX1231_ReadFei(void)
{ // Must be called while in RX
	int16_t value;
	SX1231_internalWriteRegister(SX1231_REG_AFCFEI, RegistersCfg[SX1231_REG_AFCFEI] | SX1231_RF_AFCFEI_FEI_START);   // Triggers FEI measurement
	while ((SX1231_internalReadRegister(SX1231_REG_AFCFEI) & SX1231_RF_AFCFEI_FEI_DONE) == 0x00);             // Waits for FEI measurement to be completed
	value = ((SX1231_internalReadRegister(SX1231_REG_FEIMSB) << 8) | SX1231_internalReadRegister(SX1231_REG_FEILSB));        // Reads the FEI result
	return value;
}

/*******************************************************************
** AutoFreqControl : Calibrates the receiver frequency to the     **
**               transmitter frequency                            **
********************************************************************
** In  : -                                                        **
** Out : *pReturnCode                                             **
*******************************************************************/
RET_VALUE   SX1231_AutoFreqControl(void)
{ // Must be called while in RX
	SX1231_internalWriteRegister(SX1231_REG_AFCFEI, RegistersCfg[SX1231_REG_AFCFEI] | SX1231_RF_AFCFEI_AFC_START);   // Triggers AFC measurement
	while ((SX1231_internalReadRegister(SX1231_REG_AFCFEI) & SX1231_RF_AFCFEI_AFC_DONE) == 0x00);             // Waits for AFC measurement to be completed

    return  RET_OK;
}

/*******************************************************************
**                                                                **
********************************************************************
** In  : -                                                        **
** Out : -                                                        **
*******************************************************************/
__weak bool SX1231_getDIO0(void)
{
    return  0;
}

/*******************************************************************
** SX1231_SPI_transmit :                                                 **
********************************************************************
** In              : -                                            **
** Out             : -                                            **
*******************************************************************/
__weak  bool    SX1231_SPI_transmit(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    return  false;
}

/*******************************************************************
** SX1231_SPI_receive :                                                 **
********************************************************************
** In              : -                                            **
** Out             : -                                            **
*******************************************************************/
__weak  bool    SX1231_SPI_receive(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    return  false;
}

/*******************************************************************
** SX1231_SPI_select :                                                 **
********************************************************************
** In              : -                                            **
** Out             : -                                            **
*******************************************************************/
__weak  void SX1231_SPI_select(bool select)
{
}


/*******************************************************************
** SX1231_writeRegister : Writes the register value at the given address **
**                  on the SX1231                                 **
********************************************************************
** In  : address, value                                           **
** Out : -                                                        **
*******************************************************************/
void SX1231_internalWriteRegister(uint8_t address, uint16_t value)
{
    uint8_t buffer[2];

    buffer[0] = 0x80 | address;
    buffer[1] = value;

    SX1231_SPI_select(true);

    SX1231_SPI_transmit(buffer, 2, timeoutSPI_);
    SX1231_SPI_select(false);
}

/*******************************************************************
** SX1231_internalReadRegister : Reads the register value at the given address on**
**                the SX1231                                      **
********************************************************************
** In  : address                                                  **
** Out : value                                                    **
*******************************************************************/
uint16_t SX1231_internalReadRegister(uint8_t address)
{
    uint8_t buffer;

    SX1231_SPI_select(true);

    SX1231_SPI_transmit(&address, 1, timeoutSPI_);
    SX1231_SPI_receive(&buffer, 1, timeoutSPI_);

    SX1231_SPI_select(false);

    return buffer;
}

void SX1231_internalSetMode(uint8_t mode)
{
    if(mode != previousMode)
    {
        if(mode == SX1231_RF_TRANSMITTER)
        {
            SX1231_internalWriteRegister(SX1231_REG_OPMODE, (RegistersCfg[SX1231_REG_OPMODE] & 0xE3) | SX1231_RF_TRANSMITTER);
            while ((SX1231_internalReadRegister(SX1231_REG_IRQFLAGS1) & SX1231_RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
            previousMode = SX1231_RF_TRANSMITTER;
        }

        else if(mode == SX1231_RF_RECEIVER)
        {
            SX1231_internalWriteRegister(SX1231_REG_OPMODE, (RegistersCfg[SX1231_REG_OPMODE] & 0xE3) | SX1231_RF_RECEIVER);
            while ((SX1231_internalReadRegister(SX1231_REG_IRQFLAGS1) & SX1231_RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
            previousMode = SX1231_RF_RECEIVER;
        }

        else if(mode == SX1231_RF_SYNTHESIZER)
        {
            SX1231_internalWriteRegister(SX1231_REG_OPMODE, (RegistersCfg[SX1231_REG_OPMODE] & 0xE3) | SX1231_RF_SYNTHESIZER);
            while ((SX1231_internalReadRegister(SX1231_REG_IRQFLAGS1) & SX1231_RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
            previousMode = SX1231_RF_SYNTHESIZER;
        }

        else if(mode == SX1231_RF_STANDBY)
        {
            SX1231_internalWriteRegister(SX1231_REG_OPMODE, (RegistersCfg[SX1231_REG_OPMODE] & 0xE3) | SX1231_RF_STANDBY);
            while ((SX1231_internalReadRegister(SX1231_REG_IRQFLAGS1) & SX1231_RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
            previousMode = SX1231_RF_STANDBY;
        }

        else
        {// mode == SX1231_RF_SLEEP
            SX1231_internalWriteRegister(SX1231_REG_OPMODE, (RegistersCfg[SX1231_REG_OPMODE] & 0xE3) | SX1231_RF_SLEEP);
            while ((SX1231_internalReadRegister(SX1231_REG_IRQFLAGS1) & SX1231_RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
            previousMode = SX1231_RF_SLEEP;
        }
    }
}

RET_VALUE   SX1231_lockAPI(uint32_t _timeout)
{
    TickType_t  tickTimeout = _timeout / portTICK_PERIOD_MS;

    if (xSemaphoreTake( apiSemaphore_, tickTimeout )  != pdTRUE)
    {
        DEBUG("Take failed\n");
        return  RET_TIMEOUT;
    }

//    apiLocked = true;


    return  RET_OK;
}

RET_VALUE   SX1231_unlockAPI(void)
{
    xSemaphoreGive( apiSemaphore_ );
//    apiLocked = false;

    return  RET_OK;
}

