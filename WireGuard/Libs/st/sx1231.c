#include <stdbool.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "sx1231.h"

/*******************************************************************
** Global variables                                               **
*******************************************************************/
static   uint8_t RFState = SX1231_RF_STOP;     // RF state machine
static   uint8_t *pRFFrame;             // Pointer to the RF frame
static   uint8_t RFFramePos;            // RF payload current position
static   uint8_t RFFrameSize;           // RF payload size
static  uint16_t ByteCounter = 0;       // RF payload byte counter
static   uint8_t PreMode = SX1231_RF_STANDBY;  // Previous chip operating mode
static   uint8_t SyncSize = 8;          // Size of sync word
static   uint8_t SyncValue[8];          // Value of sync word
static  uint32_t RFFrameTimeOut = SX1231_RF_FRAME_TIMEOUT(1200); // Reception counter value (full frame timeout generation)

static  uint16_t RegistersCfg[] =
{   // SX1231 configuration registers values
	SX1231_DEF_FIFO, // Left for convenience, not to be changed
	SX1231_DEF_OPMODE | SX1231_RF_OPMODE_SEQUENCER_ON | SX1231_RF_OPMODE_LISTEN_OFF | SX1231_RF_OPMODE_STANDBY,
	SX1231_DEF_DATAMODUL | SX1231_RF_DATAMODUL_DATAMODE_PACKET | SX1231_RF_DATAMODUL_MODULATIONTYPE_FSK | SX1231_RF_DATAMODUL_MODULATIONSHAPING_00,
	SX1231_DEF_BITRATEMSB | SX1231_RF_BITRATEMSB_4800,
	SX1231_DEF_BITRATELSB | SX1231_RF_BITRATELSB_4800,
	SX1231_DEF_FDEVMSB | SX1231_RF_FDEVMSB_5000,
	SX1231_DEF_FDEVLSB | SX1231_RF_FDEVLSB_5000,
	SX1231_DEF_FRFMSB | SX1231_RF_FRFMSB_922,
	SX1231_DEF_FRFMID | SX1231_RF_FRFMID_922,
	SX1231_DEF_FRFLSB | SX1231_RF_FRFLSB_922,
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
static  SX1231_CONFIG   _config =
{
    .select = NULL,
    .reset = NULL
};

void    SX1231_init(SX1231_CONFIG* config)
{
    if (config != NULL)
    {
        memcpy(&_config, config, sizeof(SX1231_CONFIG));
    }
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

    SX1231_reset();

    /////// RC CALIBRATION (Once at POR) ///////
    SX1231_setRFMode(SX1231_RF_STANDBY);
    SX1231_writeRegister(0x57, 0x80);
    SX1231_writeRegister(SX1231_REG_OSC1, SX1231_readRegister(SX1231_REG_OSC1) | SX1231_RF_OSC1_RCCAL_START);
    while ((SX1231_readRegister(SX1231_REG_OSC1) & SX1231_RF_OSC1_RCCAL_DONE) == 0x00);
    SX1231_writeRegister(SX1231_REG_OSC1, SX1231_readRegister(SX1231_REG_OSC1) | SX1231_RF_OSC1_RCCAL_START);
    while ((SX1231_readRegister(SX1231_REG_OSC1) & SX1231_RF_OSC1_RCCAL_DONE) == 0x00);
    SX1231_writeRegister(0x57, 0x00);
    ////////////////////////////////////////////

    for(i = 1; i <= SX1231_REG_TEMP2; i++)
    {
        SX1231_writeRegister(i, RegistersCfg[i]);
    }

    SyncSize = ((RegistersCfg[SX1231_REG_SYNCCONFIG] >> 3) & 0x07) + 1;
    for(i = 0; i < SyncSize; i++)
    {
        SyncValue[i] = RegistersCfg[SX1231_REG_SYNCVALUE1 + i];
    }

    RFFrameTimeOut = SX1231_RF_FRAME_TIMEOUT(1200); // Worst case bitrate

    SX1231_setRFMode(SX1231_RF_SLEEP);
}

/*******************************************************************
** SX1231_setRFMode : Sets the SX1231 operating mode                     **
********************************************************************
** In  : mode                                                     **
** Out : -                                                        **
*******************************************************************/
void SX1231_setRFMode(uint8_t mode)
{
    if(mode != PreMode)
    {
        if(mode == SX1231_RF_TRANSMITTER)
        {
            SX1231_writeRegister(SX1231_REG_OPMODE, (RegistersCfg[SX1231_REG_OPMODE] & 0xE3) | SX1231_RF_TRANSMITTER);
            while ((SX1231_readRegister(SX1231_REG_IRQFLAGS1) & SX1231_RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
            PreMode = SX1231_RF_TRANSMITTER;
        }

        else if(mode == SX1231_RF_RECEIVER)
        {
            SX1231_writeRegister(SX1231_REG_OPMODE, (RegistersCfg[SX1231_REG_OPMODE] & 0xE3) | SX1231_RF_RECEIVER);
            while ((SX1231_readRegister(SX1231_REG_IRQFLAGS1) & SX1231_RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
            PreMode = SX1231_RF_RECEIVER;
        }

        else if(mode == SX1231_RF_SYNTHESIZER)
        {
            SX1231_writeRegister(SX1231_REG_OPMODE, (RegistersCfg[SX1231_REG_OPMODE] & 0xE3) | SX1231_RF_SYNTHESIZER);
            while ((SX1231_readRegister(SX1231_REG_IRQFLAGS1) & SX1231_RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
            PreMode = SX1231_RF_SYNTHESIZER;
        }

        else if(mode == SX1231_RF_STANDBY)
        {
            SX1231_writeRegister(SX1231_REG_OPMODE, (RegistersCfg[SX1231_REG_OPMODE] & 0xE3) | SX1231_RF_STANDBY);
            while ((SX1231_readRegister(SX1231_REG_IRQFLAGS1) & SX1231_RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
            PreMode = SX1231_RF_STANDBY;
        }

        else
        {// mode == SX1231_RF_SLEEP
            SX1231_writeRegister(SX1231_REG_OPMODE, (RegistersCfg[SX1231_REG_OPMODE] & 0xE3) | SX1231_RF_SLEEP);
            while ((SX1231_readRegister(SX1231_REG_IRQFLAGS1) & SX1231_RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
            PreMode = SX1231_RF_SLEEP;
        }
    }
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
    uint8_t buffer[2];

    buffer[0] = 0x80 | address;
    buffer[1] = value;

    if (_config.select != NULL)
    {
        _config.select(true);
    }

    if (_config.transmit != NULL)
    {
        _config.transmit(buffer, 2, 10);
    }

    if (_config.select != NULL)
    {
        _config.select(false);
    }
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
    uint8_t buffer;

    if (_config.select != NULL)
    {
        _config.select(true);
    }

    if (_config.transmit != NULL)
    {
        _config.transmit(&address, 1, 10);
        if (_config.receive != NULL)
        {
            _config.receive(&buffer, 1, 10);
        }
    }

    if (_config.select != NULL)
    {
        _config.select(false);
    }

    return buffer;
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
void SX1231_SendRfFrame(uint8_t *buffer, uint8_t size, uint8_t *pReturnCode)
{
    if((size+1) > SX1231_RF_BUFFER_SIZE_MAX)
    {
        RFState |= SX1231_RF_STOP;
        *pReturnCode = ERROR;
        return;
    }

    RFState |= SX1231_RF_BUSY;
    RFState &= ~SX1231_RF_STOP;
    RFFrameSize = size;
    pRFFrame = buffer;

    SX1231_writeRegister(SX1231_REG_DIOMAPPING1, (RegistersCfg[SX1231_REG_DIOMAPPING1] & 0x3F) | SX1231_RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
    SX1231_writeRegister(SX1231_REG_FIFOTHRESH, (RegistersCfg[SX1231_REG_FIFOTHRESH] & 0x7F) | SX1231_RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY);

    SX1231_setRFMode(SX1231_RF_SLEEP);

    SX1231_sendByte(RFFrameSize);
    for(ByteCounter = 0, RFFramePos = 0; ByteCounter < RFFrameSize;)
    {
        SX1231_sendByte(pRFFrame[RFFramePos++]);
        ByteCounter++;
    }

    SX1231_setRFMode(SX1231_RF_TRANSMITTER); //   => Tx starts since FIFO is not empty

    //    do{
    //    }while(!(RegPAIn & DIO0)); // Wait for Packet sent

    SX1231_setRFMode(SX1231_RF_SLEEP);


    RFState |= SX1231_RF_STOP;
    RFState &= ~SX1231_RF_TX_DONE;
    *pReturnCode = OK;
} // void SendRfFrame(uint8_t *buffer, uint8_t size, uint8_t *pReturnCode)

/*******************************************************************
** ReceiveRfFrame : Receives a RF frame                           **
********************************************************************
** In  : -                                                        **
** Out : *buffer, size, *pReturnCode                              **
*******************************************************************/
void SX1231_ReceiveRfFrame(uint8_t *buffer, uint8_t *size, uint8_t *pReturnCode)
{

    uint8_t TempRFState;

    *pReturnCode = RX_RUNNING;

    TempRFState = RFState;

    if(TempRFState & SX1231_RF_STOP)
    {
        pRFFrame = buffer;
        RFFramePos = 0;
        RFFrameSize = 2;

        SX1231_writeRegister(SX1231_REG_DIOMAPPING1, (RegistersCfg[SX1231_REG_DIOMAPPING1] & 0x3F) | SX1231_RF_DIOMAPPING1_DIO0_01); // DIO0 is "PAYLOADREADY"
    	SX1231_writeRegister(SX1231_REG_SYNCCONFIG, (RegistersCfg[SX1231_REG_SYNCCONFIG] & 0xBF) | SX1231_RF_SYNC_FIFOFILL_AUTO);

        //        RegIrqEnLow |= 0x04; // Enables Port A pin 2 interrupt DIO0 (PAYLOADREADY)

        SX1231_setRFMode(SX1231_RF_RECEIVER);
        SX1231_enableTimeOut(true);
        RFState |= SX1231_RF_BUSY;
        RFState &= ~SX1231_RF_STOP;
        RFState &= ~SX1231_RF_TIMEOUT;
        return;
    }
    else if(TempRFState & SX1231_RF_RX_DONE)
    {

        SX1231_setRFMode(SX1231_RF_SLEEP);
        RFFrameSize = SX1231_receiveByte();
        for(ByteCounter = 0, RFFramePos = 0; ByteCounter < RFFrameSize; ){
            pRFFrame[RFFramePos++] = SX1231_receiveByte();
            ByteCounter++;
        }

        *size = RFFrameSize;
        *pReturnCode = OK;
        RFState |= SX1231_RF_STOP;
        SX1231_enableTimeOut(false);
        RFState &= ~SX1231_RF_RX_DONE;
        //        RegIrqEnLow &= ~0x04; // Disables Port A pin 2 interrupt DIO0 (PAYLOADREADY)
        //        SPINss(1);
        return;
    }
    else if(TempRFState & SX1231_RF_ERROR)
    {

        SX1231_setRFMode(SX1231_RF_SLEEP);

        RFState |= SX1231_RF_STOP;
        RFState &= ~SX1231_RF_ERROR;
        *pReturnCode = ERROR;
        //        RegIrqEnLow &= ~0x04; // Disables Port A pin 2 interrupt DIO0 (PAYLOADREADY)
        //        SPINss(1);
        return;
    }
    else if(TempRFState & SX1231_RF_TIMEOUT)
    {

        SX1231_setRFMode(SX1231_RF_SLEEP);

        RFState |= SX1231_RF_STOP;
        RFState &= ~SX1231_RF_TIMEOUT;
        SX1231_enableTimeOut(false);
        *pReturnCode = RX_TIMEOUT;
        //        RegIrqEnLow &= ~0x04; // Disables Port A pin 2 interrupt DIO0 (PAYLOADREADY)
        //        SPINss(1);
        return;
    }
} // void ReceiveRfFrame(uint8_t *buffer, uint8_t size, uint8_t *pReturnCode)

/*******************************************************************
** SX1231_sendByte : Sends a data to the transceiver through the SPI     **
**            interface                                           **
********************************************************************
** In  : b                                                        **
** Out : -                                                        **
*******************************************************************/
void SX1231_sendByte(uint8_t b)
{
	SX1231_writeRegister(SX1231_REG_FIFO, b); // SPI burst mode not used in this implementation
} // void SX1231_sendByte(uint8_t b)

/*******************************************************************
** SX1231_receiveByte : Receives a data from the transceiver through the **
**               SPI interface                                    **
********************************************************************
** In  : -                                                        **
** Out : b                                                        **
*******************************************************************/
uint8_t SX1231_receiveByte(void)
{
	return SX1231_readRegister(SX1231_REG_FIFO); //SPI burst mode not used in this implementation
}
/*******************************************************************
** Transceiver specific functions                                 **
*******************************************************************/

/*******************************************************************
** ReadRssi : Reads the Rssi value from the SX1231                **
********************************************************************
** In  : -                                                        **
** Out : value                                                    **
*******************************************************************/
uint16_t SX1231_ReadRssi(void)
{ // Must be called while in RX
	uint16_t value;
	SX1231_writeRegister(SX1231_REG_RSSICONFIG, RegistersCfg[SX1231_REG_RSSICONFIG] | SX1231_RF_RSSI_START); // Triggers RSSI measurement
	while ((SX1231_readRegister(SX1231_REG_RSSICONFIG) & SX1231_RF_RSSI_DONE) == 0x00);               // Waits for RSSI measurement to be completed
	value = SX1231_readRegister(SX1231_REG_RSSIVALUE);                                         // Reads the RSSI result
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
	SX1231_writeRegister(SX1231_REG_AFCFEI, RegistersCfg[SX1231_REG_AFCFEI] | SX1231_RF_AFCFEI_FEI_START);   // Triggers FEI measurement
	while ((SX1231_readRegister(SX1231_REG_AFCFEI) & SX1231_RF_AFCFEI_FEI_DONE) == 0x00);             // Waits for FEI measurement to be completed
	value = ((SX1231_readRegister(SX1231_REG_FEIMSB) << 8) | SX1231_readRegister(SX1231_REG_FEILSB));        // Reads the FEI result
	return value;
}

/*******************************************************************
** AutoFreqControl : Calibrates the receiver frequency to the     **
**               transmitter frequency                            **
********************************************************************
** In  : -                                                        **
** Out : *pReturnCode                                             **
*******************************************************************/
void SX1231_AutoFreqControl(uint8_t *pReturnCode)
{ // Must be called while in RX
	SX1231_writeRegister(SX1231_REG_AFCFEI, RegistersCfg[SX1231_REG_AFCFEI] | SX1231_RF_AFCFEI_AFC_START);   // Triggers AFC measurement
	while ((SX1231_readRegister(SX1231_REG_AFCFEI) & SX1231_RF_AFCFEI_AFC_DONE) == 0x00);             // Waits for AFC measurement to be completed
    *pReturnCode = OK;
}

/*******************************************************************
** Utility functions                                              **
*******************************************************************/

/*******************************************************************
** Wait : This routine uses the counter A&B to create a delay     **
**        using the RC ck source                                  **
********************************************************************
** In   : cntVal                                                  **
** Out  : -                                                       **
*******************************************************************/
void SX1231_wait(uint16_t cntVal)
{
    for(int i = 0 ; i < cntVal ; i++)
    {
    }
}

/*******************************************************************
** SX1231_enableTimeOut : Enables/Disables the software RF frame timeout **
********************************************************************
** In  : enable                                                   **
** Out : -                                                        **
*******************************************************************/
void SX1231_enableTimeOut(uint8_t enable)
{
} // void SX1231_enableTimeOut(uint8_t enable)

/*******************************************************************
** Handle_Irq_Pa1 : Handles the interruption from the Pin 1 of    **
**                  Port A                                        **
********************************************************************
** In  : -                                                        **
** Out : -                                                        **
*******************************************************************/
void SX1231_Handle_Irq_Pa1 (void)
{ // DIO1 (Not used in this implementation)

} //End Handle_Irq_Pa1

/*******************************************************************
** Handle_Irq_Pa2 : Handles the interruption from the Pin 2 of    **
**                  Port A                                        **
********************************************************************
** In  : -                                                        **
** Out : -                                                        **
*******************************************************************/
void SX1231_Handle_Irq_Pa2 (void)
{ // DIO0 = PAYLOADREADY in RX

    RFState |= SX1231_RF_RX_DONE;
    RFState &= ~SX1231_RF_BUSY;

} //End Handle_Irq_Pa2

/*******************************************************************
** Handle_Irq_CntA : Handles the interruption from the Counter A  **
********************************************************************
** In              : -                                            **
** Out             : -                                            **
*******************************************************************/
void SX1231_Handle_Irq_CntA (void)
{
    RFState |= SX1231_RF_TIMEOUT;
    RFState &= ~SX1231_RF_BUSY;
} //End Handle_Irq_CntA



/*******************************************************************
** SX1231_reset :                                                 **
********************************************************************
** In              : -                                            **
** Out             : -                                            **
*******************************************************************/
void SX1231_reset(void)
{
    if (_config.reset != NULL)
    {
        _config.reset();
    }
}
