#include "target.h"

RET_VALUE   ADC_start(uint32_t adcId, uint32_t channel)
{
    ADC_ChannelConfTypeDef  config;

    /* ### - 3 - Channel configuration ######################################## */
    config.Channel      = ADCx_CHANNEL;                /* Sampled channel number */
    config.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
    config.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;   /* Sampling time (number of clock cycles unit) */
    config.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
    config.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
    config.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
    if (HAL_ADC_ConfigChannel(&AdcHandle, &config) != HAL_OK)
    {
        Error_Handler();
    }

  /* ### - 4 - Start conversion in DMA mode ################################# */
  if (HAL_ADC_Start_DMA(&AdcHandle,
                        (uint32_t *)aADCxConvertedData,
                        ADC_CONVERTED_DATA_BUFFER_SIZE
                       ) != HAL_OK)
  {
    Error_Handler();
  }
  
