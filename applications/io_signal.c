/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#include "io_signal.h"
#include "main.h"
#include "small_modbus_port.h"
#if defined(SMODBUS_USING_INPUT_REGISTER) || defined(SMODBUS_USING_HOLD_REGISTER)
//#include "Mcp4822.h"
#endif

#ifdef DBG_TAG
#undef DBG_TAG
#define DBG_TAG "io_signal"
#endif
#define DBG_LVL DBG_LOG

#define Get_DI_GPIO_Port(__Port, __P1, __P2) \
    ((__Port) ? (__P1) : (__P2))

#define Get_DI_GPIO_Pin(__Pin, __Offset, __P1, __P2) \
    ((__Pin) < (__Offset) ? ((__P1) >> (__Pin)) : (__P2) >> ((__Pin) - (__Offset)))

#define Get_DO_Port(__Pin)                       \
    ((__Pin) < 1U ? GPIOA : (__Pin) < 4U ? GPIOC \
                                         : GPIOD)

#define Get_DO_Pin(__Pin)                                            \
    ((__Pin) < 1U ? Q_0_Pin : (__Pin) < 4U ? Q_1_Pin << ((__Pin)-1U) \
                                           : Q_4_Pin << ((__Pin)-4U))

#define Get_ADC_Channel(__Ch)                                 \
    ((__Ch) < 4U ? ((__Ch) + 8U) : (__Ch) < 12U ? ((__Ch)-4U) \
                                                : (__Ch))
#define AI_CP 0.005378F
#define AI_CQ 0.375224F
#define Get_Current_Value(__Value) \
    ((__Value) ? (AI_CP * (float)(__Value) + AI_CQ) : 0)

static Dac_HandleTypeDef dac_map[] = {
    {.Channel = DAC_OUT1, .Mcpxx_Id = Input_Other},
    {.Channel = DAC_OUT2, .Mcpxx_Id = Input_Other},
};
static float dac_param[][2] = {
    {201.8393825F, -6.724913779F},
    {201.3104013F, -7.379197379F},
};

/*dac校准互斥标志*/
static bool set_dac_flag = false;

static void Output_Current(Dac_HandleTypeDef *p_ch, float data);
/*通过DMA循环采集的ADC数据缓存区*/
unsigned int Adc_buffer[ADC_DMA_SIZE] = {0};

/**
 * @brief  Get ADC channel value of DMA transmission
 * @note   Channel 1 and Channel 2 of ADC1 are used
 * @param  Channel Channel ID
 * @retval ADC average value
 */
static unsigned int Get_AdcValue(const unsigned int Channel)
{
    unsigned int sum = 0;
    unsigned int *pAdc_Str = &Adc_buffer[0];

  /*Prevent array out of bounds*/
  if (Channel > ADC_DMA_CHANNEL)
  {
    return 0;
  }
  /*Add channel offset*/
     pAdc_Str += Channel;

    for( ; pAdc_Str < Adc_buffer + ADC_DMA_SIZE ; pAdc_Str += ADC_DMA_CHANNEL)
    {
      sum += *pAdc_Str;
    }

    return (sum >> ADC_FILTER_SHIFT);
}

/**
 * @brief   查看数字输入状态
 * @details
 * @param   none
 * @retval  none
 */
static void see_di(void)
{
    pModbusHandle pd = Modbus_Object;
    uint8_t rbits[EXTERN_DIGITALIN_MAX];
    memset(rbits, 0x00, sizeof(rbits));

    if (pd)
    {
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, InputCoil, Read, INPUT_DIGITAL_START_ADDR,
                              rbits, EXTERN_DIGITALIN_MAX))
        {
#if (SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:Input coil write failed.");
#endif
            return;
        }
        SMODBUS_DEBUG_R("\t\t\t[----digital_output_info----]\nsn\tpinx\n");
        SMODBUS_DEBUG_R("--\t----\n");
        for (uint16_t i = 0; i < EXTERN_DIGITALIN_MAX; i++)
            SMODBUS_DEBUG_R("%d\t%d\n", i, rbits[i]);
    }
}
MSH_CMD_EXPORT(see_di, display digital output value.);

/**
 * @brief   查看数字输出状态
 * @details
 * @param   none
 * @retval  none
 */
static void see_do(void)
{
    pModbusHandle pd = Modbus_Object;
    uint8_t rbits[EXTERN_DIGITALIN_MAX];
    memset(rbits, 0x00, sizeof(rbits));
    if (pd)
    {
        if (!pd->Mod_Operatex(pd, Coil, Read, INPUT_DIGITAL_START_ADDR,
                              rbits, EXTERN_DIGITALOUT_MAX))
        {
#if (SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:coil read failed.");
#endif
        }
        SMODBUS_DEBUG_R("\t\t\t[----digital_output_info----]\nsn\tpinx\n");
        SMODBUS_DEBUG_R("---\t----\n");
        for (uint16_t i = 0; i < EXTERN_DIGITALOUT_MAX; ++i)
            SMODBUS_DEBUG_R("%d\t%d\n", i, rbits[i]);
    }
}
MSH_CMD_EXPORT(see_do, display digital output value.);

/**
 * @brief   设置数字输出状态
 * @details
 * @param   none
 * @retval  none
 */
static void set_do(int argc, char **argv)
{
    pModbusHandle pd = Modbus_Object;

    if (argc < 2)
    {
        SMODBUS_DEBUG_R("@error: Please input'set_do <(0~9) | (0/1)>.'\n");
        return;
    }
    if (argc > 3)
    {
        SMODBUS_DEBUG_R("@error:parameter is too long,please input'set_do <(0~9) | (0/1)>.'\n");
        return;
    }

    uint8_t pin = *argv[1] - '0', state = *argv[2] - '0';

    if (pin > 9)
    {
        SMODBUS_DEBUG_D("@error:parameter[1]%d error,please input'0~9'.\n", pin);
        return;
    }
    if (state > 1U)
    {
        SMODBUS_DEBUG_D("@error:parameter[2]:%d error,please input'0/1'.\n", state);
        return;
    }

    if (pd)
    {
        uint8_t start_addr = INPUT_DIGITAL_START_ADDR + pin;
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, Coil, Write, start_addr, &state, 1U))
        {
#if (SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:coil write failed.");
#endif
        }
    }
    see_do();
}
MSH_CMD_EXPORT(set_do, set_do sample
               : set_do<(0 ~9) | (0 / 1)>);

/**
 * @brief   查看adc通道值
 * @details
 * @param   none
 * @retval  none
 */
static void see_adc(void)
{
    SMODBUS_DEBUG_R("\t\t\t\t[----adc_info----]\nsn\tchannel\tvalue\tcurrent(mA)\n");
    SMODBUS_DEBUG_R("---\t--------\t-----\t-----------\n");
    for (uint16_t i = 0; i < EXTERN_ANALOGIN_MAX; ++i)
    {
        uint16_t actual_ch = Get_ADC_Channel(i);
        /*获取DAC值*/
        uint32_t adc_value = Get_AdcValue(actual_ch);
        actual_ch = actual_ch > 3 ? actual_ch + 2U : actual_ch;
        SMODBUS_DEBUG_R("%d\t%d\t%d\t%.3f\n", i, actual_ch, adc_value, Get_Current_Value(adc_value));
    }
}
MSH_CMD_EXPORT(see_adc, display adc channel value.);

/**
 * @brief   设置dac值
 * @details
 * @param   none
 * @retval  none
 */
static void set_dac(int argc, char **argv)
{
    if (argc > 3)
    {
        SMODBUS_DEBUG_R("@error:too many parameters,please input'set_dac <(0~1)|(0~4095)>.'\n");
        return;
    }
    uint32_t channel = (uint16_t)atoi(argv[1]);
    uint16_t data = (uint16_t)atoi(argv[2]);
    if (channel > 1)
    {
        SMODBUS_DEBUG_R("@error:wrong channel number,please input'set_dac (0~1)|(0~4095).'\n");
        return;
    }
    if (data > 4095U)
    {
        SMODBUS_DEBUG_R("@error:invalid input,please input'set_dac (0~1)|(0~4095).'\n");
        return;
    }
    SMODBUS_DEBUG_R("channel[%d]= %d.\n", channel, data);
    set_dac_flag = data ? true : false;
    void *pHandle = &hdac;
    HAL_DAC_SetValue((DAC_HandleTypeDef *)pHandle, channel,
                     DAC_ALIGN_12B_R, (data & 0x0FFF));
}
MSH_CMD_EXPORT(set_dac, set_dac sample
               : set_dac<(ch : 0 ~1) | (val : 0 ~4095)>);

/**
 * @brief   设置AO输出4-20ma
 * @details
 * @param   none
 * @retval  none
 */
static void set_ao(int argc, char **argv)
{
    if (argc > 3)
    {
        SMODBUS_DEBUG_R("@error:too many parameters,please input'set_ao <(0~1)|(0~20)>.'\n");
        return;
    }
    uint32_t channel = (uint16_t)atoi(argv[1]);
    float data = (float)atof(argv[2]);
    if (channel > 1)
    {
        SMODBUS_DEBUG_R("@error:wrong channel number,please input'set_ao <(0~1)|(0~20)>.'\n");
        return;
    }
    if (data > 20.0F)
    {
        SMODBUS_DEBUG_R("@error:invalid input,please input'set_ao <(0~1)|(0~20)>.'\n");
        return;
    }
    SMODBUS_DEBUG_R("channel[%d]= %.3f.\n", channel, data);
    set_dac_flag = data ? true : false;
    for (uint16_t ch = 0; ch < EXTERN_ANALOGOUT_MAX; ch++)
        Output_Current(&dac_map[ch], data);
}
MSH_CMD_EXPORT(set_ao, set_ao sample
               : set_ao<(ch : 0 ~1) | (val : 0 ~20)>);

/**
 * @brief   查看dac值
 * @details
 * @param   none
 * @retval  none
 */
static void see_dac(void)
{
    SMODBUS_DEBUG_R("\t\t\t\t[----dac_info----]\nsn\tcp\t\tcq\n");
    SMODBUS_DEBUG_R("---\t------\t------\n");
    for (uint16_t ch = 0; ch < EXTERN_ANALOGOUT_MAX; ch++)
        SMODBUS_DEBUG_R("%d\t%.3f\t\t%.3f\n", ch, dac_param[ch][0], dac_param[ch][1]);
}
MSH_CMD_EXPORT(see_dac, display dac output value.);

/**
 * @brief   外部数字量输入处理
 * @details STM32F103VET6共在io口扩展了8路数字输入
 * @param   None
 * @retval  None
 */
#if defined(SMODBUS_USING_INPUT_COIL)
void Read_Digital_Io(void)
{ /*引脚默认指向输入第一路*/
    GPIO_TypeDef *pGPIOx = NULL;
    uint16_t GPIO_Pinx = 0x00;
    pModbusHandle pd = Modbus_Object;
    uint8_t wbits[EXTERN_DIGITALIN_MAX];
    memset(wbits, 0x00, sizeof(wbits));

    for (uint16_t i = 0; i < EXTERN_DIGITALIN_MAX; i++)
    {
        pGPIOx = Get_DI_GPIO_Port(!!(i < 3U), GPIOC, GPIOD);
        GPIO_Pinx = Get_DI_GPIO_Pin(i, 3U, DI_0_Pin, DI_3_Pin);
        /*读取外部数字引脚状态:翻转光耦的输入信号*/
        wbits[i] = (uint8_t)!HAL_GPIO_ReadPin(pGPIOx, GPIO_Pinx);
#if (SMODBUS_USING_DEBUG)
        // SMODBUS_DEBUG_D("@note:write[%d] = %d.", i, wbits[i]);
#endif
    }

    if (pd && pGPIOx)
    {
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, InputCoil, Write, INPUT_DIGITAL_START_ADDR,
                              wbits, EXTERN_DIGITALIN_MAX))
        {
#if (SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:Input coil write failed.");
#endif
        }
    }
}
#endif

/**
 * @brief   数字量输出
 * @details STM32F030F4共在io口扩展了8路数字输出
 * @param   None
 * @retval  None
 */
#if defined(SMODBUS_USING_COIL)
void Write_Digital_IO(void)
{
    /*引脚默认指向输入第一路*/
    GPIO_TypeDef *pGPIOx = NULL;
    uint16_t GPIO_Pinx = 0x00;
    pModbusHandle pd = Modbus_Object;
    uint8_t rbits[EXTERN_DIGITALOUT_MAX];

    if (pd)
    {
        /*读取对应寄存器*/
        if (!pd->Mod_Operatex(pd, Coil, Read, OUT_DIGITAL_START_ADDR,
                              rbits, EXTERN_DIGITALOUT_MAX))
        {
#if (SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:Coil reading failed!\r\n");
            return;
#endif
        }

        for (uint8_t i = 0; i < EXTERN_DIGITALOUT_MAX; i++)
        {
            pGPIOx = Get_DO_Port(i);
            GPIO_Pinx = Get_DO_Pin(i);
            if (pGPIOx)
                HAL_GPIO_WritePin(pGPIOx, GPIO_Pinx, (GPIO_PinState)rbits[i]);
        }
    }
}
#endif

/**
 * @brief   外部模拟量输入处理
 * @details STM32F030F4共在io口扩展了8路模拟输入
 * @param   None
 * @retval  None
 */
#if defined(SMODBUS_USING_INPUT_REGISTER)
void Read_Analog_Io(void)
{
    static bool first_flag = false;
    pModbusHandle pd = Modbus_Object;
    /*滤波结构需要不断迭代，否则滤波器无法正常工作*/
#if defined(TOOL_USING_KALMAN)
    KFP hkfp = {
        .Last_Covariance = LASTP,
        .Kg = 0,
        .Now_Covariance = 0,
        .Output = 0,
        .Q = COVAR_Q,
        .R = COVAR_R,
    };
    static KFP pkfp[EXTERN_ANALOGIN_MAX];
#else
    SideParm side = {
        .First_Flag = true,
        .Head = &side.SideBuff[0],
        .SideBuff = {0},
        .Sum = 0,
    };
    static SideParm pside[EXTERN_ANALOGIN_MAX];
#endif
    /*保证仅首次copy*/
    if (!first_flag)
    {
        first_flag = true;
        for (uint16_t ch = 0; ch < EXTERN_ANALOGIN_MAX; ch++)
        {
#if defined(TOOL_USING_KALMAN)
            memcpy(&pkfp[ch], &hkfp, sizeof(hkfp));
#else
            memcpy(&pside[ch], &side, sizeof(pside));
#endif
        }
    }
    float *pdata = (float *)smd_malloc(EXTERN_ANALOGIN_MAX * sizeof(float));
    if (!pdata)
        goto __exit;
    memset(pdata, 0x00, EXTERN_ANALOGIN_MAX * sizeof(float));

    for (uint16_t ch = 0; ch < EXTERN_ANALOGIN_MAX; ch++)
    {
        uint16_t actual_ch = Get_ADC_Channel(ch);
        /*获取DAC值*/
        uint32_t adc_value = Get_AdcValue(actual_ch);
        pdata[ch] = Get_Current_Value(adc_value);
#if (SMODBUS_USING_DEBUG)
        //        SMODBUS_DEBUG_D("ADC[%d] = %d.", ch, adc_value);
#endif
        // pdata[ch] = (pdata[ch] <= AI_CQ) ? 0 : pdata[ch];
        /*滤波处理*/
#if defined(TOOL_USING_KALMAN)
        pdata[ch] = kalmanFilter(&pkfp[ch], pdata[ch]);
#else
        pdata[ch] = sidefilter(&pside[ch], pdata[ch]);
#endif
        /*大小端转换*/
        //        endian_swap((uint8_t *)&pdata[ch], 0U, sizeof(float));

#if (SMODBUS_USING_DEBUG)
        // SMODBUS_DEBUG_D("R_AD[%d] = %.3f.", ch, pdata[ch]);
#endif
    }
    if (pd)
    {
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, InputRegister, Write, INPUT_ANALOG_START_ADDR,
                              (uint8_t *)pdata, EXTERN_ANALOGIN_MAX * sizeof(float)))
        {
#if defined(SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:Input register write failed!\r\n");
#endif
        }
    }

__exit:
    smd_free(pdata);
}
#endif

/**
 * @brief	对目标通道输出电流
 * @details
 * @param   p_ch :目标通道
 * @param	data:写入的数据
 * @retval	None
 */
static void Output_Current(Dac_HandleTypeDef *p_ch, float data)
{
    void *pHandle = &hdac;
    uint16_t value = (uint16_t)(dac_param[p_ch->Channel][0] * data +
                                dac_param[p_ch->Channel][1]);

    value = data ? ((data > 20.0F) ? 0x0FFF : value) : 0U;

    uint32_t channel = (p_ch->Channel < DAC_OUT2) ? DAC_CHANNEL_2 : DAC_CHANNEL_1;
    if (pHandle)
        HAL_DAC_SetValue((DAC_HandleTypeDef *)pHandle, channel,
                         DAC_ALIGN_12B_R, (value & 0x0FFF));
}

/**
 * @brief   模拟量输出
 * @details STM32F030F4共在io口扩展了8路数字输出
 * @param   None
 * @retval  None
 */
#if defined(SMODBUS_USING_HOLD_REGISTER)
void Write_Analog_IO(void)
{
    pModbusHandle pd = Modbus_Object;

    float *pdata = (float *)smd_malloc(EXTERN_ANALOGOUT_MAX * sizeof(float));
    if (!pdata)
        goto __exit;
    memset(pdata, 0x00, EXTERN_ANALOGOUT_MAX * sizeof(float));

    if (!pd)
    {
        goto __exit;
    }
    /*读出保持寄存器*/
    if (!pd->Mod_Operatex(pd, HoldRegister, Read, OUT_ANALOG_START_ADDR,
                          (uint8_t *)pdata, EXTERN_ANALOGOUT_MAX * sizeof(float)))
    {
#if (SMODBUS_USING_DEBUG)
        SMODBUS_DEBUG_D("@error:Hold register read failed!\r\n");
#endif
    }

    for (uint16_t ch = 0; ch < EXTERN_ANALOGOUT_MAX; ch++)
    {
#if defined(SMODBUS_USING_DEBUG)
        // shellPrint(Shell_Object, "W_AD[%d] = %.3f\r\n", ch, pdata[ch]);
#endif
        if (!set_dac_flag)
            Output_Current(&dac_map[ch], pdata[ch]);
    }

__exit:
    smd_free(pdata);
}
#endif
