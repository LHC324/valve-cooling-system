#include "measure.h"
#include "main.h"
#include "small_modbus_port.h"
#include "io_signal.h"
// #include "list.h"
#include "flash.h"

#ifdef DBG_TAG
#undef DBG_TAG
#endif

#define MEASURE_USING_DEBUG 1
#define DBG_TAG "measure"
#define DBG_LVL DBG_LOG
/*必须位于DBG_SECTION_NAME之后*/
#include <rtdbg.h>
#define MEASURE_DEBUG_R dbg_raw
#define MEASURE_DEBUG_D LOG_D

#define Get_Sensor(__s) (((__s) >> 8U) - 1U)
#define Open_Qx(__x) (pm->presour->coil[__x] = 1)
#define Close_Qx(__x) (pm->presour->coil[__x] = 0)
#define Close_inverter(__pa) ((__pa)->comp_val = 4.0F)
#define Set_Measure_Complete(__pthis) ((__pthis)->front.state = proj_complete)
#define Set_Soft_Timer_Count(__pm, __tid, __count)     \
    do                                                 \
    {                                                  \
        if (!(__pm)->presour->timer[__tid].flag)       \
            return;                                    \
        (__pm)->presour->timer[__tid].flag = false;    \
        (__pm)->presour->timer[__tid].count = __count; \
    } while (0)

#define Reset_Action(__pm, __site, __len)                        \
    do                                                           \
    {                                                            \
        if ((__pm)->presour->coil)                               \
            memset(&(__pm)->presour->coil[__site], 0x00, __len); \
    } while (0)

/*获取可变参数宏：https://blog.csdn.net/u012028275/article/details/118853297*/

#define __init_measure(__sensor, __time, __error, __event) \
    {                                                      \
        .front.cur_sensor = __sensor,                      \
        .back =                                            \
            {                                              \
                .silence_time = __time,                    \
                .permit_error = __error,                   \
                .offset = 5.0F,                            \
            },                                             \
        .measure_event = __event,                          \
    }

/*pwm互斥标志*/
static bool set_pwm_flag = false;
static void pressure_flow_level_measure_system(pmeasure_t pm, struct measure *pthis);
static void temperature_measure_system(pmeasure_t pm, struct measure *pthis);
static void conductivity_measure_system(pmeasure_t pm, struct measure *pthis);
static void pid_init(void);

const float measure_limits[] = {
    1.6F, 0, 1.6F, 0,
    90.0F, 0, 90.0F, 0,
    17.0F, 0, 17.0F, 0,
    80.0F, -30.0F, 80.0F, -30.0F,
    3.0F, 0, 3.0F, 0,
    100.0F, 0, 100.0F, 0,
    0, 0, 0, 0};

/*系统函数组*/
static void (*mea_event[])(pmeasure_t, struct measure *) = {
    pressure_flow_level_measure_system,
    temperature_measure_system,
    conductivity_measure_system,
};

static struct measure measure_backup[SYSYTEM_NUM] = {
    __init_measure(sensor_null, 30, 2.0F, NULL),
    __init_measure(sensor_null, 30, 2.0F, NULL),
    __init_measure(sensor_null, 30, 2.0F, NULL),
};
/*定义资源池*/
measure_resources_pools_t resources_pools;
/*测试系统组*/
measure_t measure_object = {
    .presour = &resources_pools,
};

/**
 * @brief	压力检测流程
 * @details
 * @param	pm 测量系统句柄
 * @retval  none
 */
int rt_measure_init(void)
{
    pmeasure_t ps = &measure_object;

    FLASH_Read(PARAM_SAVE_ADDRESS, (uint8_t *)ps, sizeof(measure_t));
    uint16_t crc16 = get_crc16((uint8_t *)ps, sizeof(measure_t) - sizeof(ps->crc16), 0xFFFF);

    if (crc16 != ps->crc16)
    {
        LOG_D("@warning:Initialize system parameters for the first time,crc:%#x,ps->crc16:%#x!",
              crc16, ps->crc16);
        /*解决首次flash参数位初始化导致的操作0xffff指针*/
        memset(ps, 0x00, sizeof(measure_t));
        // memcpy(&ps->data, &measure_data, sizeof(measure_data));
        memcpy(ps->limits, measure_limits, sizeof(ps->limits)); /*初始化上下限*/
        memcpy(&ps->me, &measure_backup, sizeof(measure_backup));
        /*初始化所有待测传感器实验标准为：%5*/
        for (uint8_t i = 0; i < sizeof(ps->expe_std) / sizeof(ps->expe_std[0]); ++i)
            ps->expe_std[i] = 5.0F;
    }
    else
        LOG_D("@note:System parameters read successfully.");
    /*防止二次编译后，原函数在flash中地址发生改变*/
    for (uint8_t i = 0; i < sizeof(mea_event) / sizeof(mea_event[0]); ++i)
    {
        ps->me[i].measure_event = mea_event[i];
    }
    ps->presour = &resources_pools;
    ps->phandle = Modbus_Object;
    /*位置式pid初始化*/
    pid_init();
    return 0;
}
INIT_ENV_EXPORT(rt_measure_init);

/**
 * @brief	软件定时器轮询
 * @details
 * @param	pm 校准系统句柄
 * @retval  none
 */
static void soft_timer_poll(pmeasure_t pm)
{
    if (pm == NULL || pm->presour == NULL)
        return;
    for (measure_timer *p = pm->presour->timer;
         p < pm->presour->timer + SOFT_TIMER_NUM; ++p)
    {
        if (!(p->count))
            p->flag = true;
        else
            p->count--;
    }
}

/**
 * @brief	检查软件定时器标志
 * @details
 * @param	pm 测量系统句柄
 * @param   pthis 校准对象指针
 * @param   timer_id 软件定时器id
 * @retval  true/false
 */
static bool check_soft_timer_flag(pmeasure_t pm,
                                  struct measure *pthis, uint8_t timer_id)
{
    if (pm && (timer_id < SOFT_TIMER_NUM))
    {
        if (pm->presour->timer[timer_id].flag)
        {
            pm->presour->timer[timer_id].flag = false;
            pm->presour->timer[timer_id].count = (uint32_t)(pthis->back.silence_time * 60.0F); /*单位按min*/
            return true;
        }
    }
    return false;
}

/**
 * @brief	复位软件定时器计数
 * @details
 * @param   timer 目标定时器指针
 * @param   timer_id 软件定时器id
 * @retval  true/false
 */
static bool reset_soft_timer_count(measure_timer *timer,
                                   uint8_t timer_id)
{
    if (timer == NULL || timer_id >= SOFT_TIMER_NUM)
        return false;
    timer[timer_id].count = 0;
    return true;
}

/**
 * @brief	测量系统检测错误
 * @details
 * @param	pm 测量系统句柄
 * @param   me 当前校验系统
 * @retval  none
 */
static bool measure_check_error(pmeasure_t pm, uint8_t system_id)
{
    pModbusHandle pd = (pModbusHandle)pm->phandle;
    /*故障检测*/
    uint8_t state_table[EXTERN_DIGITALIN_MAX];
    static uint8_t error_limit_table[] = {18U, 24U, 30U};
    bool result = false;

    if (pd == NULL || pm == NULL || system_id < SYSYTEM_NUM)
        return true;

    uint8_t cur_error_code = error_limit_table[system_id];

    // pm->flag &= 0xFF00;
    pd->Mod_Operatex(pd, InputCoil, Read, 0x00, state_table, sizeof(state_table));
    // for (uint16_t i = 0; i < sizeof(state_table) / sizeof(state_table[0]); ++i)
    //     pm->flag |= (state_table[i] & 0x01) << i;

    /*所有系统无错误*/
    if (!pm->error_code)
    {
        if (!system_id && state_table[0])
            result = true;
        // goto __exit;
    }
    else if (pm->error_code <= cur_error_code)
        result = true;

    // __exit:
    return result;
}

/**
 * @brief	测量系统阀状态切换
 * @details
 * @param	pm 测量系统句柄
 * @retval  none
 */
static void measure_coil_handle(pmeasure_t pm)
{
    pModbusHandle pd = (pModbusHandle)pm->phandle;
    if (pd == NULL || pm == NULL || pm->presour == NULL)
        return;
    pd->Mod_Operatex(pd, Coil, Write, OUT_DIGITAL_START_ADDR,
                     pm->presour->coil, sizeof(pm->presour->coil));
}

/**
 * @brief   查看开关动作
 * @details
 * @param   none
 * @retval  none
 */
static void see_action(void)
{
    pmeasure_t pm = &measure_object;
    if (pm->presour == NULL)
        return;

    MEASURE_DEBUG_R("\t\t\t[----see_switch_info----]\nsn\tstate\n");
    MEASURE_DEBUG_R("---\t----\n");
    for (uint16_t i = 0; i < EXTERN_DIGITALOUT_MAX; i++)
    {
        MEASURE_DEBUG_R("%d\t%.d\n", i, pm->presour->coil);
    }
}
MSH_CMD_EXPORT(see_action, display switch_state.);

/**
 * @brief	检查测量系统传感器错误
 * @details
 * @param   site 传感器组位置
 * @param	std_data 标准传感器电流数据
 * @param   teset_data 待测传感器电流数据
 * @retval  none
 */
static uint16_t check_measure_sensor_error(uint8_t site, float std_data, float test_data)
{
#define BREAK_WIRE_CURRENT_LIMIT 0.5F
#define EXCEED_CURRENT_LIMIT 20.5F
    /*先检测标准传感器无错误后再检测待测传感器*/
    uint16_t error_code = std_data < BREAK_WIRE_CURRENT_LIMIT
                              ? 1U + site * 6U
                          : std_data < (CURRENT_LOWER - BREAK_WIRE_CURRENT_LIMIT) ? 2U + site * 6U
                          : std_data >
                                  EXCEED_CURRENT_LIMIT
                              ? 3U + site * 6U
                              : 0U;
    if (error_code)
        return error_code;
    error_code = std_data < BREAK_WIRE_CURRENT_LIMIT
                     ? 4U + site * 6U
                 : std_data < (CURRENT_LOWER - BREAK_WIRE_CURRENT_LIMIT) ? 5U + site * 6U
                 : std_data > EXCEED_CURRENT_LIMIT                       ? 6U + site * 6U
                                                                         : 0U;
    return error_code;
#undef BREAK_WIRE_CURRENT_LIMIT
#undef EXCEED_CURRENT_LIMIT
}

/**
 * @brief	测量系统传感器数据处理
 * @details
 * @param	pm 测量系统句柄
 * @retval  none
 */
static void measure_sensor_data_handle(pmeasure_t pm)
{
    pModbusHandle pd = (pModbusHandle)pm->phandle;
    float temp_data[EXTERN_ANALOGIN_MAX];

    if (pd == NULL || pm == NULL || pm->presour == NULL)
        return;
    padjust_t pa = &pm->presour->adjust[Get_Sensor(sensor_pressure)];
    /*清空系统错误代码*/
    pm->error_code = 0;
    memset(temp_data, 0x00, sizeof(temp_data));
    pd->Mod_Operatex(pd, InputRegister, Read, INPUT_ANALOG_START_ADDR,
                     (uint8_t *)&temp_data, sizeof(temp_data));
    /*按照上下限计算实际数据*/
    for (uint16_t i = 0; i < MEASURE_MAX_SENSOR_NUM; ++i)
    {
        float std_data = temp_data[i * 2];
        float test_data = temp_data[i * 2 + 1];
        float std_upper = pm->limits[i * 4U], std_lower = pm->limits[i * 4U + 1U];
        float test_upper = pm->limits[i * 4U + 2U], test_lower = pm->limits[i * 4U + 3U];

        pm->presour->data[i][0] = (float)Get_Target(std_data, std_upper, std_lower);
        pm->presour->data[i][1] = (float)Get_Target(test_data, test_upper, test_lower);
        pm->presour->data[i][2] = (float)Get_Error(pm->presour->data[i][0], pm->presour->data[i][1]); /*不直接采用流量信号计算误差*/
        /*软件识别传感器错误*/
        if (!pm->error_code)
        {
            pm->error_code = check_measure_sensor_error(i, std_data, test_data);
        }

        /*对存在调整对象的传感器采集当前值*/
        pa[i].cur_val = pm->presour->data[i][0];
    }
}

/**
 * @brief   查看外部传感器值
 * @details
 * @param   none
 * @retval  none
 */
static void see_sensor_data(void)
{
    pmeasure_t pm = &measure_object;
    if (pm->presour == NULL)
        return;

    MEASURE_DEBUG_R("\t\t[----see_sensor_data_info----]\nsn\tstd\tupper\tlower\ttest\tupper\tlower\terror\n");
    MEASURE_DEBUG_R("---\t----\t----\t----\t----\t----\t----\t----\n");
    for (uint16_t i = 0; i < MEASURE_MAX_SENSOR_NUM; i++)
    {
        MEASURE_DEBUG_R("%d\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", i,
                        pm->presour->data[i][0], pm->limits[i * 4U], pm->limits[i * 4U + 1U],
                        pm->presour->data[i][1], pm->limits[i * 4U + 2U], pm->limits[i * 4U + 3U],
                        pm->presour->data[i][2]);
    }
}
MSH_CMD_EXPORT(see_sensor_data, display sensor data.);

/**
 * Get x sign bit only for little-endian
 * if x >= 0 then  1
 * if x <  0 then -1
 */
#define MathUtils_SignBit(x) \
    (((signed char *)&x)[sizeof(x) - 1] >> 7 | 1)

/**
 * @brief	获取目标补偿值
 * @details
 * @param	cur_val 当采样值
 * @param   tar_value 目标值
 * @param   ratio 倍率(must > 1)
 * @param   micr_val 补偿倍数
 * @retval	系统补偿值
 */
// static float get_target_microcompensate(const float cur_val, const float tar_val,
//                                         float ratio, float micr_val)
static float get_target_microcompensate(const float cur_val, const float tar_val,
                                        float ratio)
{
    float diff = tar_val - cur_val; /*差*/
    float abs = fabsf(diff);        /*绝对值*/
    float cur_gap = 0;              /*补偿值*/
    // float coef = tar_val / ratio;   /*目标值的系数*/
    float coef = (tar_val / ratio) * (abs / ratio); /*目标值的系数*/

    /*目标值不可能是负数*/
    if (MathUtils_SignBit(tar_val) == -1)
    {
        return 0;
    }
    if (abs && (abs > coef))
    {
        /*获取符号位*/
        // cur_gap = MathUtils_SignBit(diff) * micr_val; /*此处差距较大时可以加大系数*/
        cur_gap = MathUtils_SignBit(diff) * coef;
    }
    else /*允许误差或者已经调节在点上*/
        cur_gap = 0;

    return cur_gap;
}

#define __Get_Ratio(__val) ((float)(__val)*0.8666F + 0.33334F)

/**
 * @brief	获取调节倍率
 * @details
 * @param	pa 调整句柄
 * @retval  none
 */
float get_ratio(padjust_t pa)
{
    return (pa->tar_val * 0.8666F + 0.33334F);
}

site_pid_t pfl_pid, temp_pid;

/**
 * @brief	位置式pid初始化
 * @details
 * @param	None
 * @retval  None
 */
static void pid_init(void)
{
#define SEAMPING_TIMES 5.0e1F  // 采样时间：ms
#define INTERGRAL_TIMES 5.0e6F // 积分时间
#define DIFF_TIMES 1.0e3F      // 微分时间

#define PFL_KP 2.0e-2F
#define PFL_KI PFL_KP *(SEAMPING_TIMES / INTERGRAL_TIMES)
#define PFL_KD 2.0e-1F

#define TEMP_KP 3.0e1F
#define TEMP_KI TEMP_KP *(SEAMPING_TIMES / INTERGRAL_TIMES)
#define TEMP_KD TEMP_KP *(DIFF_TIMES / SEAMPING_TIMES)
    /*压力、流量、液位*/
    init_site_pid(&pfl_pid, PFL_KP, PFL_KI, PFL_KD);
    /*温度系统*/
    init_site_pid(&temp_pid, TEMP_KP, TEMP_KI, TEMP_KD);
    // #undef SEAMPING_TIMES
    // #undef INTERGRAL_TIMES
    // #undef DIFF_TIMES
}

/**
 * @brief	调节变频器输出目标值
 * @details
 * @param	pm 测量系统句柄
 * @retval  none
 */
static void adjust_inverter_out_handle(pmeasure_t pm)
{
    // float ratio = 0;
    pModbusHandle pd = (pModbusHandle)pm->phandle;
    padjust_t pa = &pm->presour->adjust[Get_Sensor(sensor_pressure)];
    struct measure *pthis = &pm->me[0];
    static uint8_t last_site = 0;

    if (pd == NULL || pm == NULL || pm->presour == NULL ||
        pd == NULL || pthis == NULL)
        return;

    uint8_t current_site = Get_Sensor(pthis->front.cur_sensor);
    if (pthis->front.state == proj_onging)
        pa = &pm->presour->adjust[current_site];

    /*防止变频器反向增大*/
    if (pa->comp_val < 4.0F)
        pa->comp_val = 4.0F;
    // float ratio = get_ratio(pa);
    // // ratio = __Get_Ratio(pa->tar_val);
    // pa->comp_val += get_target_microcompensate(pa->cur_val, pa->tar_val, ratio);
    /*传感器发生变化时，复位目标pid*/
    if ((!pa->tar_val) || (last_site != current_site))
    {
        init_site_pid(&pfl_pid, PFL_KP, PFL_KI, PFL_KD);
        pa->comp_val = 4.0F;
        last_site = current_site;
    }
    else
    {
        pa->comp_val += get_pid_out(&pfl_pid, pa->cur_val, pa->tar_val);
    }

    /*写入固定输出通道：默认写入通道0*/
    if (!pd->Mod_Operatex(pd, HoldRegister, Write, OUT_ANALOG_START_ADDR,
                          (uint8_t *)&pa->comp_val, sizeof(float)))
    {
#if (SMODBUS_USING_DEBUG)
        MEASURE_DEBUG_D("@error:Hold register write failed!\r\n");
#endif
    }
}

/**
 * @brief   通过占空比获得目标装载值
 * @details
 * @param   duty
 * @retval  目标装载值
 */
static uint32_t get_pwm_value_base_on_duty(TIM_HandleTypeDef *ptimer, float duty)
{
    if ((ptimer == NULL) || (duty > 1.0F))
        return 0;
    /*百分百时必须是满载值*/
    // uint32_t arr_val = duty == 1.0F
    //                        ? __HAL_TIM_GET_AUTORELOAD(ptimer) + 1U
    //                        : __HAL_TIM_GET_AUTORELOAD(ptimer);
    return (uint32_t)((float)(__HAL_TIM_GET_AUTORELOAD(ptimer) + 1U) * duty);
}

/**
 * @brief   设置AO输出4-20ma
 * @details
 * @param   none
 * @retval  none
 */
static void set_pwm(int argc, char **argv)
{
    if (argc > 2)
    {
        MEASURE_DEBUG_R("@error:too many parameters,please input'set_pwm<(0 - 100)>.'\n");
        return;
    }
    uint16_t duty = (uint16_t)atoi(argv[1]);
    if (duty > 100)
    {
        MEASURE_DEBUG_R("@error:wrong pwm duty number,please input'set_pwm<(0 - 100)>.'\n");
        return;
    }
    MEASURE_DEBUG_R("pwm duty= %d.\n", duty);
    set_pwm_flag = duty ? true : false;
    duty = (uint16_t)get_pwm_value_base_on_duty(&htim4, (float)duty / 100.0F);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, duty);
}
MSH_CMD_EXPORT(set_pwm, set_pwm sample
               : set_pwm<(0 - 100)>);

/**
 * @brief	调节加热棒输出目标值
 * @details
 * @param	pm 测量系统句柄
 * @retval  none
 */
static void adjust_temperature_out_handle(pmeasure_t pm)
{
#define PWM_DUTY_MAX 100.0F
#define MIN_COMPAL_VAL 35.0F
    pModbusHandle pd = (pModbusHandle)pm->phandle;
    if (pm == NULL || pd == NULL || pm->presour == NULL)
        return;
    padjust_t pa = &pm->presour->adjust[Get_Sensor(sensor_temperature)];

    if (!pa->tar_val)
    {
        // pid_init();
        init_site_pid(&temp_pid, TEMP_KP, TEMP_KI, TEMP_KD);
        pa->comp_val = 0;
    }
    else
    {
        pa->comp_val = get_pid_out(&temp_pid, pa->cur_val, pa->tar_val);
        pa->comp_val = pa->comp_val < MIN_COMPAL_VAL
                           ? MIN_COMPAL_VAL
                       : pa->comp_val > PWM_DUTY_MAX ? PWM_DUTY_MAX
                                                     : pa->comp_val;
    }
    /*温度调节策略*/
    uint16_t duty = (uint16_t)get_pwm_value_base_on_duty(&htim4, (float)pa->comp_val / 100.0F);
    if (!set_pwm_flag)
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, duty);

#undef PWM_DUTY_MAX
#undef MIN_COMPAL_VAL
}

/**
 * @brief	通用传感器数据采集
 * @details
 * @param	__operate 操作函数
 * @param   __next_sensor 下一个校准传感器
 * @param   __sensor_id 传感器id
 * @param   __end 结束操作
 * @retval  none
 */
#define Set_Measure(__operate, __next_sensor, __sensor_id, __end)                      \
    do                                                                                 \
    {                                                                                  \
        if (pa->point)                                                                 \
            pa->tar_val += pa->offset_val;                                             \
        __operate;                                                                     \
        pm->presour->his_data[pthis->cur_node][0] = pm->presour->data[__sensor_id][0]; \
        pm->presour->his_data[pthis->cur_node][1] = pm->presour->data[__sensor_id][1]; \
        pthis->cur_node++;                                                             \
        if (!(--pa->point))                                                            \
        {                                                                              \
            pthis->front.cur_sensor = __next_sensor;                                   \
            __end;                                                                     \
        }                                                                              \
    } while (0)

/**
 * @brief	采样系统入口参数检测
 * @details
 * @param	__operate 操作函数
 * @param   __next_sensor 下一个校准传感器
 * @param   __sensor_id 传感器id
 * @param   __end 结束操作
 * @retval  none
 */
#define Check_Measure_Param(__sn, __sensor, __flag1, __flag2)                                  \
    {                                                                                          \
        if (!pa || !pd || !pm)                                                                 \
        {                                                                                      \
            MEASURE_DEBUG_D("@error: [%d]illegal use of null pointer(pa:%p,pd:%p,pm:%p).\n",   \
                            __sn, pa, pd, pm);                                                 \
            goto __exit;                                                                       \
        }                                                                                      \
        if ((pthis->front.cur_sensor != __sensor) && (pthis->front.cur_sensor != sensor_null)) \
        {                                                                                      \
            MEASURE_DEBUG_D("@error: [%d]Calibration System SensorError,sensor type:[%#x].\n", \
                            __sn, pthis->front.cur_sensor);                                    \
            goto __exit;                                                                       \
        }                                                                                      \
        if ((__flag1) || (__flag2))                                                            \
        {                                                                                      \
            MEASURE_DEBUG_D("@note: [%d]measure system error:%#x.\n", __sn, pm->flag);         \
            goto __exit;                                                                       \
        }                                                                                      \
    }

/**
 * @brief	获取传感器上限(仅调节标准传感器)
 * @details
 * @param	pm 测量系统句柄
 * @param   site 传感器位置
 * @retval  none
 */
static float get_sensor_upper_limit(pmeasure_t pm, uint8_t site)
{
    if (pm && site < MEASURE_MAX_SENSOR_NUM)
        return pm->limits[site * 4U];
    return 0;
}

/**
 * @brief	获取传感器下限(仅调节标准传感器)
 * @details
 * @param	pm 测量系统句柄
 * @param   site 传感器位置
 * @retval  none
 */
static float get_sensor_lower_limit(pmeasure_t pm, uint8_t site)
{
    if (pm && site < MEASURE_MAX_SENSOR_NUM)
        return pm->limits[site * 4U + 1U];
    return 0;
}

/**
 * @brief	获取调整对象补偿值
 * @details
 * @param	pm 测量系统句柄
 * @param   pthis 校准对象指针
 * @param   site 传感器位置
 * @retval  none
 */
static float get_compensate_value(pmeasure_t pm,
                                  typeof(&measure_object.me[0]) pthis, uint8_t site)
{
    float sensor_upper = get_sensor_upper_limit(pm, site);
    float sensor_lower = get_sensor_lower_limit(pm, site);
    if (pthis->back.offset)
        return (sensor_upper - sensor_lower) / pthis->back.offset;
    return 0;
}

/**
 * @brief	获取当前系统百分比
 * @details
 * @param   cur_val  当前系统值
 * @param   toal_val 系统总值
 * @retval  none
 */
static uint16_t get_system_percentage(uint8_t cur_val,
                                      uint8_t toal_val)
{
#define PERCENTAGE_MAX 100U
    uint16_t percentage = !toal_val
                              ? 0
                              : (cur_val * PERCENTAGE_MAX) / toal_val;

    if (percentage > PERCENTAGE_MAX)
        percentage = PERCENTAGE_MAX;

#if (MEASURE_USING_DEBUG)
    MEASURE_DEBUG_R("@note:percentage:%#x.\n", percentage);
#endif
    percentage = (percentage & 0x00FF) << 8U;

    return percentage;
#undef PERCENTAGE_MAX
}

/**
 * @brief	设置动画状态
 * @details
 * @param   pm 校准系统指针
 * @param   op 动画操作类型
 * @param   cart_site 动画位置
 * @retval  none
 */
static void set_cartoon_state(pmeasure_t pm,
                              uint8_t cart_site,
                              measure_operate_type op)
{
    if (pm && cart_site < sizeof(pm->presour->cartoon) / sizeof(pm->presour->cartoon[0]))
        pm->presour->cartoon[cart_site] = op;
}

/**
 * @brief	操作目标开关
 * @details
 * @param	pm 测量系统句柄
 * @param   pthis 校准对象指针
 * @param   op 操作类型
 * @retval  true/false
 */
static void operate_target_switch(pmeasure_t pm,
                                  uint8_t site, measure_operate_type op)
{
    if (pm && site < sizeof(pm->presour->coil) / sizeof(pm->presour->coil[0]))
        pm->presour->coil[site] = (op >> 8U) & 0x01;
}

/**
 * @brief	设置检验目标传感器检验结果
 * @details
 * @param	pm 测量系统句柄
 * @param   sensor_id 当前传感器id
 * @param   sensor_flag 传感器检验结果标准
 * @retval  true/false
 */
static void set_test_sensor_result(pmeasure_t pm,
                                   uint8_t sensor_id,
                                   measure_flag_group sensor_flag)
{
    if ((sensor_id >= MEASURE_MAX_SENSOR_NUM) ||
        (sensor_flag >= measure_flag_max))
        return;

    if (pm->presour->data[sensor_id][2U] > pm->expe_std[sensor_id])
        __RESET_FLAG(pm->flag, (uint8_t)sensor_flag);
    else
        __SET_FLAG(pm->flag, (uint8_t)sensor_flag);
}

/**
 * @brief	通用传感器校准处理
 * @details
 * @param	pm 测量系统句柄
 * @param   pthis 当前校准系统指针
 * @param   pa 校准调整对象指针
 * @param   sw 开关对象
 * @param   next_sensor 下一个目标传感器
 * @param   sensor_flag 传感器检验结果标准
 * @param   state 系统状态
 * @retval  None
 */
void set_measure_information(pmeasure_t pm,
                             padjust_t pa,
                             struct measure *pthis,
                             measure_switch_group sw,
                             uint8_t sensor_id,
                             measure_sensor next_sensor,
                             measure_flag_group sensor_flag,
                             bool state,
                             measure_timer_id timer_id)
{
    if (pm == NULL || pm->presour == NULL || pthis == NULL)
        return;
    // if (pa->point)
    //     pa->tar_val += pa->offset_val;
    operate_target_switch(pm, sw, mea_set);
    uint8_t this_offset = pthis - pm->me;
    uint8_t base_addr = this_offset ? 12U + this_offset * 6U : 0U;
    if ((base_addr + pthis->cur_node) < sizeof(pm->presour->his_data) / sizeof(pm->presour->his_data[0]))
    {
        memcpy(&pm->presour->his_data[base_addr + pthis->cur_node][0], &pm->presour->data[sensor_id][0],
               sizeof(pm->presour->data[0]));
    }
    pthis->cur_node++;

    if (pa->point <= 0)
        return;
    // if (pa->point > 0)
    // {// }
    // pa->point--;
    if (!--pa->point)
    {
        pthis->front.cur_sensor = next_sensor;
        reset_soft_timer_count(pm->presour->timer, timer_id); // 解决多个传感器间产生暂停周期的问题
        if (state)
        {
            pthis->front.state = proj_complete;
            /*得到系统最后一个传感器检验结果*/
            set_test_sensor_result(pm, sensor_id, sensor_flag);
        }
    }
    else /*只有压力、液位、流量系统支持偏移值递增*/
        if (sensor_id < Get_Sensor(sensor_level))
            pa->tar_val += pa->offset_val;
}

/**
 * @brief	压力、流量、液位测量系统
 * @details
 * @param	pm 测量系统句柄
 * @param   pthis 当前系统指针
 * @retval  none
 */
static void pressure_flow_level_measure_system(pmeasure_t pm, struct measure *pthis)
{
    // pModbusHandle pd = (pModbusHandle)pm->phandle;
    uint8_t sensor_site = Get_Sensor(sensor_pressure);
    padjust_t pa = &pm->presour->adjust[sensor_site];
    // struct measure *pthis = &pm->me[0];
    uint8_t current_site = 0;
/*检测系统是否存在错误*/
#if (SMODBUS_USING_DEBUG)
    // Check_Measure_Param(0, pthis->front.cur_sensor,
    //                     __GET_FLAG(pm->flag, proj_err_stop),
    //                     __GET_FLAG(pm->flag, proj_err_conv));
#endif

    /*系统是否存在错误*/
    // if (measure_check_error(pm, 0))
    //     return;

    current_site = Get_Sensor(pthis->front.cur_sensor);
    if (pthis->front.cur_sensor != sensor_null)
        pa = &pm->presour->adjust[current_site];
    if (pthis->front.state != proj_onging)
    {
        for (adjust_t *p = &pm->presour->adjust[0];
             p < pm->presour->adjust + 3U; ++p)
        {
            p->point = 0;
            p->offset_val = 0;
            p->comp_val = 4.0F;
            p->tar_val = 0;
        }
        /*清空软件定时器*/
        pm->presour->timer[tim_id_pfl].count = 0;
        /*关闭变频器动画*/
        set_cartoon_state(pm, cartoon_inverter, mea_reset);
        /*关闭流量动画*/
        set_cartoon_state(pm, cartoon_flow, mea_reset);
        /*关闭液位动画*/
        set_cartoon_state(pm, cartoon_level, mea_reset);

        goto __exit;
    }
    /*打开变频器输出信号*/
    Open_Qx(sw_fwd);
    /*设置系统调整结构:压力、流量、液位分别调整*/
    pa->offset_val = get_compensate_value(pm, pthis, current_site);
    /*设置阶段定时器*/
    if (check_soft_timer_flag(pm, pthis, (uint8_t)tim_id_pfl) == false)
        return;
#if (MEASURE_USING_DEBUG)
    MEASURE_DEBUG_D("\t\t[----pfl_system----]\nsite\tcur_val\t\ttar_val\t\tcomp_val\tpoint\tnode\tcount\tper");
    MEASURE_DEBUG_R("----\t--------\t--------\t--------\t-----\t----\t----\t----\n");
    MEASURE_DEBUG_R("%d\t%.2f\t\t%.2f\t\t%.2f\t\t%u\t%u\t%d\t%d\n\n", current_site, pa->cur_val, pa->tar_val, pa->comp_val,
                    pa->point, pthis->cur_node, pm->presour->timer[tim_id_pfl].count, pthis->front.percentage >> 8U);
#endif
    /*校准系统百分比*/
    pthis->front.percentage = get_system_percentage(pthis->cur_node, (pthis->back.offset + 1.0F) * 3U - 1U);

    /*清空对应阀门*/
    Reset_Action(pm, sw_pressure, 2U);
    /*关闭变频器*/
    // Close_inverter(pa);

    switch (pthis->front.cur_sensor)
    {
    case sensor_pressure:
    {
        // Set_Measure(, sensor_flow, Get_Sensor(sensor_pressure), );
        set_measure_information(pm,
                                pa,
                                pthis,
                                sw_max,
                                Get_Sensor(sensor_pressure),
                                sensor_flow,
                                measure_flag_max,
                                false,
                                tim_id_pfl);
        /*打开动画*/
        set_cartoon_state(pm, cartoon_inverter, mea_set);
    }
    break;
    case sensor_flow:
    {
        /*检测流量时，获得压力结果*/
        set_test_sensor_result(pm, Get_Sensor(sensor_pressure), pre_measure_flag);
        // Set_Measure(Open_Qx(sw_pressure), sensor_level, Get_Sensor(sensor_flow), );
        set_measure_information(pm,
                                pa,
                                pthis,
                                sw_pressure,
                                Get_Sensor(sensor_flow),
                                sensor_level,
                                measure_flag_max,
                                false,
                                tim_id_pfl);
        /*打开动画*/
        set_cartoon_state(pm, cartoon_flow, mea_set);
        set_cartoon_state(pm, cartoon_inverter, mea_reset);
    }
    break;
    case sensor_level:
    {
        /*检测液位时，获得流量结果*/
        set_test_sensor_result(pm, Get_Sensor(sensor_flow), flo_measure_flag);
        /*标志本轮传感器校准完毕*/
        // Set_Measure(Open_Qx(sw_level), sensor_null, Get_Sensor(sensor_level),
        //             Set_Measure_Complete(pthis));
        set_measure_information(pm,
                                pa,
                                pthis,
                                sw_level,
                                Get_Sensor(sensor_level),
                                sensor_null,
                                lel_measure_flag,
                                true,
                                tim_id_pfl);
        /*打开动画*/
        set_cartoon_state(pm, cartoon_level, mea_set);
        set_cartoon_state(pm, cartoon_flow, mea_reset);
        /*每次进来刷新液位电磁阀关闭延时:30s*/
        pm->presour->timer[tim_id_close_level].count = 30000U;
    }
    break;
    default:
        break;
    }

    return;
__exit:
    /*清空对应阀门*/
    // Reset_Action(pm, sw_pressure, 2U);
    /*延时关闭液位电磁阀*/
    if (pm->presour->timer[tim_id_close_level].flag)
    {
        pm->presour->timer[tim_id_close_level].flag = false;
        Close_Qx(sw_level);
    }
    /*关闭压力电磁阀*/
    Close_Qx(sw_pressure);
    /*关闭变频器*/
    Close_inverter(pa);
    /*关闭动画*/
    /*关闭变频器启动信号*/
    Close_Qx(sw_fwd);
}

/**
 * @brief	温度测量系统
 * @details
 * @param	pm 测量系统句柄
 * @param   pthis 当前系统指针
 * @retval  none
 */
static void temperature_measure_system(pmeasure_t pm, struct measure *pthis)
{
    // pModbusHandle pd = (pModbusHandle)pm->phandle;
    uint8_t sensor_site = Get_Sensor(sensor_temperature);
    padjust_t pa = &pm->presour->adjust[sensor_site];
    // struct measure *pthis = &pm->me[1];

#if (SMODBUS_USING_DEBUG)
    // Check_Measure_Param(1, sensor_temperature,
    //                     __GET_FLAG(pm->flag, proj_err_stop), 0);
#endif

    // /*检查是否需要打开风扇动画*/
    // if (pm->presour->coil[sw_fan])
    // {
    //     /*打开风扇动画*/
    //     set_cartoon_state(pm, cartoon_fan, mea_set);
    // }
    // else
    //     /*关闭风扇动画*/
    //     set_cartoon_state(pm, cartoon_fan, mea_reset);

    pm->presour->coil[sw_fan]
        ? set_cartoon_state(pm, cartoon_fan, mea_set)    /*打开风扇动画*/
        : set_cartoon_state(pm, cartoon_fan, mea_reset); /*关闭风扇动画*/

    /*系统是否存在错误*/
    // if (measure_check_error(pm, 1))
    //     return;

    if (pthis->front.state != proj_onging)
    {
        adjust_t *p = &pm->presour->adjust[sensor_site];
        p->point = 0;
        p->offset_val = 0;
        p->comp_val = 0;
        p->tar_val = 0;

        /*清空软件定时器*/
        pm->presour->timer[tim_id_tempe].count = 0;
        /*关闭风扇动画*/
        // set_cartoon_state(pm, cartoon_fan, mea_reset);
        goto __exit;
        // return;
    }

    /*设置系统调整结构*/
    // pa->offset_val = get_compensate_value(pm, pthis, sensor_site);
    pa->tar_val = pthis->back.permit_error; // 目标温度值
    /*设置阶段定时器*/
    if (check_soft_timer_flag(pm, pthis, (uint8_t)tim_id_tempe) == false)
        return;
#if (MEASURE_USING_DEBUG)
    MEASURE_DEBUG_D("\t\t[----temp_system----]\nsite\tcur_val\t\ttar_val\t\tcomp_val\tpoint\tnode\tcount\tper");
    MEASURE_DEBUG_R("----\t--------\t--------\t--------\t-----\t----\t----\t----\n");
    MEASURE_DEBUG_R("%d\t%.2f\t\t%.2f\t\t%.2f\t\t%u\t%u\t%d\t%d\n\n", sensor_site, pa->cur_val, pa->tar_val, pa->comp_val,
                    pa->point, pthis->cur_node, pm->presour->timer[tim_id_tempe].count, pthis->front.percentage >> 8U);
#endif
    /*校准系统百分比*/
    pthis->front.percentage = get_system_percentage(pthis->cur_node, pthis->back.offset);

    /*关闭风扇*/
    // Reset_Action(pm, 2, 1U);
    Open_Qx(sw_fan);

    // Set_Measure(, sensor_null, Get_Sensor(sensor_temperature),
    //             Set_Measure_Complete(pthis));
    set_measure_information(pm,
                            pa,
                            pthis,
                            sw_max,
                            Get_Sensor(sensor_temperature),
                            sensor_null,
                            temp_measure_flag,
                            true,
                            tim_id_tempe);
    /*打开风扇动画*/
    // set_cartoon_state(pm, cartoon_fan, mea_set);

    return;
__exit:
    /*关闭风扇*/
    Reset_Action(pm, sw_fan, 1U);
    /*关闭风扇动画*/
}

/**
 * @brief	电导率测量系统
 * @details
 * @param	pm 测量系统句柄
 * @param   pthis 当前系统指针
 * @retval  none
 */
static void conductivity_measure_system(pmeasure_t pm, struct measure *pthis)
{
    // pModbusHandle pd = (pModbusHandle)pm->phandle;
    uint8_t sensor_site = Get_Sensor(sensor_conductivity);
    padjust_t pa = &pm->presour->adjust[sensor_site];
    // struct measure *pthis = &pm->me[2];

/*检测系统是否存在错误*/
#if (SMODBUS_USING_DEBUG)
    // Check_Measure_Param(2, sensor_conductivity,
    //                     __GET_FLAG(pm->flag, proj_err_stop), 0);
#endif

    /*系统是否存在错误*/
    // if (measure_check_error(pm, 2))
    //     return;

    if (pthis->front.state != proj_onging)
    {
        adjust_t *p = &pm->presour->adjust[sensor_site];
        p->point = 0;
        p->offset_val = 0;
        p->comp_val = 4.0F;
        p->tar_val = 0;

        /*清空软件定时器*/
        pm->presour->timer[tim_id_conv].count = 0;
        /*关闭电解质投入示意动画*/
        set_cartoon_state(pm, cartoon_con, mea_reset);
        //        goto __exit;
        return;
    }

    /*设置系统调整结构*/
    // pa->offset_val = get_compensate_value(pm, pthis, sensor_site);
    /*先打开废液排空阀固定时间、再打开加水阀一段时间*/
    /*设置阶段定时器*/
    if (check_soft_timer_flag(pm, pthis, (uint8_t)tim_id_conv) == false)
        return;
#if (MEASURE_USING_DEBUG)
    MEASURE_DEBUG_D("\t\t[----con_system----]\nsite\tcur_val\t\ttar_val\t\tcomp_val\tpoint\tnode\tcount\tper");
    MEASURE_DEBUG_R("----\t--------\t--------\t--------\t-----\t----\t----\t----\n");
    MEASURE_DEBUG_R("%d\t%.2f\t\t%.2f\t\t%.2f\t\t%u\t%u\t%d\t%d\n\t", sensor_site, pa->cur_val, pa->tar_val, pa->comp_val,
                    pa->point, pthis->cur_node, pm->presour->timer[tim_id_conv].count, pthis->front.percentage >> 8U);
#endif

    /*校准系统百分比*/
    pthis->front.percentage = get_system_percentage(pthis->cur_node, pthis->back.offset);

    /*关闭加水、废液排空电磁阀*/
    // Reset_Action(pm, sw_add_level, 2U);

    // Set_Measure(, sensor_null, Get_Sensor(sensor_conductivity),
    //             Set_Measure_Complete(pthis));
    set_measure_information(pm,
                            pa,
                            pthis,
                            sw_max,
                            Get_Sensor(sensor_conductivity),
                            sensor_null,
                            con_measure_flag,
                            true,
                            tim_id_conv);
    /*打开电解质投入示意动画*/
    set_cartoon_state(pm, cartoon_con, mea_set);

    // return;
    //__exit:
    /*关闭加水、废液排空电磁阀*/
    // Reset_Action(pm, sw_add_level, 2U);
    /*关闭动画*/
}

/**
 * @brief	轮询校准定时器轮询系统
 * @details
 * @param	none
 * @retval  none
 */
void measure_timer_poll(void)
{
    soft_timer_poll(&measure_object);
}

/**
 * @brief	轮询校准系统
 * @details
 * @param	none
 * @retval  none
 */
void measure_poll(void)
{
    // measure_check_error(&measure_object);
    measure_sensor_data_handle(&measure_object);

    for (typeof(&measure_object.me[0]) pthis = measure_object.me;
         pthis < measure_object.me + SYSYTEM_NUM; ++pthis)
    {
        if (pthis->measure_event)
            pthis->measure_event(&measure_object, pthis);
    }
    measure_coil_handle(&measure_object);
    // adjust_inverter_out_handle(&measure_object);
    // adjust_temperature_out_handle(&measure_object);
}

/**
 * @brief	校准系统采样事件
 * @details
 * @param	none
 * @retval  none
 */
void measure_sampling(void)
{
    // measure_sensor_data_handle(&measure_object);
    adjust_inverter_out_handle(&measure_object);
    adjust_temperature_out_handle(&measure_object);
}
