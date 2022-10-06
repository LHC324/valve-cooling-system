#include "measure.h"
#include "main.h"
#include "small_modbus_port.h"
#include "io_signal.h"
// #include "list.h"
#include "flash.h"

#ifdef DBG_TAG
#undef DBG_TAG
#define DBG_TAG "measure"
#define DBG_LVL DBG_LOG
/*必须位于DBG_SECTION_NAME之后*/
#include <rtdbg.h>
#endif
#define Get_Sensor(__s) (((__s) >> 8U) - 1U)
#define Open_Qx(__x) (pm->coil[__x] = 1)
#define Close_Qx(__x) (pm->coil[__x] = 0)
#define Close_inverter(__pa) ((__pa)->comp_val = 4.0F)
#define Set_Measure_Complete(__pthis) ((__pthis)->front.state = proj_complete)
#define Set_Soft_Timer_Count(__pm, __tid, __count) \
    do                                             \
    {                                              \
        if (!(__pm)->timer[__tid].flag)            \
            return;                                \
        (__pm)->timer[__tid].flag = false;         \
        (__pm)->timer[__tid].count = __count;      \
    } while (0)
#define Reset_Action(__pm, __site, __len)               \
    do                                                  \
    {                                                   \
        if ((__pm)->coil)                               \
            memset(&(__pm)->coil[__site], 0x00, __len); \
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

static void pressure_flow_level_measure_system(pmeasure_handle pm, struct measure *pthis);
static void temperature_measure_system(pmeasure_handle pm, struct measure *pthis);
static void conductivity_measure_system(pmeasure_handle pm, struct measure *pthis);

static float measure_data[][4U] = {

    {0, 0, 0, 1.0F},
    {0, 0, 0, 100.0F},
    {0, 0, 0, 40.0F},
    {0, 0, -30.0F, 80.0F},
    {0, 0, 0.1F, 200.0F},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
};
/*系统函数组*/
static void (*mea_event[])(pmeasure_handle, struct measure *) = {
    pressure_flow_level_measure_system,
    temperature_measure_system,
    conductivity_measure_system,
};

static struct measure measure_backup[SYSYTEM_NUM] = {
    __init_measure(sensor_null, 30, 2.0F, pressure_flow_level_measure_system),
    __init_measure(sensor_null, 30, 2.0F, temperature_measure_system),
    __init_measure(sensor_null, 30, 2.0F, conductivity_measure_system),
};
/*测试系统组*/
measure_handle measure_object;
// measure_handle measure_object = {
//     .data =
//         {
//             {0, 0, 0, 1.0F},
//             {0, 0, 0, 100.0F},
//             {0, 0, 0, 40.0F},
//             {0, 0, -30.0F, 80.0F},
//             {0, 0, 0.1F, 200.0F},
//             {0, 0, 0, 0},
//             {0, 0, 0, 0},
//         },
//     .me =
//         {
//             __init_measure(sensor_null, 30U, 2.0F, pressure_flow_level_measure_system),
//             __init_measure(sensor_null, 30U, 2.0F, temperature_measure_system),
//             __init_measure(sensor_null, 30U, 2.0F, conductivity_measure_system),
//         }

// };
/*校准系统存储结构*/
// measure_storage measure_storage_object = {
//     .current_system = null_system,
//     .pm = &measure_object,
// };

/**
 * @brief	压力检测流程
 * @details
 * @param	pm 测量系统句柄
 * @retval  none
 */
// int rt_measure_init(void)
// {
//     measure_object.phandle = Modbus_Object;
// #if (SMODBUS_USING_DEBUG)
//     SMODBUS_DEBUG_D("@note:measure_group init.\n");
// #endif
//     return 0;
// }
// INIT_ENV_EXPORT(rt_measure_init);
int rt_measure_init(void)
{
    pmeasure_handle ps = &measure_object;
    FLASH_Read(PARAM_SAVE_ADDRESS, (uint8_t *)ps, sizeof(measure_handle));
    uint16_t crc16 = get_crc16((uint8_t *)ps, sizeof(measure_handle) - sizeof(ps->crc16), 0xFFFF);

    if (crc16 != ps->crc16)
    {
        LOG_D("@warning:Initialize system parameters for the first time,crc:%#x,ps->crc16:%#x!",
              crc16, ps->crc16);
        /*解决首次flash参数位初始化导致的操作0xffff指针*/
        memset(ps, 0x00, sizeof(measure_handle));
        memcpy(&ps->data, &measure_data, sizeof(measure_data));
        memcpy(&ps->me, &measure_backup, sizeof(measure_backup));
        // ps->phandle = Modbus_Object;
    }
    else
        LOG_D("@note:System parameters read successfully.");
    /*防止二次编译后，原函数在flash中地址发生改变*/
    ps->me[0].measure_event = pressure_flow_level_measure_system;
    for (uint8_t i = 0; i < sizeof(mea_event) / sizeof(mea_event[0]); ++i)
    {
        ps->me[i].measure_event = mea_event[i];
    }

    ps->phandle = Modbus_Object;
    return 0;
}
INIT_ENV_EXPORT(rt_measure_init);

/**
 * @brief	检查软件定时器标志
 * @details
 * @param	pm 校准系统句柄
 * @retval  none
 */
static void detect_soft_timer_flag(pmeasure_handle pm)
{
    if (!pm)
        return;
    for (measure_timer *p = pm->timer;
         p < pm->timer + sizeof(pm->timer) / sizeof(pm->timer[0]); ++p)
    {
        if (!(p->count))
            p->flag = true;
        else
            p->count--;
    }
}

/**
 * @brief	测量系统检测错误
 * @details
 * @param	pm 测量系统句柄
 * @retval  none
 */
static void measure_check_error(pmeasure_handle pm)
{
    pModbusHandle pd = (pModbusHandle)pm->phandle;
    /*故障检测*/
    uint8_t state_table[EXTERN_DIGITALIN_MAX];
    if (pd == NULL || pm == NULL)
        return;
    pm->flag &= 0xFF00;
    pd->Mod_Operatex(pd, InputCoil, Read, 0x00, state_table, sizeof(state_table));
    for (uint16_t i = 0; i < sizeof(state_table) / sizeof(state_table[0]); ++i)
        pm->flag |= (state_table[i] & 0x01) << i;
}

/**
 * @brief	测量系统阀状态切换
 * @details
 * @param	pm 测量系统句柄
 * @retval  none
 */
static void measure_coil_handle(pmeasure_handle pm)
{
    pModbusHandle pd = (pModbusHandle)pm->phandle;
    if (!pd || !pm)
        return;
    pd->Mod_Operatex(pd, Coil, Write, OUT_DIGITAL_START_ADDR,
                     pm->coil, sizeof(pm->coil));
}

/**
 * @brief   查看开关动作
 * @details
 * @param   none
 * @retval  none
 */
static void see_action(void)
{
    pmeasure_handle pm = &measure_object;
    SMODBUS_DEBUG_R("\t\t\t[----see_switch_info----]\nsn\tstate\n");
    SMODBUS_DEBUG_R("---\t----\n");
    for (uint16_t i = 0; i < EXTERN_DIGITALOUT_MAX; i++)
    {
        SMODBUS_DEBUG_R("%d\t%.d\n", i, pm->coil);
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
                          : std_data > EXCEED_CURRENT_LIMIT                       ? 3U + site * 6U
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
static void measure_sensor_data_handle(pmeasure_handle pm)
{
    pModbusHandle pd = (pModbusHandle)pm->phandle;
    padjust_handle pa = &pm->adjust[Get_Sensor(sensor_pressure)];
    float temp_data[EXTERN_ANALOGIN_MAX];

    if (pd == NULL || pm == NULL)
        return;
    /*清空系统错误代码*/
    pm->error_code = 0;
    memset(temp_data, 0x00, sizeof(temp_data));
    pd->Mod_Operatex(pd, InputRegister, Read, INPUT_ANALOG_START_ADDR,
                     (uint8_t *)&temp_data, sizeof(temp_data));
    /*按照上下限计算实际数据*/
    for (uint16_t i = 0; i < sizeof(pm->data) / sizeof(pm->data[0]); ++i)
    {
        float std_data = temp_data[i * 2];
        float test_data = temp_data[i * 2 + 1];
        float data_lower = pm->data[i][2];
        float data_upper = pm->data[i][3];
        pm->data[i][0] = (float)Get_Target(std_data, data_upper, data_lower);
        pm->data[i][1] = (float)Get_Target(test_data, data_upper, data_lower);
        /*软件识别传感器错误*/
        if (!pm->error_code)
        {
            pm->error_code = check_measure_sensor_error(i, std_data, test_data);
        }

        /*对存在调整对象的传感器采集当前值*/
        pa[i].cur_val = pm->data[i][0];
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
    pmeasure_handle pm = &measure_object;
    SMODBUS_DEBUG_R("\t\t[----see_sensor_data_info----]\nsn\tdata1\tdata2\tlower\tupper\n");
    SMODBUS_DEBUG_R("---\t----\t----\t----\t----\n");
    for (uint16_t i = 0; i < sizeof(pm->data) / sizeof(pm->data[0]); i++)
    {
        SMODBUS_DEBUG_R("%d\t%.3f\t%.3f\t%.3f\t%.3f\n", i,
                        pm->data[i][0], pm->data[i][1],
                        pm->data[i][2], pm->data[i][3]);
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
    float abs = fabs(diff);         /*绝对值*/
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
float get_ratio(padjust_handle pa)
{
    return (pa->tar_val * 0.8666F + 0.33334F);
}

/**
 * @brief	调节变频器输出目标值
 * @details
 * @param	pm 测量系统句柄
 * @retval  none
 */
static void adjust_inverter_out_handle(pmeasure_handle pm)
{
    // float ratio = 0;
    pModbusHandle pd = (pModbusHandle)pm->phandle;
    if ((!pd) || (!pm))
        return;
    padjust_handle pa = &pm->adjust[Get_Sensor(sensor_pressure)];
    struct measure *pthis = &pm->me[0];

    // if ((!pd) || (!pm) || (pthis->front.state != proj_onging))
    //     return;
    uint8_t current_site = Get_Sensor(pthis->front.cur_sensor);
    if (pthis->front.state == proj_onging)
        pa = &pm->adjust[current_site];

    /*防止变频器反向增大*/
    if (pa->comp_val < 4.0F)
        pa->comp_val = 4.0F;
    float ratio = get_ratio(pa);
    // ratio = __Get_Ratio(pa->tar_val);
    pa->comp_val += get_target_microcompensate(pa->cur_val, pa->tar_val, ratio);
    /*写入固定输出通道：默认写入通道0*/
    if (!pd->Mod_Operatex(pd, HoldRegister, Write, OUT_ANALOG_START_ADDR,
                          (uint8_t *)&pa->comp_val, sizeof(float)))
    {
#if (SMODBUS_USING_DEBUG)
        SMODBUS_DEBUG_D("@error:Hold register write failed!\r\n");
#endif
    }
}

/**
 * @brief	调节加热棒输出目标值
 * @details
 * @param	pm 测量系统句柄
 * @retval  none
 */
static void adjust_temperature_out_handle(pmeasure_handle pm)
{
    pModbusHandle pd = (pModbusHandle)pm->phandle;
    if ((!pm) || (!pd))
        return;
    padjust_handle pa = &pm->adjust[Get_Sensor(sensor_temperature)];
    //    measure *pthis = &pm->me[1];

    // if ((!pm) || (!pd) || (pthis->front.state != proj_onging))
    //     return;
    // float ratio = Get_Ratio(pa->cur_val);
    float ratio = get_ratio(pa);
    pa->comp_val += get_target_microcompensate(pa->cur_val, pa->tar_val, ratio);
    /*温度调节策略*/
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
#define Set_Measure(__operate, __next_sensor, __sensor_id, __end)    \
    do                                                               \
    {                                                                \
        if (pa->point)                                               \
            pa->tar_val += pa->offset_val;                           \
        __operate;                                                   \
        pm->his_data[pthis->cur_node][0] = pm->data[__sensor_id][0]; \
        pm->his_data[pthis->cur_node][1] = pm->data[__sensor_id][1]; \
        pthis->cur_node++;                                           \
        if (!(--pa->point))                                          \
        {                                                            \
            pthis->front.cur_sensor = __next_sensor;                 \
            __end;                                                   \
        }                                                            \
    } while (0)
// pthis->cur_node = 0;
// pa->tar_val = 0;
// pa->comp_val = 0;

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
            SMODBUS_DEBUG_D("@error: [%d]illegal use of null pointer(pa:%p,pd:%p,pm:%p).\n",   \
                            __sn, pa, pd, pm);                                                 \
            goto __exit;                                                                       \
        }                                                                                      \
        if ((pthis->front.cur_sensor != __sensor) && (pthis->front.cur_sensor != sensor_null)) \
        {                                                                                      \
            SMODBUS_DEBUG_D("@error: [%d]Calibration System SensorError,sensor type:[%#x].\n", \
                            __sn, pthis->front.cur_sensor);                                    \
            goto __exit;                                                                       \
        }                                                                                      \
        if ((__flag1) || (__flag2))                                                            \
        {                                                                                      \
            SMODBUS_DEBUG_D("@note: [%d]measure system error:%#x.\n", __sn, pm->flag);         \
            goto __exit;                                                                       \
        }                                                                                      \
    }

/**
 * @brief	获取传感器上限
 * @details
 * @param	pm 测量系统句柄
 * @param   site 传感器位置
 * @retval  none
 */
static float get_sensor_upper_limit(pmeasure_handle pm, uint8_t site)
{
    if (pm && site < sizeof(pm->data) / sizeof(pm->data[0]))
        return pm->data[site][3];
    return 0;
}

/**
 * @brief	获取传感器下限
 * @details
 * @param	pm 测量系统句柄
 * @param   site 传感器位置
 * @retval  none
 */
static float get_sensor_lower_limit(pmeasure_handle pm, uint8_t site)
{
    if (pm && site < sizeof(pm->data) / sizeof(pm->data[0]))
        return pm->data[site][2];
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
static float get_compensate_value(pmeasure_handle pm,
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
 * @param   pthis 校准对象指针
 * @param   sensor_num 系统传感器数量
 * @retval  none
 */
static uint16_t get_system_percentage(struct measure *pthis,
                                      uint8_t sensor_num)
{
    if (!sensor_num)
        sensor_num = 1U;
    return (uint16_t)((pthis->cur_node * 100U /
                       ((uint16_t)(pthis->back.offset + 1.0F) * sensor_num - 1U))
                      << 8U);
}

/**
 * @brief	设置动画状态
 * @details
 * @param   pm 校准系统指针
 * @param   op 动画操作类型
 * @param   cart_site 动画位置
 * @retval  none
 */
static void set_cartoon_state(pmeasure_handle pm,
                              uint8_t cart_site,
                              measure_operate_type op)
{
    if (pm && cart_site < sizeof(pm->cartoon) / sizeof(pm->cartoon[0]))
        pm->cartoon[cart_site] = op;
}

/**
 * @brief	操作目标开关
 * @details
 * @param	pm 测量系统句柄
 * @param   pthis 校准对象指针
 * @param   op 操作类型
 * @retval  true/false
 */
static void operate_target_switch(pmeasure_handle pm,
                                  uint8_t site, measure_operate_type op)
{
    if (pm && site < sizeof(pm->coil) / sizeof(pm->coil[0]))
        pm->coil[site] = (op >> 8U) & 0x01;
}

/**
 * @brief	检查软件定时器标志
 * @details
 * @param	pm 测量系统句柄
 * @param   pthis 校准对象指针
 * @param   timer_id 软件定时器id
 * @retval  true/false
 */
static bool check_soft_timer_flag(pmeasure_handle pm,
                                  struct measure *pthis, uint8_t timer_id)
{
    if (pm && (timer_id < sizeof(pm->timer) / sizeof(pm->timer[0])))
    {
        if (pm->timer[timer_id].flag)
        {
            pm->timer[timer_id].flag = false;
            pm->timer[timer_id].count = (uint32_t)(pthis->back.silence_time * 60.0F); /*单位按min*/
            return true;
        }
    }
    return false;
}

/**
 * @brief	通用传感器校准处理
 * @details
 * @param	pm 测量系统句柄
 * @param   pthis 当前校准系统指针
 * @param   pa 校准调整对象指针
 * @param   sw 开关对象
 * @param   next_sensor 下一个目标传感器
 * @param   state 系统状态
 * @retval  None
 */
void set_measure_information(pmeasure_handle pm, padjust_handle pa,
                             struct measure *pthis, measure_switch_group sw,
                             uint8_t sensor_id, measure_sensor next_sensor,
                             bool state)
{
    if (pm == NULL || pthis == NULL)
        return;
    // if (pa->point)
    //     pa->tar_val += pa->offset_val;
    operate_target_switch(pm, sw, mea_set);
    uint8_t this_offset = pthis - pm->me;
    uint8_t base_addr = this_offset ? 12U + this_offset * 6U : 0U;
    if ((base_addr + pthis->cur_node) < sizeof(pm->his_data) / sizeof(pm->his_data[0]))
    {
        memcpy(&pm->his_data[base_addr + pthis->cur_node][0], &pm->data[sensor_id][0], sizeof(pm->data[0]) / 2U);
    }
    pthis->cur_node++;
    // if (!(--pa->point))
    if (pa->point)
    {
        pa->point--;
        if (!pa->point)
        {
            pthis->front.cur_sensor = next_sensor;
            if (state)
                pthis->front.state = proj_complete;
        }
        else
            pa->tar_val += pa->offset_val;
    }
}

/**
 * @brief	压力、流量、液位测量系统
 * @details
 * @param	pm 测量系统句柄
 * @param   pthis 当前系统指针
 * @retval  none
 */
static void pressure_flow_level_measure_system(pmeasure_handle pm, struct measure *pthis)
{
    // pModbusHandle pd = (pModbusHandle)pm->phandle;
    uint8_t sensor_site = Get_Sensor(sensor_pressure);
    padjust_handle pa = &pm->adjust[sensor_site];
    // struct measure *pthis = &pm->me[0];
    uint8_t current_site = 0;
/*检测系统是否存在错误*/
#if (SMODBUS_USING_DEBUG)
    // Check_Measure_Param(0, pthis->front.cur_sensor,
    //                     __GET_FLAG(pm->flag, proj_err_stop),
    //                     __GET_FLAG(pm->flag, proj_err_conv));
#endif

    current_site = Get_Sensor(pthis->front.cur_sensor);
    if (pthis->front.cur_sensor != sensor_null)
        pa = &pm->adjust[current_site];
    if (pthis->front.state != proj_onging)
    {
        for (adjust_handle *p = &pm->adjust[0]; p < pm->adjust + SYSYTEM_NUM; ++p)
        {
            p->point = 0;
            p->offset_val = 0;
            p->comp_val = 4.0F;
            p->tar_val = 0;
        }
        /*清空软件定时器*/
        pm->timer[tim_id_pfl].count = 0;
        /*关闭变频器动画*/
        set_cartoon_state(pm, cartoon_inverter, mea_reset);
        /*关闭流量动画*/
        set_cartoon_state(pm, cartoon_flow, mea_reset);
        /*关闭液位动画*/
        set_cartoon_state(pm, cartoon_level, mea_reset);

        goto __exit;
    }

    /*设置系统调整结构:压力、流量、液位分别调整*/
    // current_site = Get_Sensor(pthis->front.cur_sensor);
    pa->offset_val = get_compensate_value(pm, pthis, current_site);
    /*设置阶段定时器*/
    if (check_soft_timer_flag(pm, pthis, (uint8_t)tim_id_pfl) == false)
        return;
#if (SMODBUS_USING_DEBUG)
    SMODBUS_DEBUG_D("\t\t[----pfl_system----]\nsite\tcur_val\t\ttar_val\t\tcomp_val\tpoint\tnode\tcount\tper");
    SMODBUS_DEBUG_R("----\t--------\t--------\t--------\t-----\t----\t----\t----\n");
    SMODBUS_DEBUG_R("%d\t%.3f\t\t%.3f\t\t%.3f\t\t%u\t%u\t%d\t%d\n\n", current_site, pa->cur_val, pa->tar_val, pa->comp_val,
                    pa->point, pthis->cur_node, pm->timer[tim_id_pfl].count, pthis->front.percentage >> 8U);
#endif
    /*校准系统百分比*/
    pthis->front.percentage = get_system_percentage(pthis, 3);
    /*清空对应阀门*/
    Reset_Action(pm, sw_pressure, 2U);
    /*关闭变频器*/
    // Close_inverter(pa);

    switch (pthis->front.cur_sensor)
    {
    case sensor_pressure:
    {
        // Set_Measure(, sensor_flow, Get_Sensor(sensor_pressure), );
        set_measure_information(pm, pa, pthis, sw_max,
                                Get_Sensor(sensor_pressure), sensor_flow,
                                false);
        /*打开动画*/
        set_cartoon_state(pm, cartoon_inverter, mea_set);
    }
    break;
    case sensor_flow:
    {
        // Set_Measure(Open_Qx(sw_pressure), sensor_level, Get_Sensor(sensor_flow), );
        set_measure_information(pm, pa, pthis, sw_pressure,
                                Get_Sensor(sensor_flow), sensor_level,
                                false);
        /*打开动画*/
        set_cartoon_state(pm, cartoon_flow, mea_set);
        set_cartoon_state(pm, cartoon_inverter, mea_reset);
    }
    break;
    case sensor_level:
    {
        /*标志本轮传感器校准完毕*/
        // Set_Measure(Open_Qx(sw_level), sensor_null, Get_Sensor(sensor_level),
        //             Set_Measure_Complete(pthis));
        set_measure_information(pm, pa, pthis, sw_level,
                                Get_Sensor(sensor_level), sensor_null,
                                true);
        /*打开动画*/
        set_cartoon_state(pm, cartoon_level, mea_set);
        set_cartoon_state(pm, cartoon_flow, mea_reset);
    }
    break;
    default:
        break;
    }

    return;
__exit:
    /*清空对应阀门*/
    Reset_Action(pm, sw_pressure, 2U);
    /*关闭变频器*/
    Close_inverter(pa);
    /*关闭动画*/
}

/**
 * @brief	温度测量系统
 * @details
 * @param	pm 测量系统句柄
 * @param   pthis 当前系统指针
 * @retval  none
 */
static void temperature_measure_system(pmeasure_handle pm, struct measure *pthis)
{
    // pModbusHandle pd = (pModbusHandle)pm->phandle;
    uint8_t sensor_site = Get_Sensor(sensor_temperature);
    padjust_handle pa = &pm->adjust[sensor_site];
    // struct measure *pthis = &pm->me[1];

#if (SMODBUS_USING_DEBUG)
    // Check_Measure_Param(1, sensor_temperature,
    //                     __GET_FLAG(pm->flag, proj_err_stop), 0);
#endif

    if (pthis->front.state != proj_onging)
    {
        adjust_handle *p = &pm->adjust[sensor_site];
        p->point = 0;
        p->offset_val = 0;
        p->comp_val = 4.0F;
        p->tar_val = 0;

        /*清空软件定时器*/
        pm->timer[tim_id_tempe].count = 0;
        /*关闭风扇动画*/
        set_cartoon_state(pm, cartoon_fan, mea_reset);
        goto __exit;
    }

    /*设置系统调整结构*/
    pa->offset_val = get_compensate_value(pm, pthis, sensor_site);
    /*设置阶段定时器*/
    if (check_soft_timer_flag(pm, pthis, (uint8_t)tim_id_tempe) == false)
        return;
#if (SMODBUS_USING_DEBUG)
    SMODBUS_DEBUG_D("\t\t[----temp_system----]\nsite\ttar_val\t\tcomp_val\tpoint\tnode\tcount\tper");
    SMODBUS_DEBUG_R("----\t--------\t--------\t--------\t-----\t----\t----\t----\n");
    SMODBUS_DEBUG_R("%d\t%.3f\t\t%.3f\t\t%.3f\t\t%u\t%u\t%d\t%d\n\n", sensor_site, pa->cur_val, pa->tar_val, pa->comp_val,
                    pa->point, pthis->cur_node, pm->timer[tim_id_tempe].count, pthis->front.percentage >> 8U);
#endif
    /*校准系统百分比*/
    pthis->front.percentage = get_system_percentage(pthis, 1);

    /*关闭风扇*/
    // Reset_Action(pm, 2, 1U);
    Open_Qx(sw_fan);

    // Set_Measure(, sensor_null, Get_Sensor(sensor_temperature),
    //             Set_Measure_Complete(pthis));
    set_measure_information(pm, pa, pthis, sw_max,
                            Get_Sensor(sensor_temperature), sensor_null,
                            true);
    /*打开风扇动画*/
    set_cartoon_state(pm, cartoon_fan, mea_set);

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
static void conductivity_measure_system(pmeasure_handle pm, struct measure *pthis)
{
    // pModbusHandle pd = (pModbusHandle)pm->phandle;
    uint8_t sensor_site = Get_Sensor(sensor_conductivity);
    padjust_handle pa = &pm->adjust[sensor_site];
    // struct measure *pthis = &pm->me[2];

/*检测系统是否存在错误*/
#if (SMODBUS_USING_DEBUG)
    // Check_Measure_Param(2, sensor_conductivity,
    //                     __GET_FLAG(pm->flag, proj_err_stop), 0);
#endif

    if (pthis->front.state != proj_onging)
    {
        adjust_handle *p = &pm->adjust[sensor_site];
        p->point = 0;
        p->offset_val = 0;
        p->comp_val = 4.0F;
        p->tar_val = 0;

        /*清空软件定时器*/
        pm->timer[tim_id_conv].count = 0;
        /*关闭电解质投入示意动画*/
        set_cartoon_state(pm, cartoon_con, mea_reset);
        goto __exit;
    }

    /*设置系统调整结构*/
    pa->offset_val = get_compensate_value(pm, pthis, sensor_site);
    /*先打开废液排空阀固定时间、再打开加水阀一段时间*/
    /*设置阶段定时器*/
    if (check_soft_timer_flag(pm, pthis, (uint8_t)tim_id_conv) == false)
        return;
#if (SMODBUS_USING_DEBUG)
    SMODBUS_DEBUG_D("\t\t[----con_system----]\nsite\ttar_val\t\tcomp_val\tpoint\tnode\tcount\tper");
    SMODBUS_DEBUG_R("----\t--------\t--------\t-----\t----\t----\t----\n");
    SMODBUS_DEBUG_R("%d\t%.3f\t\t%.3f\t\t%u\t%u\t%d\t%d\n\t", sensor_site, pa->tar_val, pa->comp_val,
                    pa->point, pthis->cur_node, pm->timer[tim_id_conv].count, pthis->front.percentage >> 8U);
#endif

    /*校准系统百分比*/
    pthis->front.percentage = get_system_percentage(pthis, 1);

    /*关闭加水、废液排空电磁阀*/
    Reset_Action(pm, sw_add_level, 2U);

    // Set_Measure(, sensor_null, Get_Sensor(sensor_conductivity),
    //             Set_Measure_Complete(pthis));
    set_measure_information(pm, pa, pthis, sw_max,
                            Get_Sensor(sensor_conductivity), sensor_null,
                            true);
    /*打开电解质投入示意动画*/
    set_cartoon_state(pm, cartoon_con, mea_set);

    return;
__exit:
    /*关闭加水、废液排空电磁阀*/
    Reset_Action(pm, sw_add_level, 2U);
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
    detect_soft_timer_flag(&measure_object);
}

/**
 * @brief	轮询校准系统
 * @details
 * @param	none
 * @retval  none
 */
void measure_poll(void)
{
    measure_check_error(&measure_object);
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

// /**
//  * @brief	校准系统事件处理
//  * @details
//  * @param	none
//  * @retval  none
//  */
// void measure_event_handle(void)
// {
//     measure_check_error(&measure_object);
//     measure_coil_handle(&measure_object);

//     adjust_inverter_out_handle(&measure_object);
//     adjust_temperature_out_handle(&measure_object);
// }
