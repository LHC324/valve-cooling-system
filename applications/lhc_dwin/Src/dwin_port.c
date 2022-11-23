/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 *  @verbatim
 *      使用： 1、用户需要完善"rt_dwin_init/MX_DwinInit"、"Dwin_Send"函数
 *             2、用户需要初始化"Dwin_ObjMap[]"迪文事件组
 *             3、"rt_dwin_init/MX_DwinInit"初始化时需要明确指定"UartHandle"参数
 *             4、"Dwin_ErrorHandle"函数用户按需编写
 *             5、按需配置"dwin_cfg.h"
 */
#include "dwin_port.h"
#include "measure.h"
#include "flash.h"
#include <flashdb.h>

/*用户函数声明区*/
static void Dwin_Send(pDwinHandle pd);
static void Dwin_ErrorHandle(pDwinHandle pd, dwin_result err_code, uint8_t site, void *pdata);
static void Dwin_LimitHandle(pDwinHandle pd, uint8_t site);
static void Dwin_Back_ParamHandle(pDwinHandle pd, uint8_t site);
static void Dwin_Cur_Operate_Distinguish(pDwinHandle pd, uint8_t site);
// static void Dwin_SureHandle(pDwinHandle pd, uint8_t site);
// static void Dwin_CancelHandle(pDwinHandle pd, uint8_t site);
static void Dwin_SureOrCancelHandle(pDwinHandle pd, uint8_t site);
static void Dwin_ButtonEventHandle(pDwinHandle pd, uint8_t site);
static void Dwin_ClearHistoryData(pDwinHandle pd, uint8_t site);
static void Dwin_DeveloperMode(pDwinHandle pd, uint8_t site);
static void Dwin_Get_Rtc(pDwinHandle pd, uint8_t site);
static void Dwin_Get_Rtc_From_Network(pDwinHandle pd, uint8_t site);
// static void Dwin_Set_ExpeStd(pDwinHandle pd, uint8_t site);
static void set_measure_system_state(struct measure *pe, measure_project_state state);

typedef void (*dwin_struct__)(void *, uint8_t);
/*迪文响应线程*/
Event_Map Dwin_ObjMap[] = {
    /*系统传感器上下限设置区*/
    {.addr = PRE_SENSOR_STD_UPPER_ADDR, .upper = 10.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = PRE_SENSOR_STD_LOWER_ADDR, .upper = 10.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = PRE_SENSOR_TEST_UPPER_ADDR, .upper = 10.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = PRE_SENSOR_TEST_LOWER_ADDR, .upper = 10.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},

    {.addr = FLO_SENSOR_STD_UPPER_ADDR, .upper = 100.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = FLO_SENSOR_STD_LOWER_ADDR, .upper = 100.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = FLO_SENSOR_TEST_UPPER_ADDR, .upper = 100.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = FLO_SENSOR_TEST_LOWER_ADDR, .upper = 100.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},

    {.addr = LEL_SENSOR_STD_UPPER_ADDR, .upper = 400.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = LEL_SENSOR_STD_LOWER_ADDR, .upper = 50.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = LEL_SENSOR_TEST_UPPER_ADDR, .upper = 400.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = LEL_SENSOR_TEST_LOWER_ADDR, .upper = 50.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},

    {.addr = TEMP_SENSOR_STD_UPPER_ADDR, .upper = 200.0F, .lower = -50.0F, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = TEMP_SENSOR_STD_LOWER_ADDR, .upper = 200.0F, .lower = -50.0F, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = TEMP_SENSOR_TEST_UPPER_ADDR, .upper = 200.0F, .lower = -50.0F, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = TEMP_SENSOR_TEST_LOWER_ADDR, .upper = 200.0F, .lower = -50.0F, .event = (dwin_struct__)Dwin_LimitHandle},

    {.addr = CON_SENSOR_STD_UPPER_ADDR, .upper = 200.0F, .lower = 0.1F, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = CON_SENSOR_STD_LOWER_ADDR, .upper = 200.0F, .lower = 0.1F, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = CON_SENSOR_TEST_UPPER_ADDR, .upper = 200.0F, .lower = 0.1F, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = CON_SENSOR_TEST_LOWER_ADDR, .upper = 200.0F, .lower = 0.1F, .event = (dwin_struct__)Dwin_LimitHandle},

    {.addr = VOR_SENSOR_STD_UPPER_ADDR, .upper = 200.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = VOR_SENSOR_STD_LOWER_ADDR, .upper = 200.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = VOR_SENSOR_TEST_UPPER_ADDR, .upper = 200.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = VOR_SENSOR_TEST_LOWER_ADDR, .upper = 200.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    /*实验标准设置*/
    {.addr = PRE_EXPE_STD_ADDR, .upper = 100.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = FLO_EXPE_STD_ADDR, .upper = 100.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = LEL_EXPE_STD_ADDR, .upper = 100.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = TEMP_EXPE_STD_ADDR, .upper = 100.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = CON_EXPE_STD_ADDR, .upper = 100.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = VOR_EXPE_STD_ADDR, .upper = 100.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},

    /*系统后台参数*/
    {.addr = PFL_SYSTEM_SILENCE_TIMERS_ADDR, .upper = 500000.0F, .lower = 0, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = PFL_SYSTEM_PERMIT_ERROR_ADDR, .upper = 30.0F, .lower = 0, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = PFL_SYSTEM_OFFSET, .upper = 20.0F, .lower = 1.0F, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = TEMP_SYSTEM_SILENCE_TIMERS_ADDR, .upper = 500000.0F, .lower = 0, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = TEMP_SYSTEM_PERMIT_ERROR_ADDR, .upper = 300.0F, .lower = -50.0F, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = TEMP_SYSTEM_OFFSET, .upper = 20.0F, .lower = 1.0F, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = CON_SYSTEM_SILENCE_TIMERS_ADDR, .upper = 500000.0F, .lower = 0, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = CON_SYSTEM_PERMIT_ERROR_ADDR, .upper = 30.0F, .lower = 0, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = CON_SYSTEM_OFFSET, .upper = 20.0F, .lower = 1.0F, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    /*特殊变量*/
    {.addr = COMM_DATA_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Cur_Operate_Distinguish},
    {.addr = SYSTEM_START_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_SureOrCancelHandle},
    // {.addr = SYSTEM_END_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_SureOrCancelHandle},
    {.addr = SYSTEM_STOP_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_ButtonEventHandle},
    {.addr = HISTORY_DATA_CLEAN_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_ClearHistoryData},
    {.addr = DEVELOPER_MODE_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_DeveloperMode},
    /*读取迪文系统变量*/
    {.addr = DWIN_SYSTEM_READ_RTC_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Get_Rtc},
    {.addr = NETWORK_TIME_UPDATE_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Get_Rtc_From_Network},
    {.addr = Dwin_SCREEN_TIME_UPDATE_ADDR, .upper = 0xFFFF, .lower = 0, .event = (dwin_struct__)Dwin_Get_Rtc},
};

#define Dwin_EventSize (sizeof(Dwin_ObjMap) / sizeof(Event_Map))

/*定义迪文屏幕对象*/
pDwinHandle Dwin_Object;

/**
 * @brief  迪文屏幕初始化
 * @param  None
 * @retval None
 */
#if (DWIN_USING_RTOS == 2)
#if (DWIN_USING_MALLOC)
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart2;

int rt_dwin_init(void)
{
    UartHandle dwin_uart = {
        .huart = &huart2,
        .phdma = &hdma_usart2_rx,
#if (DWIN_USING_RTOS)
        .semaphore = NULL,
#else
        .recive_finish_flag = false,
#endif
        .tx = {
            .size = DWIN_TX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
        .rx = {
            .size = DWIN_RX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
    };
    DwinHandle temp_dwin = {
        .Id = 0x00,
        .Slave.pMap = Dwin_ObjMap,
        .Slave.Events_Size = Dwin_EventSize,
        .Uart = dwin_uart,
        // .Slave.pHandle = &measure_storage_object,
        .Slave.pHandle = &measure_object,
        .Dw_Delay = (void (*)(uint32_t))rt_thread_mdelay,
        .Dw_Transmit = Dwin_Send,
        .Dw_Error = Dwin_ErrorHandle,
    };

    Create_DwinObject(&Dwin_Object, &temp_dwin);
    return 0;
}
/*在内核对象中初始化:https://blog.csdn.net/yang1111111112/article/details/93982354*/
// INIT_COMPONENT_EXPORT(rt_dwin_init);
// INIT_ENV_EXPORT(rt_dwin_init);
INIT_DEVICE_EXPORT(rt_dwin_init);
#else
void rt_dwin_init(void)
{
    Init_Dwin_Object(0);
    pDwinHandle pd = Get_Dwin_Object(0);
    DwinHandle temp_dwin = {
        /*User init info*/
    };
    Create_DwinObject(&pd, &temp_dwin);
}
#endif

#else
void MX_DwinInit(void)
{
}
#endif

/**
 * @brief  带CRC的发送数据帧
 * @param  pd 迪文屏幕句柄
 * @retval None
 */
static void Dwin_Send(pDwinHandle pd)
{
#if (DWIN_USING_CRC == 1U)
    uint16_t crc16 = get_crc16(&dwin_tx_buf[3U], dwin_tx_count(pd) - 3U, 0xffff);

    memcpy(&dwin_tx_buf[dwin_tx_count(pd)], (uint8_t *)&crc16, sizeof(crc16));
    dwin_tx_count(pd) += sizeof(crc16);
#endif

#if (DWIN_USING_DMA)
    HAL_UART_Transmit_DMA((UART_HandleTypeDef *)pd->Uart.huart, dwin_tx_buf, dwin_tx_count(pd));
    /*https://blog.csdn.net/mickey35/article/details/80186124*/
    /*https://blog.csdn.net/qq_40452910/article/details/80022619*/
    while (__HAL_UART_GET_FLAG((UART_HandleTypeDef *)pd->Uart.huart, UART_FLAG_TC) == RESET)
    {
        if (pd->Dw_Delay)
            pd->Dw_Delay(1);
    }
#else
    HAL_UART_Transmit((UART_HandleTypeDef *)pd->Uart.huart, dwin_tx_buf, dwin_tx_count, 0xffff);
#endif
}

/**
 * @brief  迪文屏幕错误处理
 * @param  pd 迪文屏幕对象句柄
 * @param  err_code 错误代码
 * @param  site 当前对象出错位置
 * @param  pdata 当前出错数据
 * @retval None
 */
static void Dwin_ErrorHandle(pDwinHandle pd, dwin_result err_code, uint8_t site, void *pdata)
{
#define ERROR_NOTE_PAGE 31U

    float tdata = *(float *)pdata;
    uint16_t tarry[] = {0, 0, 0};

    if (err_code < err_min_lower_limit)
    {
        tarry[0] = 0x0100;
    }

    memcpy(&tarry[1], (void *)&tdata, sizeof(tdata));
    pd->Dw_Write(pd, LIMIT_NOTE_PAGE_ADDR, (uint8_t *)&tarry, sizeof(tarry));
    pd->Dw_Delay(NEXT_DELAT_TIMES);
    /*切换到提示页面*/
    pd->Dw_Page(pd, ERROR_NOTE_PAGE);
    pd->Dw_Delay(NEXT_DELAT_TIMES);
#undef ERROR_NOTE_PAGE
#undef LIMIT_NOTE_PAGE_ADDR
}

/**
 * @brief  迪文屏幕检测变量数据范围
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
dwin_result dwin_check_scope(pDwinHandle pd, float *pdata, uint8_t site)
{
    if (*pdata < pd->Slave.pMap[site].lower)
    {
        *pdata = pd->Slave.pMap[site].lower;
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("Error: Below lower limit %.3f.\r\n", *pdata);
#endif
        return err_min_lower_limit;
    }

    if (*pdata > pd->Slave.pMap[site].upper)
    {
        *pdata = pd->Slave.pMap[site].upper;
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("Error: Above upper limit %.3f.\r\n", *pdata);
#endif
        return err_max_upper_limit;
    }
    return dwin_ok;
}

/**
 * @brief  迪文屏幕存储系统参数到flash
 * @param  pd 迪文屏幕对象句柄
 * @param  None
 * @retval None
 */
static void Dwin_Save(pDwinHandle pd)
{
    pmeasure_t ps = (pmeasure_t)pd->Slave.pHandle;

    if (ps == NULL)
        return;

    /*运行过程中不允许修改系统参数*/
    for (uint8_t i = 0; i < SYSYTEM_NUM; ++i)
        set_measure_system_state(&ps->me[i], proj_null);
    /*清除系统错误标志、校验结果*/
    ps->flag &= 0x8000;

    /*系统参数存储*/
    // if (__GET_FLAG(ps->flag, measure_save))
    {
        // __RESET_FLAG(ps->flag, measure_save);
        ps->crc16 = get_crc16((uint8_t *)ps, sizeof(measure_t) - sizeof(ps->crc16), 0xFFFF);
        onchip_flash_init();
        operate_onchip_flash(PARAM_SAVE_ADDRESS, (uint16_t *)ps, sizeof(measure_t));
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("@note:DwinObject save param,crc: %#x\r\n", ps->crc16);
#endif
    }
}

/**
 * @brief  迪文屏幕参数限制数据处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_LimitHandle(pDwinHandle pd, uint8_t site)
{
#define DWIN_RATIO 10.0F
#define LIMIT_MAX (MEASURE_SENSOR_NUM * 4U)
#define EXPE_STD_MAX 6U

    uint16_t udata = (uint16_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    // short int sdata = 0;
    // /*识别负数*/
    // if (udata & 0x1000)
    //     sdata = -1 * (short int)(~udata + 1U);
    // else
    //     sdata = (short int)udata;

    short int sdata = pd->Dw_GetSignedData(pd, udata);
    pmeasure_t ps = (pmeasure_t)pd->Slave.pHandle;
    float data = (float)sdata / DWIN_RATIO;

    /*上下限和实验标准均使用此结构*/
    if (ps == NULL || site >= (LIMIT_MAX + EXPE_STD_MAX))
        return;

    dwin_result dwin_code = dwin_check_scope(pd, &data, site);
    if (site < (MEASURE_SENSOR_NUM * 4U))
    {
        /*std_upper|std_lower|test_upper|test_lower*/
        ps->limits[site] = data;
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("@note:exe'Dwin_LimitHandle',error_code:%#x,limit[%#x]: %.3f\r\n", dwin_code,
                   site, data);
#endif
    }
    else
    {
        uint8_t start_site = site - 24U;
        if (start_site < sizeof(ps->expe_std) / sizeof(ps->expe_std[0]))
            ps->expe_std[start_site] = data;
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("@note:exe'Dwin_LimitHandle',error_code:%#x,std[%#x]: %.3f\r\n", dwin_code,
                   start_site, data);
#endif
    }

    endian_swap((uint8_t *)&data, 0, sizeof(float));
    if (dwin_code != dwin_ok)
    {
        if (pd->Dw_Error)
            pd->Dw_Error(pd, dwin_code, site, &data);
    }
    else
    {
        /*错误写入导致的系统状态改变*/
        for (uint8_t i = 0; i < SYSYTEM_NUM; ++i)
        {
            memset(&ps->me[i].front, 0x00, sizeof(ps->me[0].front));
        }
        /*置位存储标志*/
        // __SET_FLAG(ps->flag, measure_save);
        Dwin_Save(pd);
    }
    /*确认数据回传到屏幕:*/
    pd->Dw_Write(pd, pd->Slave.pMap[site].addr, (uint8_t *)&data, sizeof(float));

#undef LIMIT_MAX
#undef EXPE_STD_MAX
}

/**
 * @brief  迪文屏幕后台数据处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Back_ParamHandle(pDwinHandle pd, uint8_t site)
{
#define DWIN_PARAM_OFFSET_SIT 30U
    pmeasure_t ps = (pmeasure_t)pd->Slave.pHandle;
    float data = (float)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]) / DWIN_RATIO;
#define BACK_NUMBER() (sizeof(ps->me[0].back) / sizeof(ps->me[0].back.silence_time))
    uint8_t start_site = site - DWIN_PARAM_OFFSET_SIT;
    uint8_t back_site = start_site / BACK_NUMBER();

    if (ps == NULL)
        return;

    while (1)
    {
        if (start_site < BACK_NUMBER())
            break;
        start_site -= BACK_NUMBER();
    }

    dwin_result dwin_code = dwin_check_scope(pd, &data, site);
    float *p = &ps->me[back_site].back.silence_time;
    if (p)
        *(p + start_site) = data;

#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:exe'Dwin_Back_ParamHandle',error_code:%#x,back[%#x]: %.3f\r\n", dwin_code,
               start_site, data);
#endif
    endian_swap((uint8_t *)&data, 0, sizeof(float));
    if (dwin_code != dwin_ok)
    {
        if (pd->Dw_Error)
            pd->Dw_Error(pd, dwin_code, site, &data);
    }
    /*置位存储标志*/
    // __SET_FLAG(ps->flag, measure_save);
    Dwin_Save(pd);
    /*确认数据回传到屏幕*/
    // pd->Dw_Write(pd, pd->Slave.pMap[site].addr, (uint8_t *)&data, sizeof(float));

#undef DWIN_RATIO
#undef DWIN_PARAM_OFFSET_SIT
}

#define HISTORY_DATA_RECORD_MAX 20U

typedef struct
{
    /*当前传感器id*/
    uint8_t cur_sensor;
    /*当前迭代次数*/
    uint8_t cur_count;
    /*传感器历史数据暂存缓冲区*/
    uint8_t dwin_his_data_buf[HISTORY_DATA_RECORD_MAX * 2U * sizeof(float)];
} dwin_his_t;

static dwin_his_t dwin_his;
#undef HISTORY_DATA_RECORD_MAX
/**
 * @brief   通过 TSDB 的迭代器 API ，在每次迭代时会自动执行 query_cb 回调函数，实现对 TSDB 中所有记录的查询
 * @details
 * @param   none
 * @retval  none
 */
static bool dwin_query_cb(fdb_tsl_t tsl, void *arg)
{
    struct fdb_blob blob;
    meas_his_data_t history;
    fdb_tsdb_t db = arg;
    dwin_his_t *ph = &dwin_his;
    uint8_t cur_site = ph->cur_count * sizeof(float) * 2U;
    rtc_t cur_rtc;

    if (tsl == NULL || arg == NULL)
        return true;

    linux_stamp_to_std_time(&cur_rtc, tsl->time, 8);

    fdb_blob_read((fdb_db_t)db, fdb_tsl_to_blob(tsl, fdb_blob_make(&blob, &history, sizeof(history))));

    /*确保数据地址不越界*/
    if (cur_site >= sizeof(ph->dwin_his_data_buf))
    {
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG_R("@error:The current data position is out of bounds:%d.\r\n", cur_site);
#endif
        return true;
    }
    /*找到目标传感器数据*/
    if (history.sensor_id == ph->cur_sensor)
    {
        memcpy(&ph->dwin_his_data_buf[cur_site], history.data, sizeof(history.data) - sizeof(history.data[2]));
        ph->cur_count++;
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG_R("@note:[dwin_query_cb] queried a TSL: time: %d/%d/%d/%d %d:%d:%d, sn: %d, std: %f, test: %f, error: %f\r\n",
                     cur_rtc.date.year + 2000, cur_rtc.date.month, cur_rtc.date.date, cur_rtc.date.weelday,
                     cur_rtc.time.hours, cur_rtc.time.minutes, cur_rtc.time.seconds, history.sensor_id,
                     history.data[0], history.data[1], history.data[2]);
#endif
    }

    return false;
}

/**
 * @brief   查询tsdb数据库所有数据,并筛选出目标数据
 * @details
 * @param   none
 * @retval  none
 */
static void select_target_data(void)
{
    extern struct fdb_tsdb measure_tsdb;
    fdb_tsdb_t ptsdb = &measure_tsdb;

    /* query all TSL in TSDB by iterator */
    fdb_tsl_iter(ptsdb, dwin_query_cb, ptsdb);
}
typedef struct
{
    measure_system code;
    uint8_t sensor_id;
} history_t;

/**
 * @brief  获取目标传感器
 * @param  ms_code 系统编码
 * @retval None
 */
static uint8_t get_target_sensor(measure_system ms_code)
{
    history_t his_group[] = {
        {pre_his_cmd, 0},
        {flo_his_cmd, 1},
        {lel_his_cmd, 2},
        {temp_his_cmd, 3},
        {con_his_cmd, 4},
    };
#define HIS_GROUP_SIZE() (sizeof(his_group) / sizeof(his_group[0]))

    for (history_t *p = his_group; p < his_group + HIS_GROUP_SIZE(); ++p)
    {
        if (p->code == ms_code)
            return p->sensor_id;
    }
    return 0xFF;
}

/**
 * @brief  迪文屏幕当前操作的系统
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Cur_Operate_Distinguish(pDwinHandle pd, uint8_t site)
{
#define DWIN_PAGE_TOTAL_NUM 27U

    pmeasure_t ps = (pmeasure_t)pd->Slave.pHandle;
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    uint16_t page = 0x01;
    dwin_his_t *ph = &dwin_his;

    if (ps == NULL)
        return;

    ps->current_system = (measure_system)data;
    uint8_t low_byte = (data & 0x0F);
    /*主系统界面*/
    if (!low_byte)
    {
        // page = data > (uint8_t)con_system ? 0x01 : (data >> 4U) + 1U;
        page = ((data & 0xF0) >> 4U) + 1U;
    }
    else
    {
        if (data < (uint8_t)pre_cur_cmd)
            page = low_byte + 9U;
        else if (data < temp_his_cmd)
            page = low_byte + 11U;
        else if (data < temp_cur_cmd)
            page = low_byte + 12U;
        else if (data < con_his_cmd)
            page = low_byte + 14U;
        else if (data < con_cur_cmd)
            page = low_byte + 13U;
        else
            page = low_byte + 15U;
    }
    if (page > DWIN_PAGE_TOTAL_NUM)
        page = 0x01;

    /*历史数上报*/
    memset(ph->dwin_his_data_buf, 0x00, sizeof(ph->dwin_his_data_buf));

    ph->cur_sensor = get_target_sensor(ps->current_system);
    /*清空查询记录*/
    ph->cur_count = 0;
    /*查询历史数据*/
    select_target_data();
    /*历史数据非空*/
    if (ph->cur_sensor != 0xFF)
    {
        /*大小端数据交换*/
        for (uint8_t i = 0; i < sizeof(ph->dwin_his_data_buf) / sizeof(float); ++i)
            endian_swap(&ph->dwin_his_data_buf[i * sizeof(float)], 0, sizeof(float));

        pd->Dw_Write(pd, PRE_SENSOR_HIS_DATA_ADDR, ph->dwin_his_data_buf, sizeof(ph->dwin_his_data_buf));
        /*确保屏幕能够反应*/
        if (pd->Dw_Delay)
            pd->Dw_Delay(5);
    }

#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:exe'Dwin_Cur_Operate_Distinguish',switch page: %#x\r\n", page);
#endif
    pd->Dw_Page(pd, page);

#undef DWIN_PAGE_TOTAL_NUM
}

/**
 * @brief  迪文屏幕存储系统参数到flash
 * @param  pe 校准对象句柄
 * @param  state
 * @retval None
 */
static void set_measure_system_state(struct measure *pe, measure_project_state state)
{
    if (!pe)
        return;

    /*系统工作转态*/
    pe->front.state = state;
    /*清空当前节点计数值*/
    pe->cur_node = 0;
    pe->front.cur_time = 0;
    pe->front.percentage = 0;
    /*强制停止，则复位检测项目*/
    if (state == proj_null)
        pe->front.cur_sensor = sensor_null;
}

/**
 * @brief  迪文屏幕开始校准/取消校准指令处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_SureOrCancelHandle(pDwinHandle pd, uint8_t site)
{
    pmeasure_t ps = (pmeasure_t)pd->Slave.pHandle;
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);

    if (ps == NULL)
        return;

    uint16_t page = ((ps->current_system & 0xF0) >> 4U) + 1U;
    uint8_t me_site = page - 2U;
    uint8_t adjust_site = !me_site ? 0 : page;
    typeof(&measure_object.me[0]) pthis = &ps->me[me_site];

    if (data == RSURE_CODE)
    {
        /*切换到校准系统界面*/
        pd->Dw_Page(pd, page);

        /*设置‘adjust_t’参数结构*/
        measure_back *pback = &ps->me[me_site].back;
        measure_sensor *psensor = &ps->me[me_site].front.cur_sensor;
        if (!pback->offset)
            pback->offset = 5.0F;

        /*点数为段数+1u*/
        ps->presour->adjust[adjust_site].point = (uint32_t)(pback->offset + 1.0F);
        /*如果时pfl系统，则同时调整3个结构*/
        if (!adjust_site)
        {
            for (adjust_t *p = &ps->presour->adjust[1]; p < &ps->presour->adjust[1] + (SYSYTEM_NUM - 1U); ++p)
                p->point = ps->presour->adjust[0].point;

#if (DWIN_USING_DEBUG)
            DWIN_DEBUG("\t\t[----Dwin_SureHandle----]\nsite\tpoint");
            DWIN_DEBUG_R("----\t-----\n");
            for (uint8_t i = 0; i < SYSYTEM_NUM; ++i)
            {
                DWIN_DEBUG_R("%d\t%d\n", i, ps->presour->adjust[i].point);
            }
#endif
        }
        // else /*温度和电导率：直接使用次数*/
        //     ps->presour->adjust[adjust_site].point = pback->offset;

        ps->presour->timer[me_site].count = ps->me[me_site].back.silence_time;
        /*设置开始传感器*/
        switch (me_site)
        {
        case 0:
            *psensor = sensor_pressure;
            break;
        case 1:
            *psensor = sensor_temperature;
            break;
        case 2:
            *psensor = sensor_conductivity;
            break;
        default:
            *psensor = sensor_null;
            break;
        }
#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("\t\t[----Dwin_SureHandle----]\nsite\tcomp_val\tpoint\tsensor\tsure");
        DWIN_DEBUG_R("----\t--------\t-----\t----\t----\n");
        DWIN_DEBUG_R("%d\t%.3f\t\t%d\t%#x\t%d\n", me_site, ps->presour->adjust[me_site].comp_val,
                     ps->presour->adjust[me_site].point, *psensor, data);
#endif

        set_measure_system_state(pthis, proj_onging);
    }
    else if (data == RCANCEL_CODE)
    {
        /*切换到校准系统界面*/
        pd->Dw_Page(pd, page);
        /*强制取消：点数清零,清空历史数据*/
        set_measure_system_state(pthis, proj_null);

#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("@note:exe'Dwin_CancelHandle',cancel:%#x,system site:%#x.\r\n", data, me_site);
#endif
    }
}

/**
 * @brief  迪文屏按钮事件处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_ButtonEventHandle(pDwinHandle pd, uint8_t site)
{
    pmeasure_t ps = (pmeasure_t)pd->Slave.pHandle;
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);

    if (ps == NULL)
        return;

    measure_switch_group switchx = sw_max;
    switch (data)
    {
    case 0xF0: /*急停:停止所有系统*/
    {
        for (uint8_t i = 0; i < SYSYTEM_NUM; ++i)
            set_measure_system_state(&ps->me[i], proj_null);
        /*清空所有动作*/
        memset(ps->presour->coil, 0x00, sizeof(ps->presour->coil));
        /*清除检验系统检验结果*/
        ps->flag &= 0x80FF;
    }
    break;
    case 0xF1: /*改变风扇开关状态*/
    {
        switchx = sw_fan;
    }
    break;
    case 0xF2: /*改变电导率加水开关状态*/
    {
        switchx = sw_add_level;
    }
    break;
    case 0xF3: /*改变电导率加水开关状态*/
    {
        switchx = sw_empty;
    }
    break;
    case 0xF4: /*改变压力电磁阀状态*/
    {
    }
    break;
    case 0xF5: /*改变液位电磁阀状态*/
    {
    }
    break;
    default:
        break;
    }
    if (switchx < sw_max)
        ps->presour->coil[(uint8_t)switchx] = !ps->presour->coil[(uint8_t)switchx];

#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:exe'Dwin_ButtonEventHandle',b_value:%#x.\r\n", data);
#endif
}

/**
 * @brief  迪文屏清除历史数据
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_ClearHistoryData(pDwinHandle pd, uint8_t site)
{
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    extern struct fdb_tsdb measure_tsdb;
    fdb_tsdb_t ptsdb = &measure_tsdb;

    fdb_tsl_clean(ptsdb);
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:exe'Dwin_ClearHistoryData',key_code:%#x.\r\n", data);
#endif
}

/**
 * @brief  迪文屏进入开发者模式
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_DeveloperMode(pDwinHandle pd, uint8_t site)
{
#define DEVELOPER_PAGE 27
#define ENTRY_TIMEOUT 5U
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    pmeasure_t ps = (pmeasure_t)pd->Slave.pHandle;
    static uint8_t count = 0;
    static uint8_t step_table[] = {0xF0, 0xF1, 0xF2, 0xF3};

    if (ps == NULL || (tim_id_invalid > SOFT_TIMER_NUM))
        return;

#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:exe'Dwin_DeveloperMode',key_code:%#x,count:%#x.\r\n", data, count);
#endif

    measure_timer *ptimer = &ps->presour->timer[tim_id_invalid];

    if (ptimer->flag)
    {
        ptimer->flag = false;
        count = 0;
    }

    /*刷新超时定时器*/
    ptimer->count = ENTRY_TIMEOUT;
    if (step_table[count] == data)
    {
        count++;
    }
    else
    {
        count = 0;
        return;
    }

    if (data == step_table[sizeof(step_table) - 1U])
    {
        ptimer->flag = false;
        count = 0;
        /*切换到开发者页面*/
        pd->Dw_Page(pd, DEVELOPER_PAGE);
    }

#undef DEVOLOPER_PAGE
}

static uint32_t dwin_rtc_count;

/**
 * @brief  保存迪文屏幕时间到本地
 * @param  time_stamp 当前时间戳
 * @retval None
 */
static void set_dwin_rtc(uint32_t time_stamp)
{
    dwin_rtc_count = time_stamp;
}

/**
 * @brief  获取迪文屏幕时间戳
 * @param  None
 * @retval None
 */
uint32_t get_dwin_rtc(void)
{
    return dwin_rtc_count;
}

/**
 * @brief  获取迪文屏幕时间戳自更新
 * @note   处于精度考虑可以放在标准1s定时器中断中
 * @param  None
 * @retval None
 */
void dwin_rtc_count_inc(void)
{
    dwin_rtc_count++;
}

/**
 * @brief   查看当前系统时间
 * @details
 * @param   none
 * @retval  none
 */
static void see_cur_time(void)
{
    rtc_t cur_rtc;

    linux_stamp_to_std_time(&cur_rtc, dwin_rtc_count, 8);

    rt_kprintf("@note:time_stamp:%dS.\r\n%d/%d/%d/%d %d:%d:%d.\r\n",
               dwin_rtc_count, cur_rtc.date.year + 2000, cur_rtc.date.month, cur_rtc.date.date,
               cur_rtc.date.weelday, cur_rtc.time.hours, cur_rtc.time.minutes, cur_rtc.time.seconds);
}
MSH_CMD_EXPORT(see_cur_time, View the current system time.);

/**
 * @brief  迪文屏获取实时时钟
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Get_Rtc(pDwinHandle pd, uint8_t site)
{
    rtc_t dwin_rtc = {
        .date.year = pd->Uart.rx.pbuf[7],
        .date.month = pd->Uart.rx.pbuf[8],
        .date.date = pd->Uart.rx.pbuf[9],
        .date.weelday = pd->Uart.rx.pbuf[10],
        .time.hours = pd->Uart.rx.pbuf[11],
        .time.minutes = pd->Uart.rx.pbuf[12],
        .time.seconds = pd->Uart.rx.pbuf[13],
    };
    uint32_t temp_rtc = std_time_to_linux_stamp(&dwin_rtc);
    set_dwin_rtc(temp_rtc);
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:exe'Dwin_Get_Rtc',time_stamp:%dS.\r\n%d/%d/%d/%d %d:%d:%d.\r\n",
               temp_rtc, dwin_rtc.date.year + 2000, dwin_rtc.date.month, dwin_rtc.date.date,
               dwin_rtc.date.weelday, dwin_rtc.time.hours, dwin_rtc.time.minutes, dwin_rtc.time.seconds);
#endif
}

/**
 * @brief  迪文屏获取从网络模块获取实时时钟
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Get_Rtc_From_Network(pDwinHandle pd, uint8_t site)
{
#if (DWIN_USING_DEBUG)
    DWIN_DEBUG("@note:exe'Dwin_Get_Rtc_From_Network'.\r\n");
#endif
}

/**
 * @brief  迪文屏设置实验标准
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
// static void Dwin_Set_ExpeStd(pDwinHandle pd, uint8_t site)
// {
//     uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
//     pmeasure_t ps = (pmeasure_t)pd->Slave.pHandle;

//     uint8_t start_site = site - PRE_EXPE_STD_ADDR;

// #if (DWIN_USING_DEBUG)
//     DWIN_DEBUG("@note:exe'Dwin_Set_ExpeStd',site:%#x,data:%#x.\r\n", start_site, data);
// #endif
// }
