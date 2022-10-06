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

/*用户函数声明区*/
static void Dwin_Send(pDwinHandle pd);
static void Dwin_ErrorHandle(pDwinHandle pd, dwin_result err_code, uint8_t site);
static void Dwin_LimitHandle(pDwinHandle pd, uint8_t site);
static void Dwin_Back_ParamHandle(pDwinHandle pd, uint8_t site);
static void Dwin_Cur_Operate_Distinguish(pDwinHandle pd, uint8_t site);
static void Dwin_SureHandle(pDwinHandle pd, uint8_t site);
static void Dwin_CancelHandle(pDwinHandle pd, uint8_t site);

typedef void (*dwin_struct__)(void *, uint8_t);
/*迪文响应线程*/
Event_Map Dwin_ObjMap[] = {
    /*系统传感器上下限设置区*/
    {.addr = PRE_SENSOR_UPPER_ADDR, .upper = 10.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = PRE_SENSOR_LOWER_ADDR, .upper = 10.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = FLO_SENSOR_UPPER_ADDR, .upper = 100.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = FLO_SENSOR_LOWER_ADDR, .upper = 100.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = LEL_SENSOR_UPPER_ADDR, .upper = 50.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = LEL_SENSOR_LOWER_ADDR, .upper = 50.0F, .lower = 0, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = TEMP_SENSOR_UPPER_ADDR, .upper = 80.0F, .lower = -30.0F, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = TEMP_SENSOR_LOWER_ADDR, .upper = 80.0F, .lower = -30.0F, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = CON_SENSOR_UPPER_ADDR, .upper = 200.0F, .lower = 0.1F, .event = (dwin_struct__)Dwin_LimitHandle},
    {.addr = CON_SENSOR_LOWER_ADDR, .upper = 200.0F, .lower = 0.1F, .event = (dwin_struct__)Dwin_LimitHandle},
    /*系统后台参数*/
    {.addr = PFL_SYSTEM_SILENCE_TIMERS_ADDR, .upper = 500000.0F, .lower = 0, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = PFL_SYSTEM_PERMIT_ERROR_ADDR, .upper = 30.0F, .lower = 0, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = PFL_SYSTEM_OFFSET, .upper = 20.0F, .lower = 1.0F, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = TEMP_SYSTEM_SILENCE_TIMERS_ADDR, .upper = 500000.0F, .lower = 0, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = TEMP_SYSTEM_PERMIT_ERROR_ADDR, .upper = 30.0F, .lower = 0, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = TEMP_SYSTEM_OFFSET, .upper = 20.0F, .lower = 1.0F, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = CON_SYSTEM_SILENCE_TIMERS_ADDR, .upper = 500000.0F, .lower = 0, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = CON_SYSTEM_PERMIT_ERROR_ADDR, .upper = 30.0F, .lower = 0, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    {.addr = CON_SYSTEM_OFFSET, .upper = 20.0F, .lower = 1.0F, .event = (dwin_struct__)Dwin_Back_ParamHandle},
    /*特殊变量*/
    {.addr = COMM_DATA_ADDR, .upper = 200.0F, .lower = 0.1F, .event = (dwin_struct__)Dwin_Cur_Operate_Distinguish},
    {.addr = SYSTEM_START_ADDR, .upper = 200.0F, .lower = 0.1F, .event = (dwin_struct__)Dwin_SureHandle},
    {.addr = SYSTEM_END_ADDR, .upper = 200.0F, .lower = 0.1F, .event = (dwin_struct__)Dwin_CancelHandle},
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
 * @retval None
 */
static void Dwin_ErrorHandle(pDwinHandle pd, dwin_result err_code, uint8_t site)
{
    // #define ERROR_NOTE_PAGE 30U
    //  TYPEDEF_STRUCT tdata = (error_code == BELOW_LOWER_LIMIT) ? pd->Slave.pMap[site].lower : pd->Slave.pMap[site].upper;
    //  uint16_t tarry[] = {0, 0, 0};
    // #if (DWIN_USING_DEBUG)
    //  if (error_code == BELOW_LOWER_LIMIT)
    //  {
    //      shellPrint(Shell_Object, "Error: Below lower limit %.3f.\r\n", tdata);
    //  }
    //  else
    //  {
    //      tarry[0] = 0x0100;
    //      shellPrint(Shell_Object, "Error: Above upper limit %.3f.\r\n", tdata);
    //  }
    // #endif
    //  Endian_Swap((uint8_t *)&tdata, 0U, sizeof(TYPEDEF_STRUCT));
    //  /*设置错误时将显示上下限*/
    //  pd->Dw_Write(pd, pd->Slave.pMap[site].addr, (uint8_t *)&tdata, sizeof(TYPEDEF_STRUCT));
    //  pd->Dw_Delay(NEXT_DELAT_TIMES);
    //  /*切换到提示页面*/
    //  pd->Dw_Page(pd, ERROR_NOTE_PAGE);
    //  pd->Dw_Delay(NEXT_DELAT_TIMES);
    //  memcpy(&tarry[1], (void *)&tdata, sizeof(tdata));
    //  pd->Dw_Write(pd, NOTE_PAGE_ADDR, (uint8_t *)&tarry, sizeof(tarry));
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
        return err_min_lower_limit;
    }

    if (*pdata > pd->Slave.pMap[site].upper)
    {
        *pdata = pd->Slave.pMap[site].upper;
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
    pmeasure_handle ps = (pmeasure_handle)pd->Slave.pHandle;
    /*系统参数存储*/
    if (__GET_FLAG(ps->flag, measure_save))
    {
        __RESET_FLAG(ps->flag, measure_save);
        ps->crc16 = get_crc16((uint8_t *)ps, sizeof(measure_handle) - sizeof(ps->crc16), 0xFFFF);
        FLASH_Init();
        FLASH_Write(PARAM_SAVE_ADDRESS, (uint16_t *)ps, sizeof(measure_handle));
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
    pmeasure_handle ps = (pmeasure_handle)pd->Slave.pHandle;
    float data = (float)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]) / DWIN_RATIO;
    /*量程范围*/
    // float range = 0;

    if (ps && site < (MEASURE_SENSOR_NUM * 2U))
    {
        uint8_t adjust_site = site / (sizeof(ps->data[0][0]) / 2U); // site / 2U;
        // uint8_t back_site = site < 6U ? 0 : site / 4U;

        dwin_result dwin_code = dwin_check_scope(pd, &data, site);
        /*下限------上限*/
        if (site % 2U)
        {
            // ps->data[site - (site + 1U) / 2U][2] = data;
            ps->data[adjust_site][2] = data;
            // range = data - ps->data[site][3];
        }
        else
        {
            // ps->data[site - site / 2U][3] = data;
            ps->data[adjust_site][3] = data;
            // range = ps->data[site][2] - data;
        }

        // /*设置‘adjust_handle’参数结构*/
        // measure_back *pback = &ps->me[back_site].back;
        // if (!pback->offset)
        //     pback->offset = 5.0;

        // ps->adjust[adjust_site].comp_val = range / pback->offset;
        // /*点数为段数+1u*/
        // ps->adjust[adjust_site].point = (uint32_t)(pback->offset + 1.0F);

#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("@note:exe'Dwin_LimitHandle',error_code:%#x,limit[%#x]: %.3f\r\n", dwin_code,
                   site, data);
#endif
        if (dwin_code != dwin_ok)
        {
            if (pd->Dw_Error)
                pd->Dw_Error(pd, dwin_code, site);
        }
        /*错误写入导致的系统状态改变*/
        for (uint8_t i = 0; i < SYSYTEM_NUM; ++i)
        {
            // ps->me[i].front.state = proj_null;
            // ps->me[i].front.cur_sensor = sensor_null;
            // ps->me[i].front.percentage = 0;
            // ps->me[i].front.cur_time = 0;
            memset(&ps->me[i].front, 0x00, sizeof(&ps->me[0].front));
        }
        /*置位存储标志*/
        __SET_FLAG(ps->flag, measure_save);
        Dwin_Save(pd);
        // /*确认数据回传到屏幕:*/
        // endian_swap((uint8_t *)&data, 0, sizeof(float));
        // pd->Dw_Write(pd, pd->Slave.pMap[site].addr, (uint8_t *)&data, sizeof(float));
    }
}

/**
 * @brief  迪文屏幕后台数据处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_Back_ParamHandle(pDwinHandle pd, uint8_t site)
{
#define DWIN_PARAM_OFFSET_SIT 10U
    pmeasure_handle ps = (pmeasure_handle)pd->Slave.pHandle;
    float data = (float)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]) / DWIN_RATIO;
#define BACK_NUMBER() (sizeof(ps->me[0].back) / sizeof(ps->me[0].back.silence_time))
    uint8_t start_site = site - DWIN_PARAM_OFFSET_SIT;
    uint8_t back_site = start_site / BACK_NUMBER();

    if (ps)
    {
        while (1)
        {
            // if (((start_site == 0U) || (start_site == 1U) ||
            //      (start_site == 2U)))
            if (start_site < BACK_NUMBER())
                break;
            start_site -= BACK_NUMBER();
        }

        dwin_result dwin_code = dwin_check_scope(pd, &data, site);
        float *p = &ps->me[back_site].back.silence_time;
        if (p)
            *(p + start_site) = data;
            // *(p + start_site * sizeof(float)) = !start_site ? (uint32_t)(data * DWIN_RATIO) : data;

#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("@note:exe'Dwin_Back_ParamHandle',error_code:%#x,back[%#x]: %.3f\r\n", dwin_code,
                   start_site, data);
#endif
        if (dwin_code != dwin_ok)
        {
            if (pd->Dw_Error)
                pd->Dw_Error(pd, dwin_code, site);
        }
        /*置位存储标志*/
        __SET_FLAG(ps->flag, measure_save);
        Dwin_Save(pd);
        /*确认数据回传到屏幕*/
        // pd->Dw_Write(pd, pd->Slave.pMap[site].addr, (uint8_t *)&data, sizeof(float));
    }
#undef DWIN_RATIO
#undef DWIN_PARAM_OFFSET_SIT
    // #undef BACK_NUMBER()
}

typedef struct
{
    measure_system code;
    uint8_t base_addr;
} history_struct;

/**
 * @brief  获取历史数据基地址
 * @param  ms_code 系统编码
 * @retval None
 */
static uint8_t get_history_base_addr(measure_system ms_code)
{
    history_struct his_group[] = {
        {pre_his_cmd, 0},
        {flo_his_cmd, 6},
        {lel_his_cmd, 12},
        {temp_his_cmd, 18},
        {con_his_cmd, 24},
    };
#define HIS_GROUP_SIZE() (sizeof(his_group) / sizeof(his_group[0]))

    for (history_struct *p = his_group; p < his_group + HIS_GROUP_SIZE(); ++p)
    {
        if (p->code == ms_code)
            return p->base_addr;
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
// #define DW_PFL_SYSTEM_PAGE 0x02
// #define DW_TEMP_SYSTEM_PAGE 0x03
// #define DW_CON_SYSTEM_PAGE 0x04
#define DWIN_PAGE_TOTAL_NUM 27U
    pmeasure_handle ps = (pmeasure_handle)pd->Slave.pHandle;
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);
    uint16_t page = 0x01;
    uint8_t buf[MEASURE_MAX_POINT * 2U * sizeof(float)];
    if (ps)
    {
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
        memset(buf, 0x00, sizeof(buf));
        uint8_t base_addr = get_history_base_addr(ps->current_system);
        /*确认用户触控区域为历史数据显示*/
        if (base_addr != 0xFF)
        {
            /*拷贝数据到临时缓冲区*/
            for (uint8_t i = 0; i < MEASURE_MAX_POINT; ++i)
                memcpy(&buf[i * 8U], &ps->his_data[base_addr + i][0], sizeof(ps->his_data[0]));
            /*大小端数据交换*/
            for (uint8_t i = 0; i < MEASURE_MAX_POINT * 2U; ++i)
                endian_swap(&buf[i * sizeof(float)], 0, sizeof(float));

            pd->Dw_Write(pd, PRE_SENSOR_HIS_DATA_ADDR, buf, sizeof(buf));
            /*确保屏幕能够反应*/
            if (pd->Dw_Delay)
                pd->Dw_Delay(5);
        }

#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("@note:exe'Dwin_Cur_Operate_Distinguish',switch page: %#x\r\n", page);
#endif
        pd->Dw_Page(pd, page);
    }
}

/**
 * @brief  从临时结构拷贝数据到系统运行区
 * @param  ps 临时对象句柄
 * @retval None
 */
// void set_measure_system_work_state(measure_storage *ps)
// {

// }

/**
 * @brief  从临时结构拷贝数据到系统运行区
 * @param  ps 临时对象句柄
 * @retval None
 */
// void from_temp_copy_data_to_run(measure_storage *ps)
// {
//     if (ps)
//         return;
//     for (uint8_t i = 0; i < SYSYTEM_NUM; ++i)
//     {
//         memcpy(&ps->pm->me[i].front, &ps->front[i], sizeof(ps->front));
//         memcpy(&ps->pm->me[i].back, &ps->back[i], sizeof(ps->back));
//     }
//     /*拷贝调整结构*/
//     memcpy(ps->pm->adjust, ps->adjust, sizeof(ps->adjust));
//     /*拷贝上下限数据*/
//     for (uint8_t i = 0; i < sizeof(ps->measure_limit) / sizeof(ps->measure_limit[0]); ++i)
//     {
//         memcpy(&ps->pm->data[i][2], &ps->measure_limit[i][0], sizeof(ps->measure_limit[0][0]) * 2U);
//     }
// }

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
 * @brief  迪文屏幕开始校准指令处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_SureHandle(pDwinHandle pd, uint8_t site)
{
    pmeasure_handle ps = (pmeasure_handle)pd->Slave.pHandle;
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);

    if (ps && (data == RSURE_CODE))
    {
        uint16_t page = ((ps->current_system & 0xF0) >> 4U) + 1U;
        uint8_t me_site = page - 2U;
        uint8_t adjust_site = !me_site ? 0 : page;
        typeof(&measure_object.me[0]) pthis = &ps->me[me_site];

        /*切换到校准系统界面*/
        pd->Dw_Page(pd, page);

        /*量程范围*/
        // float range = 0;
        /*设置‘adjust_handle’参数结构*/
        measure_back *pback = &ps->me[me_site].back;
        measure_sensor *psensor = &ps->me[me_site].front.cur_sensor;
        if (!pback->offset)
            pback->offset = 5.0F;

        /*点数为段数+1u*/
        ps->adjust[adjust_site].point = (uint32_t)(pback->offset + 1.0F);
        /*如果时pfl系统，则同时调整3个结构*/
        if (!adjust_site)
        {
            for (adjust_handle *p = &ps->adjust[1]; p < &ps->adjust[1] + (SYSYTEM_NUM - 1U); ++p)
                p->point = ps->adjust[0].point;
        }
        ps->timer[me_site].count = ps->me[me_site].back.silence_time;
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
        DWIN_DEBUG_R("%d\t%.3f\t\t%d\t%#x\t%d\n", me_site, ps->adjust[me_site].comp_val,
                     ps->adjust[me_site].point, *psensor, data);
#endif

        set_measure_system_state(pthis, proj_onging);
    }
    // #undef BACK_NUMBER()
}

/**
 * @brief  迪文屏幕取消校准指令处理
 * @param  pd 迪文屏幕对象句柄
 * @param  site 记录当前Map中位置
 * @retval None
 */
static void Dwin_CancelHandle(pDwinHandle pd, uint8_t site)
{
    pmeasure_handle ps = (pmeasure_handle)pd->Slave.pHandle;
    uint8_t data = (uint8_t)Get_Dwin_Data(pd->Uart.rx.pbuf, 7U, pd->Uart.rx.pbuf[6U]);

    if (ps && (data == RCANCEL_CODE))
    {
        uint16_t page = ((ps->current_system & 0xF0) >> 4U) + 1U;
        uint8_t me_site = page - 2U;
        typeof(&measure_object.me[0]) pthis = &ps->me[me_site];
        /*切换到校准系统界面*/
        pd->Dw_Page(pd, page);
        /*强制取消：点数清零,清空历史数据*/
        // uint8_t site_count = 0;
        // if (!me_site)
        // { /*如果时pfl系统，则同时调整3个结构*/
        //     site_count = SYSYTEM_NUM;
        // }
        // else
        //     site_count = 1U;
        // for (adjust_handle *p = &ps->adjust[me_site]; p < &ps->adjust[me_site] + site_count; ++p)
        // {
        //     p->point = 0;
        //     p->offset_val = 0;
        //     p->comp_val = 4.0F;
        //     p->tar_val = 0;
        // }

        // /*清空软件定时器*/
        // ps->timer[me_site].count = 0;
        // Dwin_Save(pd);
        set_measure_system_state(pthis, proj_null);

#if (DWIN_USING_DEBUG)
        DWIN_DEBUG("@note:exe'Dwin_CancelHandle',cancel:%#x,system site:%#x.\r\n", data, me_site);
#endif
    }
}
