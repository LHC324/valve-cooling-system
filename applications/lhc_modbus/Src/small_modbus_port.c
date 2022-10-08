/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 *  @verbatim
 *  使用： 1、用户需要完善"rt_small_modbus_init/MX_ModbusInit"、"Modbus_Send"函数
 *         2、用户需要定义"ModbusPools"寄存器池
 *         3、"rt_small_modbus_init/MX_ModbusInit"初始化时需要明确指定"UartHandle"参数
 *         4、"Modbus_CallBack"函数用户按需编写
 *         5、按需配置"small_modbus_cfg.h"
 */
#include "small_modbus_port.h"

#define __init_modbus(__name, __type, __master_id, __slave_id,                    \
                      __callback, __lock, __unlock, __ota_update, __error_handle, \
                      __transmit, __uart, __pools, __user_handle)                 \
    MdbusHandle __name##_small_modbus = {                                         \
        .type = __type,                                                           \
        .Master.id = __master_id,                                                 \
        .Slave.id = __slave_id,                                                   \
        .Mod_CallBack = __callback,                                               \
        .Mod_Lock = __lock,                                                       \
        .Mod_Unlock = __unlock,                                                   \
        .Mod_Ota = __ota_update,                                                  \
        .Mod_Error = __error_handle,                                              \
        .Mod_Transmit = __transmit,                                               \
        .Uart = __uart,                                                           \
        .pPools = &__pools,                                                       \
        .Slave.pHandle = __user_handle,                                           \
    };

/*定义Modbus对象*/
pModbusHandle Modbus_Object;
static ModbusPools Spool;
/* 指向互斥量的指针 */
static rt_mutex_t modbus_mutex = RT_NULL;

static void Modbus_CallBack(pModbusHandle pd, Function_Code code);
static void Modbus_ErrorHadle(pModbusHandle pd, Lhc_Modbus_State_Code error_code);
#if (SMODBUS_USING_RTOS)
static void Modbus_Lock(void);
static void Modbus_UnLock(void);
#endif
static void lhc_ota_update(pModbusHandle pd);
static void Modbus_Send(pModbusHandle pd, enum Using_Crc crc);

#if (SMODBUS_USING_RTOS == 2)
#if (SMODBUS_USING_MALLOC)
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;

/**
 * @brief  初始化Modbus协议库
 * @retval None
 */
int rt_small_modbus_init(void)
{
    /* 创建一个动态互斥量 */
    modbus_mutex = rt_mutex_create("modbus_mutex", RT_IPC_FLAG_PRIO);
    UartHandle small_modbus_uart = {
        .huart = &huart3,
        .phdma = &hdma_usart3_rx,
#if (SMODBUS_USING_RTOS)
        .semaphore = NULL,
#else
        .recive_finish_flag = false,
#endif
        .tx = {
            .size = SMODBUS_TX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
        .rx = {
            .size = SMODBUS_RX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
    };
    __init_modbus(temp, Smd_Slave, SMALL_MODBUS_MASTER_ADDR, SMALL_MODBUS_SLAVE_ADDR,
                  Modbus_CallBack, Modbus_Lock, Modbus_UnLock, lhc_ota_update, Modbus_ErrorHadle,
                  Modbus_Send, small_modbus_uart, Spool, NULL);
    Create_ModObject(&Modbus_Object, &temp_small_modbus);
    return 0;
}
/*在内核对象中初始化:https://blog.csdn.net/yang1111111112/article/details/93982354*/
// INIT_COMPONENT_EXPORT(rt_small_modbus_init);
// INIT_ENV_EXPORT(rt_small_modbus_init);
INIT_DEVICE_EXPORT(rt_small_modbus_init);

/*主机使用example*/
/*void small_modbus_master_request(void)
{
    pModbusHandle pd = Modbus_Object;
    Request_HandleTypeDef request = {
        .code = ReadHoldReg,
        .reg_start_addr = 0x0000,
        .reg_len = 0x01,
    };

    if (pd)
    {
        memcpy(&pd->Master.request_data, &request, sizeof(request));
        pd->Mod_Request(pd);
    }
}*/
#else
int rt_small_modbus_init(void)
{
    MdbusHandle temp_small_modbus = {
        /*User init info*/
    };
    Create_ModObject(&Modbus_Object, &temp_small_modbus);
    return 0;
}
#endif

#else
void MX_ModbusInit(void)
{
}

#endif

/**
 * @brief  modbus协议栈外部回调函数
 * @param  pd 需要初始化对象指针
 * @param  code 功能码
 * @retval None
 */
static void Modbus_CallBack(pModbusHandle pd, Function_Code code)
{
}

#if (SMODBUS_USING_RTOS)
/**
 * @brief  modbus协议栈加锁函数
 * @param  None
 * @retval None
 */
static void Modbus_Lock(void)
{
    rt_mutex_take(modbus_mutex, RT_WAITING_FOREVER);
}

/**
 * @brief  modbus协议栈解锁函数
 * @param  None
 * @retval None
 */
static void Modbus_UnLock(void)
{
    rt_mutex_release(modbus_mutex);
}
#endif

/**
 * @brief  modbus协议栈进行ota升级
 * @param  pd modbus协议站句柄
 * @retval None
 */
static void lhc_ota_update(pModbusHandle pd)
{
#include <rtthread.h>
#include "flash.h"
#include "dwin_port.h"

#if (SMODBUS_USING_DEBUG)
    SMODBUS_DEBUG_D("@note:About to enter upgrade mode .......\r\n");
#endif
    /*Switch to the upgrade page*/
    if (Dwin_Object)
    {
        Dwin_Object->Dw_Page(Dwin_Object, Update_Page);
    }

    uint32_t update_flag = (*(__IO uint32_t *)OTA_UPDATE_SAVE_ADDRESS);

    if (((update_flag & 0xFFFF0000) >> 16U) == OTA_UPDATE_APP1)
    {
        update_flag = (((uint32_t)OTA_UPDATE_APP2 << 16U) | OTA_UPDATE_CMD);
    }
    else
    {
        update_flag = (((uint32_t)OTA_UPDATE_APP1 << 16U) | OTA_UPDATE_CMD);
    }
/*调度器上锁，上锁后不再切换到其他线程，仅响应中断*/
#if (SMODBUS_USING_RTOS == 1)
    taskENTER_CRITICAL();
#elif (SMODBUS_USING_RTOS == 2)
    rt_enter_critical();
#endif
    FLASH_Write(OTA_UPDATE_SAVE_ADDRESS, (uint16_t *)&update_flag, sizeof(update_flag));
#if (SMODBUS_USING_RTOS == 1)
    taskEXIT_CRITICAL();
#define __RESET_SYSTEM
    {
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }
#elif (SMODBUS_USING_RTOS == 2)
    /*调度器解锁*/
    rt_exit_critical();
#if (SMODBUS_USING_DEBUG)
    SMODBUS_DEBUG_D("@note:The system starts to restart, please wait .......\r\n");
#endif
    extern void rt_hw_cpu_reset(void);
    /*重启系统*/
    rt_hw_cpu_reset();
#endif
}

/**
 * @brief  modbus协议栈接收帧错误处理
 * @param  pd 需要初始化对象指针
 * @param  error_code 错误码
 * @retval None
 */
static void Modbus_ErrorHadle(pModbusHandle pd, Lhc_Modbus_State_Code error_code)
{
}

/**
 * @brief  Modbus协议发送
 * @param  pd 需要初始化对象指针
 * @retval None
 */
static void Modbus_Send(pModbusHandle pd, enum Using_Crc crc)
{
    if (crc == UsedCrc)
    {
        uint16_t crc16 = get_crc16(smd_tx_buf, smd_tx_count(pd), 0xffff);

        memcpy(&smd_tx_buf[smd_tx_count(pd)], (uint8_t *)&crc16, sizeof(crc16));
        smd_tx_count(pd) += sizeof(crc16);
    }

    HAL_UART_Transmit_DMA((UART_HandleTypeDef *)pd->Uart.huart, smd_tx_buf, smd_tx_count(pd));
    while (__HAL_UART_GET_FLAG((UART_HandleTypeDef *)pd->Uart.huart, UART_FLAG_TC) == RESET)
    {
    }
}
