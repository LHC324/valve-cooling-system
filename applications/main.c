/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-10     RT-Thread    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "main.h"
#include "tool.h"
#include "dwin_port.h"
#include "small_modbus_port.h"
#include "io_signal.h"
#include "measure.h"
#include "flash.h"
// #include <fal.h>

typedef enum
{
    using_semaphore = 0x00,
    unusing_semaphore,
} semaphore_state;

/*rt_thread 线程池*/
typedef struct rt_thread_pools
{
    rt_thread_t thread_handle;
    const char name[RT_NAME_MAX];
    void *parameter;
    rt_uint32_t stack_size;
    rt_uint8_t priority;
    rt_uint32_t tick;
    void (*thread)(void *parameter);
    semaphore_state sema_state;
    rt_sem_t semaphore;
} rt_thread_pools_t;

typedef struct
{
    // rt_thread_t thread_handle_t;
    rt_thread_pools_t *pools;
    rt_uint32_t rt_thread_numbers;
} rt_thread_pool_map_t;

/*线程入口函数声明区*/
static void modbus_slave_thread_entry(void *parameter);
static void dwin_recive_thread_entry(void *parameter);
static void sampling_thread_entry(void *parameter);
static void contrl_thread_entry(void *parameter);
static void measure_poll_thread_entry(void *parameter);
static void measure_control_thread_entry(void *parameter);
static void report_thread_entry(void *parameter);

#define __init_rt_thread_pools(__name, __handle, __param, __statck_size,    \
                               __prio, __tick, __fun, __sema_state, __sema) \
    {                                                                       \
        .name = __name,                                                     \
        .thread_handle = __handle,                                          \
        .parameter = __param,                                               \
        .stack_size = __statck_size,                                        \
        .priority = __prio,                                                 \
        .tick = __tick,                                                     \
        .thread = __fun,                                                    \
        .sema_state = __sema_state,                                         \
        .semaphore = __sema,                                                \
    }

static rt_thread_pools_t thread_pools[] = {
    __init_rt_thread_pools("md_slave_thread", RT_NULL, RT_NULL, 1024U, 0x0F,
                           10, modbus_slave_thread_entry, using_semaphore, RT_NULL),
    __init_rt_thread_pools("dwin_thread", RT_NULL, RT_NULL, 1024U, 0x0F,
                           10, dwin_recive_thread_entry, using_semaphore, RT_NULL),
    __init_rt_thread_pools("sampling_thread", RT_NULL, RT_NULL, 1024U, 0x05,
                           50, sampling_thread_entry, unusing_semaphore, RT_NULL),
    __init_rt_thread_pools("contrl_thread", RT_NULL, RT_NULL, 512U, 0x0F,
                           10, contrl_thread_entry, unusing_semaphore, RT_NULL),
    __init_rt_thread_pools("measure_thread", RT_NULL, RT_NULL, 2048U, 0x06,
                           10, measure_poll_thread_entry, unusing_semaphore, RT_NULL),
    __init_rt_thread_pools("measure_out", RT_NULL, RT_NULL, 1024U, 0x10,
                           10, measure_control_thread_entry, unusing_semaphore, RT_NULL),
    __init_rt_thread_pools("report", RT_NULL, RT_NULL, 2048U, 0x11,
                           10, report_thread_entry, unusing_semaphore, RT_NULL),

};

rt_thread_pool_map_t rt_thread_pool_map = {
    // .thread_handle_t = RT_NULL,
    .pools = thread_pools,
    .rt_thread_numbers = sizeof(thread_pools) / sizeof(rt_thread_pools_t),
};

/**
 * @brief	rt thread 线程池初始化
 * @details
 * @param	none
 * @retval  none
 */
// static int rt_thread_pools_init(rt_thread_pool_map_t *p_rt_thread_map)
static void rt_thread_pools_init(rt_thread_pool_map_t *p_rt_thread_map)
{
    rt_thread_pools_t *p_rt_thread_pools = p_rt_thread_map->pools;
    if (p_rt_thread_map->rt_thread_numbers && p_rt_thread_pools)
    {
        // rt_thread_t thread_handle = RT_NULL;
        /*从线程池中初始化线程*/
        for (rt_thread_pools_t *p = p_rt_thread_pools;
             p < p_rt_thread_pools + p_rt_thread_map->rt_thread_numbers; ++p)
        {
            if (p->thread)
            {
                /*线程自生参数信息传递给线程入口函数*/
                p->parameter = p;
                p->thread_handle = rt_thread_create(p->name, p->thread, p->parameter,
                                                    p->stack_size, p->priority, p->tick);
            }
            /* 创建一个动态信号量，初始值是 0 */
            if (p->sema_state == using_semaphore)
                p->semaphore = rt_sem_create("sempx", 0, RT_IPC_FLAG_PRIO);
            if (p->thread_handle)
                rt_thread_startup(p->thread_handle);
        }
    }
    //    return 0;
}
/*在内核对象中初始化:https://blog.csdn.net/yang1111111112/article/details/93982354*/
// INIT_COMPONENT_EXPORT(rt_thread_pools_init);
// INIT_APP_EXPORT(rt_thread_pools_init);
// INIT_ENV_EXPORT(rt_thread_pools_init);

/**
 * @brief	rt_thread 初始化目标串口的串口空闲中断+DMA接收
 * @details
 * @param	puart uartx句柄
 * @param   p_pool 线程池句柄
 * @retval  none
 */
static void rt_thread_hal_uartx_dma_info_init(rt_thread_pools_t *p_pool, pUartHandle puart)
{
    /*初始化目标串口DMA配置*/
    if (puart && puart->huart && puart->phdma && puart->rx.pbuf)
    {
        __HAL_UART_ENABLE_IT((UART_HandleTypeDef *)puart->huart, UART_IT_IDLE);
        /*DMA buffer must point to an entity address!!!*/
        HAL_UART_Receive_DMA((UART_HandleTypeDef *)puart->huart, puart->rx.pbuf, puart->rx.size);
        puart->rx.count = 0;
        rt_memset(puart->rx.pbuf, 0x00, puart->rx.size);
    }
    if (p_pool && p_pool->semaphore)
        puart->semaphore = p_pool->semaphore;
}

static rt_timer_t timer1 = RT_NULL, rtc_timer = RT_NULL;
static void timer1_callback(void *parameter);
static void rtc_timer_callback(void *parameter);
/**
 * @brief	main线程
 * @details
 * @param	none
 * @retval 线程状态
 */
int main(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Adc_buffer, ADC_DMA_SIZE);
    /*Turn on the DAC*/
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
    /*初始化线程池*/
    rt_thread_pools_init(&rt_thread_pool_map);
    /* 创建定时器1  周期定时器 */
    timer1 = rt_timer_create("timer1",
                             timer1_callback,
                             RT_NULL,
                             1000,
                             RT_TIMER_FLAG_PERIODIC);
    /* 启动rtc定时器 */
    if (timer1 != RT_NULL)
        rt_timer_start(timer1);
    /* 创建rtc定时器  周期定时器 */
    rtc_timer = rt_timer_create("rtc_timer",
                                rtc_timer_callback,
                                RT_NULL,
                                1000,
                                RT_TIMER_FLAG_PERIODIC);
    /* 启动rtc定时器 */
    if (rtc_timer != RT_NULL)
        rt_timer_start(rtc_timer);
    //    int count = 1;
    //    while (count++)
    // fal_init();
    for (;;)
    {
        //        LOG_D("Hello RT-Thread!");
        // HAL_UART_Transmit_IT(&huart2, "hello word.\r\n", 14);
        HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}

/**
 * @brief	rt_thread 软件定时器回调函数
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void timer1_callback(void *parameter)
{
    UNUSED(parameter);
    measure_timer_poll();
}

/**
 * @brief	rt_thread 软件定时器回调函数
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void rtc_timer_callback(void *parameter)
{
    UNUSED(parameter);
    /*本地rtc时间维持*/
    extern void dwin_rtc_count_inc(void);
    dwin_rtc_count_inc();
}

/**
 * @brief	modbus 从机接收线程
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void modbus_slave_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    /*初始化modbus接口信号量*/
    // extern UartHandle small_modbus_uart;
    rt_thread_hal_uartx_dma_info_init(p_rt_thread_pool, &Modbus_Object->Uart);

    for (;;)
    {
        /* 永久方式等待信号量，获取到信号量则解析modbus协议*/
        if (p_rt_thread_pool->semaphore &&
            rt_sem_take(p_rt_thread_pool->semaphore, RT_WAITING_FOREVER) == RT_EOK)
        {
            small_modbus_handler(Modbus_Object);
            /*远程升级*/
            LOG_D("small modbus recive a data.");
        }
    }
}

/**
 * @brief	dwin 线程解析接收的数据
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void dwin_recive_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    rt_thread_hal_uartx_dma_info_init(p_rt_thread_pool, &Dwin_Object->Uart);

    for (;;)
    {
        /* 永久方式等待信号量*/
        if (p_rt_thread_pool->semaphore &&
            rt_sem_take(p_rt_thread_pool->semaphore, RT_WAITING_FOREVER) == RT_EOK)
        {
            dwin_handler(Dwin_Object);
            // LOG_D("dwin recive a data.");
        }
    }
}

/**
 * @brief	采样线程定时采样数据
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void sampling_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    UNUSED(p_rt_thread_pool);
    // extern DmaHandle rt_dma_dwin;
    // rt_thread_hal_uartx_dma_info_init(p_rt_thread_pool, &rt_dma_dwin);

    for (;;)
    {
        /* 永久方式等待信号量*/
        // if (p_rt_thread_pool->semaphore &&
        //     rt_sem_take(p_rt_thread_pool->semaphore, RT_WAITING_FOREVER) == RT_EOK)
        // {
        // }
        //        LOG_D("samling is running!");
        // extern void measure_sampling(void);
        Read_Io_Handle();
        // measure_sampling();
        rt_thread_mdelay(50);
    }
}

/**
 * @brief	控制线程定时动作
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void contrl_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    UNUSED(p_rt_thread_pool);

    for (;;)
    {
        Write_Io_Handle();
        rt_thread_mdelay(100);
    }
}

/**
 * @brief	测量系统轮询采样事件
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void measure_poll_thread_entry(void *parameter)
{
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    UNUSED(p_rt_thread_pool);

    for (;;)
    {
        measure_poll();
        rt_thread_mdelay(500);
    }
}

/**
 * @brief   获取wifi状态
 * @details
 * @param	None
 * @retval  None
 */
static uint8_t get_wifi_state(void)
{
    static Gpiox_info wifi_gpio[] = {
        {.pGPIOx = WIIF_READY_GPIO_Port, .Gpio_Pin = WIIF_READY_Pin},
        {.pGPIOx = WIFI_LINK_GPIO_Port, .Gpio_Pin = WIFI_LINK_Pin},
    };
    uint8_t wifi_state = 0;
    for (Gpiox_info *p = wifi_gpio;
         p < wifi_gpio + sizeof(wifi_gpio) / sizeof(wifi_gpio[0]); ++p)
    {
        uint8_t site = p - wifi_gpio;
        uint8_t bit = HAL_GPIO_ReadPin((GPIO_TypeDef *)p->pGPIOx, p->Gpio_Pin) ? 0 : 1;
        wifi_state |= bit << site;
    }
    return wifi_state;
}

typedef struct
{
    uint32_t system_version;
    uint32_t update_flag;
} System_InfoTypeDef;

#define CURRENT_HARDWARE_VERSION 100
#define CURRENT_SOFT_VERSION 106
#define SYSTEM_VERSION() ((uint32_t)((CURRENT_SOFT_VERSION << 16U) | CURRENT_HARDWARE_VERSION))
/**
 * @brief   获取系统信息
 * @details
 * @param	None
 * @retval  None
 */
static void get_system_infomation(void)
{
    pModbusHandle pd = Modbus_Object;
    pmeasure_t pm = &measure_object;
    System_InfoTypeDef system_info = {
        .system_version = SYSTEM_VERSION(),
        .update_flag = (*(__IO uint32_t *)OTA_UPDATE_SAVE_ADDRESS),
    };
#define SYSTEM_INFO_SIZE()                          \
    (sizeof(system_info) + sizeof(pm->error_code) + \
     sizeof(pm->me[0].back) * SYSYTEM_NUM)

    uint8_t info_buf[SYSTEM_INFO_SIZE()];
    /*版本信息存储到保持寄存器:跟在模拟输出的后面*/
    if (pd)
    {
        memset(info_buf, 0x00, sizeof(info_buf));
        memcpy(info_buf, &system_info, sizeof(system_info));
        memcpy(&info_buf[sizeof(system_info)], &pm->error_code, sizeof(pm->error_code));
        uint8_t base_addr = sizeof(system_info) + sizeof(pm->error_code);
        for (uint8_t i = 0; i < SYSYTEM_NUM; ++i)
        {
            memcpy(&info_buf[base_addr + i * sizeof(pm->me[0].back)],
                   &pm->me[0].back, sizeof(pm->me[0].back));
        }

        if (!pd->Mod_Operatex(pd, HoldRegister, Write,
                              EXTERN_ANALOGOUT_MAX * sizeof(pd->pPools->HoldRegister[0]),
                              info_buf, sizeof(info_buf)))
        {
            LOG_D("@error:Error writing system information.");
        }
    }
#undef CURRENT_HARDWARE_VERSION
#undef CURRENT_SOFT_VERSION
}

/**
 * @brief   校准系统控制线程
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void measure_control_thread_entry(void *parameter)
{
    for (;;)
    {
        measure_output_control_event();
        rt_thread_mdelay(250);
    }
}

/**
 * @brief   定时上报数据到屏幕
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void report_thread_entry(void *parameter)
{
#define VAL_FLOAT_NUM 45U // 浮点数据数量（不包括历史数据）
    rt_thread_pools_t *p_rt_thread_pool = (rt_thread_pools_t *)parameter;
    pModbusHandle pd = Modbus_Object;
    pDwinHandle pw = Dwin_Object;
    pmeasure_t pm = &measure_object;
    uint8_t in_coil[EXTERN_DIGITALIN_MAX], out_coil[EXTERN_DIGITALOUT_MAX];
    uint8_t buf[22U * 2U + VAL_FLOAT_NUM * 4U];
    UNUSED(p_rt_thread_pool);

    /*首次切换到主界面*/
    if (pw)
    {
        rt_thread_mdelay(3000); // 解决首次上电迪文屏幕不接收参数问题
        pw->Dw_Page(pw, MAIN_PAGE);
        rt_thread_mdelay(10);
        /*首次上报上下限实验标准*/
        memcpy(buf, pm->limits, sizeof(pm->limits));
        memcpy(&buf[sizeof(float) * 24U], pm->expe_std, sizeof(pm->expe_std));
        /*4字节数据交换*/
        for (uint8_t i = 0; i < VAL_FLOAT_NUM; ++i)
            endian_swap(&buf[i * sizeof(float)], 0, sizeof(float));
        pw->Dw_Write(pw, PRE_SENSOR_STD_UPPER_ADDR, (uint8_t *)buf, sizeof(float) * 30U);
        rt_thread_mdelay(10);
        /*主动请求迪文屏幕更新本地RTC时间*/
        pw->Dw_Read(pw, DWIN_SYSTEM_READ_RTC_ADDR, 0x04);
        rt_thread_mdelay(50);
    }
    for (;;)
    {
        if (pd && pw && pm)
        {
            get_system_infomation();
            memset(buf, 0x00, sizeof(buf));
            /*WiFi状态*/
            buf[0] = get_wifi_state();
            /*数字输入*/
            pd->Mod_Operatex(pd, InputCoil, Read, 0x00, in_coil, sizeof(in_coil));
            /*数字输出*/
            pd->Mod_Operatex(pd, Coil, Read, 0x00, out_coil, sizeof(out_coil));
            for (uint8_t i = 0; i < sizeof(in_coil); ++i)
            {
                buf[1] |= in_coil[i] << i;
                if (i < sizeof(out_coil))
                    buf[3] |= out_coil[i] << i;
            }
            /*错误代码*/
            buf[5] = (uint8_t)(pm->error_code & 0x00FF);
            /*检验结果*/
            buf[6] = (uint8_t)((pm->flag & 0xFF00) >> 8U);
            /*系统项目*/
            for (uint8_t this = 0; this < sizeof(pm->me) / sizeof(pm->me[0]); ++this)
            {
                /*前台参数*/
                memcpy(&buf[(PFL_SYSTWM_PROJECT_ADDR - DIGITAL_INPUT_ADDR) * 2U + sizeof(pm->me[this].front) * this],
                       &pm->me[this].front.cur_sensor, sizeof(pm->me[this].front));
                /*后台参数*/
                memcpy(&buf[(PFL_SYSTEM_SILENCE_TIMERS_ADDR - DIGITAL_INPUT_ADDR) * 2U + sizeof(pm->me[this].back) * this],
                       &pm->me[this].back.silence_time, sizeof(pm->me[this].back));
            }

            /*动画组*/
            memcpy(&buf[(ANIMATION_START_ADDR - DIGITAL_INPUT_ADDR) * 2U], &pm->presour->cartoon,
                   sizeof(pm->presour->cartoon));
            /*4字节数据读取*/
            /*模拟输入*/
            pd->Mod_Operatex(pd, InputRegister, Read, 0x00, &buf[(22 * 2 + 9 * 4)],
                             sizeof(float) * EXTERN_ANALOGIN_MAX);
            /*模拟输出*/
            pd->Mod_Operatex(pd, HoldRegister, Read, 0x00, &buf[(22 * 2 + 23 * 4)],
                             sizeof(float) * EXTERN_ANALOGOUT_MAX);
            for (uint8_t i = 0; i < MEASURE_SENSOR_NUM; ++i)
            {
                // /*std sensor data*/
                // memcpy(&buf[(22 * 2 + 25 * 4) + i * sizeof(float) * 2U], &pm->data[i][0], sizeof(float));
                // /*test sensor data*/
                // memcpy(&buf[(22 * 2 + 25 * 4) + i * sizeof(float) * 2U + 4U], &pm->data[i][1], sizeof(float));

                /*std sensor data、test sensor data*/
                memcpy(&buf[(22 * 2 + 25 * 4) + i * sizeof(pm->presour->data[0])],
                       &pm->presour->data[i][0], sizeof(pm->presour->data[0]));

                /*sensor upper limit*/
                // memcpy(&buf[(22 * 2 + 35 * 4) + i * sizeof(float) * 2U], &pm->data[i][3], sizeof(float));
                /*sensor lower limit*/
                // memcpy(&buf[(22 * 2 + 35 * 4) + i * sizeof(float) * 2U + 4U], &pm->data[i][2], sizeof(float));
                /*根据屏幕回传传感器信息获取历史数据*/
                // /*std sensor history data*/
                // memcpy(&buf[(22 * 2 + 45 * 4) + i * sizeof(float) + 4U], &pm->his_data[i][0], sizeof(float));
                // /*test sensor history data*/
                // memcpy(&buf[(22 * 2 + 45 * 4) + i * sizeof(float) + 4U], &pm->his_data[i][1], sizeof(float));
            }

            /*4字节数据交换*/
            for (uint8_t i = 0; i < VAL_FLOAT_NUM; ++i)
                endian_swap(&buf[(PFL_SYSTEM_SILENCE_TIMERS_ADDR - DIGITAL_INPUT_ADDR) * 2U + i * sizeof(float)],
                            0, sizeof(float));

            pw->Dw_Write(pw, DIGITAL_INPUT_ADDR, (uint8_t *)buf, sizeof(buf));
        }
        rt_thread_mdelay(1000);
    }
}
