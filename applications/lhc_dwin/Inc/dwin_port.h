/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef PACKAGES_DWIN_INC_DWIN_PORT_H_
#define PACKAGES_DWIN_INC_DWIN_PORT_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include "dwin.h"

    extern pDwinHandle Dwin_Object;
#if (DWIN_USING_RTOS == 2)
    extern int rt_dwin_init(void);
#else
extern void MX_DwinInit(void);
#endif

/*迪文屏幕页面*/
#define MAIN_PAGE 0x01
#define DIGITAL_INPUT_PAGE 0x04
#define DIFITAL_OUTPUT_PAGE 0x05
#define ANALOG_INPUT_PAGE 0x06
#define ANALOG_OUTPUT_PAGE 0x07
#define NONE_PAGE 0x08
#define COMMUNICATION_PAGE 0x0F
#define ERROR_PAGE 0x10
#define RESET_POEWR_NOTE_PAGE 28U
#define NEXT_DELAT_TIMES 50U
#define Update_Page 28U

#define DIGITAL_INPUT_ADDR 0x2000              //数字输入开始地址
#define PFL_SYSTWM_PROJECT_ADDR 0x2004         //压力、流量、液位系统项目显示地址
#define ANIMATION_START_ADDR 0x2010            //动画组开始地址
#define PFL_SYSTEM_SILENCE_TIMERS_ADDR 0x2016  //压力、流量、液位系统后台显示地址-------
#define PFL_SYSTEM_PERMIT_ERROR_ADDR 0x2018    //压力、流量、液位系统允许误差
#define PFL_SYSTEM_OFFSET 0x201A               //压力、流量、液位系统偏移点
#define TEMP_SYSTEM_SILENCE_TIMERS_ADDR 0x201C //温度系统静默时间
#define TEMP_SYSTEM_PERMIT_ERROR_ADDR 0x201E   //温度系统允许误差
#define TEMP_SYSTEM_OFFSET 0x2020              //温度系统偏移点
#define CON_SYSTEM_SILENCE_TIMERS_ADDR 0x2022  //温度系统静默时间
#define CON_SYSTEM_PERMIT_ERROR_ADDR 0x2024    //温度系统允许误差
#define CON_SYSTEM_OFFSET 0x2026               //温度系统偏移点-----------

#define PRE_SENSOR_CURRENT_DATA_ADDR 0x2048 //压力传感器当前数据地址
#define PRE_SENSOR_UPPER_ADDR 0x205C        //压力传感器上限地址----------
#define PRE_SENSOR_LOWER_ADDR 0x205E        //压力传感器下限地址
#define FLO_SENSOR_UPPER_ADDR 0x2060        //流量传感器上限地址
#define FLO_SENSOR_LOWER_ADDR 0x2062        //流量传感器下限地址
#define LEL_SENSOR_UPPER_ADDR 0x2064        //液位传感器上限地址
#define LEL_SENSOR_LOWER_ADDR 0x2066        //液位传感器下限地址
#define TEMP_SENSOR_UPPER_ADDR 0x2068       //温度传感器上限地址
#define TEMP_SENSOR_LOWER_ADDR 0x206A       //温度传感器下限地址
#define CON_SENSOR_UPPER_ADDR 0x2068        //电导率传感器上限地址
#define CON_SENSOR_LOWER_ADDR 0x206A        //电导率传感器下限地址---------
#define PRE_SENSOR_HIS_DATA_ADDR 0x2070     //压力传感器历史数据地址
#define COMM_DATA_ADDR 0x3000               //当前系统区分地址
#define SYSTEM_START_ADDR 0x3001            //系统开始按钮确认地址
#define SYSTEM_END_ADDR 0x3002              //系统结束按钮确认地址

#define PFL_SYSTEM_PAGE_CMD 0x10  //进入压力、流量、液位系统命令
#define PRE_HIS_PAGE_CMD 0x11     //进入压力系统历史数据显示命令
#define FLO_HIS_PAGE_CMD 0x12     //进入流量系统历史数据命令
#define LEL_HIS_PAGE_CMD 0x13     //进入液位系统历史数据命令
#define PRE_CUR_PAGE_CMD 0x14     //进入压力系统曲线显示
#define FLO_CUR_PAGE_CMD 0x15     //进入流量系统曲线显示
#define LEL_CUR_PAGE_CMD 0x16     //进入液位系统曲线显示
#define TEMP_SYSTEM_PAGE_CMD 0x20 //进入温度系统命令
#define TEMP_HIS_PAGE_CMD 0x21    //进入温度系统历史数据显示命令
#define TEMP_CUR_PAGE_CMD 0x24    //进入温度系统曲线显示命令
#define CON_SYSTEM_PAGE_CMD 0x30  //进入电导率系统命令
#define CON_HIS_PAGE_CMD 0x31     //进入电导率系统历史数据显示命令
#define CON_CUR_PAGE_CMD 0x34     //进入电导率系统曲线显示命令

#define RSURE_CODE 0x00F1   //确认键值
#define RCANCEL_CODE 0x00F0 //注销键值
#ifdef __cplusplus
}
#endif
#endif /* PACKAGES_DWIN_INC_DWIN_PORT_H_ */
