/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef _MEASURE_H_
#define _MEASURE_H_
#ifdef __cplusplus
extern "C"
{
#endif

/*独立检测系统数量*/
#define SYSYTEM_NUM 3U
#define MEASURE_MAX_SENSOR_NUM 7U                //最大传感器数量
#define MEASURE_SENSOR_NUM 5U                    //传感器数量
#define MEASURE_MAX_ROW 5U                       //测量数据最大行
#define MEASURE_MAX_POINT (MEASURE_MAX_ROW + 1U) // 测量的最大点数:段数+1U
#define MEASURE_MAX_COLUMN 2U                    //测量数据最大列
#define SOFT_TIMER_NUM 6U                        //软件定时器数量
#define ADJUST_NUM 2U                            //调整对象数量
#define COIL_NUM 7U                              //线圈数量
#define CARTOON_NUM 6U                           //动画数量
#define CURRENT_UPPER 16.0F
#define CURRENT_LOWER 4.0F
/*https://blog.csdn.net/weixin_36443823/article/details/112775994*/
#define Get_Target(__current, __upper, __lower) \
    (((__current)-CURRENT_LOWER) / CURRENT_UPPER * ((__upper) - (__lower)) + (__lower))

    // /*工作模式*/
    // // enum measure_mode
    // // {
    // //     proj_auto = 0,
    // //     proj_fp,
    // // };
    typedef enum
    {
        null_system = 0,
        pfl_system = 0x10,   //压力流量系统---------
        pre_his_cmd = 0x11,  /*压力系统历史数据*/
        flo_his_cmd = 0x12,  /*流量系统历史数据*/
        lel_his_cmd = 0x13,  /*液位系统历史数据*/
        pre_cur_cmd = 0x14,  /*压力系统曲线数据*/
        flo_cur_cmd = 0x15,  /*流量系统曲线数据*/
        lel_cur_cmd = 0x16,  /*液位系统曲线数据*/
        temp_system = 0x20,  //温度系统-------
        temp_his_cmd = 0x21, /*温度系统历史数据*/
        temp_cur_cmd = 0x24, /*温度系统曲线数据*/
        con_system = 0x30,   //电导率系统------
        con_his_cmd = 0x31,  /*温度系统历史数据*/
        con_cur_cmd = 0x34,  /*温度系统曲线数据*/
    } measure_system;

    /*校准传感器*/
    typedef enum
    {
        sensor_null = 0x0000,         /*无测试传感器*/
        sensor_pressure = 0x0100,     /*压力*/
        sensor_flow = 0x0200,         /*流量*/
        sensor_level = 0x0300,        /*液位*/
        sensor_temperature = 0x0400,  /*温度*/
        sensor_conductivity = 0x0500, /*电导率*/
        senser_max = 0xFFFF,          /*enum占用2byte*/
    } measure_sensor __attribute__((aligned(2)));
    /*项目状态*/
    typedef enum
    {
        proj_null = 0x0000,     /*无项目*/
        proj_onging = 0x0100,   /*项目进行中*/
        proj_complete = 0x0200, /*项目完成*/
        proj_max = 0xFFFF,      /*enum占用2byte*/
    } measure_project_state __attribute__((aligned(2)));
    /*系统故障码*/
    typedef enum
    {
        proj_err_stop = 0,      /*急停*/
        proj_err_conv,          /*变频器故障*/
        proj_err_sensor,        /*传感器故障*/
        pre_measure_flag = 8,   /*压力传感器校准标志*/
        flo_measure_flag = 9,   /*流量传感器校准标志*/
        lel_measure_flag = 10,  /*液位传感器校准标志*/
        temp_measure_flag = 11, /*温度传感器校准标志*/
        con_measure_flag = 12,  /*电导率传感器校准标志*/
        measure_save = 15,      /*参数存储标志*/
    } measure_flag_group;
    /*开关组*/
    typedef enum
    {
        sw_pressure = 0, /*加压开关*/
        sw_level,        /*液位测量开关*/
        sw_add_level,    /*电导率：加液开关*/
        sw_empty,        /*电导率:排空开关*/
        sw_fan,          /*温度:风扇开关*/
        sw_max = 0xFF,   /*最大开关量*/
    } measure_switch_group;
    /*动画组*/
    typedef enum
    {
        cartoon_inverter = 0, /*变频器动画*/
        cartoon_flow,         /*测流量动画*/
        cartoon_level,        /*测液位动画*/
        cartoon_fan,          /*风扇动画*/
        cartoon_con,          /*示意电解质投入动画*/
    } measure_cartoon_group;
    typedef enum
    {
        mea_reset = 0x000,
        mea_set = 0x0100,
        mea_max = 0xFFFF,
    } measure_operate_type;
    /*定时器组*/
    typedef enum
    {
        tim_id_pfl = 0, /*压力 流量 液位系统静默定时器*/
        tim_id_tempe,   /*温度系统静默定时器*/
        tim_id_conv,    /*电导率系统静默定时器*/
        tim_id_fan,     /*风扇定时器*/
        tim_id_dra,     /*废液排空定时器*/
        tim_id_add_le,  /*加水定时器*/
    } measure_timer_id;

    /*前台显示参数*/
    typedef struct
    { /*项目信息*/
        measure_sensor cur_sensor;
        measure_project_state state;
        unsigned short percentage; /*项目百分比*/
        unsigned short cur_time;   /*当前用时*/
        // struct
        // {
        //     float *p;            /*用户数据*/
        //     unsigned short size; /*数据尺寸*/
        // } data;
    } measure_front __attribute__((aligned(4)));
    /*后台参数*/
    typedef struct
    {
        float silence_time; /*静默时间:auto和fp均生效*/
        float permit_error; /*允许的误差*/
        float offset;       /*测试点偏移量*/

        // struct
        // {
        //     float *p; /*传感器数据上下限*/
        //     unsigned int size;
        // } limit;
        // unsigned short limit_size; /*数据尺寸*/
    } measure_back __attribute__((aligned(4)));

    typedef struct measure_adjust adjust_handle;
    typedef struct measure_adjust *padjust_handle;
    struct measure_adjust
    {
        float comp_val;     /*当前补偿值*/
        float cur_val;      /*当前采样值*/
        float tar_val;      /*目标要求值*/
        float offset_val;   /*每个采样点偏移值*/
        unsigned int point; /*采样点数*/
    };

    typedef struct
    {
        unsigned char flag; /*定时器的溢出标志*/
        unsigned int count; /*时间计数器*/
    } measure_timer;

    typedef struct measure_systemx measure_handle;
    typedef struct measure_systemx *pmeasure_handle;
    struct measure
    {
        measure_front front;
        measure_back back;
        /*当前节点*/
        unsigned char cur_node;
        void (*measure_event)(pmeasure_handle, struct measure *);
    };
    struct measure_systemx
    {
        unsigned short flag;       /*系统标志：低8位系统错误，高7位校验结果.第8bit存储标志*/
        unsigned short error_code; /*系统错误代码*/
        measure_system current_system;
        /*传感器历史记录*/
        float his_data[(MEASURE_MAX_ROW + 1U) * MEASURE_SENSOR_NUM][MEASURE_MAX_COLUMN];
        float data[7U][4U];
        /*阀门*/
        unsigned char coil[COIL_NUM];
        /*动画标志组*/
        unsigned short cartoon[CARTOON_NUM];
        struct measure me[SYSYTEM_NUM];
        measure_timer timer[SOFT_TIMER_NUM];
        adjust_handle adjust[MEASURE_MAX_SENSOR_NUM]; /*调整对象*/
        void *phandle;                                /*外部结构*/
        unsigned int crc16;
    } __attribute__((aligned(4)));

    // typedef enum
    // {
    //     measure_other = 0,
    //     measure_save,
    // } measure_storage_flag_group;
    // /*存储参数*/
    // typedef struct
    // {
    //     measure_system current_system;
    //     unsigned int flag;  /*存储参数标志组*/
    //     pmeasure_handle pm; /*校准系统指针*/
    //     measure_front front[SYSYTEM_NUM];
    //     measure_back back[SYSYTEM_NUM];
    //     adjust_handle adjust[MEASURE_SENSOR_NUM]; /*调整对象*/
    //     float measure_limit[MEASURE_MAX_ROW][MEASURE_MAX_COLUMN];
    //     unsigned int crc16;
    // } measure_storage __attribute__((aligned(4)));

#define sssss sizeof(struct measure_systemx)

    extern measure_handle measure_object;
    //    extern measure_storage measure_storage_object;
    extern void measure_timer_poll(void);
    extern void measure_poll(void);
    // extern void measure_event_handle(void);

#ifdef __cplusplus
}
#endif
#endif /* _MEASURE_H_ */
