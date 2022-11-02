/*
 * myflash.h
 *
 *  Created on: 2021年07月31日
 *      Author: LHC
 */

/*
 *@note:STM32F103C8T6的Flash有效容量为64KB，但是实际可以使用超过64KB，上限是128KB
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "main.h"

/*flash操作ok码*/
#define FLASH_OK 0x55AA
/*flash操作失败码*/
#define FLASH_FAILED 0xAA55
/* FLASH大小：512KB ,SRAM : 64KB*/
#define STM32FLASH_SIZE 0x00080000UL /*512KB*/

/* FLASH起始地址 */
#define STM32FLASH_BASE FLASH_BASE

/* FLASH结束地址:此芯片实际flah大小256k，但是使用了VET6的配置*/
#define STM32FLASH_END (STM32FLASH_BASE | STM32FLASH_SIZE)

/* FLASH页大小：2K */
#define STM32FLASH_PAGE_SIZE FLASH_PAGE_SIZE

/* FLASH总页数 */
#define STM32FLASH_PAGE_NUM (STM32FLASH_SIZE / STM32FLASH_PAGE_SIZE)

/* 获取页地址，X=0~STM32FLASH_PAGE_NUM */
#define ADDR_FLASH_PAGE_X(X) (STM32FLASH_BASE | (X * STM32FLASH_PAGE_SIZE))

/*用户数据保存地址*/
#define OTA_UPDATE_FLAG_FLASH_PAGE 254U
#define OTA_UPDATE_SAVE_ADDRESS (FLASH_BASE + OTA_UPDATE_FLAG_FLASH_PAGE * FLASH_PAGE_SIZE)
#define OTA_UPDATE_CMD 0x1234
#define OTA_UPDATE_APP1 0x5AA5
#define OTA_UPDATE_APP2 0xA55A
#define USER_SAVE_FLASH_PAGE 255U
#define PARAM_SAVE_ADDRESS (FLASH_BASE + USER_SAVE_FLASH_PAGE * FLASH_PAGE_SIZE)

#define ONCHIP_FLASH_OK_CODE 0xFFFFFFFF
typedef enum
{
    flash_use_null_buf = -7,   /*flash操作使用空数据缓冲区*/
    flash_addr_err = -6,       /*flash操作地址错误*/
    flah_len_err = -5,         /*flash数据长度错误*/
    flash_addr_not_align = -4, /*flash操作地址未对齐*/
    flash_write_err = -3,      /*flash写入数据失败：写入后对比错误*/
    flash_erase_err = -2,      /*flash擦除错误*/
    flash_fail = -1,           /*falsh局部操作导致的整体操作失败*/
    flash_ok,                  /*flash操作无错误*/
} flash_result;

/*函数声明*/
// void FLASH_Init(void);
// bool FLASH_Read(uint32_t Address, void *Buffer, uint32_t Size);
// uint32_t FLASH_Write(uint32_t Address, const uint16_t *Buffer, uint32_t Size);

extern void onchip_flash_init(void);
extern flash_result onchip_flash_read(uint32_t addr,
                                      uint8_t *pbuf,
                                      size_t size);
extern flash_result operate_onchip_flash(uint32_t addr,
                                         const uint16_t *pbuf,
                                         uint32_t size);

#endif /* INC_FLASH_H_ */
