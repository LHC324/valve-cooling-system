/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-8      zylx         first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <rtthread.h>
#include <board.h>

extern const struct fal_flash_dev stm32_onchip_flash;

/* flash device table */
#define FAL_FLASH_DEV_TABLE  \
    {                        \
        &stm32_onchip_flash, \
    }
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG

/* partition table */
#define FAL_PART_TABLE                                                                                          \
    {                                                                                                           \
        {FAL_PART_MAGIC_WROD, "bootloader", "stm32_onchip", 0, 20 * 1024, 0},                                   \
            {FAL_PART_MAGIC_WROD, "app1", "stm32_onchip", 20 * 1024, 244 * 1024, 0},                            \
            {FAL_PART_MAGIC_WROD, "app2", "stm32_onchip", (20 + 244) * 1024, 224 * 1024, 0},                    \
            {FAL_PART_MAGIC_WROD, "history_data", "stm32_onchip", (20 + 244 + 224) * 1024, 20 * 1024, 0},       \
            {FAL_PART_MAGIC_WROD, "update_param", "stm32_onchip", (20 + 244 + 224 + 20) * 1024, 2 * 1024, 0},   \
            {FAL_PART_MAGIC_WROD, "user_param", "stm32_onchip", (20 + 244 + 224 + 20 + 2) * 1024, 2 * 1024, 0}, \
    }
#endif /* FAL_PART_HAS_TABLE_CFG */
#endif /* _FAL_CFG_H_ */
