/*
 * myflash.c
 *
 *  Created on: 2022年01月23日
 *      Author: LHC
 */

#include "flash.h"
#include "string.h"
#include <fal.h>

/*===================================================================================*/
/* Flash 分配
 * @用户flash区域：0-125页(1KB/页)
 * @系统参数区： 126页
 * @校准系数存放区域：127页
 */
/*===================================================================================*/

// /*
//  *  初始化FLASH
//  */
// void FLASH_Init(void)
// {
// 	HAL_FLASH_Unlock();
// 	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
// 	HAL_FLASH_Lock();
// }

// /**
//  * 读FLASH
//  * @param  Address 地址
//  * @note   为了兼容各种数据类型，按字节读取
//  * @param  pBuf  存放读取的数据
//  * @param  Size    要读取的数据大小，单位字节
//  * @return         读出成功的字节数
//  */
// bool FLASH_Read(uint32_t Address, void *pBuf, uint32_t Size)
// {
// 	uint8_t *pdata = (uint8_t *)pBuf;

// 	/* 非法地址 */
// 	if (Address < STM32FLASH_BASE || (Address > STM32FLASH_END) || Size == 0 || pBuf == NULL)
// 		return false;

// 	for (uint32_t i = 0; i < Size; i++)
// 	{
// 		pdata[i] = *(__IO uint16_t *)Address;
// 		Address += 1U;
// 	}
// 	return true;
// }

// /**
//  * 写FLASH
//  * @param  Address    写入起始地址，！！！要求2字节对齐！！！
//  * @param  pBuf     待写入的数据，！！！要求2字节对齐！！！
//  * @param  Size 要写入的数据量，单位：半字，！！！要求2字节对齐！！！
//  * @return            成功：0，错误：非0值
//  */
// uint32_t FLASH_Write(uint32_t Address, const uint16_t *pBuf, uint32_t Size)
// {
// 	/*初始化FLASH_EraseInitTypeDef*/
// 	FLASH_EraseInitTypeDef pEraseInit;
// 	/*设置PageError*/
// 	uint32_t PageError = 0;

// 	/* 非法地址 */
// 	if ((Address % 2) ||
// 		Address < STM32FLASH_BASE ||
// 		(Address > STM32FLASH_END) ||
// 		Size == 0 || pBuf == NULL)
// 		return true;

// 	/*1、解锁FLASH*/
// 	HAL_FLASH_Unlock();
// 	/*2、擦除FLASH*/
// 	pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
// 	pEraseInit.PageAddress = Address;
// 	pEraseInit.NbPages = 1U;
// 	/*清除flash标志位*/
// 	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
// 	HAL_FLASHEx_Erase(&pEraseInit, &PageError);
// 	/*数据擦除成功：返回0xffffffff,否则错误地址*/
// 	PageError = PageError == 0xFFFFFFFF ? 0 : PageError;
// 	/*3、对FLASH烧写*/
// 	uint16_t data = 0;
// 	/*size是按字节，写入是按半字*/
// 	for (uint32_t i = 0; i < (Size / 2U); i++)
// 	{
// 		data = *(pBuf + i);
// 		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address + i * 2, data);
// 		/*写完数据之后，再读出来校验*/
// 		if ((*(__IO uint16_t *)(Address + i * 2) != data))
// 		{
// 			return FLASH_FAILED;
// 		}
// 		// Usart1_Printf("addr is  %d\r\n", Address);
// 	}
// 	/*4、锁住FLASH*/
// 	HAL_FLASH_Lock();

// 	return PageError;
// }

/**
 * @brief	片内flash操作检查
 * @details
 * @param	type 片内flash擦除类型
 * @param   addr 片内flash擦除地址
 * @param   nb_pages 片内flash擦除页数
 * @retval  成功：0xFFFFFFFF;失败：错误地址
 */
static flash_result onchip_flash_operate_check(uint32_t addr,
											   const uint16_t *pbuf,
											   uint32_t size)
{
#define ONCHIP_FLASH_BYTE_ALIGNED 2U
	flash_result result = flash_ok;

	if (addr % ONCHIP_FLASH_BYTE_ALIGNED)
		result = flash_addr_not_align;

	if (addr < STM32FLASH_BASE || addr > STM32FLASH_END)
		result = flash_addr_err;

	if (pbuf == NULL)
		result = flash_use_null_buf;

	if (!size)
		result = flah_len_err;

	return result;
#undef ONCHIP_FLASH_BYTE_ALIGNED
}

/**
 * @brief	片内flash初始化
 * @details
 * @param	None
 * @retval  None
 */
void onchip_flash_init(void)
{
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	HAL_FLASH_Lock();
}

/**
 * @brief	片内flash读
 * @details
 * @param	addr 片内flash写入地址
 * @param   pbuf 目标数据指针
 * @param   size 片内flash写入的数据长度
 * @retval  flash_result：0成功 非0错误
 */
flash_result onchip_flash_read(uint32_t addr,
							   uint8_t *pbuf,
							   size_t size)
{
	/* flash地址检查地址 */
	if (onchip_flash_operate_check(addr, (uint16_t *)pbuf, size) != flash_ok)
		return flash_fail;

	// for (size_t i = 0; i < size; i++)
	// {
	// 	pbuf[i] = *(__IO uint16_t *)addr;
	// 	addr += 1U;
	// }

	for (uint8_t *p = pbuf; p < pbuf + size;)
	{
		*p++ = *(__IO uint16_t *)addr++;
	}

	return flash_ok;
}

/**
 * @brief	片内flash写
 * @details
 * @param	addr 片内flash写入地址
 * @param   pbuf 目标数据指针
 * @param   size 片内flash写入的数据长度
 * @retval  flash_result：0成功 非0错误
 */
static flash_result onchip_flash_write(uint32_t addr,
									   const uint16_t *pbuf,
									   size_t size)
{
	/* flash地址检查地址 */
	if (onchip_flash_operate_check(addr, pbuf, size) != flash_ok)
		return flash_fail;

	/*1、解锁FLASH*/
	HAL_FLASH_Unlock();
	/*2、清除flash标志位*/
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	/*3、数据写入目标地址并校验*/
	for (size_t i = 0; i < (size / 2U); i++) // size是按字节，写入是按半字
	{
		uint16_t target_data = *(pbuf + i);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i * 2, target_data);
		/*写完数据之后，再读出来校验*/
		if ((*(__IO uint16_t *)(addr + i * 2) != target_data))
		{
			return flash_write_err;
		}
	}
	/*4、锁住FLASH*/
	HAL_FLASH_Lock();

	return flash_ok;
}

/**
 * @brief	片内flash擦除
 * @details
 * @param	type 片内flash擦除类型
 * @param   addr 片内flash擦除地址
 * @param   nb_pages 片内flash擦除页数
 * @retval  成功：0xFFFFFFFF;失败：错误地址
 */
static size_t onchip_flash_erase(uint32_t type,
								 size_t addr,
								 uint32_t nb_pages)
{
	/*初始化FLASH_EraseInitTypeDef*/
	FLASH_EraseInitTypeDef erase = {
		.TypeErase = type,
		.PageAddress = addr,
		.NbPages = nb_pages,
	};
	/*设置PageError*/
	size_t erase_error = ONCHIP_FLASH_OK_CODE;

	/*1、解锁FLASH*/
	HAL_FLASH_Unlock();
	/*2、清除flash标志位*/
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	/*3、擦除FLASH:数据擦除成功：返回0xffffffff,否则错误地址*/
	HAL_FLASHEx_Erase(&erase, &erase_error);
	/*4、锁住FLASH*/
	HAL_FLASH_Lock();

	return erase_error;
}

/**
 * @brief	片内flash操作
 * @details
 * @param	ptsdb 时序数据库句柄
 * @param   his_data
 * @retval  none
 */
flash_result operate_onchip_flash(uint32_t addr,
								  const uint16_t *pbuf,
								  uint32_t size)
{
	flash_result result = flash_ok;

	if (onchip_flash_erase(FLASH_TYPEERASE_PAGES, addr, 1U) != ONCHIP_FLASH_OK_CODE)
		result = flash_erase_err;
	result = onchip_flash_write(addr, pbuf, size);

	return result;
}

static int init(void)
{
	/* do nothing now */
	onchip_flash_init();
	return 0;
}

static int read(long offset, uint8_t *buf, size_t size)
{
	uint32_t addr = stm32_onchip_flash.addr + offset;

	if (onchip_flash_read(addr, buf, size) != flash_ok)
		return -1;

	return size;
}

static int write(long offset, const uint8_t *buf, size_t size)
{
	uint32_t addr = stm32_onchip_flash.addr + offset;

	if (onchip_flash_write(addr, (const uint16_t *)buf, size) != flash_ok)
		return -1;

	return size;
}

static int erase(long offset, size_t size)
{
	UNUSED(size);
	uint32_t addr = stm32_onchip_flash.addr + offset;

	if (onchip_flash_erase(FLASH_TYPEERASE_PAGES, addr, 1U) != ONCHIP_FLASH_OK_CODE)
		return -1;

	return size;
}

const struct fal_flash_dev stm32_onchip_flash = {

	.name = "stm32_onchip",
	.addr = 0x08000000,
	.len = 512 * 1024,
	.blk_size = 2 * 1024,
	.ops = {init, read, write, erase},
	.write_gran = 32,
};
