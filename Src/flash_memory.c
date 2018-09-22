#include "flash_memory.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"


//#define DATA_16         ((uint16_t)0x1234)

//uint32_t  startAddress , endAddress ;
uint32_t data16 = 0 , MemoryProgramStatus = 0;
//volatile FLASH_Status FLASHStatus=FLASH_BUSY;
//FLASH写入数据测试
/*void WriteFlash(uint16_t DATA_16)
{
    //1.解锁FLASH
  HAL_FLASH_Unlock();

    //2.擦除FLASH
    //初始化FLASH_EraseInitTypeDef
    FLASH_EraseInitTypeDef f;
    f.TypeErase = TYPEERASE_SECTORS;
    f.VoltageRange= VOLTAGE_RANGE_3;
    f.Sector=FLASH_SECTOR_5;
		f.NbSectors=1;
    //调用擦除函数,SectorError存储错误信息
		uint32_t SectorError=0;
    HAL_FLASHEx_Erase(&f, &SectorError);

    //3.对FLASH烧写
	startAddress = 0x08020000;
	endAddress = 0x0803FFFF;
   while (startAddress < endAddress)
  {
    if (HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, startAddress, DATA_16) == HAL_OK)
    {
      startAddress = startAddress + 2;
			//32bits的data，占用4个字节，加上4
			//16bits的data，占用2个字节，加上2
    }

  }
    //4.锁住FLASH
  HAL_FLASH_Lock();
	
}

//FLASH读取数据测试
void PrintFlash(void)
{
	startAddress=0x08020000;
	MemoryProgramStatus=0x0;
  uint32_t temp ;
	while (startAddress < endAddress)
	{
			temp= *(__IO uint16_t*)(startAddress);
			printf("addr:0x%x, data:0x%x\r\n", startAddress, temp);
			startAddress = startAddress + 2;
			//printf("DATA:0x%x \r\n",DATA_16);
	}
}*/
/*void FlashInit(void);
void WriteFlash(uint32_t addr, uint16_t DATA_16);
void PrintFlash(void);
uint16_t ReadFlash(uint32_t addr);*/

void FlashInit(void)
{
	 //1.解锁FLASH
   HAL_FLASH_Unlock();

   //2.擦除FLASH
   //初始化FLASH_EraseInitTypeDef
   FLASH_EraseInitTypeDef f;
   f.TypeErase = TYPEERASE_SECTORS;
   f.VoltageRange= VOLTAGE_RANGE_3;
   f.Sector=FLASH_SECTOR_5;
	 f.NbSectors=1;
	 //调用擦除函数,SectorError存储错误信息
	 uint32_t SectorError=0;
   HAL_FLASHEx_Erase(&f, &SectorError);
	HAL_FLASH_Lock();
	//debug
	//PrintFlash();
}

void WriteFlash(uint32_t addr,uint16_t DATA_16)
{
	//重写的函数
//	HAL_FLASH_Unlock();
//	if (HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, addr, DATA_16) == HAL_OK)
//	{
//		printf("OK   \r\n\n\n\n");
//	}
//	else
//	{
//		printf("AD \r\n");
//	}
//	HAL_FLASH_Lock();

	  //3.对FLASH烧写
	//startAddress = addr;
	//endAddress = 0x0803FFFF;
	 HAL_FLASH_Unlock();
	//printf("data: 0x%x \r\n",DATA_16);
	HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, addr, DATA_16);
	//uint32_t temp ;
	//temp= *(__IO uint16_t*)(addr);
	//printf("Write Flash  addr:0x%x, data:0x%x\r\n", addr, temp);
   // if (HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, addr, DATA_16) == HAL_OK)
   // {
      //addr = addr + 2;
			//32bits的data，占用4个字节，加上4
			//16bits的data，占用2个字节，加上2
    //}
    //4.锁住FLASH
  HAL_FLASH_Lock();
}

uint16_t ReadFlash(uint32_t addr)
{
	//startAddress=0x08020000;
	//MemoryProgramStatus=0x0;
  uint16_t temp ;
	temp= *(__IO uint16_t*)(addr);
	//printf("addr:0x%x, data:0x%x\r\n", addr, temp);
	//startAddress = startAddress + 2;h
	return temp;
	//debug
	//printf("DATA:0x%x \r\n",DATA_16);
}

void PrintFlash(void)
{
	uint32_t sA=0x08020000;
	uint32_t eA=0x0803FFFF;
	//MemoryProgramStatus=0x0;
  uint32_t temp ;
	while (sA < eA)
	{
			temp= *(__IO uint16_t*)(sA);
			printf("addr:0x%x, data:0x%x\r\n", sA, temp);
			sA = sA + 2;
			//printf("DATA:0x%x \r\n",DATA_16);
	}
}















