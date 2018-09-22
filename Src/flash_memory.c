#include "flash_memory.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"


//#define DATA_16         ((uint16_t)0x1234)

//uint32_t  startAddress , endAddress ;
uint32_t data16 = 0 , MemoryProgramStatus = 0;
//volatile FLASH_Status FLASHStatus=FLASH_BUSY;
//FLASHд�����ݲ���
/*void WriteFlash(uint16_t DATA_16)
{
    //1.����FLASH
  HAL_FLASH_Unlock();

    //2.����FLASH
    //��ʼ��FLASH_EraseInitTypeDef
    FLASH_EraseInitTypeDef f;
    f.TypeErase = TYPEERASE_SECTORS;
    f.VoltageRange= VOLTAGE_RANGE_3;
    f.Sector=FLASH_SECTOR_5;
		f.NbSectors=1;
    //���ò�������,SectorError�洢������Ϣ
		uint32_t SectorError=0;
    HAL_FLASHEx_Erase(&f, &SectorError);

    //3.��FLASH��д
	startAddress = 0x08020000;
	endAddress = 0x0803FFFF;
   while (startAddress < endAddress)
  {
    if (HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, startAddress, DATA_16) == HAL_OK)
    {
      startAddress = startAddress + 2;
			//32bits��data��ռ��4���ֽڣ�����4
			//16bits��data��ռ��2���ֽڣ�����2
    }

  }
    //4.��סFLASH
  HAL_FLASH_Lock();
	
}

//FLASH��ȡ���ݲ���
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
	 //1.����FLASH
   HAL_FLASH_Unlock();

   //2.����FLASH
   //��ʼ��FLASH_EraseInitTypeDef
   FLASH_EraseInitTypeDef f;
   f.TypeErase = TYPEERASE_SECTORS;
   f.VoltageRange= VOLTAGE_RANGE_3;
   f.Sector=FLASH_SECTOR_5;
	 f.NbSectors=1;
	 //���ò�������,SectorError�洢������Ϣ
	 uint32_t SectorError=0;
   HAL_FLASHEx_Erase(&f, &SectorError);
	HAL_FLASH_Lock();
	//debug
	//PrintFlash();
}

void WriteFlash(uint32_t addr,uint16_t DATA_16)
{
	//��д�ĺ���
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

	  //3.��FLASH��д
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
			//32bits��data��ռ��4���ֽڣ�����4
			//16bits��data��ռ��2���ֽڣ�����2
    //}
    //4.��סFLASH
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















