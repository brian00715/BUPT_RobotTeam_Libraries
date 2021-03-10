#ifndef __FLASH_H
#define __FLASH_H
// 详细解释见外部文档

#include "simplelib_cfg.h"
#include "stdint.h"

#ifdef SLIB_USE_FLASH

#define u32 uint32_t
#define u16 uint16_t
#define u8 uint8_t
#define vu32 volatile unsigned long
#define vu16 volatile uint16_t
#define vu8 volatile uint8_t
u32 STMFLASH_ReadWord_Inc(u32* faddr);
u16 STMFLASH_ReadHalfWord_Inc(u32* faddr);
u8 STMFLASH_ReadByte_Inc(u32* faddr);
float STMFLASH_ReadFloat_Inc(u32* faddr);
u32 STMFLASH_ReadWord(u32 faddr);
u16 STMFLASH_ReadHalfWord(u32 faddr);
u8 STMFLASH_ReadByte(u32 faddr);
float STMFLASH_ReadFloat(u32 faddr);
//FLASH_Status FLASH_ProgramFloat(uint32_t Address, float Data);
#endif // SLIB_USE_FLASH

#endif
