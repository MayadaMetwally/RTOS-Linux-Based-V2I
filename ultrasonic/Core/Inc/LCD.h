


#ifndef __LCD_HAL_H
#define __LCD_HAL_H

#include "LCD_Config.h"
#include "cmsis_os.h"       // Include FreeRTOS header



// Declare semaphore handle
extern osSemaphoreId lcdSemaphoreHandle;

// Function prototypes
void LCD_Init(void);
void LCD_Send_Command(uint8_t cmd);
void LCD_Send_Data(uint8_t data);
void LCD_Write_String(char* str);
void LCD_Clear(void);
void LCD_Set_Cursor(uint8_t row, uint8_t col);

// Internal functions for semaphore handling
void LCD_SemaphoreWait(void);
void LCD_SemaphoreRelease(void);

#endif // __LCD_HAL_H

