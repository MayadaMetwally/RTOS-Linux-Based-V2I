#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdint.h>
#include "LCD.h"

extern osMutexId LcdHandle;

// Helper functions for sending commands/data
void LCD_Enable(void);
void LCD_Write8Bits(uint8_t value);

void LCD_Init(void) {
    LCD_SemaphoreWait();

    // Initialize the LCD to 8-bit mode
    osDelay(40); // Wait for more than 40ms after VDD rises to 2.7V
    LCD_Send_Command(0b00111000);
    LCD_Send_Command(0b00001111);

    LCD_Send_Command(1);

   LCD_SemaphoreRelease();
}

void LCD_Send_Command(uint8_t cmd) {

    LCD_Send(cmd, 0);

}

void LCD_Send_Data(uint8_t data) {

    LCD_Send(data, 1);

}

void LCD_Send(uint8_t value, uint8_t mode) {
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, mode);
    HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
    LCD_Write8Bits(value);
}

void LCD_Write8Bits(uint8_t value) {
    HAL_GPIO_WritePin(LCD_D0_GPIO_Port, LCD_D0_Pin, (value >> 0) & 0x01);
    HAL_GPIO_WritePin(LCD_D1_GPIO_Port, LCD_D1_Pin, (value >> 1) & 0x01);
    HAL_GPIO_WritePin(LCD_D2_GPIO_Port, LCD_D2_Pin, (value >> 2) & 0x01);
    HAL_GPIO_WritePin(LCD_D3_GPIO_Port, LCD_D3_Pin, (value >> 3) & 0x01);
    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, (value >> 4) & 0x01);
    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, (value >> 5) & 0x01);
    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, (value >> 6) & 0x01);
    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, (value >> 7) & 0x01);
    LCD_Enable();
}

void LCD_Enable(void) {
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_SET);
    osDelay(2); // Enable pulse must be >450ns
    HAL_GPIO_WritePin(LCD_EN_GPIO_Port, LCD_EN_Pin, GPIO_PIN_RESET);

}

void LCD_Clear(void) {
    LCD_SemaphoreWait();
    LCD_Send_Command(0x01); // Clear display
    osDelay(2); // Clearing the display takes 1.53ms
    LCD_SemaphoreRelease();
}

void LCD_Set_Cursor(uint8_t row, uint8_t col) {
    uint8_t address;
    switch(row) {
        case 0: address = 0x00 + col; break;
        case 1: address = 0x40 + col; break;
        default: address = 0x00 + col; break;
    }
    LCD_SemaphoreWait();
    LCD_Send_Command(0x80 | address);
    LCD_SemaphoreRelease();
}

void LCD_Write_String(char* str) {
   LCD_SemaphoreWait();
    while(*str) {
        LCD_Send_Data(*str++);
    }
  LCD_SemaphoreRelease();
}

void LCD_SemaphoreWait(void) {
	osMutexWait(LcdHandle, osWaitForever);

}

void LCD_SemaphoreRelease(void) {
	osMutexRelease(LcdHandle);
}
