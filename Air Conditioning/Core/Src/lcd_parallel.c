#include "lcd_parallel.h"
#include <stdint.h>
#include "stm32f4xx.h"
#include "delay.h"

char display_settings;

/**
 * @brief  Send a falling edge to the LCD
 * This function generates a falling edge on the Enable pin of the LCD.
 */
static void fallingEdge(void) {
    uint32_t primask_bit;
    primask_bit = __get_PRIMASK(); // Lưu trạng thái ngắt hiện tại
    __disable_irq(); // <-- TẮT NGẮT TOÀN CỤC

    /* Set Enable pin high */
    LCD_DATA_PORT_B->ODR |= (1 << E_Pin);
    delay_us(2);
    /* Set Enable pin low */
    LCD_DATA_PORT_B->ODR &= ~(1 << E_Pin);

    if (!primask_bit) { // Chỉ bật lại ngắt nếu trước đó nó đã được bật
        __enable_irq();  // <-- BẬT LẠI NGẮT NGAY LẬP TỨC
    }
}

#ifndef LCD8Bit
static void LCD_sendData4Bit(char data) {
    uint32_t primask_bit;
        primask_bit = __get_PRIMASK();
        __disable_irq(); // <-- TẮT NGẮT TOÀN CỤC
    LCD_DATA_PORT_B->ODR &= ~((1 << DATA5_Pin) | (1 << DATA6_Pin));
    LCD_DATA_PORT_A->ODR &= ~((1 << DATA7_Pin) | (1 << DATA8_Pin));

    /* Set data pin 5 */
    if (data & 0x01)
        LCD_DATA_PORT_B->ODR |= (1 << DATA5_Pin);
    /* Set data pin 6 */
    if (data & 0x02)
        LCD_DATA_PORT_B->ODR |= (1 << DATA6_Pin);
    /* Set data pin 7 */
    if (data & 0x04)
        LCD_DATA_PORT_A->ODR |= (1 << DATA7_Pin);
    /* Set data pin 8 */
    if (data & 0x08)
        LCD_DATA_PORT_A->ODR |= (1 << DATA8_Pin);

    // Gọi fallingEdge phiên bản không có disable/enable irq bên trong
        LCD_DATA_PORT_B->ODR |= (1 << E_Pin);
        delay_us(2);
        LCD_DATA_PORT_B->ODR &= ~(1 << E_Pin);

        if (!primask_bit) {
            __enable_irq();
        }
        delay_us(45); // Delay chờ LCD xử lý, để ngoài vùng critical
}
#endif

/**
 * @brief  Send a command to the LCD
 * @param  command: Command to send
 * Sends a control instruction (e.g., clear, set cursor, shift)
 to the LCD by splitting it into two 4-bit transmissions.
 */
static void LCD_sendCommand(char command) {
    /* Clear RS pin for command */
    LCD_DATA_PORT_B->ODR &= ~(1 << RS_Pin);
    /* Send upper nibble*/
    LCD_sendData4Bit(command >> 4);
    /* Send lower nibble */
    LCD_sendData4Bit(command); // Send lower nibble
}

/**
 * @brief  Send data to the LCD
 * @param  data: Data to send
 * @retval None
 * Sends a character to be displayed on the LCD.
 */
static void LCD_sendData(char data) {
    LCD_DATA_PORT_B->ODR |= (1 << RS_Pin); // Set RS pin for data
    LCD_sendData4Bit(data >> 4); // Send upper nibble
    LCD_sendData4Bit(data); // Send lower nibble
}

/**
 * @brief  Turn off display without clearing data
 * Sends command to disable display (Display OFF).
 */
void LCD_Clear(void) {
    LCD_sendCommand(LCD_CMD_CLEAR_DISPLAY);
    delay_ms(2); // Wait for the command to complete
}

/**
 * @brief  Put a character on the LCD
 * @param  c: Character to display
 * @retval None
 * This function sends a character to the LCD.
 */
void LCD_Put(char c) {
    LCD_sendData(c);
}

/**
 * @brief  Write a string to the LCD
 * @param  str: Pointer to the string to display
 * @retval None
 * This function sends a string to the LCD.
 */
void LCD_Write(char *str) {
    while (*str) {
        LCD_sendData(*str++);
    }
}

/**
 * @brief  Initialize the LCD in 4-bit mode
 * Sends the required startup sequence and configuration commands to prepare the LCD for operation.
 */
void LCD_Init(void) {
    /* Clear RS pin for command */
    LCD_DATA_PORT_B->ODR &= ~(1 << RS_Pin);
    /* Clear Enable pin */
    LCD_DATA_PORT_B->ODR &= ~(1 << E_Pin);
    delay_ms(50);

#if 1
    display_settings =
    LCD_CMD_4BIT_MODE | LCD_CMD_2LINE_MODE | LCD_CMD_5x8_DOTS;
    LCD_sendData4Bit(0x03);
    delay_ms(5);
    LCD_sendData4Bit(0x03);
    delay_us(150);
    LCD_sendData4Bit(0x03);
    delay_us(50);
    LCD_sendData4Bit(0x02);
    delay_us(50);
#endif
    LCD_sendCommand(LCD_CMD_FUNCTION_SET | display_settings);
    delay_ms(1);
    display_settings |= LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF;
    LCD_sendCommand(LCD_CMD_DISPLAY_CONTROL | display_settings);
    delay_us(50);

    LCD_Clear();
    display_settings |= LCD_CMD_SET_ENTRY_LEFT | LCD_CMD_SET_ENTRY_NO_SHIFT;
    LCD_sendCommand(LCD_CMD_ENTRY_MODE_SET | display_settings);
    delay_us(50);
}

/**
 * @brief  Set cursor to specified column and row
 * @param  x: Column position (0–15)
 * @param  y: Row position (0: top row, 1: bottom row)
 * @retval None
 * Moves the LCD cursor to the given (x, y) position.
 */
void LCD_setCursor(char x, char y) {
    uint8_t address;
    if (y == 0) {
        /* First line */
        address = 0x00 + x;
    } else {
        /* Second line */
        address = 0x40 + x;
    }
    /* Set DDRAM address */
    LCD_sendCommand(LCD_CMD_SET_DDRAM_ADDR | address);
}

/**
 * @brief  Turn on the cursor
 * This function turns on the cursor on the LCD.
 */
void LCD_cursorOn(void) {
    LCD_sendCommand(LCD_CMD_DISPLAY_CONTROL | 0x04 | 0x02);
}

/**
 * @brief  Turn on the blinking cursor
 * This function turns on the blinking cursor on the LCD.
 */
void LCD_blinkOn(void) {
    LCD_sendCommand(LCD_CMD_DISPLAY_CONTROL | 0x04 | 0x01);
}

/**
 * @brief Clears the cursor and the blink bits.
 */
void LCD_clearDisplay(void) {
    LCD_sendCommand(0x08 | 0x04 | 0x00);
}

/**
 * @brief  Set display settings
 * @param  settings: Display settings (ON/OFF, cursor, blink)
 * @retval None
 * This function sets the display settings for the LCD.
 */
void LCD_setDisplaySettings(LCD_Display_Settings settings) {
    LCD_sendCommand(LCD_CMD_DISPLAY_CONTROL | (settings & 0x07));
}

