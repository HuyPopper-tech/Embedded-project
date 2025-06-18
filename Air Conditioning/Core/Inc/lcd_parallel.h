#ifndef _LCD_PARALLEL_H_
#define _LCD_PARALLEL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "lcd_config.h"
#include "stm32f4xx.h"

#define LCD_CMD_CLEAR_DISPLAY       0x01
#define LCD_CMD_RETURN_HOME         0x02
#define LCD_CMD_ENTRY_MODE_SET      0x04
#define LCD_CMD_DISPLAY_CONTROL     0x08
#define LCD_CMD_CURSOR_SHIFT        0x10
#define LCD_CMD_FUNCTION_SET        0x20
#define LCD_CMD_SET_DDRAM_ADDR      0x80
#define LCD_CMD_SET_CGRAM_ADDR      0x40

/* flags for display entry mode */
#define LCD_CMD_SET_ENTRY_LEFT      0x02
#define LCD_CMD_SET_ENTRY_RIGHT     0x00
#define LCD_CMD_SET_ENTRY_SHIFT     0x01
#define LCD_CMD_SET_ENTRY_NO_SHIFT  0x00

/* flags for display/cursor shift */
#define LCD_CMD_DISPLAY_MOVE        0x08
#define LCD_CMD_CURSOR_MOVE         0x00
#define LCD_CMD_DISPLAY_MOVE_RIGHT  0x04
#define LCD_CMD_DISPLAY_MOVE_LEFT   0x00

/* flags for function set */
#define LCD_CMD_8BIT_MODE           0x10
#define LCD_CMD_4BIT_MODE           0x00
#define LCD_CMD_2LINE_MODE          0x08
#define LCD_CMD_1LINE_MODE          0x00
#define LCD_CMD_5x10_DOTS           0x04
#define LCD_CMD_5x8_DOTS            0x00

/* Flags for display/cursor/blink on/off control */
typedef enum {
    LCD_DISPLAY_ON = 0x04,
    LCD_DISPLAY_OFF = 0x00,
    LCD_CURSOR_ON = 0x02,
    LCD_CURSOR_OFF = 0x00,
    LCD_BLINK_ON = 0x01,
    LCD_BLINK_OFF = 0x00
} LCD_Display_Settings;

/**
 * @brief Initializes the LCD in 4-bit mode.
 */
void LCD_Init(void);

/**
 * @brief Sends a command to the LCD.
 * @param cmd Command to send.
 */
void LCD_Clear(void);

/**
 * @brief Sends a command to the LCD.
 */
void LCD_Put(char c);

/**
 * @brief Writes a string to the LCD.
 */
void LCD_Write(char *str);

/**
 * @brief Sets the cursor position on the LCD.
 */
void LCD_setCursor(char x, char y);

/**
 * @brief Turns the LCD cursor display on.
 */
void LCD_cursorOn(void);

/**
 * @brief Turns on blink for the LCD cursor.
 */
void LCD_blinkOn(void);

/**
 * @brief Clears the LCD display.
 */
void LCD_clearDisplay(void);

/**
 * @brief Sets the display settings for the LCD.
 */
void LCD_setDisplaySettings(LCD_Display_Settings settings);

#endif /* _LCD_PARALLEL_H_ */
