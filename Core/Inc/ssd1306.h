/* ssd1306.h */
#ifndef __SSD1306_H
#define __SSD1306_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h" // Change this based on your specific STM32 series if needed
#include <stdint.h>
#include <stddef.h>

/* Exported types ------------------------------------------------------------*/
// SSD1306 Color definition (monochrome)
typedef enum {
    Black = 0x00, // Pixel OFF
    White = 0x01  // Pixel ON
} SSD1306_COLOR;

// SSD1306 State structure
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} SSD1306_t;

// Font definition structure
typedef struct {
    const uint8_t FontWidth;    /*!< Font width in pixels */
    uint8_t FontHeight;   /*!< Font height in pixels */
    // CORRECTED: Pointer should be to uint8_t for typical font data
    const uint8_t *data; /*!< Pointer to font data array */
} FontDef;

/* Exported constants --------------------------------------------------------*/
// IMPORTANT: Verify this I2C address is correct for your module (usually 0x3C or 0x3D)
#define SSD1306_I2C_ADDR        (0x3C << 1) // 7-bit address 0x3C shifted left for HAL
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64         // Or 32 if using a 128x32 display
#define SSD1306_I2C_TIMEOUT     100        // Adjust timeout as needed (ms)

// Optional: Define for error reporting
// #define SSD1306_DEBUG_LOG // Uncomment to enable printf logging for errors

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 * @brief Initializes the SSD1306 OLED display.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure.
 * @retval 0 on success, 1 on error (I2C communication failure).
 */
uint8_t ssd1306_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Fills the entire screen buffer with a specified color.
 * @param color Color to fill with (Black or White).
 */
void ssd1306_Fill(SSD1306_COLOR color);

/**
 * @brief Sends the screen buffer content to the OLED display RAM.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure.
 * @retval 0 on success, 1 on error (I2C communication failure).
 */
uint8_t ssd1306_UpdateScreen(I2C_HandleTypeDef *hi2c);

/**
 * @brief Draws a single pixel at the specified coordinates in the buffer.
 * @param x X coordinate (0 to SSD1306_WIDTH - 1).
 * @param y Y coordinate (0 to SSD1306_HEIGHT - 1).
 * @param color Pixel color (Black or White).
 */
void ssd1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR color);

/**
 * @brief Sets the current cursor position for text writing.
 * @param x X coordinate.
 * @param y Y coordinate.
 */
void ssd1306_SetCursor(uint16_t x, uint16_t y);

/**
 * @brief Writes a single character to the screen buffer at the current cursor position using the specified font.
 * @note This version assumes font data is W bytes per character, each byte representing a column of 8 pixels.
 * It will only draw up to Font.FontHeight pixels vertically, clipping if FontHeight > 8 and data is only 8 bits high per column.
 * @param ch Character to write.
 * @param Font Font definition structure.
 * @param color Character color (Black or White).
 * @return The character written, or 0 if there was not enough space.
 */
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);

/**
 * @brief Writes a string to the screen buffer at the current cursor position using the specified font.
 * @note Handles newline ('\n') character for line wrapping.
 * @param str Pointer to the string to write.
 * @param Font Font definition structure.
 * @param color String color (Black or White).
 * @return The last character successfully written, or 0 if the string is empty or writing failed early.
 */
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color);

/* Exported font variables -------------------------------------------------- */
// Ensure these are defined in your C file (or a separate font file)
extern FontDef Font_7x10;
// extern FontDef Font_11x18; // Uncomment if you have these fonts
// extern FontDef Font_16x26;

#ifdef __cplusplus
}
#endif

#endif /* __SSD1306_H */
