

/* Includes ------------------------------------------------------------------*/
#include "ssd1306.h"
#include <string.h> // For memset
#include <math.h>   // For potential future shape drawing
#ifdef SSD1306_DEBUG_LOG
#include <stdio.h>  // For printf debugging
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SSD1306_CMD  0x00  // I2C Control byte for commands
#define SSD1306_DATA 0x40  // I2C Control byte for data

/* Private macro -------------------------------------------------------------*/
// Macro for debug logging (optional)
#ifdef SSD1306_DEBUG_LOG
#define SSD1306_LOG(...) printf(__VA_ARGS__)
#else
#define SSD1306_LOG(...)
#endif

/* Private variables ---------------------------------------------------------*/
// Screen buffer: SSD1306 is page-organized. Buffer stores pixel data.
// Size: 128 * (64 / 8) = 1024 bytes for a 128x64 display.
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// SSD1306 state instance
static SSD1306_t SSD1306;

/* --- Font Data --- */
// Font 7x10 Data (Assuming 7 bytes per character, each byte is a column of 8 pixels)
// The '10' in the name might refer to total height including spacing,
// but the data likely only defines 8 rows of pixels.
const uint8_t Font7x10_Table[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sp ' ' (32)
    0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, // !
    0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, // "
    0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, // #
    0x00, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00, // $
    0x00, 0x23, 0x13, 0x08, 0x64, 0x62, 0x00, // %
    0x00, 0x36, 0x49, 0x55, 0x22, 0x50, 0x00, // &
    0x00, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, // '
    0x00, 0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, // (
    0x00, 0x00, 0x41, 0x22, 0x1C, 0x00, 0x00, // )
    0x00, 0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00, // *
    0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, // +
    0x00, 0x00, 0xA0, 0x60, 0x00, 0x00, 0x00, // ,
    0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, // -
    0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, // .
    0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, // /
    0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, // 0
    0x00, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, // 1
    0x00, 0x42, 0x61, 0x51, 0x49, 0x46, 0x00, // 2
    0x00, 0x21, 0x41, 0x45, 0x4B, 0x31, 0x00, // 3
    0x00, 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, // 4
    0x00, 0x27, 0x45, 0x45, 0x45, 0x39, 0x00, // 5
    0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, // 6
    0x00, 0x01, 0x71, 0x09, 0x05, 0x03, 0x00, // 7
    0x00, 0x36, 0x49, 0x49, 0x49, 0x36, 0x00, // 8
    0x00, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, // 9
    0x00, 0x00, 0x36, 0x36, 0x00, 0x00, 0x00, // :
    0x00, 0x00, 0xAC, 0x6C, 0x00, 0x00, 0x00, // ;
    0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00, // <
    0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, // =
    0x00, 0x00, 0x41, 0x22, 0x14, 0x08, 0x00, // >
    0x00, 0x02, 0x01, 0x51, 0x09, 0x06, 0x00, // ?
    0x00, 0x32, 0x49, 0x79, 0x41, 0x3E, 0x00, // @
    0x00, 0x7E, 0x09, 0x09, 0x09, 0x7E, 0x00, // A
    0x00, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, // B
    0x00, 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, // C
    0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00, // D
    0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, // E
    0x00, 0x7F, 0x09, 0x09, 0x01, 0x01, 0x00, // F
    0x00, 0x3E, 0x41, 0x41, 0x51, 0x72, 0x00, // G
    0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, // H
    0x00, 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, // I
    0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, // J
    0x00, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, // K
    0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, // L
    0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00, // M
    0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, // N
    0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, // O
    0x00, 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, // P
    0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, // Q
    0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, // R
    0x00, 0x46, 0x49, 0x49, 0x49, 0x31, 0x00, // S
    0x00, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00, // T
    0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, // U
    0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, // V
    0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00, // W
    0x00, 0x63, 0x14, 0x08, 0x14, 0x63, 0x00, // X
    0x00, 0x03, 0x04, 0x78, 0x04, 0x03, 0x00, // Y
    0x00, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, // Z
    0x00, 0x00, 0x7F, 0x41, 0x41, 0x00, 0x00, // [
    0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, // \ backslash
    0x00, 0x00, 0x41, 0x41, 0x7F, 0x00, 0x00, // ]
    0x00, 0x04, 0x02, 0x01, 0x02, 0x04, 0x00, // ^
    0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, // _
    0x00, 0x00, 0x01, 0x02, 0x04, 0x00, 0x00, // `
    0x00, 0x20, 0x54, 0x54, 0x54, 0x78, 0x00, // a
    0x00, 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00, // b
    0x00, 0x38, 0x44, 0x44, 0x44, 0x20, 0x00, // c
    0x00, 0x38, 0x44, 0x44, 0x48, 0x7F, 0x00, // d
    0x00, 0x38, 0x54, 0x54, 0x54, 0x18, 0x00, // e
    0x00, 0x08, 0x7E, 0x09, 0x01, 0x02, 0x00, // f
    0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C, 0x00, // g ('g' might exceed 8 pixels in some fonts)
    0x00, 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, // h
    0x00, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x00, // i
    0x00, 0x40, 0x80, 0x84, 0x7D, 0x00, 0x00, // j ('j' might exceed 8 pixels)
    0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, // k
    0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, 0x00, // l
    0x00, 0x7C, 0x04, 0x18, 0x04, 0x78, 0x00, // m
    0x00, 0x7C, 0x08, 0x04, 0x04, 0x78, 0x00, // n
    0x00, 0x38, 0x44, 0x44, 0x44, 0x38, 0x00, // o
    0x00, 0xFC, 0x24, 0x24, 0x24, 0x18, 0x00, // p ('p' might exceed 8 pixels)
    0x00, 0x18, 0x24, 0x24, 0x18, 0xFC, 0x00, // q ('q' might exceed 8 pixels)
    0x00, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x00, // r
    0x00, 0x48, 0x54, 0x54, 0x54, 0x20, 0x00, // s
    0x00, 0x04, 0x3F, 0x44, 0x40, 0x20, 0x00, // t
    0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00, // u
    0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00, // v
    0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00, // w
    0x00, 0x44, 0x28, 0x10, 0x28, 0x44, 0x00, // x
    0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C, 0x00, // y ('y' might exceed 8 pixels)
    0x00, 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, // z
    0x00, 0x08, 0x36, 0x41, 0x41, 0x00, 0x00, // {
    0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, // |
    0x00, 0x00, 0x41, 0x41, 0x36, 0x08, 0x00, // }
    0x00, 0x02, 0x01, 0x02, 0x01, 0x00, 0x00  // ~
};

// Define the FontDef structure instance for Font_7x10
// NOTE: FontHeight is 10, but the data table above and the drawing function
//       will likely only use the first 8 rows of pixels per column byte.
FontDef Font_7x10 = { 7, 10, Font7x10_Table };

// Define other fonts if you have them and uncomment the extern declarations in the header
// FontDef Font_11x18 = { 11, 18, Font11x18_Table };
// FontDef Font_16x26 = { 16, 26, Font16x26_Table };

/* Private function prototypes -----------------------------------------------*/
// Made static as they are internal helper functions
static uint8_t ssd1306_WriteCommand(I2C_HandleTypeDef *hi2c, uint8_t command);
static uint8_t ssd1306_WriteData(I2C_HandleTypeDef *hi2c, uint8_t* data, size_t size);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Sends a command byte to SSD1306.
 * @param hi2c Pointer to I2C handle.
 * @param command Command byte to send.
 * @retval 0 on success, 1 on error.
 */
static uint8_t ssd1306_WriteCommand(I2C_HandleTypeDef *hi2c, uint8_t command) {
    if (HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, SSD1306_CMD, 1, &command, 1, SSD1306_I2C_TIMEOUT) != HAL_OK) {
        SSD1306_LOG("I2C Error: Write Command 0x%02X failed\n", command);
        return 1; // Error
    }
    return 0; // Success
}

/**
 * @brief Sends data bytes to SSD1306.
 * @param hi2c Pointer to I2C handle.
 * @param data Pointer to data buffer.
 * @param size Number of bytes to send.
 * @retval 0 on success, 1 on error.
 */
static uint8_t ssd1306_WriteData(I2C_HandleTypeDef *hi2c, uint8_t* data, size_t size) {
    if (HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, SSD1306_DATA, 1, data, size, SSD1306_I2C_TIMEOUT) != HAL_OK) {
         SSD1306_LOG("I2C Error: Write Data (%d bytes) failed\n", size);
        return 1; // Error
    }
    return 0; // Success
}

/**
 * @brief Initializes the SSD1306 OLED display.
 */
uint8_t ssd1306_Init(I2C_HandleTypeDef *hi2c) {
    // Wait for I2C device ready (important!)
    if (HAL_I2C_IsDeviceReady(hi2c, SSD1306_I2C_ADDR, 3, SSD1306_I2C_TIMEOUT) != HAL_OK) {
         SSD1306_LOG("I2C Error: Device not ready at address 0x%02X\n", SSD1306_I2C_ADDR);
        return 1; // Device not found or communication error
    }

    // Short delay after power-on can sometimes help
    HAL_Delay(100);

    // Initialization Sequence
    if (ssd1306_WriteCommand(hi2c, 0xAE)) return 1; // Display OFF

    if (ssd1306_WriteCommand(hi2c, 0xD5)) return 1; // Set Display Clock Divide Ratio/Oscillator Frequency
    if (ssd1306_WriteCommand(hi2c, 0x80)) return 1; // Default Ratio 0x80

    if (ssd1306_WriteCommand(hi2c, 0xA8)) return 1; // Set MUX Ratio
    if (ssd1306_WriteCommand(hi2c, SSD1306_HEIGHT - 1)) return 1; // Height - 1 (e.g., 63 for 128x64)

    if (ssd1306_WriteCommand(hi2c, 0xD3)) return 1; // Set Display Offset
    if (ssd1306_WriteCommand(hi2c, 0x00)) return 1; // No offset

    if (ssd1306_WriteCommand(hi2c, 0x40 | 0x0)) return 1; // Set Display Start Line (0)

    if (ssd1306_WriteCommand(hi2c, 0x8D)) return 1; // Charge Pump Setting
    if (ssd1306_WriteCommand(hi2c, 0x14)) return 1; // Enable Charge Pump (0x10 to disable if VCC is external)

    if (ssd1306_WriteCommand(hi2c, 0x20)) return 1; // Set Memory Addressing Mode
    if (ssd1306_WriteCommand(hi2c, 0x00)) return 1; // 0x00 = Horizontal Addressing Mode

    // Set Segment Re-map (controls left-right orientation)
    // 0xA1 mirrors horizontally (column 127 is mapped to SEG0)
    // 0xA0 is normal (column 0 is mapped to SEG0)
    if (ssd1306_WriteCommand(hi2c, 0xA1)) return 1; // Choose 0xA0 or 0xA1 based on your wiring/module

    // Set COM Output Scan Direction (controls up-down orientation)
    // 0xC8 mirrors vertically (scan from COM[N-1] down to COM0)
    // 0xC0 is normal (scan from COM0 up to COM[N-1])
    if (ssd1306_WriteCommand(hi2c, 0xC8)) return 1; // Choose 0xC0 or 0xC8 based on your wiring/module

    if (ssd1306_WriteCommand(hi2c, 0xDA)) return 1; // Set COM Pins Hardware Configuration
    if (SSD1306_HEIGHT == 32) {
        if (ssd1306_WriteCommand(hi2c, 0x02)) return 1; // Config for 128x32
    } else { // Default to 64
         if (ssd1306_WriteCommand(hi2c, 0x12)) return 1; // Config for 128x64
    }

    if (ssd1306_WriteCommand(hi2c, 0x81)) return 1; // Contrast Control
    if (ssd1306_WriteCommand(hi2c, 0xCF)) return 1; // Set Contrast Value (adjust as needed 0x00-0xFF)

    if (ssd1306_WriteCommand(hi2c, 0xD9)) return 1; // Set Pre-charge Period
    if (ssd1306_WriteCommand(hi2c, 0xF1)) return 1; // Recommended value

    if (ssd1306_WriteCommand(hi2c, 0xDB)) return 1; // Set VCOMH Deselect Level
    if (ssd1306_WriteCommand(hi2c, 0x40)) return 1; // Recommended value 0x40 (~0.77 VCC)

    if (ssd1306_WriteCommand(hi2c, 0xA4)) return 1; // Set Entire Display ON/OFF (A4=Output follows RAM, A5=Output ignores RAM)
    if (ssd1306_WriteCommand(hi2c, 0xA6)) return 1; // Set Normal/Inverse Display (A6=Normal, A7=Inverse)

    // Clear screen buffer
    ssd1306_Fill(Black);
    // Send buffer to screen RAM (initial clear)
    // No need to check return here, as failure likely means init already failed
    ssd1306_UpdateScreen(hi2c); // Ignore return value here, focus on init commands

    // Turn display ON
    if (ssd1306_WriteCommand(hi2c, 0xAF)) return 1;

    // Initialize state
    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;
    SSD1306.Initialized = 1;
    SSD1306.Inverted = 0; // Default to non-inverted

    SSD1306_LOG("SSD1306 Initialized Successfully.\n");
    return 0; // Initialization successful
}

/**
 * @brief Fills the entire screen buffer with a specified color.
 */
void ssd1306_Fill(SSD1306_COLOR color) {
    uint8_t fill_byte = (color == Black) ? 0x00 : 0xFF;
    memset(SSD1306_Buffer, fill_byte, sizeof(SSD1306_Buffer));
}

/**
 * @brief Sends the screen buffer content to the OLED display RAM.
 */
uint8_t ssd1306_UpdateScreen(I2C_HandleTypeDef *hi2c) {
    if (!SSD1306.Initialized) return 1; // Don't update if not initialized

    // Set Column Address Range (0 - 127)
    if (ssd1306_WriteCommand(hi2c, 0x21)) return 1; // Set column address
    if (ssd1306_WriteCommand(hi2c, 0)) return 1;    // Column start address (0)
    if (ssd1306_WriteCommand(hi2c, SSD1306_WIDTH - 1)) return 1; // Column end address (127)

    // Set Page Address Range (0 - 7 for 64 high, 0-3 for 32 high)
    if (ssd1306_WriteCommand(hi2c, 0x22)) return 1; // Set page address
    if (ssd1306_WriteCommand(hi2c, 0)) return 1;    // Page start address (0)
    if (ssd1306_WriteCommand(hi2c, (SSD1306_HEIGHT / 8) - 1)) return 1; // Page end address

    // Send the entire buffer data
    if (ssd1306_WriteData(hi2c, SSD1306_Buffer, sizeof(SSD1306_Buffer))) {
        return 1; // I2C data transfer failed
    }

    return 0; // Success
}

/**
 * @brief Draws a single pixel at the specified coordinates in the buffer.
 */
void ssd1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR color) {
    // Check boundaries
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return; // Out of bounds
    }

    // Handle inversion if needed
    if (SSD1306.Inverted) {
        color = (SSD1306_COLOR)!color;
    }

    // Calculate buffer index and bit mask
    // Buffer is organized as horizontal pages of 8 rows
    uint16_t byte_index = x + (y / 8) * SSD1306_WIDTH;
    uint8_t bit_mask = 1 << (y % 8);

    // Modify buffer
    if (color == White) {
        SSD1306_Buffer[byte_index] |= bit_mask;  // Set bit
    } else {
        SSD1306_Buffer[byte_index] &= ~bit_mask; // Clear bit
    }
}

/**
 * @brief Sets the current cursor position for text writing.
 */
void ssd1306_SetCursor(uint16_t x, uint16_t y) {
    SSD1306.CurrentX = (x < SSD1306_WIDTH) ? x : SSD1306_WIDTH -1;
    SSD1306.CurrentY = (y < SSD1306_HEIGHT) ? y : SSD1306_HEIGHT -1;
}


/**
 * @brief Writes a single character to the screen buffer (Modified Logic).
 */
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color) {
    // Handle non-printable ASCII characters or replace with space
    if (ch < ' ' || ch > '~') {
        ch = ' '; // Replace unsupported characters with a space
    }

    // Check remaining space on screen
    if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
        SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight)) {
        // Not enough space to draw the character fully
        SSD1306_LOG("WriteChar Error: Not enough space for char '%c' at (%d, %d)\n", ch, SSD1306.CurrentX, SSD1306.CurrentY);
        return 0; // Indicate failure (no space)
    }

    // Calculate offset in the font data array
    // Assumes font data is Width bytes per char, contiguous for ASCII ' ' through '~'
    // Assumes Font.data is uint8_t* as corrected in header
    const uint8_t* font_char_ptr = &Font.data[(ch - ' ') * Font.FontWidth];

    // Draw character column by column
    for (uint32_t i = 0; i < Font.FontWidth; i++) { // Iterate through columns
        uint8_t column_data = font_char_ptr[i];     // Get the byte representing the column (8 pixels)

        for (uint32_t j = 0; j < 8; j++) {          // Iterate through the 8 pixels in the column byte
            // Check if the pixel is within the defined FontHeight
            if (j >= Font.FontHeight) {
                continue; // Don't draw pixels beyond the font's defined height
            }

            // Calculate target pixel coordinates
            uint16_t target_x = SSD1306.CurrentX + i;
            uint16_t target_y = SSD1306.CurrentY + j;

            // Check if the current pixel bit is set in the font data
            if ((column_data >> j) & 0x01) {
                ssd1306_DrawPixel(target_x, target_y, color); // Draw foreground pixel
            } else {
                // Draw background pixel (important to overwrite previous content)
                ssd1306_DrawPixel(target_x, target_y, (SSD1306_COLOR)!color);
            }
        }
         // Note: If FontHeight > 8, this logic *only* draws the first 8 rows
         //       unless the font data format and this loop are adapted.
         //       For the provided Font7x10_Table, this is likely the intended behavior.
    }

    // Update cursor position for the next character
    SSD1306.CurrentX += Font.FontWidth;

    return ch; // Return the character that was written
}

/**
 * @brief Writes a string to the screen buffer.
 */
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color) {
    char last_char_written = 0;
    while (*str) {
        // Handle newline character '\n'
        if (*str == '\n') {
            SSD1306.CurrentX = 0; // Move cursor to beginning of the line
            SSD1306.CurrentY += Font.FontHeight; // Move cursor down one font height
            // Optional: Implement screen wrap-around or scrolling if Y exceeds height
            if (SSD1306.CurrentY >= SSD1306_HEIGHT) {
                 SSD1306.CurrentY = 0; // Simple wrap-around to top
            }
        }
        // Handle carriage return '\r' (often ignored or treated like \n)
        else if (*str == '\r') {
             SSD1306.CurrentX = 0; // Move cursor to beginning of the line
        }
        // Write normal character
        else {
            char written_char = ssd1306_WriteChar(*str, Font, color);
            if (written_char == 0) {
                // Writing failed (likely out of space), stop processing the string
                 SSD1306_LOG("WriteString Error: Failed to write char '%c', stopping.\n", *str);
                return last_char_written; // Return the last char that *was* successfully written
            }
            last_char_written = written_char;
        }
        str++; // Move to the next character in the string
    }
    return last_char_written; // Return the last character written from the string
}

// --- Add other drawing functions here if needed ---
// void ssd1306_DrawLine(...)
// void ssd1306_DrawRectangle(...)
// void ssd1306_DrawBitmap(...)
