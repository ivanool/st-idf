


static const char* TAG = "ST7796S_LIB";
#define CMD_MODE  0
#define DATA_MODE 1
#define NOP         0x00
#define SWRESET     0x01
#define SLPOUT      0x11
#define NORON       0x13
#define INVOFF      0x20
#define INVON       0x21
#define DISPON      0x29
#define CASET       0x2A
#define RASET       0x2B
#define RAMWR       0x2C
#define COLMOD      0x3A
#define MADCTL      0x36
#define PORCTRL     0xB2
#define GCTRL       0xB7
#define VCOMS       0xBB
#define ST7796S


#ifdef ST7796S
    #define USED_SCREEN "ST7796S"
    #define TFT_CS    1    // CS - Chip Select
    #define TFT_DC    2    // DC - Data/Command
    #define TFT_RST   3    // RST - Reset
    #define TFT_SCLK  7    // SCL - SPI Clock (ya configurado en ESP32-S3)
    #define TFT_MOSI  9    // SDA - SPI MOSI (ya configurado en ESP32-S3)
    #define TFT_MISO  8    // MISO (ya configurado en ESP32-S3)
    #define TFT_BL    45   // Backlight (cambiado de GPIO 4 a 45 por conflicto)
    #define TFT_WIDTH 480
    #define TFT_HEIGHT 320

#elif defined(ST7789)

    #define TFT_HEIGHT 240
    #define TFT_WIDTH 135

    #endif