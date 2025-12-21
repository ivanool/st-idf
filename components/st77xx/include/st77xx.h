/**
 * @file st77xx.h
 * @brief Driver unificado para displays ST7789/ST7796S sobre ESP32
 * 
 * Soporta detección automática de chip, PSRAM, double buffering y modo stripe.
 */

#ifndef ST77XX_H
#define ST77XX_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "sdkconfig.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * Detección automática de chip ESP32
 * ═══════════════════════════════════════════════════════════════════════════ */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    #define ST77XX_CHIP_NAME      "ESP32-S3"
    #define ST77XX_HAS_PSRAM      1
    #define ST77XX_MAX_SPI_SPEED  (80 * 1000 * 1000)
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    #define ST77XX_CHIP_NAME      "ESP32-S2"
    #define ST77XX_HAS_PSRAM      0
    #define ST77XX_MAX_SPI_SPEED  (80 * 1000 * 1000)
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
    #define ST77XX_CHIP_NAME      "ESP32-C3"
    #define ST77XX_HAS_PSRAM      0
    #define ST77XX_MAX_SPI_SPEED  (40 * 1000 * 1000)
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
    #define ST77XX_CHIP_NAME      "ESP32-C6"
    #define ST77XX_HAS_PSRAM      0
    #define ST77XX_MAX_SPI_SPEED  (40 * 1000 * 1000)
#else
    #define ST77XX_CHIP_NAME      "ESP32"
    #define ST77XX_HAS_PSRAM      0
    #define ST77XX_MAX_SPI_SPEED  (40 * 1000 * 1000)
#endif

#if defined(CONFIG_SPIRAM) || defined(CONFIG_ESP32S3_SPIRAM_SUPPORT) || defined(CONFIG_ESP32_SPIRAM_SUPPORT)
    #undef ST77XX_HAS_PSRAM
    #define ST77XX_HAS_PSRAM      1
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Selección de controlador (desde Kconfig o manual)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Si Kconfig está disponible, usar su configuración */
#if defined(CONFIG_ST77XX_MODEL_ST7789)
    #define ST77XX_MODEL_ST7789
#elif defined(CONFIG_ST77XX_MODEL_ST7796S)
    #define ST77XX_MODEL_ST7796S
#endif

/* Fallback: si no hay Kconfig, usar define manual (descomentar uno) */
#if !defined(ST77XX_MODEL_ST7789) && !defined(ST77XX_MODEL_ST7796S)
    // #define ST77XX_MODEL_ST7789
    #define ST77XX_MODEL_ST7796S
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Configuración de pines SPI
 * ═══════════════════════════════════════════════════════════════════════════ */

#if ST77XX_HAS_PSRAM
    #define ST77XX_PIN_CS    1
    #define ST77XX_PIN_DC    2
    #define ST77XX_PIN_RST   3
    #define ST77XX_PIN_SCLK  7
    #define ST77XX_PIN_MOSI  9
    #define ST77XX_PIN_MISO  8
    #define ST77XX_PIN_BL    43
#else
    #define ST77XX_PIN_CS    5
    #define ST77XX_PIN_DC    16
    #define ST77XX_PIN_RST   23
    #define ST77XX_PIN_SCLK  18
    #define ST77XX_PIN_MOSI  19
    #define ST77XX_PIN_MISO  -1
    #define ST77XX_PIN_BL    4
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Configuración por modelo de controlador
 * ═══════════════════════════════════════════════════════════════════════════ */

#if defined(ST77XX_MODEL_ST7789)
    #define ST77XX_WIDTH           240
    #define ST77XX_HEIGHT          135
    #define ST77XX_X_OFFSET        40
    #define ST77XX_Y_OFFSET        52
    #define ST77XX_USE_INVERSION   1
    #define ST77XX_SPI_SPEED_HZ    (40 * 1000 * 1000)
    #define ST77XX_CONTROLLER_NAME "ST7789"
#elif defined(ST77XX_MODEL_ST7796S)
    #define ST77XX_WIDTH           480
    #define ST77XX_HEIGHT          320
    #define ST77XX_X_OFFSET        0
    #define ST77XX_Y_OFFSET        0
    #define ST77XX_USE_INVERSION   0
    #define ST77XX_SPI_SPEED_HZ    ST77XX_MAX_SPI_SPEED
    #define ST77XX_CONTROLLER_NAME "ST7796S"
#else
    #error "Define ST77XX_MODEL_ST7789 o ST77XX_MODEL_ST7796S"
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Configuración de buffers y memoria
 * ═══════════════════════════════════════════════════════════════════════════ */

#define ST77XX_USE_PSRAM       ST77XX_HAS_PSRAM
#define ST77XX_FB_SIZE         ((size_t)ST77XX_WIDTH * ST77XX_HEIGHT * sizeof(uint16_t))

/** @brief Modo stripe: divide pantalla en franjas para menor uso de RAM */
#define ST77XX_STRIPE_HEIGHT   27
#define ST77XX_STRIPE_COUNT    (ST77XX_HEIGHT / ST77XX_STRIPE_HEIGHT)
#define ST77XX_STRIPE_SIZE     ((size_t)ST77XX_WIDTH * ST77XX_STRIPE_HEIGHT * sizeof(uint16_t))

/** @brief Configuración DMA */
#define ST77XX_DMA_BUFFER_SIZE (32 * 1024)
#define ST77XX_SPI_QUEUE_SIZE  8
#define ST77XX_SWAP_BYTES_DMA  1

/* ═══════════════════════════════════════════════════════════════════════════
 * Configuración Backlight (LEDC PWM)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define ST77XX_LEDC_TIMER      LEDC_TIMER_0
#define ST77XX_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define ST77XX_LEDC_CHANNEL    LEDC_CHANNEL_0
#define ST77XX_LEDC_DUTY_RES   LEDC_TIMER_8_BIT
#define ST77XX_LEDC_FREQUENCY  5000

/* ═══════════════════════════════════════════════════════════════════════════
 * Configuración SPIFFS
 * ═══════════════════════════════════════════════════════════════════════════ */

#define ST77XX_SPIFFS_LABEL    "spiffs_image"
#define ST77XX_FONT_FILE       "/spiffs/font.bin"
#define ST77XX_FONT_WIDTH      8
#define ST77XX_FONT_HEIGHT     12
#define ST77XX_FONT_CHARS      108

/* ═══════════════════════════════════════════════════════════════════════════
 * Tipos de datos
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Orientaciones de pantalla soportadas
 */
typedef enum {
    ST77XX_PORTRAIT = 0,
    ST77XX_LANDSCAPE = 1,
    ST77XX_PORTRAIT_INV = 2,
    ST77XX_LANDSCAPE_INV = 3
} st77xx_orientation_t;

/**
 * @brief Información del estado del driver
 */
typedef struct {
    const char* controller_name;
    uint16_t width;
    uint16_t height;
    uint32_t spi_speed_hz;
    bool psram_enabled;
    bool initialized;
} st77xx_info_t;

/* ═══════════════════════════════════════════════════════════════════════════
 * API - Inicialización y Sistema
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Inicializa el driver ST77xx (SPI, GPIO, secuencia de inicio)
 */
void st77xx_init(void);

/**
 * @brief Inicialización rápida con double-buffering (requiere PSRAM)
 */
void st77xx_init_fast(void);

/**
 * @brief Obtiene información del estado actual del driver
 * @return Estructura con información del driver
 */
st77xx_info_t st77xx_get_info(void);

/**
 * @brief Libera todos los recursos del driver
 */
void st77xx_cleanup(void);

/**
 * @brief Monta el sistema de archivos SPIFFS
 */
void st77xx_mount_spiffs(void);

/**
 * @brief Carga datos de fuente desde SPIFFS
 * @param font_data Buffer donde almacenar la fuente
 */
void st77xx_load_font(uint8_t* font_data);

/* ═══════════════════════════════════════════════════════════════════════════
 * API - Control de pantalla
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Envía framebuffer completo a la pantalla
 * @param frame_buffer Puntero al buffer RGB565
 */
void st77xx_flush(const uint16_t* frame_buffer);

/**
 * @brief Flush inmediato sin validación de ventana
 * @param frame_buffer Puntero al buffer RGB565
 */
void st77xx_flush_immediate(const uint16_t* frame_buffer);

/**
 * @brief Cambia la orientación de la pantalla
 * @param orientation Nueva orientación
 */
void st77xx_set_orientation(st77xx_orientation_t orientation);

/**
 * @brief Ajusta el brillo del backlight
 * @param duty Nivel de brillo (0-255)
 */
void st77xx_backlight(uint8_t duty);

/**
 * @brief Define la ventana de dibujo activa
 * @param x0, y0 Esquina superior izquierda
 * @param x1, y1 Esquina inferior derecha
 */
void st77xx_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/* ═══════════════════════════════════════════════════════════════════════════
 * API - Double Buffering
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Inicializa sistema de doble buffer
 */
void st77xx_init_double_buffers(void);

/**
 * @brief Obtiene el buffer de dibujo actual (back buffer)
 * @return Puntero al buffer de dibujo
 */
uint16_t* st77xx_get_draw_buffer(void);

/**
 * @brief Intercambia buffers y muestra el contenido
 */
void st77xx_swap_and_display(void);

/**
 * @brief Libera los buffers dobles
 */
void st77xx_cleanup_double_buffers(void);

/* ═══════════════════════════════════════════════════════════════════════════
 * API - Modo Stripe (bajo consumo de RAM)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Inicializa el modo stripe buffer
 */
void st77xx_init_stripe_mode(void);

/**
 * @brief Obtiene el buffer de la franja actual
 * @return Puntero al buffer de stripe
 */
uint16_t* st77xx_stripe_get_buffer(void);

/**
 * @brief Rellena la franja actual con un color
 * @param color Color RGB565
 */
void st77xx_stripe_fill(uint16_t color);

/**
 * @brief Dibuja rectángulo en la franja actual
 * @param x, y Posición (coordenadas locales a la franja)
 * @param w, h Dimensiones
 * @param color Color RGB565
 */
void st77xx_stripe_fill_rect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color);

/**
 * @brief Envía franja actual y avanza a la siguiente
 * @return Índice de siguiente franja, -1 si terminó el frame
 */
int st77xx_stripe_flush_next(void);

/**
 * @brief Reinicia el contador de franjas para nuevo frame
 */
void st77xx_stripe_begin_frame(void);

/**
 * @brief Dibuja imagen usando modo stripe
 * @param path Ruta al archivo RGB565 raw
 * @return true si éxito
 */
bool st77xx_stripe_draw_image(const char* path);

/**
 * @brief Libera recursos del modo stripe
 */
void st77xx_cleanup_stripe_mode(void);

/* ═══════════════════════════════════════════════════════════════════════════
 * API - Funciones de dibujo
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Rellena el framebuffer con un color sólido
 * @param fb Framebuffer destino
 * @param color Color RGB565
 */
void st77xx_fill_screen(uint16_t* fb, uint16_t color);

/**
 * @brief Dibuja un píxel individual
 * @param fb Framebuffer destino
 * @param x, y Coordenadas
 * @param color Color RGB565
 */
void st77xx_draw_pixel(uint16_t* fb, int32_t x, int32_t y, uint16_t color);

/**
 * @brief Dibuja un rectángulo relleno
 * @param fb Framebuffer destino
 * @param x, y Posición
 * @param w, h Dimensiones
 * @param color Color RGB565
 */
void st77xx_fill_rect(uint16_t* fb, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color);

/**
 * @brief Carga imagen RGB565 desde archivo
 * @param fb Framebuffer destino
 * @param path Ruta al archivo
 * @return true si éxito
 */
bool st77xx_draw_image(uint16_t* fb, const char* path);

/**
 * @brief Dibuja texto con escala
 * @param fb Framebuffer destino
 * @param text Texto a dibujar
 * @param x, y Posición
 * @param color Color RGB565
 * @param scale Factor de escala
 * @param font Datos de fuente
 */
void st77xx_draw_text(uint16_t* fb, const char* text, int32_t x, int32_t y, 
                      uint16_t color, uint8_t scale, const uint8_t* font);

/**
 * @brief Dibuja texto UTF-8 con soporte extendido
 * @param fb Framebuffer destino
 * @param text Texto UTF-8
 * @param x, y Posición
 * @param color Color RGB565
 * @param scale Factor de escala
 * @param font Datos de fuente
 */
void st77xx_draw_text_unicode(uint16_t* fb, const char* text, int32_t x, int32_t y,
                              uint16_t color, uint8_t scale, const uint8_t* font);

/* ═══════════════════════════════════════════════════════════════════════════
 * API - Utilidades
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Convierte color RGB888 a RGB565
 * @param r, g, b Componentes RGB (0-255)
 * @return Color en formato RGB565
 */
uint16_t st77xx_rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Convierte color BGR888 a RGB565
 */
static inline uint16_t st77xx_bgr888_to_rgb565(uint8_t b, uint8_t g, uint8_t r) {
    return st77xx_rgb888_to_rgb565(r, g, b);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * API - Preload de frames (animaciones)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Precarga frames desde directorio para animaciones
 * @param base_dir Directorio con los frames
 * @param max_preload Máximo número de frames a cargar
 * @return Número de frames cargados
 */
int st77xx_preload_frames(const char* base_dir, int max_preload);

/**
 * @brief Obtiene un frame precargado
 * @param index Índice del frame
 * @return Puntero a los datos del frame
 */
const uint8_t* st77xx_get_preloaded_frame(int index);

/**
 * @brief Obtiene cantidad de frames precargados
 * @return Número de frames
 */
int st77xx_get_preloaded_count(void);

/**
 * @brief Libera todos los frames precargados
 */
void st77xx_free_preloaded_frames(void);

/* ═══════════════════════════════════════════════════════════════════════════
 * Macros de compatibilidad (legacy)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define TFT_WIDTH                ST77XX_WIDTH
#define TFT_HEIGHT               ST77XX_HEIGHT
#define X_OFFSET                 ST77XX_X_OFFSET
#define Y_OFFSET                 ST77XX_Y_OFFSET

#define tft_init()               st77xx_init()
#define flush(fb)                st77xx_flush(fb)
#define fillScreen(fb, c)        st77xx_fill_screen(fb, c)
#define fillRect(fb,x,y,w,h,c)   st77xx_fill_rect(fb,x,y,w,h,c)
#define draw_pixel(fb,x,y,c)     st77xx_draw_pixel(fb,x,y,c)
#define backlight(d)             st77xx_backlight(d)
#define set_orientation(o)       st77xx_set_orientation((st77xx_orientation_t)(o))
#define rgb888_to_rgb565(r,g,b)  st77xx_rgb888_to_rgb565(r,g,b)
#define mount_spiffs()           st77xx_mount_spiffs()
#define load_font(f)             st77xx_load_font(f)
#define drawText(fb,t,x,y,c,s,f) st77xx_draw_text(fb,t,x,y,c,s,f)

#endif /* ST77XX_H */
