/**
 * @file st77xx.c
 * @brief Implementación del driver ST77xx para ESP-IDF
 */

#include "st77xx.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Constantes privadas
 * ═══════════════════════════════════════════════════════════════════════════ */

static const char* TAG = "ST77XX";

#define CMD_MODE  0
#define DATA_MODE 1

/** @brief Comandos del controlador ST77xx */
#define CMD_NOP         0x00
#define CMD_SWRESET     0x01
#define CMD_SLPOUT      0x11
#define CMD_NORON       0x13
#define CMD_INVOFF      0x20
#define CMD_INVON       0x21
#define CMD_DISPON      0x29
#define CMD_CASET       0x2A
#define CMD_RASET       0x2B
#define CMD_RAMWR       0x2C
#define CMD_COLMOD      0x3A
#define CMD_MADCTL      0x36
#define CMD_PORCTRL     0xB2
#define CMD_GCTRL       0xB7
#define CMD_VCOMS       0xBB

/* ═══════════════════════════════════════════════════════════════════════════
 * Variables estáticas
 * ═══════════════════════════════════════════════════════════════════════════ */

static spi_device_handle_t spi_handle = NULL;
static uint8_t* dma_buffer = NULL;
static size_t dma_buffer_size = 0;
static bool window_set = false;
static bool backlight_initialized = false;
static bool driver_initialized = false;

static uint16_t* fb_front = NULL;
static uint16_t* fb_back = NULL;

static uint16_t* stripe_buffer = NULL;
static int current_stripe = 0;

static uint8_t** preloaded_frames = NULL;
static int preloaded_count = 0;

/** @brief Mapeo Unicode -> índice de glifo en la fuente */
static const uint32_t font_char_map[] = {
    32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,
    48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
    64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,
    80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,
    96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,
    112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,
    161,191,209,225,233,237,241,243,250,252,26376,20320
};
static const int font_char_map_count = sizeof(font_char_map) / sizeof(font_char_map[0]);

/* ═══════════════════════════════════════════════════════════════════════════
 * Prototipos privados
 * ═══════════════════════════════════════════════════════════════════════════ */

static void gpio_init_pins(void);
static void spi_init_bus(void);
static void display_reset(void);
static void send_cmd(uint8_t cmd);
static void send_data(const uint8_t* data, size_t size);
static void send_word(uint16_t data);
static void send_data_dma(const uint8_t* data, size_t size);
static void init_backlight_once(void);
static int find_char_index(uint32_t code);
static uint32_t utf8_next_codepoint(const char** p);
static void draw_glyph(uint16_t* fb, int32_t x, int32_t y, int index, 
                       uint16_t color, uint8_t scale, const uint8_t* font);

/* ═══════════════════════════════════════════════════════════════════════════
 * Inicialización
 * ═══════════════════════════════════════════════════════════════════════════ */

void st77xx_init(void) {
    if (driver_initialized) {
        ESP_LOGW(TAG, "Driver ya inicializado");
        return;
    }
    
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  ST77XX Driver - Detección automática        ║");
    ESP_LOGI(TAG, "╠══════════════════════════════════════════════╣");
    ESP_LOGI(TAG, "║  Chip: %-20s              ║", ST77XX_CHIP_NAME);
    ESP_LOGI(TAG, "║  PSRAM: %-19s              ║", ST77XX_HAS_PSRAM ? "Disponible" : "No disponible");
    ESP_LOGI(TAG, "║  Display: %-17s              ║", ST77XX_CONTROLLER_NAME);
    ESP_LOGI(TAG, "║  Resolución: %dx%-22d  ║", ST77XX_WIDTH, ST77XX_HEIGHT);
    ESP_LOGI(TAG, "║  SPI: %lu MHz                              ║", (unsigned long)(ST77XX_SPI_SPEED_HZ / 1000000));
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════╝");
    
    gpio_init_pins();
    spi_init_bus();
    display_reset();
    
    // Secuencia de inicialización común ST77xx
    ESP_LOGD(TAG, "Enviando SLPOUT...");
    send_cmd(CMD_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));  // Datasheet: mínimo 120ms
    
    // Color mode: 16-bit RGB565
    ESP_LOGD(TAG, "Configurando modo color 16-bit...");
    send_cmd(CMD_COLMOD);
    send_data((uint8_t[]){0x55}, 1);
    
    // Orientación por defecto
    st77xx_set_orientation(ST77XX_LANDSCAPE_INV);
    
    // Porch Control
    send_cmd(CMD_PORCTRL);
    send_data((uint8_t[]){0x0C, 0x0C, 0x00, 0x33, 0x33}, 5);
    
    // Gate Control
    send_cmd(CMD_GCTRL);
#if defined(ST77XX_MODEL_ST7789)
    send_data((uint8_t[]){0x75}, 1);  // ST7789
#else
    send_data((uint8_t[]){0x35}, 1);  // ST7796S
#endif
    
    // VCOMS
    send_cmd(CMD_VCOMS);
#if defined(ST77XX_MODEL_ST7789)
    send_data((uint8_t[]){0x2B}, 1);  // ST7789
#else
    send_data((uint8_t[]){0x1A}, 1);  // ST7796S
#endif
    
    // Inversión según modelo
#if ST77XX_USE_INVERSION
    send_cmd(CMD_INVON);
    ESP_LOGD(TAG, "Inversión: ON");
#else
    send_cmd(CMD_INVOFF);
    ESP_LOGD(TAG, "Inversión: OFF");
#endif
    
    // Normal mode y display on
    send_cmd(CMD_NORON);
    send_cmd(CMD_DISPON);
    vTaskDelay(pdMS_TO_TICKS(120));
    
    // Backlight al máximo
    st77xx_backlight(255);
    
    driver_initialized = true;
    ESP_LOGI(TAG, "✅ Inicialización completada");
}

void st77xx_init_fast(void) {
    st77xx_init();
    st77xx_init_double_buffers();
}

st77xx_info_t st77xx_get_info(void) {
    st77xx_info_t info = {
        .controller_name = ST77XX_CONTROLLER_NAME,
        .width = ST77XX_WIDTH,
        .height = ST77XX_HEIGHT,
        .spi_speed_hz = ST77XX_SPI_SPEED_HZ,
        .psram_enabled = ST77XX_USE_PSRAM,
        .initialized = driver_initialized
    };
    return info;
}

void st77xx_cleanup(void) {
    st77xx_cleanup_double_buffers();
    st77xx_free_preloaded_frames();
    
    if (dma_buffer) {
        heap_caps_free(dma_buffer);
        dma_buffer = NULL;
        dma_buffer_size = 0;
    }
    
    window_set = false;
    driver_initialized = false;
    ESP_LOGI(TAG, "Recursos liberados");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SPIFFS
 * ═══════════════════════════════════════════════════════════════════════════ */

void st77xx_mount_spiffs(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = ST77XX_SPIFFS_LABEL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Mount con label '%s' falló. Intentando partición por defecto...", 
                 ST77XX_SPIFFS_LABEL);
        conf.partition_label = NULL;
        ret = esp_vfs_spiffs_register(&conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Fallo al montar SPIFFS: %s", esp_err_to_name(ret));
            return;
        }
    }
    
    size_t total = 0, used = 0;
    esp_spiffs_info(conf.partition_label, &total, &used);
    ESP_LOGI(TAG, "SPIFFS montado: %u/%u bytes usados", (unsigned)used, (unsigned)total);
}

void st77xx_load_font(uint8_t* font_data) {
    if (!font_data) return;
    
    size_t font_size = (size_t)ST77XX_FONT_CHARS * ST77XX_FONT_HEIGHT;
    FILE* file = fopen(ST77XX_FONT_FILE, "rb");
    
    if (file) {
        size_t read = fread(font_data, 1, font_size, file);
        fclose(file);
        if (read == font_size) {
            ESP_LOGI(TAG, "Fuente cargada: %u bytes", (unsigned)font_size);
            return;
        }
        ESP_LOGW(TAG, "Fuente incompleta: %u/%u bytes", (unsigned)read, (unsigned)font_size);
    } else {
        ESP_LOGW(TAG, "No se encontró fuente, usando patrón por defecto");
    }
    
    // Patrón por defecto
    for (size_t i = 0; i < font_size; i++) {
        font_data[i] = 0xAA;  // Patrón visible para debug
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Control de pantalla
 * ═══════════════════════════════════════════════════════════════════════════ */

void st77xx_flush(const uint16_t* frame_buffer) {
    if (!frame_buffer) return;
    
    if (!window_set) {
        st77xx_set_window(0, 0, ST77XX_WIDTH - 1, ST77XX_HEIGHT - 1);
        window_set = true;
    } else {
        send_cmd(CMD_RAMWR);
    }
    
    send_data_dma((const uint8_t*)frame_buffer, ST77XX_FB_SIZE);
}

void st77xx_flush_immediate(const uint16_t* frame_buffer) {
    if (!frame_buffer) return;
    send_cmd(CMD_RAMWR);
    send_data_dma((const uint8_t*)frame_buffer, ST77XX_FB_SIZE);
}

void st77xx_set_orientation(st77xx_orientation_t orientation) {
    uint8_t madctl = 0;
    
#if defined(ST77XX_MODEL_ST7789)
    // ST7789 MADCTL values (con MX=1 para corregir espejo horizontal)
    switch (orientation) {
        case ST77XX_PORTRAIT:      madctl = 0x40; break;  // MX
        case ST77XX_LANDSCAPE:     madctl = 0x20; break;  // MV
        case ST77XX_PORTRAIT_INV:  madctl = 0x80; break;  // MY
        case ST77XX_LANDSCAPE_INV: madctl = 0xE0; break;  // MY+MX+MV
        default:                   madctl = 0x40; break;
    }
#else
    // ST7796S MADCTL values (con MX para corregir espejo horizontal)
    switch (orientation) {
        case ST77XX_PORTRAIT:      madctl = 0x48; break;  // MX + BGR
        case ST77XX_LANDSCAPE:     madctl = 0x28; break;  // MV + BGR
        case ST77XX_PORTRAIT_INV:  madctl = 0x88; break;  // MY + BGR
        case ST77XX_LANDSCAPE_INV: madctl = 0xE8; break;  // MY+MX+MV + BGR
        default:                   madctl = 0x48; break;
    }
#endif
    
    send_cmd(CMD_MADCTL);
    send_data(&madctl, 1);
    window_set = false;  // Reset ventana al cambiar orientación
}

void st77xx_backlight(uint8_t duty) {
    init_backlight_once();
    ledc_set_duty(ST77XX_LEDC_MODE, ST77XX_LEDC_CHANNEL, duty);
    ledc_update_duty(ST77XX_LEDC_MODE, ST77XX_LEDC_CHANNEL);
}

void st77xx_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // Clamp y ordenar
    if (x0 > x1) { uint16_t t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { uint16_t t = y0; y0 = y1; y1 = t; }
    if (x1 >= ST77XX_WIDTH) x1 = ST77XX_WIDTH - 1;
    if (y1 >= ST77XX_HEIGHT) y1 = ST77XX_HEIGHT - 1;
    
    send_cmd(CMD_CASET);
    send_word(x0 + ST77XX_X_OFFSET);
    send_word(x1 + ST77XX_X_OFFSET);
    
    send_cmd(CMD_RASET);
    send_word(y0 + ST77XX_Y_OFFSET);
    send_word(y1 + ST77XX_Y_OFFSET);
    
    send_cmd(CMD_RAMWR);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Double Buffering
 * ═══════════════════════════════════════════════════════════════════════════ */

void st77xx_init_double_buffers(void) {
    if (fb_front && fb_back) return;  // Ya inicializado
    
    uint32_t caps = MALLOC_CAP_8BIT;
#if ST77XX_USE_PSRAM
    caps |= MALLOC_CAP_SPIRAM;
#endif
    
    fb_front = (uint16_t*)heap_caps_malloc(ST77XX_FB_SIZE, caps);
    fb_back = (uint16_t*)heap_caps_malloc(ST77XX_FB_SIZE, caps);
    
    if (!fb_front || !fb_back) {
        ESP_LOGE(TAG, "❌ Error al asignar double buffers");
        if (fb_front) { heap_caps_free(fb_front); fb_front = NULL; }
        if (fb_back) { heap_caps_free(fb_back); fb_back = NULL; }
        return;
    }
    
    memset(fb_front, 0, ST77XX_FB_SIZE);
    memset(fb_back, 0, ST77XX_FB_SIZE);
    ESP_LOGI(TAG, "✅ Double buffers: %u bytes cada uno", (unsigned)ST77XX_FB_SIZE);
}

uint16_t* st77xx_get_draw_buffer(void) {
    return fb_back;
}

void st77xx_swap_and_display(void) {
    if (!fb_front || !fb_back) return;
    uint16_t* tmp = fb_front;
    fb_front = fb_back;
    fb_back = tmp;
    st77xx_flush(fb_front);
}

void st77xx_cleanup_double_buffers(void) {
    if (fb_front) { heap_caps_free(fb_front); fb_front = NULL; }
    if (fb_back) { heap_caps_free(fb_back); fb_back = NULL; }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Stripe Mode (bajo consumo de RAM)
 * ═══════════════════════════════════════════════════════════════════════════ */

void st77xx_init_stripe_mode(void) {
    if (stripe_buffer) return;  // Ya inicializado
    
    stripe_buffer = (uint16_t*)heap_caps_malloc(ST77XX_STRIPE_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    if (!stripe_buffer) {
        ESP_LOGE(TAG, "❌ Error al asignar stripe buffer (%u bytes)", (unsigned)ST77XX_STRIPE_SIZE);
        return;
    }
    
    current_stripe = 0;
    ESP_LOGI(TAG, "✅ Stripe mode: %u bytes buffer, %d franjas de %d líneas", 
             (unsigned)ST77XX_STRIPE_SIZE, ST77XX_STRIPE_COUNT, ST77XX_STRIPE_HEIGHT);
}

uint16_t* st77xx_stripe_get_buffer(void) {
    return stripe_buffer;
}

void st77xx_stripe_fill(uint16_t color) {
    if (!stripe_buffer) return;
    
    size_t total = (size_t)ST77XX_WIDTH * ST77XX_STRIPE_HEIGHT;
    uint32_t color32 = ((uint32_t)color << 16) | color;
    uint32_t* ptr32 = (uint32_t*)stripe_buffer;
    size_t words = total / 2;
    
    while (words >= 8) {
        *ptr32++ = color32; *ptr32++ = color32;
        *ptr32++ = color32; *ptr32++ = color32;
        *ptr32++ = color32; *ptr32++ = color32;
        *ptr32++ = color32; *ptr32++ = color32;
        words -= 8;
    }
    while (words--) *ptr32++ = color32;
}

void st77xx_stripe_fill_rect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color) {
    if (!stripe_buffer) return;
    
    // Clipping a la franja
    if (x >= ST77XX_WIDTH || y >= ST77XX_STRIPE_HEIGHT) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > ST77XX_WIDTH) w = ST77XX_WIDTH - x;
    if (y + h > ST77XX_STRIPE_HEIGHT) h = ST77XX_STRIPE_HEIGHT - y;
    if (w <= 0 || h <= 0) return;
    
    for (int32_t row = y; row < y + h; row++) {
        uint16_t* line = &stripe_buffer[row * ST77XX_WIDTH + x];
        for (int32_t col = 0; col < w; col++) {
            line[col] = color;
        }
    }
}

void st77xx_stripe_begin_frame(void) {
    current_stripe = 0;
}

int st77xx_stripe_flush_next(void) {
    if (!stripe_buffer || current_stripe >= ST77XX_STRIPE_COUNT) {
        return -1;  // Frame terminado
    }
    
    // Calcular posición Y de esta franja
    uint16_t y0 = current_stripe * ST77XX_STRIPE_HEIGHT;
    uint16_t y1 = y0 + ST77XX_STRIPE_HEIGHT - 1;
    
    // Establecer ventana para esta franja
    st77xx_set_window(0, y0, ST77XX_WIDTH - 1, y1);
    send_cmd(CMD_RAMWR);
    
    // Enviar datos de la franja
    send_data_dma((const uint8_t*)stripe_buffer, ST77XX_STRIPE_SIZE);
    
    current_stripe++;
    return (current_stripe < ST77XX_STRIPE_COUNT) ? current_stripe : -1;
}

void st77xx_cleanup_stripe_mode(void) {
    if (stripe_buffer) {
        heap_caps_free(stripe_buffer);
        stripe_buffer = NULL;
    }
    current_stripe = 0;
}

bool st77xx_stripe_draw_image(const char* path) {
    if (!stripe_buffer || !path) return false;
    
    FILE* f = fopen(path, "rb");
    if (!f) {
        ESP_LOGW(TAG, "No se pudo abrir: %s", path);
        return false;
    }
    
    // OPTIMIZACIÓN: Establecer ventana completa UNA SOLA VEZ
    st77xx_set_window(0, 0, ST77XX_WIDTH - 1, ST77XX_HEIGHT - 1);
    send_cmd(CMD_RAMWR);
    
    // Enviar todas las franjas sin pausas (streaming continuo)
    for (int stripe = 0; stripe < ST77XX_STRIPE_COUNT; stripe++) {
        // Leer franja desde archivo
        size_t bytes_read = fread(stripe_buffer, 1, ST77XX_STRIPE_SIZE, f);
        if (bytes_read < ST77XX_STRIPE_SIZE) {
            memset(((uint8_t*)stripe_buffer) + bytes_read, 0, ST77XX_STRIPE_SIZE - bytes_read);
        }
        
        // Enviar directamente sin re-establecer ventana
        send_data_dma((const uint8_t*)stripe_buffer, ST77XX_STRIPE_SIZE);
    }
    
    fclose(f);
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Funciones de dibujo
 * ═══════════════════════════════════════════════════════════════════════════ */

void st77xx_fill_screen(uint16_t* fb, uint16_t color) {
    if (!fb) return;
    
    size_t total = (size_t)ST77XX_WIDTH * ST77XX_HEIGHT;
    uint32_t color32 = ((uint32_t)color << 16) | color;
    uint32_t* ptr32 = (uint32_t*)fb;
    size_t words = total / 2;
    
    // Loop unrolling: 8 words por iteración
    while (words >= 8) {
        *ptr32++ = color32; *ptr32++ = color32;
        *ptr32++ = color32; *ptr32++ = color32;
        *ptr32++ = color32; *ptr32++ = color32;
        *ptr32++ = color32; *ptr32++ = color32;
        words -= 8;
    }
    while (words--) *ptr32++ = color32;
    
    if (total % 2) fb[total - 1] = color;
}

void st77xx_draw_pixel(uint16_t* fb, int32_t x, int32_t y, uint16_t color) {
    if (!fb || x < 0 || x >= ST77XX_WIDTH || y < 0 || y >= ST77XX_HEIGHT) return;
    fb[y * ST77XX_WIDTH + x] = color;
}

void st77xx_fill_rect(uint16_t* fb, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color) {
    if (!fb) return;
    
    // Clipping
    if (x >= ST77XX_WIDTH || y >= ST77XX_HEIGHT || (x + w) <= 0 || (y + h) <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if ((x + w) > ST77XX_WIDTH) w = ST77XX_WIDTH - x;
    if ((y + h) > ST77XX_HEIGHT) h = ST77XX_HEIGHT - y;
    
    // Optimización para rectángulos de ancho completo
    if (x == 0 && w == ST77XX_WIDTH) {
        uint16_t* start = &fb[y * ST77XX_WIDTH];
        size_t pixels = (size_t)w * h;
        uint32_t color32 = ((uint32_t)color << 16) | color;
        uint32_t* ptr32 = (uint32_t*)start;
        size_t words = pixels / 2;
        
        while (words >= 4) {
            *ptr32++ = color32; *ptr32++ = color32;
            *ptr32++ = color32; *ptr32++ = color32;
            words -= 4;
        }
        while (words--) *ptr32++ = color32;
        if (pixels % 2) start[pixels - 1] = color;
    } else {
        // Método tradicional
        for (int32_t row = 0; row < h; row++) {
            uint16_t* dst = &fb[(y + row) * ST77XX_WIDTH + x];
            for (int32_t col = 0; col < w; col++) {
                *dst++ = color;
            }
        }
    }
}

bool st77xx_draw_image(uint16_t* fb, const char* path) {
    if (!fb || !path) return false;
    
    FILE* file = fopen(path, "rb");
    if (!file) {
        ESP_LOGE(TAG, "No se pudo abrir: %s", path);
        return false;
    }
    
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    if ((size_t)size != ST77XX_FB_SIZE) {
        ESP_LOGW(TAG, "Tamaño incorrecto: %ld (esperado %u)", size, (unsigned)ST77XX_FB_SIZE);
        fclose(file);
        return false;
    }
    
    size_t read = fread(fb, 1, ST77XX_FB_SIZE, file);
    fclose(file);
    
    return (read == ST77XX_FB_SIZE);
}

void st77xx_draw_text(uint16_t* fb, const char* text, int32_t x, int32_t y,
                      uint16_t color, uint8_t scale, const uint8_t* font) {
    st77xx_draw_text_unicode(fb, text, x, y, color, scale, font);
}

void st77xx_draw_text_unicode(uint16_t* fb, const char* text, int32_t x, int32_t y,
                              uint16_t color, uint8_t scale, const uint8_t* font) {
    if (!fb || !text || !font) return;
    
    int32_t cx = x, cy = y;
    const char* p = text;
    
    while (*p) {
        uint32_t cp = utf8_next_codepoint(&p);
        if (cp == 0) break;
        
        if (cp == '\n') {
            cy += (ST77XX_FONT_HEIGHT + 2) * scale;
            cx = x;
            continue;
        }
        
        int idx = find_char_index(cp);
        if (idx >= 0) {
            draw_glyph(fb, cx, cy, idx, color, scale, font);
        }
        cx += ST77XX_FONT_WIDTH * scale;
    }
}

uint16_t st77xx_rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Preload de frames (animaciones)
 * ═══════════════════════════════════════════════════════════════════════════ */

int st77xx_preload_frames(const char* base_dir, int max_preload) {
    if (!base_dir || max_preload <= 0) return 0;
    
    // Liberar previos
    st77xx_free_preloaded_frames();
    
    uint32_t caps = MALLOC_CAP_8BIT;
#if ST77XX_USE_PSRAM
    caps |= MALLOC_CAP_SPIRAM;
#endif
    
    preloaded_frames = heap_caps_malloc(sizeof(uint8_t*) * max_preload, caps);
    if (!preloaded_frames) {
        ESP_LOGE(TAG, "Error al asignar array de frames");
        return 0;
    }
    
    for (int i = 0; i < max_preload; i++) {
        char path[128];
        snprintf(path, sizeof(path), "%s/%d.bin", base_dir, i + 1);
        
        FILE* f = fopen(path, "rb");
        if (!f) {
            ESP_LOGW(TAG, "No se encontró: %s", path);
            break;
        }
        
        uint8_t* buf = heap_caps_malloc(ST77XX_FB_SIZE, caps);
        if (!buf) { 
            ESP_LOGE(TAG, "Error al asignar buffer para frame %d", i + 1);
            fclose(f); 
            break; 
        }
        
        size_t r = fread(buf, 1, ST77XX_FB_SIZE, f);
        fclose(f);
        
        if (r != ST77XX_FB_SIZE) { 
            ESP_LOGW(TAG, "Tamaño incorrecto en %s: %u/%u", path, (unsigned)r, (unsigned)ST77XX_FB_SIZE);
            heap_caps_free(buf); 
            break; 
        }
        
        preloaded_frames[preloaded_count++] = buf;
        ESP_LOGI(TAG, "Preloaded: %s (%d/%d)", path, i + 1, max_preload);
    }
    
    if (preloaded_count == 0) {
        heap_caps_free(preloaded_frames);
        preloaded_frames = NULL;
        ESP_LOGE(TAG, "No se cargó ningún frame desde %s", base_dir);
    }
    
    return preloaded_count;
}

const uint8_t* st77xx_get_preloaded_frame(int index) {
    if (index < 0 || index >= preloaded_count) return NULL;
    return preloaded_frames[index];
}

int st77xx_get_preloaded_count(void) {
    return preloaded_count;
}

void st77xx_free_preloaded_frames(void) {
    if (!preloaded_frames) return;
    for (int i = 0; i < preloaded_count; i++) {
        if (preloaded_frames[i]) heap_caps_free(preloaded_frames[i]);
    }
    heap_caps_free(preloaded_frames);
    preloaded_frames = NULL;
    preloaded_count = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Funciones privadas - GPIO/SPI
 * ═══════════════════════════════════════════════════════════════════════════ */

static void gpio_init_pins(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ST77XX_PIN_DC) | (1ULL << ST77XX_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

static void spi_init_bus(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = ST77XX_PIN_MOSI,
        .miso_io_num = ST77XX_PIN_MISO,
        .sclk_io_num = ST77XX_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ST77XX_DMA_BUFFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = ST77XX_SPI_SPEED_HZ,
        .mode = 0,
        .spics_io_num = ST77XX_PIN_CS,
        .queue_size = ST77XX_SPI_QUEUE_SIZE,
        .flags = SPI_DEVICE_NO_DUMMY
    };
    
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fallo SPI bus: %s", esp_err_to_name(ret));
    }
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fallo SPI device: %s", esp_err_to_name(ret));
    }
    
    // Buffer DMA estático
    dma_buffer_size = ST77XX_DMA_BUFFER_SIZE;
    dma_buffer = heap_caps_malloc(dma_buffer_size, MALLOC_CAP_DMA);
    if (!dma_buffer) {
        ESP_LOGE(TAG, "Fallo al asignar DMA buffer");
    }
}

static void display_reset(void) {
    gpio_set_level(ST77XX_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(ST77XX_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    send_cmd(CMD_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(120));
}

static void send_cmd(uint8_t cmd) {
    gpio_set_level(ST77XX_PIN_DC, CMD_MODE);
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_polling_transmit(spi_handle, &t);
}

static void send_data(const uint8_t* data, size_t size) {
    if (!size) return;
    gpio_set_level(ST77XX_PIN_DC, DATA_MODE);
    
    while (size > 0) {
        size_t chunk = (size > ST77XX_DMA_BUFFER_SIZE) ? ST77XX_DMA_BUFFER_SIZE : size;
        spi_transaction_t t = {0};
        t.length = chunk * 8;
        t.tx_buffer = data;
        spi_device_polling_transmit(spi_handle, &t);
        data += chunk;
        size -= chunk;
    }
}

static void send_word(uint16_t data) {
    uint8_t buf[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    send_data(buf, 2);
}

static void send_data_dma(const uint8_t* data, size_t size) {
    if (!size || !dma_buffer) return;
    
    gpio_set_level(ST77XX_PIN_DC, DATA_MODE);
    
    while (size > 0) {
        size_t chunk = (size > dma_buffer_size) ? dma_buffer_size : size;
        
#if ST77XX_SWAP_BYTES_DMA
        // Swap bytes para RGB565 correcto
        for (size_t i = 0; i + 1 < chunk; i += 2) {
            dma_buffer[i] = data[i + 1];
            dma_buffer[i + 1] = data[i];
        }
        if (chunk % 2) dma_buffer[chunk - 1] = data[chunk - 1];
#else
        memcpy(dma_buffer, data, chunk);
#endif
        
        spi_transaction_t t = {0};
        t.length = chunk * 8;
        t.tx_buffer = dma_buffer;
        spi_device_polling_transmit(spi_handle, &t);
        
        data += chunk;
        size -= chunk;
    }
}

static void init_backlight_once(void) {
    if (backlight_initialized) return;
    
    ledc_timer_config_t timer = {
        .speed_mode = ST77XX_LEDC_MODE,
        .timer_num = ST77XX_LEDC_TIMER,
        .duty_resolution = ST77XX_LEDC_DUTY_RES,
        .freq_hz = ST77XX_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);
    
    ledc_channel_config_t channel = {
        .speed_mode = ST77XX_LEDC_MODE,
        .channel = ST77XX_LEDC_CHANNEL,
        .gpio_num = ST77XX_PIN_BL,
        .timer_sel = ST77XX_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
    
    backlight_initialized = true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Funciones privadas - Texto/UTF-8
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Busca el índice de un codepoint Unicode en el mapa de fuente
 */
static int find_char_index(uint32_t code) {
    for (int i = 0; i < font_char_map_count; i++) {
        if (font_char_map[i] == code) return i;
    }
    return -1;
}

static uint32_t utf8_next_codepoint(const char** p) {
    const unsigned char* s = (const unsigned char*)*p;
    if (!s || !*s) return 0;
    
    uint32_t cp = 0;
    if (s[0] < 0x80) {
        cp = s[0];
        *p += 1;
    } else if ((s[0] & 0xE0) == 0xC0) {
        cp = ((s[0] & 0x1F) << 6) | (s[1] & 0x3F);
        *p += 2;
    } else if ((s[0] & 0xF0) == 0xE0) {
        cp = ((s[0] & 0x0F) << 12) | ((s[1] & 0x3F) << 6) | (s[2] & 0x3F);
        *p += 3;
    } else if ((s[0] & 0xF8) == 0xF0) {
        cp = ((s[0] & 0x07) << 18) | ((s[1] & 0x3F) << 12) | ((s[2] & 0x3F) << 6) | (s[3] & 0x3F);
        *p += 4;
    } else {
        *p += 1;
    }
    return cp;
}

static void draw_glyph(uint16_t* fb, int32_t x, int32_t y, int index,
                       uint16_t color, uint8_t scale, const uint8_t* font) {
    if (index < 0 || index >= ST77XX_FONT_CHARS || !font) return;
    
    const uint8_t* glyph = &font[index * ST77XX_FONT_HEIGHT];
    
    for (int32_t row = 0; row < ST77XX_FONT_HEIGHT; row++) {
        uint8_t line = glyph[row];
        for (int32_t col = 0; col < ST77XX_FONT_WIDTH; col++) {
            if (line & (1 << (ST77XX_FONT_WIDTH - 1 - col))) {
                if (scale == 1) {
                    st77xx_draw_pixel(fb, x + col, y + row, color);
                } else {
                    st77xx_fill_rect(fb, x + col * scale, y + row * scale, scale, scale, color);
                }
            }
        }
    }
}