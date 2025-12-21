/**
 * @file st-idf.c
 * @brief Aplicación principal - Demo del driver ST77xx con decodificación JPG
 * 
 * Muestra imágenes JPG desde SPIFFS usando el driver ST77xx.
 * Soporta modo stripe (bajo RAM) o PSRAM según disponibilidad.
 */

#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "st77xx.h"
#include "mem_monitor.h"
#include "sdkconfig.h"
#include "jpeg_decoder.h"

static const char* TAG = "st-idf";

/**
 * @brief Lista archivos en un directorio SPIFFS
 * @param dir_path Ruta del directorio
 */
static void list_spiffs_files(const char* dir_path)
{
    DIR* dir = opendir(dir_path);
    if (!dir) {
        ESP_LOGE(TAG, "No se pudo abrir directorio: %s", dir_path);
        return;
    }

    ESP_LOGI(TAG, "Archivos en %s:", dir_path);
    struct dirent* entry;
    int count = 0;
    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI(TAG, "  [%d] %s", count++, entry->d_name);
    }
    ESP_LOGI(TAG, "Total: %d archivos", count);
    closedir(dir);
}

/**
 * @brief Decodifica y muestra imagen JPG usando modo stripe (bajo RAM)
 * 
 * Escala la imagen para llenar la pantalla completa (cover mode).
 * Puede recortar bordes si la relación de aspecto no coincide.
 * 
 * @param path Ruta al archivo JPG en SPIFFS
 * @return true si éxito, false en caso de error
 */
static bool load_and_display_jpg_stripe(const char* path)
{
    FILE* f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "No se pudo abrir: %s", path);
        return false;
    }

    fseek(f, 0, SEEK_END);
    size_t file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t* jpg_buf = heap_caps_malloc(file_size, MALLOC_CAP_8BIT);
    if (!jpg_buf) {
        ESP_LOGE(TAG, "Sin memoria para JPG (%u bytes)", (unsigned)file_size);
        fclose(f);
        return false;
    }

    fread(jpg_buf, 1, file_size, f);
    fclose(f);

    size_t free_heap = esp_get_free_heap_size();
    ESP_LOGI(TAG, "JPG: %u bytes, RAM libre: %u", (unsigned)file_size, (unsigned)free_heap);

    esp_jpeg_image_scale_t scale;
    size_t max_decode_size;
    
    if (free_heap > 130000) {
        scale = JPEG_IMAGE_SCALE_1_2;
        max_decode_size = 240 * 160 * sizeof(uint16_t);
        ESP_LOGI(TAG, "Escala 1/2");
    } else if (free_heap > 70000) {
        scale = JPEG_IMAGE_SCALE_1_4;
        max_decode_size = 120 * 80 * sizeof(uint16_t);
        ESP_LOGI(TAG, "Escala 1/4");
    } else {
        scale = JPEG_IMAGE_SCALE_1_8;
        max_decode_size = 60 * 40 * sizeof(uint16_t);
        ESP_LOGI(TAG, "Escala 1/8");
    }

    uint8_t* decode_buf = heap_caps_malloc(max_decode_size, MALLOC_CAP_8BIT);
    if (!decode_buf) {
        ESP_LOGE(TAG, "Sin memoria para buffer");
        free(jpg_buf);
        return false;
    }

    esp_jpeg_image_output_t img_info;
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = jpg_buf,
        .indata_size = file_size,
        .outbuf = decode_buf,
        .outbuf_size = max_decode_size,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = scale,
        .flags = { .swap_color_bytes = 0 }
    };

    esp_err_t ret = esp_jpeg_decode(&jpeg_cfg, &img_info);
    free(jpg_buf);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error JPEG: %s", esp_err_to_name(ret));
        free(decode_buf);
        return false;
    }

    int src_w = img_info.width;
    int src_h = img_info.height;
    ESP_LOGI(TAG, "Imagen: %dx%d → Pantalla: %dx%d", src_w, src_h, ST77XX_WIDTH, ST77XX_HEIGHT);

    /* Escalar para cubrir pantalla (cover mode) */
    float scale_x = (float)ST77XX_WIDTH / src_w;
    float scale_y = (float)ST77XX_HEIGHT / src_h;
    float fill_scale = fmaxf(scale_x, scale_y);
    
    int scaled_w = (int)(src_w * fill_scale);
    int scaled_h = (int)(src_h * fill_scale);
    int offset_x = (ST77XX_WIDTH - scaled_w) / 2;
    int offset_y = (ST77XX_HEIGHT - scaled_h) / 2;
    
    ESP_LOGI(TAG, "Fill: %dx%d, escala=%.2f, offset(%d,%d)", 
             scaled_w, scaled_h, fill_scale, offset_x, offset_y);

    st77xx_init_stripe_mode();
    uint16_t* stripe = st77xx_stripe_get_buffer();
    if (!stripe) {
        ESP_LOGE(TAG, "Stripe mode no disponible");
        free(decode_buf);
        return false;
    }

    uint16_t* src = (uint16_t*)decode_buf;
    st77xx_stripe_begin_frame();
    
    for (int stripe_idx = 0; stripe_idx < ST77XX_STRIPE_COUNT; stripe_idx++) {
        int stripe_y_start = stripe_idx * ST77XX_STRIPE_HEIGHT;
        
        for (int y = 0; y < ST77XX_STRIPE_HEIGHT; y++) {
            int screen_y = stripe_y_start + y;
            float src_y_f = (float)(screen_y - offset_y) / fill_scale;
            int src_y = (int)src_y_f;
            
            if (src_y < 0 || src_y >= src_h) {
                for (int x = 0; x < ST77XX_WIDTH; x++) {
                    stripe[y * ST77XX_WIDTH + x] = 0;
                }
                continue;
            }
            
            for (int screen_x = 0; screen_x < ST77XX_WIDTH; screen_x++) {
                float src_x_f = (float)(screen_x - offset_x) / fill_scale;
                int src_x = (int)src_x_f;
                
                if (src_x < 0 || src_x >= src_w) {
                    stripe[y * ST77XX_WIDTH + screen_x] = 0;
                } else {
                    stripe[y * ST77XX_WIDTH + screen_x] = src[src_y * src_w + src_x];
                }
            }
        }
        
        st77xx_stripe_flush_next();
    }

    free(decode_buf);
    st77xx_cleanup_stripe_mode();
    
    ESP_LOGI(TAG, "JPG mostrado: %s", path);
    return true;
}

#if ST77XX_USE_PSRAM
/**
 * @brief Decodifica y muestra imagen JPG usando PSRAM
 * 
 * Permite decodificación a resolución completa sin escalado.
 * Requiere ESP32-S3 con PSRAM habilitada.
 * 
 * @param path Ruta al archivo JPG en SPIFFS
 * @return true si éxito, false en caso de error
 */
static bool load_and_display_jpg_psram(const char* path)
{
    FILE* f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "No se pudo abrir: %s", path);
        return false;
    }

    fseek(f, 0, SEEK_END);
    size_t file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t* jpg_buf = heap_caps_malloc(file_size, MALLOC_CAP_8BIT);
    if (!jpg_buf) {
        ESP_LOGE(TAG, "Sin memoria para JPG");
        fclose(f);
        return false;
    }

    fread(jpg_buf, 1, file_size, f);
    fclose(f);

    // Buffer suficiente para imágenes hasta 800x600
    size_t max_out_size = (size_t)800 * 600 * sizeof(uint16_t);
    uint8_t* decode_buf = heap_caps_malloc(max_out_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!decode_buf) {
        // Fallback a RAM interna si PSRAM falla
        ESP_LOGW(TAG, "PSRAM no disponible, usando RAM interna");
        max_out_size = (size_t)ST77XX_WIDTH * ST77XX_HEIGHT * sizeof(uint16_t);
        decode_buf = heap_caps_malloc(max_out_size, MALLOC_CAP_8BIT);
        if (!decode_buf) {
            ESP_LOGE(TAG, "Sin memoria para decodificación");
            free(jpg_buf);
            return false;
        }
    } else {
        ESP_LOGI(TAG, "Usando PSRAM para decodificación (%u bytes)", (unsigned)max_out_size);
    }

    esp_jpeg_image_output_t img_info;
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = jpg_buf,
        .indata_size = file_size,
        .outbuf = decode_buf,
        .outbuf_size = max_out_size,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = { .swap_color_bytes = 0 }
    };

    esp_err_t ret = esp_jpeg_decode(&jpeg_cfg, &img_info);
    free(jpg_buf);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error decodificando: %s", esp_err_to_name(ret));
        free(decode_buf);
        return false;
    }

    ESP_LOGI(TAG, "JPG: %dx%d", img_info.width, img_info.height);

    // Framebuffer en PSRAM
    uint16_t* frame_buffer = heap_caps_malloc(
        (size_t)ST77XX_WIDTH * ST77XX_HEIGHT * sizeof(uint16_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    if (!frame_buffer) {
        // Mostrar directo si no hay más memoria
        st77xx_flush((uint16_t*)decode_buf);
        free(decode_buf);
        return true;
    }

    memset(frame_buffer, 0, (size_t)ST77XX_WIDTH * ST77XX_HEIGHT * sizeof(uint16_t));

    uint16_t* src = (uint16_t*)decode_buf;
    int img_w = img_info.width;
    int img_h = img_info.height;
    int offset_x = (ST77XX_WIDTH - img_w) / 2;
    int offset_y = (ST77XX_HEIGHT - img_h) / 2;
    
    int src_start_x = 0, src_start_y = 0;
    int copy_w = img_w, copy_h = img_h;
    
    if (offset_x < 0) { src_start_x = -offset_x; copy_w = ST77XX_WIDTH; offset_x = 0; }
    if (offset_y < 0) { src_start_y = -offset_y; copy_h = ST77XX_HEIGHT; offset_y = 0; }
    if (offset_x + copy_w > ST77XX_WIDTH) copy_w = ST77XX_WIDTH - offset_x;
    if (offset_y + copy_h > ST77XX_HEIGHT) copy_h = ST77XX_HEIGHT - offset_y;

    for (int y = 0; y < copy_h; y++) {
        uint16_t* dst_row = &frame_buffer[(offset_y + y) * ST77XX_WIDTH + offset_x];
        uint16_t* src_row = &src[(src_start_y + y) * img_w + src_start_x];
        memcpy(dst_row, src_row, copy_w * sizeof(uint16_t));
    }

    free(decode_buf);
    st77xx_flush(frame_buffer);
    free(frame_buffer);
    
    ESP_LOGI(TAG, "JPG mostrado: %s", path);
    return true;
}
#endif

/**
 * @brief Muestra imagen JPG (selecciona método según hardware)
 * @param path Ruta al archivo JPG
 * @return true si éxito
 */
static bool load_and_display_jpg(const char* path)
{
#if ST77XX_USE_PSRAM
    return load_and_display_jpg_psram(path);
#else
    return load_and_display_jpg_stripe(path);
#endif
}

/**
 * @brief Punto de entrada de la aplicación
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Heap libre inicial: %lu bytes", (unsigned long)esp_get_free_heap_size());
    
    st77xx_mount_spiffs();
    st77xx_init();
    st77xx_backlight(77);
    mem_monitor_start();
    
    ESP_LOGI(TAG, "Heap libre después init: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Display: %s %dx%d, PSRAM: %s", 
             ST77XX_CONTROLLER_NAME, ST77XX_WIDTH, ST77XX_HEIGHT,
             ST77XX_USE_PSRAM ? "SI" : "NO");
    
    list_spiffs_files("/spiffs");
    
    ESP_LOGI(TAG, "Cargando imagen...");
    bool result = load_and_display_jpg("/spiffs/cammy.jpg");
    if (result) {
        ESP_LOGI(TAG, "=== Imagen mostrada exitosamente ===");
    } else {
        ESP_LOGE(TAG, "=== ERROR mostrando imagen ===");
    }
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}