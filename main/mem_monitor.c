#include "mem_monitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char* TAG = "MEM_MON";

#ifndef CONFIG_MEM_MONITOR_INTERVAL_MS
#define CONFIG_MEM_MONITOR_INTERVAL_MS 5000
#endif

static void mem_monitor_task(void* arg)
{
    (void)arg;
    while (1) {
        size_t heap_free = esp_get_free_heap_size();
        size_t heap_min = esp_get_minimum_free_heap_size();
        size_t sram_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
        size_t sram_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

#if CONFIG_SPIRAM_SUPPORT
        size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
        size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        ESP_LOGI(TAG, "Heap free:%u min:%u | SRAM total/free:%u/%u | PSRAM total/free:%u/%u",
                 (unsigned)heap_free, (unsigned)heap_min,
                 (unsigned)sram_total, (unsigned)sram_free,
                 (unsigned)psram_total, (unsigned)psram_free);
#else
        ESP_LOGI(TAG, "Heap free:%u min:%u | SRAM total/free:%u/%u",
                 (unsigned)heap_free, (unsigned)heap_min,
                 (unsigned)sram_total, (unsigned)sram_free);
#endif

        vTaskDelay(pdMS_TO_TICKS(CONFIG_MEM_MONITOR_INTERVAL_MS));
    }
}

void mem_monitor_start(void)
{
#if CONFIG_MEM_MONITOR_ENABLE
    xTaskCreatePinnedToCore(mem_monitor_task, "mem_mon", 4096, NULL, 5, NULL, tskNO_AFFINITY);
#else
    (void)mem_monitor_task;
#endif
}
