idf_component_register(SRCS "mem_monitor.c" "st-idf.c"
                    INCLUDE_DIRS "."
                    REQUIRES st77xx esp_lcd driver esp_driver_spi)
