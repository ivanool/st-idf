# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(st-idf)

# Usar tabla de particiones personalizada
set(PARTITION_TABLE_CUSTOM_FILENAME "partitions.csv")

# Crear imagen SPIFFS desde la carpeta spiffs_image
# El nombre 'storage' debe coincidir con el de partitions.csv
spiffs_create_partition_image(storage ${CMAKE_CURRENT_SOURCE_DIR}/spiffs_image FLASH_IN_PROJECT)
