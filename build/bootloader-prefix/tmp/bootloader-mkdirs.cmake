# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/alfredochavez/esp/esp-idf/components/bootloader/subproject"
  "/Users/alfredochavez/Desktop/Courses/EECS149/projects/ble_esp_integration/gatt_client/build/bootloader"
  "/Users/alfredochavez/Desktop/Courses/EECS149/projects/ble_esp_integration/gatt_client/build/bootloader-prefix"
  "/Users/alfredochavez/Desktop/Courses/EECS149/projects/ble_esp_integration/gatt_client/build/bootloader-prefix/tmp"
  "/Users/alfredochavez/Desktop/Courses/EECS149/projects/ble_esp_integration/gatt_client/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/alfredochavez/Desktop/Courses/EECS149/projects/ble_esp_integration/gatt_client/build/bootloader-prefix/src"
  "/Users/alfredochavez/Desktop/Courses/EECS149/projects/ble_esp_integration/gatt_client/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/alfredochavez/Desktop/Courses/EECS149/projects/ble_esp_integration/gatt_client/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
