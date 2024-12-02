# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/hello/esp/v5.3.1/esp-idf/components/bootloader/subproject"
  "C:/Users/hello/OneDrive/Documents/PlatformIO/Projects/Speedo/ESP32-S3-Touch-LCD-2.1/build/bootloader"
  "C:/Users/hello/OneDrive/Documents/PlatformIO/Projects/Speedo/ESP32-S3-Touch-LCD-2.1/build/bootloader-prefix"
  "C:/Users/hello/OneDrive/Documents/PlatformIO/Projects/Speedo/ESP32-S3-Touch-LCD-2.1/build/bootloader-prefix/tmp"
  "C:/Users/hello/OneDrive/Documents/PlatformIO/Projects/Speedo/ESP32-S3-Touch-LCD-2.1/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/hello/OneDrive/Documents/PlatformIO/Projects/Speedo/ESP32-S3-Touch-LCD-2.1/build/bootloader-prefix/src"
  "C:/Users/hello/OneDrive/Documents/PlatformIO/Projects/Speedo/ESP32-S3-Touch-LCD-2.1/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/hello/OneDrive/Documents/PlatformIO/Projects/Speedo/ESP32-S3-Touch-LCD-2.1/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/hello/OneDrive/Documents/PlatformIO/Projects/Speedo/ESP32-S3-Touch-LCD-2.1/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
