# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/brbzb/esp/v5.2.1/esp-idf/components/bootloader/subproject"
  "C:/GITHUB/SLAM/Firmware_Novo_Robo/build/bootloader"
  "C:/GITHUB/SLAM/Firmware_Novo_Robo/build/bootloader-prefix"
  "C:/GITHUB/SLAM/Firmware_Novo_Robo/build/bootloader-prefix/tmp"
  "C:/GITHUB/SLAM/Firmware_Novo_Robo/build/bootloader-prefix/src/bootloader-stamp"
  "C:/GITHUB/SLAM/Firmware_Novo_Robo/build/bootloader-prefix/src"
  "C:/GITHUB/SLAM/Firmware_Novo_Robo/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/GITHUB/SLAM/Firmware_Novo_Robo/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/GITHUB/SLAM/Firmware_Novo_Robo/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
