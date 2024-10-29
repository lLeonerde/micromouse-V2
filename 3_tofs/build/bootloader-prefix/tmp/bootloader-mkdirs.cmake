# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Leo/esp/v5.2/esp-idf/components/bootloader/subproject"
  "C:/workspace/micromouse-V2/3_tofs/build/bootloader"
  "C:/workspace/micromouse-V2/3_tofs/build/bootloader-prefix"
  "C:/workspace/micromouse-V2/3_tofs/build/bootloader-prefix/tmp"
  "C:/workspace/micromouse-V2/3_tofs/build/bootloader-prefix/src/bootloader-stamp"
  "C:/workspace/micromouse-V2/3_tofs/build/bootloader-prefix/src"
  "C:/workspace/micromouse-V2/3_tofs/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/workspace/micromouse-V2/3_tofs/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/workspace/micromouse-V2/3_tofs/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()