cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Keypad_Polling)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/keypad.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    -DVECT_TAB_OFFSET=0x4000  # Offset for vector table
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Building ${CMAKE_PROJECT_NAME}.bin for DFU..."

)

add_custom_target(flash

    # Add user defined flash commands here

    COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c "program ${CMAKE_PROJECT_NAME}.elf verify reset exit"

    DEPENDS ${CMAKE_PROJECT_NAME}

    COMMENT "Flash STM32F1xx using OpenOCD..."

    USES_TERMINAL

)

add_custom_target(flash_dfu
    # 这里的参数解释：
    # -a 0 : 使用 Alt Setting 0 (通常对应内部 Flash)
    # -s 0x08004000:leave : 下载到该地址，下载完后执行 leave (复位运行)
    # -D : 指定下载的文件
    COMMAND dfu-util -d 0483:df11 -a 0 -s 0x08004000:leave -D ${CMAKE_PROJECT_NAME}.bin
    
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Flashing via DFU..."
    USES_TERMINAL
)