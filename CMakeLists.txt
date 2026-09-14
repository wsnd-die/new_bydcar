cmake_minimum_required(VERSION 3.22)

# ---------------------------------------------------------------------------
# Target: STM32G491VETx (Cortex-M4F, 512K flash / 96K SRAM / 16K CCMRAM)
#
# This must be configured with the ARM cross toolchain, never with the host
# compiler:
#     cmake --preset Debug        (see CMakePresets.json)
# or
#     cmake -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -B build
#
# CLion's own CMake profile does not read CMakePresets.json and passes no
# toolchain file, which is how this project first ended up being compiled with
# the host MinGW gcc. So if no toolchain was given, select ours here -- this has
# to happen before project().
# ---------------------------------------------------------------------------

if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake"
      CACHE FILEPATH "Cross toolchain file" FORCE)
endif()

project(bydcar_g491vet6 C ASM)

if(NOT CMAKE_C_COMPILER MATCHES "arm-none-eabi")
  message(FATAL_ERROR
    "This project must be built with arm-none-eabi-gcc, but CMake picked "
    "'${CMAKE_C_COMPILER}'.\n"
    "Configure with the toolchain file:\n"
    "  cmake -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -B build")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Source files
# ---------------------------------------------------------------------------

# CubeMX-managed: startup vector table.
set(STARTUP_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/Core/Startup/startup_stm32g491xx.s)

# CubeMX-managed: HAL drivers.
# Every HAL module .c is guarded by its own HAL_xxx_MODULE_ENABLED switch from
# stm32g4xx_hal_conf.h, so compiling the whole directory only produces empty
# objects for the peripherals this project does not use. The "_template" files
# are excluded: they define HAL_MspInit / HAL_InitTick and would clash with
# Core/Src/stm32g4xx_hal_msp.c and stm32g4xx_hal.c.
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/*.c)
list(FILTER HAL_SOURCES EXCLUDE REGEX "_template\\.c$")

# CubeMX-managed: FreeRTOS kernel + the GCC Cortex-M4F port + CMSIS-RTOS2 wrapper.
set(FREERTOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source)
set(FREERTOS_SOURCES
    ${FREERTOS_DIR}/croutine.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/CMSIS_RTOS_V2/cmsis_os2.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F/port.c
        uart/msp_uart2.c
        uart/msp_uart2.h)

# Application code: CubeMX-generated Core, plus the hand-written app/,
# algorithm/ and hardware/ folders.
file(GLOB APP_SOURCES CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/app/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/hardware/*.c)

# The IMU660 drivers and their fusion code have been retired in favour of the
# WitMotion HWT906 (I2C3, see hardware/hwt_imu.c). The old sources live in
# obsolete/imu660/ and obsolete/hwt101_legacy/, which is outside every glob
# below, so they are not built.

# GNU Arm output is an .elf; without this CMake would emit an extension-less
# file on Windows, which the debug/flash configs do not expect.
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

add_executable(${CMAKE_PROJECT_NAME}
    ${STARTUP_SOURCE}
    ${HAL_SOURCES}
    ${FREERTOS_SOURCES}
    ${APP_SOURCES})

# ---------------------------------------------------------------------------
# Include paths
# ---------------------------------------------------------------------------
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    app
    algorithm
    hardware
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
    Drivers/CMSIS/DSP/Include
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/CMSIS_RTOS_V2
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F)

# ---------------------------------------------------------------------------
# Compiler options
# ---------------------------------------------------------------------------
set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    STM32G491xx
    USE_HAL_DRIVER)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    # Cortex-M4 core flags apply to C and to the vector-table assembly alike;
    # -mfloat-abi=hard on the .s file is what keeps its ABI tag consistent with
    # the rest of the objects.
    $<$<COMPILE_LANGUAGE:C>:${MCU_FLAGS}>
    $<$<COMPILE_LANGUAGE:ASM>:${MCU_FLAGS}>
    $<$<COMPILE_LANGUAGE:C>:-ffunction-sections -fdata-sections -Wall>
    $<$<COMPILE_LANGUAGE:C>:$<$<CONFIG:Debug>:-Og -g3>>
    $<$<COMPILE_LANGUAGE:C>:$<$<CONFIG:Release>:-O2 -g0>>)

# ---------------------------------------------------------------------------
# Linker options
# ---------------------------------------------------------------------------
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32G491VETx_FLASH.ld)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    -T${LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,--start-group -lc -lm -Wl,--end-group
    # newlib-nano is the only libc that fits comfortably in flash, but its
    # printf drops %f unless the float formatter is pulled in explicitly --
    # app/GrayTrace.c and friends rely on "%.2f".
    -specs=nano.specs
    -u _printf_float
    # Uncomment if the code ever calls CMSIS-DSP (libarm_cortexM4lf_math.a is
    # built for -mfloat-abi=hard, matching MCU_FLAGS):
    # ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM4lf_math.a
)

# ---------------------------------------------------------------------------
# Post-build: .hex / .bin for the programmer, plus a size report
# ---------------------------------------------------------------------------
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex / .bin")
