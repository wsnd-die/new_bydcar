# CMake toolchain file for STM32G4 (Cortex-M4F) using the GNU Arm Embedded toolchain.
#
# Pass it at configure time:
#   cmake -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -B build
#
# The toolchain ships with STM32CubeCLT (GNU-tools-for-STM32) or with the
# standalone "GNU Arm Embedded Toolchain" installer. If yours lives somewhere
# else, override it without editing this file:
#   cmake -DARM_TOOLCHAIN_PATH="C:/path/to/bin" ...

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain discovery: honor -DARM_TOOLCHAIN_PATH, otherwise look in the
# locations STM32CubeCLT / the standalone installer use.
if(NOT DEFINED ARM_TOOLCHAIN_PATH)
  file(GLOB CUBECLT_CANDIDATES "C:/ST/STM32CubeCLT*/GNU-tools-for-STM32/bin"
                               "D:/STM32CubeCLT*/GNU-tools-for-STM32/bin"
                               "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeCLT*/GNU-tools-for-STM32/bin"
                               "E:/Tools/xpack-arm-none-eabi-gcc/*/bin")
  list(SORT CUBECLT_CANDIDATES)
  list(REVERSE CUBECLT_CANDIDATES)
  foreach(candidate IN LISTS CUBECLT_CANDIDATES)
    if(EXISTS "${candidate}/arm-none-eabi-gcc.exe" OR EXISTS "${candidate}/arm-none-eabi-gcc")
      set(ARM_TOOLCHAIN_PATH "${candidate}")
      break()
    endif()
  endforeach()
endif()

if(NOT ARM_TOOLCHAIN_PATH OR NOT EXISTS "${ARM_TOOLCHAIN_PATH}")
  message(FATAL_ERROR
    "arm-none-eabi toolchain not found.\n"
    "Re-run CMake with -DARM_TOOLCHAIN_PATH=<dir containing arm-none-eabi-gcc> "
    "(e.g. D:/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin).")
endif()

set(TOOLCHAIN_PREFIX "${ARM_TOOLCHAIN_PATH}/arm-none-eabi-")

# CMake wants the literal ".exe" name on Windows hosts.
set(TOOLCHAIN_SUFFIX "")
if(CMAKE_HOST_WIN32)
  set(TOOLCHAIN_SUFFIX ".exe")
endif()

set(CMAKE_C_COMPILER   "${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX}")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX}")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_SUFFIX}")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_SUFFIX}" CACHE FILEPATH "objcopy")
set(CMAKE_SIZE         "${TOOLCHAIN_PREFIX}size${TOOLCHAIN_SUFFIX}"    CACHE FILEPATH "size")

# Firmware images are not runnable programs on the host, so the compiler
# capability probes must link as static libraries instead of executables.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)