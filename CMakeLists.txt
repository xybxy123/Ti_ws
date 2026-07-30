cmake_minimum_required(VERSION 3.10)

# 1. 交叉編譯器設定
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# ==========================================
# 加入這行！告訴 CMake 測試時跳過可執行檔連結
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
# ==========================================

# 2. 定義專案名稱 
project(MSPM0_Project C ASM)

# ==========================================
# ✅ 加入這行！定義晶片型號，讓 SDK 載入正確的硬體暫存器 (Registers)
add_compile_definitions(__MSPM0G3507__)
# ==========================================

# 3. 定義 TI SDK 路徑
set(SDK_ROOT "$ENV{HOME}/ti/mspm0_sdk_2_11_00_07")

# 4. 指定正確的 startup 檔案路徑 (替換掉原本找不到的 GLOB)
set(STARTUP_FILE "${SDK_ROOT}/source/ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g350x_gcc.c")

# 5. 設定編譯參數 (Cortex-M0+)
set(CPU_FLAGS "-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft")
set(CMAKE_C_FLAGS "${CPU_FLAGS} -O2 -ffunction-sections -fdata-sections -g -Wall")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 6. 設定 Linker (連結器) 與 SysConfig 產生的腳本，【重點：加入 -Wl,--entry,Reset_Handler】
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/generated_output/device_linker.lds")
set(CMAKE_EXE_LINKER_FLAGS "${CPU_FLAGS} -T ${LINKER_SCRIPT} -Wl,--gc-sections -Wl,-Map=output.map -Wl,--entry,Reset_Handler --specs=nano.specs --specs=nosys.specs")

# 7. 加入標頭檔搜尋路徑
include_directories(
    ${CMAKE_SOURCE_DIR}/generated_output
    ${SDK_ROOT}/source
    ${SDK_ROOT}/source/third_party/CMSIS/Core/Include
    /home/xybxy123/w_work/Ti/generated_output
)

# 8. 定義原始碼
set(SOURCES
    main.c
    motor.c
    encoder.c           
    pid_motor.c
    generated_output/ti_msp_dl_config.c
    ${STARTUP_FILE}
)

# 9. 生成 ELF 執行檔
add_executable(firmware.elf ${SOURCES})

# 10. 連結 TI 預先編譯好的 DriverLib 靜態庫
target_link_libraries(firmware.elf
    ${SDK_ROOT}/source/ti/driverlib/lib/gcc/m0p/mspm0g1x0x_g3x0x/driverlib.a
    m
    c
    gcc
)