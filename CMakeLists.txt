# ============================================================
# DummyL-Robot 主构建脚本
# 平台：Windows PC（USB 转 RS485 → 立三 LEESN 闭环步进电机）
# 目标：6 个静态库（comm/kinematics/trajectory/api/control/utils/cli）+ dummyrobot 主程序
# 用法：
#   cmake -G "Visual Studio 18 2026" -B build
#   cmake --build build --config Debug
#   build\bin\Debug\dummyrobot.exe
# ============================================================
cmake_minimum_required(VERSION 3.16)

project(DummyL-Robot VERSION 0.1.0 LANGUAGES C)

if(NOT WIN32)
    message(FATAL_ERROR "本工程仅支持 Windows（serial_win.c 直接依赖 Win32 串口 API）")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "构建类型 (Debug/Release)" FORCE)
endif()

# 产物统一输出到 build/bin 与 build/lib
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 告警：MSVC 用 /W3（/Wall 噪音过大）；GCC/Clang 用 -Wall -Wextra。
# MSVC 源码 UTF-8 无 BOM，必须 /utf-8，否则中文按 GBK 解码可能吞掉字符串结束引号。
if(MSVC)
    set(WARN_FLAGS /W3)
    add_compile_options(/utf-8)
else()
    set(WARN_FLAGS -Wall -Wextra)
endif()

# ---------- 静态库 ----------
add_library(comm        STATIC
    src/comm/modbus_rtu.c
    src/comm/serial_win.c
)
add_library(kinematics  STATIC
    src/kinematics/dh.c
    src/kinematics/fk.c
    src/kinematics/ik.c
    src/kinematics/zero.c
)
add_library(trajectory  STATIC
    src/trajectory/line.c
    src/trajectory/arc.c
)
add_library(api         STATIC
    src/api/motor_reg.c
)
add_library(control     STATIC
    src/control/robot.c
    src/control/home.c
    src/control/monitor.c
)
add_library(utils       STATIC
    src/utils/cmd_parser.c
    src/utils/err.c
    src/utils/ini_rw.c
    src/utils/telemetry.c
)
add_library(cli         STATIC
    src/cli/commands.c
)

# 公共头目录与告警等级：所有静态库统一（全项目 include 均为 "<module>/<file>.h"，仅需 src/ 一层）。
foreach(lib comm kinematics trajectory api control utils cli)
    target_include_directories(${lib} PUBLIC ${CMAKE_SOURCE_DIR}/src)
    target_compile_options(${lib} PRIVATE ${WARN_FLAGS})
endforeach()

# ---------- 依赖关系 ----------
# utils: telemetry 走 UDP 回环，Windows 下链 ws2_32（PUBLIC 让下游自动继承）。
if(WIN32)
    target_link_libraries(utils PUBLIC ws2_32)
endif()
target_link_libraries(trajectory PUBLIC kinematics)
target_link_libraries(control    PUBLIC comm kinematics trajectory api utils)
target_link_libraries(api        PUBLIC comm utils control)
target_link_libraries(cli        PUBLIC control)
# ↑ api ↔ control 存在循环：motor_reg.c 回回调 robot_request/joint_slave（属 control），
#   而 control/home.c 又调 motor_read_*（属 api）。CMake 3.3+ 对静态库循环会在
#   链接命令里自动展开，无需手工重复列目标。
# cli 只显式列 control：utils/kinematics/trajectory 均沿 control PUBLIC 传递。

# ---------- 主程序 ----------
# winmm：timeBeginPeriod/timeEndPeriod 抬高系统定时器精度，压低堵转轮询周期。
add_executable(dummyrobot src/main.c)
target_compile_options(dummyrobot PRIVATE ${WARN_FLAGS})
target_link_libraries(dummyrobot PRIVATE cli)   # control/utils 沿 cli→control 传递
target_link_libraries(dummyrobot PRIVATE winmm)
