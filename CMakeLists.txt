# ============================================================
# DummyL-Robot 主构建脚本
# 平台: Windows PC (USB 转 RS485 -> 立三 LEESN 闭环步进电机)
# 工具链: C11 + MinGW-w64 GCC + CMake/Ninja
# 目标: lib(comm/kinematics/trajectory/control/utils) + app + tests
# 用法:
#   cmake -G Ninja -B build -DCMAKE_C_COMPILER=C:/Qt/Tools/mingw1310_64/bin/gcc.exe
#   cmake --build build
#   ctest --test-dir build
# ============================================================
cmake_minimum_required(VERSION 3.16)

project(DummyL-Robot VERSION 0.1.0 LANGUAGES C)

# ---------- 语言标准: C11 ----------
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "构建类型 (Debug/Release)" FORCE)
endif()

# 产物统一输出到 build/bin 与 build/lib
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ---------- 静态库: comm ----------
# Windows 串口封装 + Modbus RTU（含 CRC16）
add_library(comm STATIC
    src/comm/modbus_rtu.c
    src/comm/serial_win.c
)

# ---------- 静态库: kinematics ----------
# 矩阵运算 + DH 建模/正解 + 球腕解耦解析 IK
add_library(kinematics STATIC
    src/kinematics/mat3.c
    src/kinematics/dh.c
    src/kinematics/ik.c
)

# ---------- 静态库: trajectory ----------
# 点到点 梯形/S 曲线 关节插补
add_library(trajectory STATIC
    src/trajectory/planner.c
)

# ---------- 静态库: api ----------
# 电机寄存器 API（封装 Modbus 读写，供 control/monitor 等模块调用）
add_library(api STATIC
    src/api/motor_reg.c
)

# ---------- 静态库: control ----------
# 高层接口 (movej/enable/disable) + 堵转回零 + 监控
add_library(control STATIC
    src/control/robot.c
    src/control/home.c
    src/control/monitor.c
)

# ---------- 静态库: utils ----------
# 精简中文日志 + 命令行解析 + 统一错误码 + 字符串字面量集中管理
add_library(utils STATIC
    src/utils/cmd_parser.c
    src/utils/err.c
    src/utils/strings.c
)

# 公共头文件目录与警告选项
# src 用于引用 comm/kinematics/... 子模块头；项目根用于引用 config/*.h
foreach(lib comm kinematics trajectory api control utils)
    target_include_directories(${lib} PUBLIC src ${CMAKE_SOURCE_DIR})
    target_compile_options(${lib} PRIVATE -Wall -Wextra)
endforeach()

# api 依赖 comm（Modbus 帧构造）
target_link_libraries(api PUBLIC comm utils)

# control 依赖 comm/kinematics/trajectory/api/utils
target_link_libraries(control PUBLIC
    comm
    kinematics
    trajectory
    api
    utils
)

# ---------- 应用: CLI 主程序 ----------
add_executable(dummyrobot src/main.c)
# winmm：timeBeginPeriod/timeEndPeriod（抬高系统定时器精度，压低堵转轮询周期）
target_link_libraries(dummyrobot PRIVATE control utils)
if(WIN32)
    target_link_libraries(dummyrobot PRIVATE winmm)
endif()
