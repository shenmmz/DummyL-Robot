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
# Windows 串口封装 + CRC16 + Modbus RTU 主站
add_library(comm STATIC
    src/comm/crc16.c
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

# ---------- 静态库: control ----------
# 高层接口 (movej/enable/disable) + 堵转回零 + 监控
add_library(control STATIC
    src/control/robot.c
    src/control/home.c
    src/control/monitor.c
)

# ---------- 静态库: utils ----------
# 精简中文日志 + 命令行解析 + 统一错误码
add_library(utils STATIC
    src/utils/logger.c
    src/utils/cmd_parser.c
    src/utils/err.c
)

# 公共头文件目录与警告选项
# src 用于引用 comm/kinematics/... 子模块头；项目根用于引用 config/*.h
foreach(lib comm kinematics trajectory control utils)
    target_include_directories(${lib} PUBLIC src ${CMAKE_SOURCE_DIR})
    target_compile_options(${lib} PRIVATE -Wall -Wextra)
endforeach()

# control 依赖 comm/kinematics/trajectory/utils
target_link_libraries(control PUBLIC
    comm
    kinematics
    trajectory
    utils
)

# ---------- 应用: CLI 主程序 ----------
add_executable(dummyrobot src/main.c)
target_link_libraries(dummyrobot PRIVATE control utils)

# ---------- 应用: 总线电机扫描工具 ----------
add_executable(scan_motors src/tools/scan_motors.c)
target_link_libraries(scan_motors PRIVATE control utils)

# ---------- 应用: 单关节标定工具 ----------
add_executable(servo_calib src/tools/servo_calib.c)
target_link_libraries(servo_calib PRIVATE control utils)

# ---------- 测试: CTest 注册 4 个单测 ----------
enable_testing()

# 1. CRC16 向量测试
add_executable(test_crc16 tests/test_crc16.c)
target_link_libraries(test_crc16 PRIVATE comm)
add_test(NAME test_crc16 COMMAND test_crc16)

# 2. Modbus RTU 帧构造/解析测试 (不连硬件)
add_executable(test_modbus tests/test_modbus.c)
target_link_libraries(test_modbus PRIVATE comm utils)
add_test(NAME test_modbus COMMAND test_modbus)

# 3. 正解/逆解闭环测试
add_executable(test_fk_ik tests/test_fk_ik.c)
target_link_libraries(test_fk_ik PRIVATE kinematics)
add_test(NAME test_fk_ik COMMAND test_fk_ik)

# 4. 轨迹规划测试
add_executable(test_planner tests/test_planner.c)
target_link_libraries(test_planner PRIVATE trajectory)
add_test(NAME test_planner COMMAND test_planner)
