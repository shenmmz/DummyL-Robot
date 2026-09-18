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
    src/kinematics/joint_zero.c
)

# ---------- 静态库: trajectory ----------
# 笛卡尔直线插补（moveL）。
# 原另有 planner.c（单关节梯形/S 曲线）已于 2026-09-18 删除：全项目零引用，
# 现行 moveL 走 line.c + 驱动器内部加减速，没用到它。需要时可从 git 取回
#   git show HEAD:src/trajectory/planner.c > src/trajectory/planner.c
#   git show HEAD:src/trajectory/planner.h > src/trajectory/planner.h
add_library(trajectory STATIC
    src/trajectory/line.c
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
# 精简中文日志 + 命令行解析（含帮助文本）+ 统一错误码
add_library(utils STATIC
    src/utils/cmd_parser.c
    src/utils/err.c
    src/utils/ini_rw.c
)

# ---------- 静态库: cli（命令实现与分发）----------
# 原 main.c 中的命令逻辑（home/movej/movel/enable/disable/motor/getpos/
# zero/zero_save/help/exit）整体迁入，main.c 只保留初始化与命令循环。
add_library(cli STATIC
    src/cli/commands.c
)
target_include_directories(cli PUBLIC src ${CMAKE_SOURCE_DIR})
target_compile_options(cli PRIVATE -Wall -Wextra)
target_link_libraries(cli PUBLIC control utils kinematics)

# 公共头文件目录与警告选项
# src 用于引用 comm/kinematics/config/... 子模块头；项目根保留给根级头文件
# 注：config/ 已于 2026-09-11 迁入 src/，故 config/*.h 走 src 路径解析
foreach(lib comm kinematics trajectory api control utils)
    target_include_directories(${lib} PUBLIC src ${CMAKE_SOURCE_DIR})
    target_compile_options(${lib} PRIVATE -Wall -Wextra)
endforeach()

# api 依赖 comm（Modbus 帧构造）
target_link_libraries(api PUBLIC comm utils)

# trajectory 依赖 kinematics（moveL 逐点逆解使用 dh/ik）
target_link_libraries(trajectory PUBLIC kinematics)

# cli 依赖 trajectory（moveL 直线插补与分段时间表）
target_link_libraries(cli PUBLIC trajectory)

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
target_compile_options(dummyrobot PRIVATE -Wall -Wextra)
# winmm：timeBeginPeriod/timeEndPeriod（抬高系统定时器精度，压低堵转轮询周期）
target_link_libraries(dummyrobot PRIVATE control utils cli)
if(WIN32)
    target_link_libraries(dummyrobot PRIVATE winmm)
endif()

# ---------- 离线工具: 驱动时序保真回放器 ----------
# 不接硬件，在 PC 上复现 MoveL 实际画出来的轨迹，用于定位
# "画不出直线 / 画不出正方形" 的成因（段弓高 vs 六轴起步相位差）。
# 构建：cmake --build build --target replay
# 用法：build/bin/replay.exe --path "150,52,175;210,52,175;..." --tx 27 --out replay.html
option(BUILD_REPLAY "构建离线回放器 replay" ON)
# 【2026-09-18】tools/replay/replay.c 当前不在工作区里，加了 EXISTS 判断：
# 源文件缺失时跳过本目标，不要让整个 CMake 配置失败、连主程序都编不了。
if(BUILD_REPLAY AND EXISTS "${CMAKE_SOURCE_DIR}/tools/replay/replay.c")
    add_executable(replay tools/replay/replay.c)
    target_include_directories(replay PRIVATE src ${CMAKE_SOURCE_DIR})
    target_compile_options(replay PRIVATE -Wall -Wextra)
    target_link_libraries(replay PRIVATE trajectory kinematics m)
    # 生成 HTML 时把 three.js / OrbitControls / 回放脚本整段内联，产出页离线自包含
    target_compile_definitions(replay PRIVATE
        REPLAY_VENDOR_DIR="${CMAKE_SOURCE_DIR}/tools/replay/vendor"
        REPLAY_SRC_DIR="${CMAKE_SOURCE_DIR}/tools/replay")
endif()

# ---------- 离线单元测试（CTest）----------
# tests 仅依赖 kinematics，不含串口与硬件，可无机械臂运行：
#   cmake --build build --target test_ik_singular
#   build/bin/test_ik_singular.exe
#
# 【2026-09-18】tests/ 目录当前不在工作区里（目录已空/被移走），
# 因此每个目标都加了 EXISTS 判断：缺哪个就跳过哪个，不要让整个 CMake
# 配置失败、连主程序 dummyrobot 都编不了。
# 把 .c 放回 tests/ 后重新 cmake 配置，这些目标会自动恢复。
enable_testing()

if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_ik_singular.c")
add_executable(test_ik_singular tests/test_ik_singular.c)
target_include_directories(test_ik_singular PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_ik_singular PRIVATE -Wall -Wextra)
target_link_libraries(test_ik_singular PRIVATE kinematics trajectory m)
add_test(NAME ik_singular COMMAND test_ik_singular)
endif()

# 腕部奇异（J5≈0）专项回归：守卫 2026-09-18 那次 home 位形 MoveL 甩出 34mm 的事故。
# 光测"有没有解"抓不到它（旧代码确实能给出能还原位姿的解，只是 8 组乱跳），
# 必须专门测解的**稳定性**：J4/J5 在 8 组解间零散布 + 相邻插补点关节不跳。
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_ik_wrist.c")
add_executable(test_ik_wrist tests/test_ik_wrist.c)
target_include_directories(test_ik_wrist PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_ik_wrist PRIVATE -Wall -Wextra)
target_link_libraries(test_ik_wrist PRIVATE kinematics trajectory m)
add_test(NAME ik_wrist COMMAND test_ik_wrist)
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_ik_verify.c")
add_executable(test_ik_verify tests/test_ik_verify.c)
target_include_directories(test_ik_verify PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_ik_verify PRIVATE -Wall -Wextra)
target_link_libraries(test_ik_verify PRIVATE kinematics trajectory m)
add_test(NAME ik_verify COMMAND test_ik_verify)
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_dh_verify.c")
add_executable(test_dh_verify tests/test_dh_verify.c)
target_include_directories(test_dh_verify PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_dh_verify PRIVATE -Wall -Wextra)
target_link_libraries(test_dh_verify PRIVATE kinematics trajectory m)
add_test(NAME dh_verify COMMAND test_dh_verify)
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_dh_calib.c")
add_executable(test_dh_calib tests/test_dh_calib.c)
target_include_directories(test_dh_calib PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_dh_calib PRIVATE -Wall -Wextra)
target_link_libraries(test_dh_calib PRIVATE kinematics m)
add_test(NAME dh_calib COMMAND test_dh_calib)
endif()

# 复现用户 MoveL:150,52,170,-180,0,-180 的规划链路（纯 Z 下降，笔是否保持竖直）
# CRC16 / 帧构造离线单测（T8）：纯算法，不碰串口，硬件不在也能跑。
# 直接把 modbus_rtu.c 编进来，避免 comm 库拉进串口依赖。
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_crc16.c")
add_executable(test_crc16 tests/test_crc16.c src/comm/modbus_rtu.c)
target_include_directories(test_crc16 PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_crc16 PRIVATE -Wall -Wextra)
add_test(NAME crc16 COMMAND test_crc16)
endif()

# 堵转/碰撞保护单测：monitor_stall_hit 判定边界 + ini [stall] 逐轴解析。
# 只调用纯函数与文件解析，不开串口、不需要机械臂在场。
# 链接 control 库仅为拿到 monitor_stall_hit 的实现（monitor.c）。
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_stall.c")
add_executable(test_stall tests/test_stall.c)
target_include_directories(test_stall PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_stall PRIVATE -Wall -Wextra)
target_link_libraries(test_stall PRIVATE control utils)
add_test(NAME stall COMMAND test_stall)
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_z170.c")
add_executable(test_z170 tests/test_z170.c)
target_include_directories(test_z170 PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_z170 PRIVATE -Wall -Wextra)
target_link_libraries(test_z170 PRIVATE kinematics trajectory m)
add_test(NAME z170 COMMAND test_z170)
endif()
