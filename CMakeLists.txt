# ============================================================
# DummyL-Robot 主构建脚本
# 平台: Windows PC (USB 转 RS485 -> 立三 LEESN 闭环步进电机)
# 工具链: C11 + MinGW-w64 GCC + CMake/Ninja（也可用 Visual Studio 生成器）
# 目标: lib(comm/kinematics/trajectory/control/utils) + app + tests
# 用法:
#   cmake -G Ninja -B build -DCMAKE_C_COMPILER=C:/Qt/Tools/mingw1310_64/bin/gcc.exe
#   cmake --build build
#   ctest --test-dir build
# ⚠️ 用 Visual Studio 生成器时，若环境同时设了 HTTP_PROXY 与 http_proxy（仅大小写不同），
#    MSBuild 会因字典键冲突报 MSB6001 ⇒ 先 unset 小写那个再编:
#      env -u http_proxy -u https_proxy cmake --build build --config Debug
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

# ---------- 告警开关: GCC/Clang 与 MSVC 不同名，别写死 ----------
# ⚠️ MSVC 不认 -Wextra（报 D8021 无效的数值参数），-Wall 会被当成 /Wall（噪音极大）。
#    本工程现在两种编译器都可能用（MinGW/Ninja 与 Visual Studio），所以按编译器分支。
if(MSVC)
    set(WARN_FLAGS /W3)
    # 源码是 UTF-8 无 BOM。不加 /utf-8 会把中文按 GBK(936) 解码 ⇒ C4819，
    # 严重时吞掉字符串字面量的结束引号 ⇒ C2001「字符串字面量中的换行符」
    # （实测 ik.c:92 case IK_SOL_OUT_OF_REACH: return "超可达"; 就是这么炸的）。
    add_compile_options(/utf-8)
else()
    set(WARN_FLAGS -Wall -Wextra)
endif()

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
    src/utils/telemetry.c
)

# telemetry 用 UDP 回环发给 3D 镜像，Windows 下要链 Winsock。
# 注意：只在 Windows 链，别把它挂到 utils 上让所有 target 都背这个依赖。
if(WIN32)
    target_link_libraries(utils PUBLIC ws2_32)
endif()

# ---------- 静态库: cli（命令实现与分发）----------
# 原 main.c 中的命令逻辑（home/movej/movel/enable/disable/motor/getpos/
# zero/zero_save/help/exit）整体迁入，main.c 只保留初始化与命令循环。
add_library(cli STATIC
    src/cli/commands.c
)
target_include_directories(cli PUBLIC src ${CMAKE_SOURCE_DIR})
target_compile_options(cli PRIVATE ${WARN_FLAGS})
target_link_libraries(cli PUBLIC control utils kinematics)

# 公共头文件目录与警告选项
# src 用于引用 comm/kinematics/config/... 子模块头；项目根保留给根级头文件
# 注：config/ 已于 2026-09-11 迁入 src/，故 config/*.h 走 src 路径解析
foreach(lib comm kinematics trajectory api control utils)
    target_include_directories(${lib} PUBLIC src ${CMAKE_SOURCE_DIR})
    target_compile_options(${lib} PRIVATE ${WARN_FLAGS})
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
target_compile_options(dummyrobot PRIVATE ${WARN_FLAGS})
# winmm：timeBeginPeriod/timeEndPeriod（抬高系统定时器精度，压低堵转轮询周期）
target_link_libraries(dummyrobot PRIVATE control utils cli)
if(WIN32)
    target_link_libraries(dummyrobot PRIVATE winmm)
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
target_compile_options(test_ik_singular PRIVATE ${WARN_FLAGS})
target_link_libraries(test_ik_singular PRIVATE kinematics trajectory m)
add_test(NAME ik_singular COMMAND test_ik_singular)
endif()

# 腕部奇异（J5≈0）专项回归：守卫 2026-09-18 那次 home 位形 MoveL 甩出 34mm 的事故。
# 光测"有没有解"抓不到它（旧代码确实能给出能还原位姿的解，只是 8 组乱跳），
# 必须专门测解的**稳定性**：J4/J5 在 8 组解间零散布 + 相邻插补点关节不跳。
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_ik_wrist.c")
add_executable(test_ik_wrist tests/test_ik_wrist.c)
target_include_directories(test_ik_wrist PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_ik_wrist PRIVATE ${WARN_FLAGS})
target_link_libraries(test_ik_wrist PRIVATE kinematics trajectory m)
add_test(NAME ik_wrist COMMAND test_ik_wrist)
endif()

# 直线插补的 NaN 防线：守卫 2026-09-18 那次"34mm 乱甩 + 超时急停"的真正根因
# —— home 姿态 Ry=90° 正踩万向锁，quat_to_euler 的 asin 自变量浮点越界
# （1.0000000000000002）返回 NaN，一路转成 ±2147483647 的垃圾目标写给驱动器。
# 三条防线（asin clamp / line_plan 查有限 / movej_issue 拒发）逐条验证。
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_line_nan.c")
add_executable(test_line_nan tests/test_line_nan.c)
target_include_directories(test_line_nan PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_line_nan PRIVATE ${WARN_FLAGS})
target_link_libraries(test_line_nan PRIVATE kinematics trajectory m)
add_test(NAME line_nan COMMAND test_line_nan)
endif()

# 下发前的目标合理性闸门：守卫"把垃圾目标写给驱动器"这一类事故。
# 本机出过两次，目标值都是【完全合法的 int32】（NaN→±2147483647、字序写反→3.28 亿），
# 所以"查 NaN"挡不住，只有量级闸门能拦。解析与量级判定是纯函数，可离线测。
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_safety_gate.c")
add_executable(test_safety_gate tests/test_safety_gate.c)
target_include_directories(test_safety_gate PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_safety_gate PRIVATE ${WARN_FLAGS})
# 注意：control 与 api 互相引用（control→api 取 motor_*，api→control 取
# robot_request/joint_slave），静态库有环，必须重复列出让链接器回扫。
target_link_libraries(test_safety_gate PRIVATE control api control utils)
add_test(NAME safety_gate COMMAND test_safety_gate)
endif()

# 球腕几何自洽性：IK 解析算法成立的前提（三轴共点、相邻轴垂直、θ5=0 为奇异点、
# TCP 在 J6 轴上、wrist-flip 等价）。同时算出清单 #5/#6（J4/J6 符号、J5 零点）
# 真机验证所需的预测值 —— 这些量光靠读数是查不出来的（读数+FK 永远自洽）。
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_wrist_geom.c")
add_executable(test_wrist_geom tests/test_wrist_geom.c)
target_include_directories(test_wrist_geom PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_wrist_geom PRIVATE ${WARN_FLAGS})
target_link_libraries(test_wrist_geom PRIVATE kinematics m)
add_test(NAME wrist_geom COMMAND test_wrist_geom)
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_ik_verify.c")
add_executable(test_ik_verify tests/test_ik_verify.c)
target_include_directories(test_ik_verify PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_ik_verify PRIVATE ${WARN_FLAGS})
target_link_libraries(test_ik_verify PRIVATE kinematics trajectory m)
add_test(NAME ik_verify COMMAND test_ik_verify)
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_dh_verify.c")
add_executable(test_dh_verify tests/test_dh_verify.c)
target_include_directories(test_dh_verify PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_dh_verify PRIVATE ${WARN_FLAGS})
target_link_libraries(test_dh_verify PRIVATE kinematics trajectory m)
add_test(NAME dh_verify COMMAND test_dh_verify)
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_dh_calib.c")
add_executable(test_dh_calib tests/test_dh_calib.c)
target_include_directories(test_dh_calib PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_dh_calib PRIVATE ${WARN_FLAGS})
target_link_libraries(test_dh_calib PRIVATE kinematics m)
add_test(NAME dh_calib COMMAND test_dh_calib)
endif()

# 复现用户 MoveL:150,52,170,-180,0,-180 的规划链路（纯 Z 下降，笔是否保持竖直）
# CRC16 / 帧构造离线单测（T8）：纯算法，不碰串口，硬件不在也能跑。
# 直接把 modbus_rtu.c 编进来，避免 comm 库拉进串口依赖。
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_crc16.c")
add_executable(test_crc16 tests/test_crc16.c src/comm/modbus_rtu.c)
target_include_directories(test_crc16 PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_crc16 PRIVATE ${WARN_FLAGS})
add_test(NAME crc16 COMMAND test_crc16)
endif()

# 堵转/碰撞保护单测：monitor_stall_hit 判定边界 + ini [stall] 逐轴解析。
# 只调用纯函数与文件解析，不开串口、不需要机械臂在场。
# 链接 control 库仅为拿到 monitor_stall_hit 的实现（monitor.c）。
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_stall.c")
add_executable(test_stall tests/test_stall.c)
target_include_directories(test_stall PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_stall PRIVATE ${WARN_FLAGS})
target_link_libraries(test_stall PRIVATE control utils)
add_test(NAME stall COMMAND test_stall)
endif()

# MoveL "省略姿态 / keep 关键字" 解析行为单测。
# 锁死【向后兼容】红线：6 个数字段不带 keep 时必须仍解释为 X,Y,Z,Rx,Ry,Rz，
# 一旦改坏，用户所有老命令会静默变成"速度参数"。
# 只调 cmd_parse 纯解析，不开串口、不需要机械臂在场。
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_movel_keep.c")
add_executable(test_movel_keep tests/test_movel_keep.c)
target_include_directories(test_movel_keep PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_movel_keep PRIVATE ${WARN_FLAGS})
target_link_libraries(test_movel_keep PRIVATE utils)
add_test(NAME movel_keep COMMAND test_movel_keep)
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/tests/test_z170.c")
add_executable(test_z170 tests/test_z170.c)
target_include_directories(test_z170 PRIVATE src ${CMAKE_SOURCE_DIR})
target_compile_options(test_z170 PRIVATE ${WARN_FLAGS})
target_link_libraries(test_z170 PRIVATE kinematics trajectory m)
add_test(NAME z170 COMMAND test_z170)
endif()

# ============================================================
# 离线诊断工具（不是 ctest —— 都需要命令行参数，无法无人值守跑）
# ------------------------------------------------------------
# 【2026-09-21 整理】这四个此前只靠文件头注释里的 gcc 命令手工编译，
# CMake 里没有目标 ⇒ 源码在但构建系统不认，属于事实上的孤儿文件。
# 现已登记，统一走 cmake --build build --target <名字>，
# 产物与其它目标一样落在 build/bin/。
# 用法摘要（详见各文件头部注释）：
#   movl_tip_trace   J1..J6  X,Y,Z,Rx,Ry,Rz [tool_length]   笔尖 vs 法兰轨迹
#   movl_reach       J1..J6  X,Y,Z,Rx,Ry,Rz                 下发前可达性预检
#   ik_pose_check    X,Y,Z,Rx,Ry,Rz  J1..J6                 位姿↔关节角双向核对
#   udp_telemetry_test                                        UDP 遥测链路自检
# ============================================================
foreach(tool movl_tip_trace movl_reach ik_pose_check movl_verify)
    if(EXISTS "${CMAKE_SOURCE_DIR}/tests/${tool}.c")
        add_executable(${tool} tests/${tool}.c)
        target_include_directories(${tool} PRIVATE src ${CMAKE_SOURCE_DIR})
        target_compile_options(${tool} PRIVATE ${WARN_FLAGS})
        target_link_libraries(${tool} PRIVATE kinematics trajectory m)
    endif()
endforeach()

# UDP 遥测要链 Winsock（utils 已 PUBLIC 引出 ws2_32）
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/udp_telemetry_test.c")
    add_executable(udp_telemetry_test tests/udp_telemetry_test.c)
    target_include_directories(udp_telemetry_test PRIVATE src ${CMAKE_SOURCE_DIR})
    target_compile_options(udp_telemetry_test PRIVATE ${WARN_FLAGS})
    target_link_libraries(udp_telemetry_test PRIVATE utils)
endif()
