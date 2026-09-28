---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: 7fd4f4eb60674eb6db7e422f0766d751_f27cb0c19c3811f19467525400287e28
    ReservedCode1: OcSOClTzdyPqhxI07H5Dvxci8RPt0VHAX/NqveMucO2dWcviT8qHN+LtIvFWTgT2Qb1DG47/m2kqzFsInOAXhkBhS33vkTEJQ/lqYZtwwsr7AjxGRSRJgjkltHK00h3ED/fyxyEFMRTXpSMF3/rpl6atgs15stPa4S6EjsJXbFpkxmNiMFWtYmN6Loo=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: 7fd4f4eb60674eb6db7e422f0766d751_f27cb0c19c3811f19467525400287e28
    ReservedCode2: OcSOClTzdyPqhxI07H5Dvxci8RPt0VHAX/NqveMucO2dWcviT8qHN+LtIvFWTgT2Qb1DG47/m2kqzFsInOAXhkBhS33vkTEJQ/lqYZtwwsr7AjxGRSRJgjkltHK00h3ED/fyxyEFMRTXpSMF3/rpl6atgs15stPa4S6EjsJXbFpkxmNiMFWtYmN6Loo=
---

# DummyL-Robot

PC 端六轴机械臂控制台（C 语言，脱离单片机部署）

代码运行在 Windows PC 上，通过 USB 转 RS485 以 Modbus RTU 协议直接驱动立三（LEESN）闭环步进电机（六轴机械臂），不再依赖单片机固件。运动学、轨迹规划、电机控制逻辑全部在 PC 端完成，算法实现保持纯 C、无重量级依赖，可随时移植回单片机。

> **协议基线**：全项目一律使用立三（LEESN）485 协议，
> 寄存器表见 [docs/protocol.md](docs/protocol.md) 与 `src/control/robot_internal.h`。

---

## 1. 技术栈与开发环境

| 项 | 方案 |
|---|---|
| 语言标准 | C11 |
| 编译器 | MinGW-w64 GCC 13.10（`C:\Qt\Tools\mingw1310_64\bin`，Qt 自带） |
| 构建系统 | CMake 4.4.2 + Ninja 1.13.2 |
| 串口通信 | Windows API 直接封装（CreateFile / SetCommState / ReadFile / WriteFile），零第三方依赖 |
| 单元测试 | CTest + 轻量断言（不引第三方框架） |
| 配置管理 | 头文件宏（静态参数）+ ini 文本（运行时可调参数） |

## 2. 项目架构

### 2.1 可视化架构图

完整可视化架构图见 [docs/architecture.html](docs/architecture.html)（浏览器打开可查看交互式架构图，支持深色模式）。

### 2.2 分层架构图

```
┌─────────────────────────── PC 软件 (Windows / MinGW) ───────────────────────────┐
│                                                                                 │
│                          ┌──────────┐                                           │
│                          │  main.c  │  命令行入口                              │
│                          └────┬─────┘                                           │
│                               │                                                 │
│  ┌──┐  ┌──────────────────────┼──────────────────────┐  ┌────────────┐          │
│  │utils│ │       控制层 (control)                     │  │  算法库    │          │
│  │     │ │  robot.c   home.c   monitor.c              │  │ mat3.c     │          │
│  │logger│ │  使能/运动  堵转回零  状态监控              │  │ dh.c       │          │
│  │cmd  │ │             ┌─────┐                       │  │ ik.c       │          │
│  │err  │ │             │核心 │                       │  ├────────────┤          │
│  └──┬──┘ └─────────────┴─────┴───────────────────────┘  │ planner.c  │          │
│     │                    │                               └─────┬──────┘          │
│     │              ┌─────┘                                     │                │
│  ┌──┴──────┐  ┌────▼────────────────────────┐                 │                │
│  │ config  │  │     通信层 (comm)            │                 │                │
│  │ config.h│  │ serial_win  modbus_rtu  crc16│                 │                │
│  │ config  │  │ Win32串口   RTU主站    CRC校验 │                 │                │
│  │ .ini    │  └─────────────┬───────────────┘                 │                │
│  └─────────┘                │                                  │                │
│                             │                                  │                │
└─────────────────────────────┼──────────────────────────────────┘                │
                              │                                                   │
                    ┌─────────▼───────────────────┐                                │
                    │  USB-RS485 → 6轴步进电机    │                                │
                    │  立三 LEESN · Modbus RTU    │                                │
                    │  115200 8N1                │                                │
                    └───────────────────────────┘                                │
```

### 2.3 层级说明

| 层 | 目录 | 模块 | 职责 |
|---|---|---|---|
| 应用层 | `src/main.c` | CLI | 命令行入口，解析用户输入，调用控制层 |
| 控制层 | `src/control/` | robot.c / home.c / monitor.c | 使能/运动/状态/屏蔽、堵转回零、状态监控 |
| 通信层 | `src/comm/` | serial_win.c / modbus_rtu.c / crc16.c | Win32串口、Modbus RTU主站、CRC16校验 |
| 算法库 | `src/kinematics/` `src/trajectory/` | mat3.c / dh.c / ik.c / planner.c | 矩阵运算、DH建模、逆运动学、轨迹规划 |
| 工具层 | `src/utils/` | logger / cmd_parser / err | 中文日志、命令解析、统一错误码 |
| 配置层 | `config/` | robot_config.h / .ini | 编译期参数（宏）+ 运行时参数（ini） |

### 2.4 关键设计

- **通信抽象**：`comm_if.h` 定义 `CommOps` 函数指针表，控制层不直接依赖串口实现，可注入假串口做单测
- **内部共享**：`robot_internal.h` 暴露寄存器定义和 `robot_request()` 给 home.c，不对外公开
- **产物**：主控台 `dummyrobot.exe` + 10 个离线单元测试（`ctest --test-dir build`，当前 10/10）。
  早期附带的独立工具（`scan_motors.exe`、`servo_calib.exe`）已删除

### 2.5 数据流向

```
用户命令 → main.c → control层 → comm层 → RS485 → 电机
                     ↑    ↑
              utils  ↑    ↑  algorithm (kinematics/trajectory)
              config ↑
```

### 2.6 目录结构

```
DummyL-Robot/
├── CMakeLists.txt              # 主构建：7 静态库（comm/kinematics/trajectory/api/control/utils/cli）+ app
├── README.md                   # 本文件
├── src/
│   ├── main.c                  # 入口 + CLI 交互循环
│   ├── config/
│   │   ├── robot_config.h      # 电机ID/减速比/堵转电流/限位（宏定义）
│   │   └── robot_config.ini    # 运行时可调参数（串口端口、波特率、默认速度）
│   ├── comm/
│   │   ├── comm_if.h           # CommOps 接口层（串口帧收发统一接口抽象）
│   │   ├── serial_win.c/h      # Windows 串口封装（实现 CommOps）
│   │   ├── crc16.c/h           # CRC16 0xA001
│   │   └── modbus_rtu.c/h      # Modbus RTU 主站：03H/06H/10H/04H 帧构造与解析
│   ├── kinematics/
│   │   ├── dh.c/h              # 建模：DH 参数表 + d6 工具长度标定
│   │   ├── fk.c/h              # 正解 FK：单关节变换 / 六轴正解 / 位姿→XYZ+RPY
│   │   ├── ik.c/h              # 球腕解耦解析 IK（8 组解）+ 限位筛选/最优解选择
│   │   └── joint_zero.c/h      # 编码器读数 → 机械角（零点/方向标定）
│   ├── trajectory/
│   │   └── line.c/h            # 笛卡尔直线插补（moveL 逐点 IK）
│   ├── control/
│   │   ├── robot.c/h           # 高层接口：movej / enable / disable / 状态查询
│   │   ├── home.c/h            # 回零流程：顶限位→电流判堵转→急停→清零→就位
│   │   ├── monitor.c/h         # 状态轮询：在线检测、堵转报警
│   │   └── robot_internal.h    # 立三（LEESN）寄存器宏表 + DWORD 字节序契约
│   └── utils/
│       ├── logger.c/h          # 精简中文日志
│       ├── err.c/h             # ErrCode 统一错误码 + err_str() 中文提示
│       └── cmd_parser.c/h      # 命令行解析（movej:1:45 / home / status）
└── docs/
    └── output/                 # 产物输出目录
```

## 3. 构建与运行

```bash
# 首次构建
cmake -G Ninja -B build -DCMAKE_C_COMPILER=C:/Qt/Tools/mingw1310_64/bin/gcc.exe
ninja -C build

# 运行
build\bin\dummyrobot.exe

# 运行单元测试
ctest --test-dir build
```

构建产物：`build\bin\dummyrobot.exe`。

## 4. 硬件与通信协议

### 4.1 硬件配置

- 6× 立三（LEESN）闭环步进电机（RS485 总线），USB 转 485 连接 PC
- 波特率 115200，8 数据位 / 无校验 / 1 停止位（115200 8N1）
- 每台电机 4096 线编码器，单圈 16384 步

| 关节 | 电机型号 | 减速比 |
|---|---|---|
| 1 | 42 | 50:1 |
| 2 | 42 | 100:1 |
| 3 | 42 | 50:1 |
| 4 | 35（IP35ET） | 50:1 |
| 5 | 35（IP35ET） | 50:1 |
| 6 | 28（IP28ET） | 50:1 |

**步数换算**：`步数 = 角度° × 减速比 ÷ 360 × 16384`

### 4.2 立三（LEESN）寄存器表（Modbus RTU）

> 来源：`external/485通讯手册_sv126.1.pdf`（V126）。完整寄存器表见 [docs/protocol.md](docs/protocol.md)；
> 代码宏定义见 `src/control/robot_internal.h`。功能码 03H=读、06H=写单寄存器、10H=写多寄存器；
> CRC16 多项式 0xA001。

#### 状态与信息区
| 地址 | 功能码 | 内容 | 初始值 |
|---|---|---|---|
| 0x0004 / 0x0005 | 03H | 电机实时位置（INT32，高<<16 \| 低，单位脉冲） | -- |
| 0x0006 / 0x0007 | 03H | 状态寄存器（UINT32，位定义：bit8~9 运行 / bit12 到位 / bit15 原点 / bit21 报警） | -- |
| 0x001A | 03H | 实时电流（mA，堵转/碰撞检测依据） | -- |
| 0x00A3 | 03H | 报警状态（低 4 位报警码，0=正常） | -- |
| 0x00A4 | 06H | 清除报警（写 0 清除） | -- |

#### 运动命令区（`robot_movej` / `robot_home` 使用）
| 地址 | 功能码 | 内容 | 初始值 |
|---|---|---|---|
| 0x00C8 | 06H | 运行/停止：写 0 减速停、1 正转、256 急停、257 反转 | -- |
| 0x00C9 | 06H | 回原点：触发电机回机械原点（速度取 0x00D8~0x00D9） | -- |
| 0x00D8 / 0x00D9 | 10H | 运行速度（INT32，单位 0.01 rpm，默认 30000=300rpm） | 30000 |
| 0x0098 / 0x0099 | 06H | 加/减速时间（UINT16 ms，出厂默认 120ms） | 120 |
| 0x00E8 / 0x00E9 | 10H | 运行到绝对位置（INT32 脉冲，相对原点，运行/停止均可执行） | -- |
| 0x00D2 / 0x00D3 | 10H | 设置当前电机位置（只改位置寄存器值，物理不动，回零清零用） | -- |

#### 使能与地址区
| 地址 | 功能码 | 内容 | 初始值 |
|---|---|---|---|
| 0x00D4 | 06H | 脱机/使能/驱动重启：**写 0 使能、写 1 释放马达**；0x0100 驱动重启 | -- |
| 0x0008 | 06H | 串口超时（UINT16，单位 10ms，写 0 取消超时） | -- |
| 0x0009 | 06H | 通讯参数（低 8 位波特率码，出厂 12=115200） | 12 |
| 0x0024 | 10H | 细分（每转脉冲数，UINT32，出厂默认 4000） | 4000 |
| 0x0066 | 06H | 驱动器基地址（默认 1，多轴需逐台设置并写 0x00DC=1 保存） | 1 |
| 0x0067 | 06H | 驱动器地址源（默认 0） | 0 |
| 0x00DC | 06H | 断电保存命令：写 1 保存参数、写 0 恢复出厂 | -- |

### 4.3 回零流程与参数

回零分两阶段：

1. **找零**：组 0 = {1,2,3,5,6} 并行回零 → 组 1 = {4} 回零。立三无归零/堵转专用寄存器，
   流程为：0x00C8 速度模式顶硬限位 → 上位机读 0x001A 实时电流超阈值判定堵转 →
   急停（0x00C8=0x0100）→ 清零位置（0x00D2~0x00D3=0）；
2. **就位**：全部找零完成后，运动到机械原点位姿 `HOME_POSE_DEG`。

- 各关节归零方向 `HOME_DIR[6] = {+1, -1, +1, -1, -1, +1}`（决定 0x00C8 写 1 正转还是 257 反转）
- 回零堵转电流阈值 `HOME_STALL_CURRENT_MA[7] = {0, 500, 500, 800, 500, 500, 0} mA`（实机标定，
  下标 0 不用；立三无 0x00E1 类堵转电流寄存器，判定由上位机电流轮询完成）
- 堵转检测电流余量 `HOME_STALL_MARGIN_MA = 400 mA`
- 回零速度 `HOME_SPEED_RPM = 100 rpm`（写 0x00D8~0x00D9 运行速度寄存器，与 robot_movej 一致；
  0x009A 为 SV113 以下固件 UINT16 速度寄存器，不再使用）
- 机械原点位姿 `HOME_POSE_DEG[6]`：6 关节目标角度（度），待用户设置

## 5. CLI 命令

命令名**大小写不敏感**（`movel` / `MoveL` 都行）。下表按 `src/utils/cmd_parser.c`
实际识别的命令列出 —— 以代码为准，不以本文档为准。

| 命令 | 格式 | 作用 | 示例 |
|---|---|---|---|
| `home` | `home` / `home:N` | 回零：组0={1,2,3,5,6}并行→组1={4}→机械原点位姿；`:N` 只回零第 N 轴 | `home:3` |
| `MoveJ` | `MoveJ:N:ANGLE[:SPD][:r\|a]` | 单关节运动到 ANGLE 度；末段 `r`=相对当前位置、`a`=绝对（默认）。省略 SPD 时 60 rpm | `MoveJ:1:45` |
| `MoveJ` | `MoveJ:A1,A2,A3,A4,A5,A6,SPD,ACC,DEC` | 六轴同步关节空间运动，按行程比例分配转速（同起同停） | `MoveJ:0,0,90,0,20,0,60,100,100` |
| `MoveL` | `MoveL:X,Y,Z,Rx,Ry,Rz[,SPD,ACC,DEC][,MODE]` | 笛卡尔**直线**：位置线性插值 + 姿态四元数 SLERP，逐点 IK 且选解连续。默认 60 rpm / 80 ms / 90 ms；`MODE`=`sync`（默认，按弓高预算分段+逐航点等到位）/ `step` / `stream` / `smooth`（流畅优先，不分段） | `MoveL:241.5,52,236,-90,90,-90` |
| `enable` | `enable` / `enable:N` | 恢复使能（全部或单轴） | `enable:1` |
| `disable` | `disable` / `disable:N` | 泄力失能（全部或单轴） | `disable` |
| `motor` | `motor` | 启动/停止电机实时监控（约 1 s/次循环显示） | `motor` |
| `getpos` | `getpos` | 读当前关节角(度)、笛卡尔坐标(X,Y,Z,RPY)与法兰倾角 | `getpos` |
| `fk` | `fk:J1,...,J6` | 离线正解预览：算该组关节角的位姿，**不动臂不下发**，用来和 `getpos` 对照 | `fk:0,0,90,0,0,0` |
| `diag` | `diag` | 总线时延体检：把单事务拆成 flush/write/read 三段计时，不动臂 | `diag` |
| `bcast` | `bcast` | 广播帧验证（地址 0 写速度再逐轴读回），定位刷新率瓶颈，不动臂 | `bcast` |
| `curtest` | `curtest` / `curtest:N[:DEG[:RPM]]` | 电流实测（标定堵转阈值用）。无参数=静止采样六轴保持电流；带参数=关节 N 走 +DEG 度再回原位并全程采样。默认摆幅 2°、30 rpm | `curtest:5:2:30` |
| `stall` | `stall` / `stall:N:MA` | 查看/运行时设置逐轴堵转阈值(mA)，`0`=该轴不启用。只改本次运行，持久化写 ini `[stall]` | `stall:1:1000` |
| `poseok` | `poseok` | 人工解除「位姿不可信」闸门（零点丢失时运动命令会被锁住；确认是误判才用） | `poseok` |
| `zero` | `zero` | 显示当前零点与机械角 | `zero` |
| `zero_save` | `zero_save:v1,...,v6` | 保存指定的零点标定值（6 个电机角，度） | `zero_save:-176.66,74.58,-180.03,4.27,115.44,84.89` |
| `help` | `help` 或 `?` | 打印命令帮助 | `help` |
| `exit` | `exit` 或 `quit` | 退出程序 | `exit` |

> 关节号 N 范围 1..6。
> **已移除的命令**（文档别再写）：`status`、`mask`/`unmask`、`scan`、`calib`，
> 以及独立可执行程序 `scan_motors.exe` / `servo_calib.exe` —— 均已随工具清理删除。
> 另有未列入帮助的调试命令 `nrtest`（连发帧间延迟实测），不常用。

> **⚠️ 下发前的安全闸门**（2026-09-18/19 两次事故后加的，命令被拒时看这里的提示）：
> 目标角 NaN/Inf、目标越该轴软限位、单次位移 > ini `[safety] max_step_deg`、
> 目标步数 \|steps\| > 1e8、MoveL 规划路径单段关节跳变 > ini `[safety] max_jump_deg`
> —— 任一命中即拒绝下发，一动不动。详见 `未完成任务清单.md` 的 #12 / #13 / #14。

## 6. 设计原则

1. **三层分离 + CommOps 接口抽象**：`comm/`（帧收发，serial_win 实现 CommOps 函数指针表）→ `control/`（业务指令，仅依赖 `comm_if.h` 接口）→ `main.c`（交互），换总线/换电机不动控制层
2. **可移植**：运动学与轨迹规划为纯 C，无平台依赖，可整体搬回单片机固件
3. **配置外置**：电机减速比、DH 参数、限位集中在 `config/`，换机型不碰业务代码
4. **离线可测**：CRC / Modbus 帧 / IK / 轨迹全部可单测，不接硬件也能验证逻辑
5. **输出精简**：串口打印全中文，过程性/调试性打印无必要即删

## 7. 开发提示词

> **项目**：DummyL-Robot — PC 端六轴机械臂控制台（C 语言，脱离单片机）
> **架构**：C11 + MinGW GCC + CMake/Ninja；`comm/` 走 RS485 Modbus RTU 直驱立三（LEESN）闭环步进电机；`control/robot.c` 提供 movej/home 等高层接口；`kinematics/` 纯 C 正逆解（球腕解耦 + 限位筛选/最优解）
> **协议**：立三（LEESN）寄存器（地址 0x0066、使能 0x00D4（写0使能/写1释放）、实时位置 0x0004/5、速度 0x00D8/9（0.01rpm）、绝对位置 0x00E8/9、状态 0x0006/7、电流 0x001A、报警 0x00A3/清 0x00A4、回零位置清零 0x00D2/3）；Modbus RTU 03H/06H/10H；CRC16 0xA001；115200 8N1；脉冲 = 角度° × 减速比 ÷ 360 × 16384
> **硬件**：6× 立三闭环步进电机（1~3号 42 机座，减速比 50:1/100:1/50:1；4/5号 35 机座 IP35ET，50:1；6号 28 机座 IP28ET，50:1），4096 线编码器单圈 16384 步，USB 转 485；1~6 号全部在线
> **约束**：① 只本地 git commit，禁止 push；② 架构/接口改动先讨论再动手；③ 串口打印精简中文；④ 单位：角度用度、长度用毫米；⑤ 优先可移植到单片机的纯 C 实现，不引入重量级依赖；⑥ 运动学/轨迹/通信逻辑保持离线可单测（不接硬件）；⑦ 新模块先调研 GitHub 开源项目确认方案再写代码；⑧ 产物输出到 docs\output\
> **开发原则**：comm/control/ui 三层分离；配置头文件 + ini；离线可单测；先方案后代码

## 8. 路线图

- [x] 串口层（serial_win.c）联调验证：打开 COM 口、收发帧
- [x] Modbus RTU 主站：读状态/写使能/写位置，单关节验证
- [x] 六轴高层接口：movej / home / status
- [x] 运动学库：DH/FK/IK + 单测闭环
- [x] 轨迹规划：梯形/S 曲线插补
- [ ] 笛卡尔空间运动（moveL）
- [ ] 3D 可视化验证（可选）
*（内容由AI生成，仅供参考）*
