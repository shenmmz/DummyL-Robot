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

```
DummyL-Robot/
├── CMakeLists.txt              # 主构建：5 静态库（comm/kinematics/trajectory/control/utils）+ app + 4 测试目标
├── README.md                   # 本文件
├── config/
│   ├── robot_config.h          # 电机ID/减速比/堵转电流/限位（宏定义）
│   ├── dh_params.h             # DH 参数表
│   └── robot_config.ini        # 运行时可调参数（串口端口、波特率、默认速度）
├── src/
│   ├── main.c                  # 入口 + CLI 交互循环
│   ├── comm/
│   │   ├── comm_if.h           # CommOps 接口层（串口帧收发统一接口抽象）
│   │   ├── serial_win.c/h      # Windows 串口封装（实现 CommOps）
│   │   ├── crc16.c/h           # CRC16 0xA001
│   │   └── modbus_rtu.c/h      # Modbus RTU 主站：03H/06H/10H/04H 帧构造与解析
│   ├── kinematics/
│   │   ├── mat3.c/h            # 3x3 矩阵运算（纯 C）
│   │   ├── dh.c/h              # DH 建模 + 正运动学 FK
│   │   └── ik.c/h              # 球腕解耦解析 IK（8 组解）+ 限位筛选/最优解选择
│   ├── trajectory/
│   │   └── planner.c/h         # 点到点梯形/S 曲线、关节插补
│   ├── control/
│   │   ├── robot.c/h           # 高层接口：movej / enable / disable / 状态查询
│   │   ├── home.c/h            # 回零流程：顶限位→电流判堵转→急停→清零→就位
│   │   ├── monitor.c/h         # 状态轮询：在线检测、堵转报警
│   │   └── robot_internal.h    # 立三（LEESN）寄存器宏表 + DWORD 字节序契约
│   ├── utils/
│   │   ├── logger.c/h          # 精简中文日志
│   │   ├── err.c/h             # ErrCode 统一错误码 + err_str() 中文提示
│   │   └── cmd_parser.c/h      # 命令行解析（movej:1:45 / home / status）
│   └── tools/
│       ├── scan_motors.c       # 总线电机扫描
│       └── servo_calib.c       # 单关节手动调试
├── tests/
│   ├── test_crc16.c            # CRC 向量测试
│   ├── test_modbus.c           # 帧构造/解析（不连硬件）
│   ├── test_fk_ik.c            # FK→IK→FK 闭环
│   └── test_planner.c          # 轨迹规划验证
└── docs/
    ├── protocol.md             # 立三（LEESN）寄存器表 + Modbus 帧格式
    └── output/                 # 产物输出目录（技术面试题库等文档）
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

| 命令 | 格式 | 作用 | 示例 |
|---|---|---|---|
| `help` | `help` 或 `?` | 打印命令帮助 | `help` |
| `status` | `status` | 查询所有关节状态（在线/状态/位置/电流）；屏蔽关节显示"已屏蔽" | `status` |
| `enable` | `enable:N` | 使能关节 N（1..6），屏蔽关节自动跳过 | `enable:1` |
| `disable` | `disable:N` | 失能关节 N | `disable:1` |
| `movej` | `movej:N:ANGLE` | 关节 N 绝对运动到 ANGLE 度（相对零位），默认速度 3000 rpm | `movej:1:45` |
| `movej` | `movej:N:ANGLE:SPEED` | 同上，指定速度（rpm） | `movej:1:45:500` |
| `home` | `home` | 回零：组0={1,2,3,5,6}并行→组1={4}→机械原点位姿 | `home` |
| `mask` | `mask:N` | 屏蔽关节 N（跳过、不发指令、不轮询） | `mask:2` |
| `unmask` | `unmask:N` | 恢复关节 N | `unmask:2` |
| `scan` | `scan` | 提示运行独立工具 `scan_motors.exe` 扫描总线 | `scan` |
| `calib` | `calib` | 提示运行独立工具 `servo_calib.exe` 单关节标定 | `calib` |
| `exit` | `exit` 或 `quit` | 退出程序 | `exit` |

> 关节号 N 范围 1..6，1~6 号电机全部在线；故障/未安装关节可用 `mask:N` 屏蔽、`unmask:N` 恢复。
> 独立可执行程序：`scan_motors.exe`（总线扫描）、`servo_calib.exe`（单关节手动标定），均位于 `build/bin/`。

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
