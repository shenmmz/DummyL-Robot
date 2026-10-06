# DummyL-Robot

PC 端六轴机械臂控制台（C 语言，脱离单片机部署）

代码运行在 Windows PC 上，通过 USB 转 RS485 以 Modbus RTU 协议直接驱动立三（LEESN）闭环步进电机（六轴机械臂），不再依赖单片机固件。运动学、轨迹规划、电机控制逻辑全部在 PC 端完成，算法实现保持纯 C、无重量级依赖，可随时移植回单片机。

> **协议基线**：全项目一律使用立三（LEESN）485 协议，寄存器来源见
> `external/485通讯手册_sv126.1.md`（V126）与代码宏表 `src/control/robot_internal.h`。

---

## 1. 技术栈与开发环境

| 项 | 方案 |
|---|---|
| 语言标准 | C11 |
| 编译器 | **当前主用 Visual Studio 2026（MSVC cl.exe）**；亦支持 MinGW-w64 GCC（`C:\Qt\Tools\mingw1310_64\bin`） |
| 构建系统 | CMake 3.16+，生成器二选一：Visual Studio（默认，产物落 `build\bin\Debug\`）或 Ninja + MinGW（产物落 `build\bin\`） |
| 串口通信 | Windows API 直接封装（CreateFile / SetCommState / ReadFile / WriteFile），零第三方依赖；CRC16（0xA001）内联在 `modbus_rtu.c` |
| 单元测试 | CTest + 轻量断言（不引第三方框架）；`tests/` 目录当前不在工作区，CMake 按文件存在与否自动跳过对应目标 |
| 配置管理 | 头文件宏（静态参数）+ ini 文本（运行时可调参数） |
| 遥测 | `utils/telemetry.c` 以 UDP 回环（ini `[telemetry] port=9900`）把位姿发给 3D 镜像程序 |

## 2. 项目架构

### 2.1 可视化架构图

完整交互式架构图见 `docs/architecture.html`（本地文件，浏览器打开，支持深色模式）。

### 2.2 分层架构图

```
              main.c  命令行入口 + 交互循环
                          │
                          ▼
   ┌────────────────  cli (commands.c)  ────────────────┐
   │  命令分发：home/movej/movel/enable/getpos/探针…     │
   └───────────────────────┬─────────────────────────────┘
                           ▼
   ┌────────────────  control  ─────────────────────────┐
   │  robot.c 高层接口   home.c 堵转回零   monitor.c 监控 │
   └──────┬───────────────────────────────┬─────────────┘
          │ 调用                           │ 调用
          ▼                               ▼
   ┌── kinematics ──┐  ┌── trajectory ──┐  ┌── api (motor_reg) ──┐
   │ dh / fk / ik   │  │ line.c 直线插补 │  │ 寄存器读写封装       │
   │   / zero       │  │ arc.c 圆弧插补  │  │ (motor_write/read…) │
   └────────────────┘  └────────────────┘  └──────────┬──────────┘
                                                      ▼
   ┌────────────────  comm  ───────────────────────────────┐
   │  serial_win.c Win32串口   modbus_rtu.c RTU主站+CRC16    │
   │  comm_if.h CommOps 抽象（可注入假串口做单测）           │
   └───────────────────────────┬────────────────────────────┘
                               ▼
              USB-RS485 → 6 轴立三 LEESN 步进电机
              Modbus RTU · 921600 8N1 · 10000 脉冲/转

   横向支撑：utils (cmd_parser / err / ini_rw / telemetry)、config (robot_config.h / .ini)
```

### 2.3 层级说明

| 层 | 目录 | 模块 | 职责 |
|---|---|---|---|
| 应用层 | `src/main.c` | — | 初始化 + CLI 交互循环 |
| 命令层 | `src/cli/` | commands.c | 各命令实现与分发（运动、标定、诊断、探针） |
| 控制层 | `src/control/` | robot.c / home.c / monitor.c | 使能/运动/状态、堵转回零、监控 |
| 电机 API | `src/api/` | motor_reg.c | 寄存器读写封装（供 control/cli 调用） |
| 通信层 | `src/comm/` | serial_win.c / modbus_rtu.c / comm_if.h | Win32 串口、Modbus RTU 主站、CRC16 |
| 算法库 | `src/kinematics/` `src/trajectory/` | dh / fk / ik / zero / line / arc | DH 建模、正解、球腕解耦逆解、零点换算、直线插补、三点圆弧插补 |
| 工具层 | `src/utils/` | cmd_parser / err / ini_rw / telemetry | 命令解析、错误码、ini 读写、UDP 遥测 |
| 配置层 | `src/config/` | robot_config.h / robot_config.ini | 编译期参数（宏）+ 运行时参数（ini） |

### 2.4 关键设计

- **通信抽象**：`comm_if.h` 定义 `CommOps` 函数指针表，控制层不直接依赖串口实现，可注入假串口做单测
- **内部共享**：`robot_internal.h` 暴露寄存器定义和 `robot_request()`，不对外公开
- **构建模块化**：7 个静态库（comm / kinematics / trajectory / api / control / utils / cli）+ 主程序 `dummyrobot`
- **产物**：主控台 `dummyrobot.exe`；离线单测与诊断工具登记在 CMake（`tests/` 缺文件时自动跳过）

### 2.5 数据流向

```
用户命令 → main.c → cli → control → api → comm → RS485 → 电机
                        ↑        ↘ kinematics/trajectory（IK/插补）
                        ↘ utils(cmd_parser/ini)   ↗
                     config(.h/.ini)
```

### 2.6 目录结构

```
DummyL-Robot/
├── CMakeLists.txt              # 7 静态库（comm/kinematics/trajectory/api/control/utils/cli）+ app + tests/tools
├── README.md                   # 本文件
├── src/
│   ├── main.c                  # 入口 + CLI 交互循环
│   ├── api/
│   │   └── motor_reg.c/h       # 电机寄存器读写 API（motor_write_u16 / motor_read_position / …）
│   ├── cli/
│   │   └── commands.c/h        # 命令实现与分发（home/movel/探针/诊断…）
│   ├── config/
│   │   ├── robot_config.h      # 减速比/编码器脉冲/零点/限位（宏定义）
│   │   └── robot_config.ini    # 运行时可调参数（串口端口/波特率、[movel]、[stall]、[tool]…）
│   ├── comm/
│   │   ├── comm_if.h           # CommOps 接口层（串口帧收发统一抽象）
│   │   ├── serial_win.c/h      # Windows 串口封装（实现 CommOps）
│   │   └── modbus_rtu.c/h      # Modbus RTU 主站：03H/06H/10H 帧构造与解析（含 CRC16 0xA001）
│   ├── kinematics/
│   │   ├── dh.c/h              # DH 参数表 + d6 工具长度标定
│   │   ├── fk.c/h              # 正解 FK：单关节变换 / 六轴正解 / 位姿→XYZ+RPY
│   │   ├── ik.c/h              # 球腕解耦解析 IK（8 组解）+ 限位筛选/最优解选择
│   │   └── zero.c/h            # 编码器读数 → 机械角（零点/方向换算）
│   ├── trajectory/
│   │   ├── line.c/h            # 笛卡尔直线插补（moveL 逐点 IK + 分段时间表）
│   │   └── arc.c/h             # 三点式空间圆弧插补（MoveC 定圆 + 等步长采样 + 近共线退化）
│   ├── control/
│   │   ├── robot.c/h           # 高层接口：movej / enable / disable / 状态查询
│   │   ├── home.c/h            # 回零流程：顶限位→电流判堵转→急停→清零→就位
│   │   ├── monitor.c/h         # 状态轮询：在线检测、堵转报警
│   │   └── robot_internal.h    # 立三（LEESN）寄存器宏表 + 字节序契约
│   └── utils/
│       ├── cmd_parser.c/h      # 命令行解析 + 分级帮助文本
│       ├── err.c/h             # ErrCode 统一错误码 + err_str() 中文提示
│       ├── ini_rw.c/h          # ini 读取
│       └── telemetry.c/h       # UDP 位姿遥测（供 3D 镜像）
├── tasks/                      # .rbt 轨迹脚本存放目录
├── external/                   # 电机驱动资料（485 手册 md/pdf、参考照片）
├── docs/                       # 本地文档/架构图（不推云端，见 .gitignore）
└── second/                     # 参考实现（本地保留，不推云端）
```

## 3. 构建与运行

**当前主用 Visual Studio 2026（MSVC）生成器：**

```bash
# 首次配置（默认走本机 Visual Studio 生成器）
cmake -B build
# 编译
cmake --build build --config Debug
# 运行（VS 生成器产物带 Debug 子目录）
build\bin\Debug\dummyrobot.exe
```

**可选：MinGW-w64 + Ninja：**

```bash
cmake -G Ninja -B build -DCMAKE_C_COMPILER=C:/Qt/Tools/mingw1310_64/bin/gcc.exe
cmake --build build
build\bin\dummyrobot.exe
```

**单元测试**（仅当 `tests/` 目录存在时才会配置）：

```bash
ctest --test-dir build
```

> ⚠️ 若同时设了 `HTTP_PROXY` 与 `http_proxy`（仅大小写不同），MSBuild 会因字典键冲突报
> `MSB6001` ⇒ 先取消小写那个：`Remove-Item Env:\http_proxy`。

## 4. 硬件与通信协议

### 4.1 硬件配置

- 6× 立三（LEESN）闭环步进电机（RS485 总线），USB 转 485 连接 PC
- 波特率 **921600**，8 数据位 / 无校验 / 1 停止位（921600 8N1；六轴驱动器 `0x0009` 已固化为档位 15）
- 每转脉冲数（细分）**10000**（`0x0024`，出厂 4000，启动时写后读回比对并对齐到 10000）

| 关节 | 电机型号 | 减速比 |
|---|---|---|
| 1 | 42 | 50:1 |
| 2 | 42 | 100:1 |
| 3 | 42 | 50:1 |
| 4 | 35（IP35ET） | 50:1 |
| 5 | 35（IP35ET） | 50:1 |
| 6 | 28（IP28ET） | 50:1 |

**步数换算**（`ENCODER_STEPS_PER_REV = 10000`）：`步数 = 角度° × 减速比 ÷ 360 × 10000`

### 4.2 立三（LEESN）寄存器表（Modbus RTU）

> 来源：`external/485通讯手册_sv126.1.md`（V126）；代码宏定义见 `src/control/robot_internal.h`。
> 功能码 03H=读、06H=写单寄存器、10H=写多寄存器；CRC16 多项式 0xA001。

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
| 0x00D8 / 0x00D9 | 10H | 运行速度（INT32，单位 0.01 rpm） | 30000 |
| 0x0098 / 0x0099 | 06H | 加/减速时间（UINT16 ms，出厂 120ms；上位机按 ini `[movel] acc_floor_ms` 抬高下限） | 120 |
| 0x00E8 / 0x00E9 | 10H | 运行到绝对位置（INT32 脉冲，相对原点） | -- |
| 0x00D2 / 0x00D3 | 10H | 设置当前电机位置（只改位置寄存器值，物理不动，回零清零用） | -- |

#### 使能与地址区
| 地址 | 功能码 | 内容 | 初始值 |
|---|---|---|---|
| 0x00D4 | 06H | 脱机/使能/驱动重启：**写 0 使能、写 1 释放马达**；0x0100 驱动重启 | -- |
| 0x0008 | 06H | 串口超时（UINT16，单位 10ms，写 0 取消超时） | -- |
| 0x0009 | 06H | 通讯参数（低 8 位波特率码，出厂 12=115200；**当前六轴固化为 15=921600**） | 12 |
| 0x0024 | 10H | 细分（每转脉冲数，UINT32，出厂 4000；**启动对齐到 10000**） | 4000 |
| 0x0066 | 06H | 驱动器基地址（默认 1，多轴需逐台设置并写 0x00DC=1 保存） | 1 |
| 0x0067 | 06H | 驱动器地址源（默认 0） | 0 |
| 0x00DC | 06H | 断电保存命令：写 1 保存参数、写 0 恢复出厂 | -- |

### 4.3 回零流程与参数

堵转回零（无传感器归零）分阶段：

1. **找零**：堵转组 `{2, 3, 5, 1}` 并行启动（进恒力矩模式顶硬限位 → 上位机读 `0x001A` 实时电流超阈值判堵转 → 急停 → `0x00D2` 清零）；
   **J4 特殊**：等 J3 撞停后才启动（避免与 J3 打架）；**J6 用传感器法**（光电/限位找原点，非堵转）。
2. **就位**：各轴清零后退让到配置的机械角，全部完成后运动到机械原点位姿 `ROBOT_HOME_MECH_DEG = {0, 0, 90, 0, 0, 0}`。

- 各关节归零方向 `HOME_DIR[6] = {+1, -1, +1, -1, -1, +1}`
- 堵转电流阈值来自 ini `[stall]`（j1..j6，单位 mA，`0`=该轴不启用）；判定由上位机电流轮询完成（立三无专用堵转寄存器）
- J6 传感器回零速度 `HOME_J6_ZERO_RPM = 100 rpm`
- 回零清零后偶见「位置非 0」警告：`0x00D2` 是 RAM 无记忆寄存器 + 齿轮回程间隙回弹所致，属固有残留，本次已改用相对位移保证行程

## 5. CLI 命令

命令名**大小写不敏感**（`movel` / `MoveL` 都行），参数分隔符用**冒号 `:`**（不是分号）。
下表与程序内 `help` 一致 —— **以 `src/utils/cmd_parser.c` 实际解析为准**。输入 `help` 看速查索引，
`help:motion / enable / state / diag / probe / all` 看分组详解。

### 运动

| 命令 | 格式 | 作用 | 示例 |
|---|---|---|---|
| `home` | `home` / `home:N` | 回零；`:N` 仅回零第 N 轴 | `home:3` |
| `MoveJ` | `MoveJ:N:ANGLE[:SPD][:r\|a]` | 单关节运动到 ANGLE 度；`r`=相对、`a`=绝对(默认)；省略 SPD=60rpm | `MoveJ:1:45` |
| `MoveJ` | `MoveJ:a1,a2,a3,a4,a5,a6,SPD,ACC,DEC` | 六轴同步关节空间运动，按行程比例分配转速（同起同停） | `MoveJ:0,0,90,0,90,0,70,80,90` |
| `MoveL` | `MoveL:X,Y,Z,Rx,Ry,Rz,SPD,ACC,DEC[,smooth]` | 笛卡尔**直线**：**必须写满 9 段（含显式姿态 Rx,Ry,Rz）**，3/6 段简写与 `keep`/`stream`/`sync` 均已移除。默认 **interp**（按偏差预算反推步长、逐航点等到位、每段过读回/超时急停保护，末端贴直线但段间有加减速停顿）；显式 `,smooth` = 终点一次 MoveJ、零段间停顿但末端走弧 | `MoveL:150,0,120,-180,0,-180,70,80,90` |
| `MoveC` | `MoveC:X,Y,Z,Rx,Ry,Rz,VX,VY,VZ,SPD,ACC,DEC` | 三点式空间**圆弧**：起点=当前位姿，toPoint=(X..Rz)，viaPoint=(VX,VY,VZ) 定凸向；恒 interp | `MoveC:100,0,80,-180,0,-180,50,50,90,60,80,90` |
| `MoveC` | `MoveC:<起点名>,<via名>,<终点名>[,SPD,ACC,DEC]` | 具名三点式（三个点须为 X 笛卡尔点位） | `MoveC:arc_start,arc_mid,arc_end` |
| `Run` | `run:<文件>` / `run <文件>` | 执行 .rbt 轨迹脚本（自动补 tasks/ 路径与 .rbt 后缀） | `run test` |
| `TARGET` | `TARGET <名>={J1:v ...}` | 定义命名点位（关节型或笛卡尔型） | `TARGET P1={J1:0 J2:0 J3:90 J4:0 J5:0 J6:0}` |
| `cd` / `save` | `cd:<文件>` / `save` | 进入/退出 .rbt 示教录制模式；`getpos:j/x` 追加点位 | `cd mypath` |

### 使能 / 状态 / 标定（均不动臂或只读）

| 命令 | 格式 | 作用 |
|---|---|---|
| `enable` | `enable` / `enable:N` | 恢复使能（全部或单轴） |
| `disable` | `disable` / `disable:N` | 泄力失能（全部或单轴） |
| `getpos` | `getpos` | 读当前关节角、笛卡尔位姿(X,Y,Z,R,P,Y)与法兰倾角 |
| `motor` | `motor` | 启/停电机实时监控（约 1s/次循环显示） |
| `fk` | `fk:J1,J2,J3,J4,J5,J6` | 离线正解预览：算该组关节角位姿，**不动臂不下发**，与 `getpos` 对照 |
| `zero` | `zero` | 显示当前零点与机械角 |
| `zero_save` | `zero_save:v1,...,v6` | 保存零点标定值（6 个电机角，度） |
| `poseok` | `poseok` | 人工解除「位姿不可信」闸门（零点丢失时运动命令被锁，确认误判才用） |
| `stall` | `stall` / `stall:N:MA` | 查看/运行时设逐轴堵转阈值(mA)，`0`=关闭该轴；持久化写 ini `[stall]` |
| `alarm` | `alarm` / `alarm:clear` | 报警查看 / 清除 |

### 总线诊断（均不动臂）

`diag`（时延体检）、`busrate`（485 极限速率）、`pipe`（流水线批量读探针）、
`progread:N`（编程区只读转储）、`bcast`（广播帧验证）、`accel`（加减速时间读写，断电即失）、
`drvbaud`（波特率读/设/固化 ⚠失联风险）、`looptest`（转换器极限 ⚠须脱离电机）。

### 驱动器探针（⚠ 会真动臂）

`curtest`（电流实测，标定堵转阈值）、`tabtest`（表格执行 0x00DD 试验）、
`trigtest`（表格重复触发试验）、`queuetest`（排队寄存器 0x00CE 试验）、`chain`（补链排队试验）。

### 系统

`help[:topic]`（分级帮助）、`exit` / `quit`（退出）。

> **已移除的命令**（文档别再写）：`status`、`mask`/`unmask`、`scan`、`calib`，独立程序
> `scan_motors.exe` / `servo_calib.exe`，`stream` 模式，MoveL 的 3/6 段简写与 `keep` 后缀。

> **⚠️ 下发前的安全闸门**（命令被拒时看这里）：目标角 NaN/Inf、目标越该轴软限位、
> 单次位移 > ini `[safety] max_step_deg`、目标步数 \|steps\| > `ROBOT_ABS_MOVE_STEPS_LIMIT`
> (1e8)、MoveL 单段关节跳变 > ini `[safety] max_jump_deg` —— 任一命中即拒绝下发、一动不动。

## 6. 设计原则

1. **三层分离 + CommOps 接口抽象**：`comm/`（帧收发）→ `control/`（业务指令）→ `cli`/`main`（交互），换总线/换电机不动控制层
2. **可移植**：运动学与轨迹规划为纯 C，无平台依赖，可整体搬回单片机固件
3. **配置外置**：减速比、DH 参数、限位、加减速/偏差预算集中在 `config/`，换机型不碰业务代码
4. **离线可测**：CRC / Modbus 帧 / IK / 轨迹 / 解析均可单测，不接硬件也能验证逻辑
5. **输出精简**：串口打印全中文，过程性/调试性打印无必要即删

## 7. 开发提示词

> **项目**：DummyL-Robot — PC 端六轴机械臂控制台（C 语言，脱离单片机）
> **架构**：C11 + CMake；生成器当前用 Visual Studio（MSVC），亦支持 MinGW/Ninja；`comm/` 走 RS485 Modbus RTU 直驱立三（LEESN）闭环步进；`control/robot.c` 提供 movej/home 高层接口；`cli/commands.c` 命令分发；`kinematics/` 纯 C 正逆解（球腕解耦 + 限位筛选/最优解）；`trajectory/` 直线插补 (line.c) + 三点圆弧插补 (arc.c)；`.rbt` 轨迹脚本 + 示教录制
> **协议**：立三（LEESN）寄存器（使能 0x00D4 写0使能/写1释放、实时位置 0x0004/5、速度 0x00D8/9（0.01rpm）、绝对位置 0x00E8/9、状态 0x0006/7、电流 0x001A、报警 0x00A3/清 0x00A4、回零清零 0x00D2/3、加减速 0x0098/99）；Modbus RTU 03H/06H/10H；CRC16 0xA001；**921600 8N1**；**脉冲 = 角度° × 减速比 ÷ 360 × 10000**
> **硬件**：6× 立三闭环步进（1~3 号 42 机座，减速比 50:1/100:1/50:1；4/5 号 35 机座 IP35ET，50:1；6 号 28 机座 IP28ET，50:1），每转 10000 脉冲（0x0024），USB 转 485；1~6 号全部在线
> **约束**：① 本地 commit 后按用户指示推送 origin（Gitee）；② 架构/接口改动先讨论再动手；③ 串口打印精简中文；④ 单位：角度用度、长度用毫米；⑤ 优先可移植到单片机的纯 C，不引重量级依赖；⑥ 运动学/轨迹/通信逻辑保持离线可单测；⑦ 新模块先调研开源方案确认再写；⑧ `docs/`、`second/` 仅本地保留、不推云端
> **开发原则**：comm/control/ui 三层分离；配置头文件 + ini；离线可单测；先方案后代码

## 8. 路线图

### 已完成

- [x] 串口层（serial_win.c）联调：打开 COM 口、精确读/写帧、避开 15.6ms 节拍坑
- [x] Modbus RTU 主站：03H/06H/10H 帧构造、CRC16、逐事务耗时统计
- [x] 六轴高层接口：movej / home / enable / disable / getpos
- [x] 回零双机制：堵转法(轴1-5) + 传感器法(轴6) + 退让就位 0-0-90
- [x] 运动学库：DH 建模 / FK 正解 / 球腕解耦解析 IK（8 组解 + 限位筛选 + 连续选解）/ 零点换算
- [x] 轨迹规划：直线插补（line.c）+ 三点圆弧插补（arc.c）+ 驱动器内部加减速
- [x] 笛卡尔空间运动（MoveL）：interp 逐点插补 + smooth 流畅，偏差预算控航点密度
- [x] 空间圆弧运动（MoveC）：三点定圆、半径自适应加密、偏差遥测量到真实弧线
- [x] .rbt 轨迹脚本：命名点位(TARGET) + 示教录制(cd/save/getpos:j/x) + run 执行器
- [x] 安全闸门体系：NaN/Inf + 软限位 + 单步位移 + 步数量级 + 关节跳变 五重检查
- [x] 堵转检测与碰撞保护：实时电流轮询 + ini [stall] 各轴阈值 + 急停
- [x] 高速进巡航护栏：段耗时 < 加速窗时按直线度红线拉长步长、减航点
- [x] UDP 遥测镜像 3D（telemetry.c + second/sim/live_mirror.py）
- [x] 驱动器探针工具集：curtest/tabtest/trigtest/queuetest/chain/busrate/pipe/diag 等 15+ 诊断命令
- [x] CLI 分级帮助 + 命令解析体系（冒号分隔、大小写不敏、命名点位自动分流）
- [x] second/ 对照框架：Hg_Robot_Arm 运动学移植 + A/B 测试
*仅供参考；寄存器与算法细节以代码和 485 手册为准。）*
