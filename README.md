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

代码运行在 Windows PC 上，通过 USB 转 RS485 以 Modbus RTU 协议直接驱动 Zeta 电机（六轴机械臂），不再依赖单片机固件。运动学、轨迹规划、电机控制逻辑全部在 PC 端完成，算法实现保持纯 C、无重量级依赖，可随时移植回单片机。

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
├── CMakeLists.txt              # 主构建：lib + app + tests 三个目标
├── README.md                   # 本文件
├── config/
│   ├── robot_config.h          # 电机ID/减速比/堵转电流/限位（宏定义）
│   ├── dh_params.h             # DH 参数表
│   └── robot_config.ini        # 运行时可调参数（串口端口、波特率、默认速度）
├── src/
│   ├── main.c                  # 入口 + CLI 交互循环
│   ├── comm/
│   │   ├── serial_win.c/h      # Windows 串口封装
│   │   ├── crc16.c/h           # CRC16 0xA001
│   │   └── modbus_rtu.c/h      # Modbus RTU 主站：03H/06H/10H 帧构造与解析
│   ├── kinematics/
│   │   ├── mat3.c/h            # 3x3 矩阵运算（纯 C）
│   │   ├── dh.c/h              # DH 建模 + 正运动学 FK
│   │   └── ik.c/h              # 球腕解耦解析 IK（8 组解）+ 限位筛选/最优解选择
│   ├── trajectory/
│   │   └── planner.c/h         # 点到点梯形/S 曲线、关节插补
│   ├── control/
│   │   ├── robot.c/h           # 高层接口：robot_home / movej / enable / disable
│   │   └── monitor.c/h         # 状态轮询：在线检测、堵转报警
│   ├── utils/
│   │   ├── logger.c/h          # 精简中文日志
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
    └── protocol.md             # Zeta 寄存器表 + Modbus 帧格式
```

## 3. 构建与运行

```bash
# 首次构建
cmake -G Ninja -B build -DCMAKE_C_COMPILER=C:/Qt/Tools/mingw1310_64/bin/gcc.exe
ninja -C build

# 运行
build\dummyrobot.exe

# 运行单元测试
ctest --test-dir build
```

构建产物：`build\dummyrobot.exe`。

## 4. 硬件与通信协议

### 4.1 硬件配置

- 6× Zeta 电机（RS485 总线），USB 转 485 连接 PC
- 波特率 115200，8 数据位 / 无校验 / 1 停止位（115200 8N1）
- 每台电机 16384 线磁编码，单圈 16384 步

| 关节 | 电机型号 | 减速比 |
|---|---|---|
| 1 | 42 | 50:1 |
| 2 | 42 | 100:1 |
| 3 | 42 | 50:1 |
| 4 | 35 | 50:1 |
| 5 | 35 | 50:1 |
| 6 | 28 | 50:1 |

**步数换算**：`步数 = 角度° × 减速比 ÷ 360 × 16384`

### 4.2 Zeta 寄存器表（Modbus RTU）

> 来源：`external/Zeta系列产品手册.md` §3.3。功能码 03H=读、06H=写单寄存器、10H=写多寄存器；CRC16 多项式 0xA001。初始值为出厂默认（十六进制）。

#### 通用状态区
| 地址 | 功能码 | 内容 | 初始值 |
|---|---|---|---|
| 0x0000 | 03H | 电机状态：0000 待机/到位、0001 运行中、0002 碰撞停、0003 正光电停、0004 反光电停 | 00 00 |
| 0x0001 / 0x0002 | 03/06/10H | 实际步数（32 位，高<<16 \| 低；写 0 = 停止并清零位置） | 00 00 |
| 0x0003 | 03H | 实际速度（rpm） | 00 00 |
| 0x0004 | 03/06/10H | 急停指令（写 1 立即停止） | -- |
| 0x0005 | 03H | 电流（mA） | -- |
| 0x0006 | 03/06/10H | 使能：1 使能 / 0 失能（默认 1） | 00 01 |
| 0x0007 | 03/06/10H | 输出 PWM 0~1000（K 引脚 0~100.0%） | 00 00 |
| 0x0008 | 03/06/10H | 限位开关：0 不起作用 / 1 起作用 | 00 01 |

#### 位置模式区（`robot_movej` 使用）
| 地址 | 功能码 | 内容 | 初始值 |
|---|---|---|---|
| 0x0010 / 0x0011 | 03/06/10H | 目标步数（32 位，高<<16 \| 低） | 00 00 |
| 0x0012 | 03/06/10H | 保留 | -- |
| 0x0013 | 03/06/10H | 目标速度（rpm，上电读 0x00E7） | 读 E7H |
| 0x0014 | 03/06/10H | 加速度 0~30000 rpm/s（上电读 0x00E8） | 读 E8H |
| 0x0015 | 03/06/10H | 到位精度（步，上电读 0x00E9） | 读 E9H |
| 0x001F | 03/06/10H | 归零指令：写入值=归零速度，正数正转/负数反转，走到堵转或限位停并清零位置 | 00 00 |

> 位置模式写多个寄存器时须从 0x0010 连续写 6 个：`{目标高, 目标低, 0(保留占位), 速度, 加速度, 精度}`，否则因 0x0012 保留位存在而错位（见手册示例）。

#### 单圈绝对值区
| 地址 | 功能码 | 内容 | 初始值 |
|---|---|---|---|
| 0x0020 | 03/06/10H | 目标步数 | 00 00 |
| 0x0021 | 03/06/10H | 目标速度 | 00 00 |
| 0x0022 | 03/06/10H | 模式：0 就近原则、1 顺时针 | 00 00 |
| 0x00D5 | 03/06/10H | 设置零位（掉电保存） | 00 00 |

#### 速度模式区
| 地址 | 功能码 | 内容 | 初始值 |
|---|---|---|---|
| 0x0060 | 03/06/10H | 保留 | -- |
| 0x0061 | 03/06/10H | 速度（rpm，上电读 0x00E7） | 读 E7H |
| 0x0062 | 03/06/10H | 加速度 0~30000 rpm/s（上电读 0x00E8） | 读 E8H |

#### 电机参数区（掉电保存）
| 地址 | 功能码 | 内容 | 初始值 |
|---|---|---|---|
| 0x00E0 | 03/06/10H | 设备地址（1~254） | 00 01 |
| 0x00E1 | 03/06/10H | 堵转电流（mA） | 0B B8(3000) |
| 0x00E2 | 03/06/10H | 找零电流（mA，回零时使用） | 02 58(600) |
| 0x00E3 | 03/06/10H | 每圈步数（485 固定 16384，不可改） | 06 40 |
| 0x00E4 | 03/06/10H | 底层方向：1 正常 / 0 反转（不改限位正反性质） | 00 01 |
| 0x00E5 | 03/06/10H | 堵转逻辑：0 泄力 / 1 人机对抗 | 00 00 |
| 0x00E6 | 03/06/10H | 堵转时间（ms） | 00 00 |
| 0x00E7 | 03/06/10H | 默认速度（上电赋给 0x13/0x43/0x61）* | — |
| 0x00E8 | 03/06/10H | 默认加速度（上电赋给 0x14/0x44/0x62） | EA 60(60000) |
| 0x00E9 | 03/06/10H | 默认精度（上电赋给 0x15/0x45） | 00 64(100) |
| 0x00EA / 0x00EB | 03/06/10H | 通讯波特率高/低 16 位（00 01 C2 00 = 115200） | 00 01 / C2 00 |
| 0x00F0 | 03H | 版本号（如 00 1F = 3.1） | 00 1F |
| 0x00F1 | 03H | 型号 | — |

> *0x00E7：手册正文多次引用（0x13/0x43/0x61 上电读取其值作为默认速度），但手册寄存器表未单列此行，按引用语义补入。

### 4.3 回零流程与参数

回零分两阶段：

1. **找零**：组 0 = {1,2,3,5,6} 并行回零 → 组 1 = {4} 回零（向 0x001F 写带符号归零速度，走到堵转/限位停并清零位置）；
2. **就位**：全部找零完成后，运动到机械原点位姿（6 关节目标角度，由指令配置并掉电保存，回零参数与工作参数分离）。

- 各关节归零方向 `HOME_DIR[6] = {+1, -1, +1, -1, -1, +1}`（决定 0x001F 写正速还是负速）
- 回零堵转电流 `HOME_STALL_CURRENT_MA[6] = {0, 500, 800, 500, 500, 800} mA`（写 0x00E1，实机标定）
- 回零找零电流 `HOME_FIND_CURRENT_MA[6]`（写 0x00E2，实机标定）
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

> 关节号 N 范围 1..6。2 号电机默认屏蔽（启动即生效），可 `unmask:2` 临时恢复。
> 独立可执行程序：`scan_motors.exe`（总线扫描）、`servo_calib.exe`（单关节手动标定），均位于 `build/bin/`。

## 6. 设计原则

1. **三层分离**：`comm/`（帧收发）→ `control/`（业务指令）→ `main.c`（交互），换电机/换总线只改 comm 层
2. **可移植**：运动学与轨迹规划为纯 C，无平台依赖，可整体搬回单片机固件
3. **配置外置**：电机减速比、DH 参数、限位集中在 `config/`，换机型不碰业务代码
4. **离线可测**：CRC / Modbus 帧 / IK / 轨迹全部可单测，不接硬件也能验证逻辑
5. **输出精简**：串口打印全中文，过程性/调试性打印无必要即删

## 7. 开发提示词

> **项目**：DummyL-Robot — PC 端六轴机械臂控制台（C 语言，脱离单片机）
> **架构**：C11 + MinGW GCC + CMake/Ninja；`comm/` 走 RS485 Modbus RTU 直驱 Zeta 电机；`control/robot.c` 提供 movej/home 等高层接口；`kinematics/` 纯 C 正逆解（球腕解耦 + 限位筛选/最优解）
> **协议**：Zeta 寄存器（地址0x00E0、使能0x0006、位置0x0001/2、速度0x0061、状态0x0000、堵转0x00E1、限位0x0008）；Modbus RTU 03H/06H/10H；CRC16 0xA001；115200 8N1；步数 = 角度° × 减速比 ÷ 360 × 16384
> **硬件**：6× Zeta 电机（1号42/50:1、2号42/100:1、3号42/50:1、4/5号35/50:1、6号28/50:1），16384 线磁编码，USB 转 485
> **约束**：① 只本地 git commit，禁止 push；② 架构/接口改动先讨论再动手；③ 串口打印精简中文；④ 单位：角度用度、长度用毫米；⑤ 优先可移植到单片机的纯 C 实现，不引入重量级依赖
> **开发原则**：comm/control/ui 三层分离；配置头文件 + ini；离线可单测；先方案后代码

## 8. 路线图

- [ ] 串口层（serial_win.c）联调验证：打开 COM 口、收发帧
- [ ] Modbus RTU 主站：读状态/写使能/写位置，单关节验证
- [ ] 六轴高层接口：movej / home / status
- [ ] 运动学库：DH/FK/IK + 单测闭环
- [ ] 轨迹规划：梯形/S 曲线插补
- [ ] 笛卡尔空间运动（moveL）
- [ ] 3D 可视化验证（可选）
*（内容由AI生成，仅供参考）*
