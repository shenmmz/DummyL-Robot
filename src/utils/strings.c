/*
 * strings.c —— main.c 用户可见字符串字面量定义
 * 与 strings.h 配套；全部为 const 只读数据，链接期确定地址。
 */
#include "utils/strings.h"

/* ---- 启动横幅 / 串口来源 ---- */
const char STR_BANNER[]            = "DummyL-Robot 控制台 (C11 + MinGW)\n";
const char STR_HELP_HINT[]         = "输入 help 查看命令，exit 退出。\n\n";
const char STR_INI_PATH[]          = "config/robot_config.ini";
const char STR_DEFAULT_PORT[]      = "COM3";
const char STR_SRC_INI[]           = "生效来源：ini（%s），串口 %s @ %lu 8N1\n";
const char STR_SRC_DEFAULT[]       = "生效来源：默认（ini 缺失，回退 robot_config.h 默认值），串口 %s @ %lu 8N1\n";

/* ---- 串口探测 / 选择 ---- */
const char STR_ERR_NO_PORT[]       = "[错误] 未发现可用串口\n";
const char STR_MULTI_PORT[]        = "\n检测到多个串口：\n";
const char STR_PORT_ITEM[]         = "  [%d] %s\n";
const char STR_SELECT_PORT[]       = "请选择串口编号 (1-%d): ";
const char STR_ERR_BAD_SELECT[]    = "[错误] 无效选择\n";
const char STR_USE_PORT[]          = "\n使用串口: %s @ %lu 8N1\n";

/* ---- 初始化 / 监控 ---- */
const char STR_ERR_INIT[]          = "[错误] 初始化失败，请检查串口连接与 robot_config.ini\n";
const char STR_WARN_MON_CREATE[]   = "[警告] 监控器创建失败，继续运行\n";
const char STR_WARN_MON_START[]    = "[警告] 监控线程启动失败，继续运行\n";

/* ---- 交互循环 / 命令反馈 ---- */
const char STR_PROMPT[]            = "DummyL> ";
const char STR_ERR_HOME[]          = "[错误] 回零失败：%s\n";
const char STR_WARN_HOME_MON[]     = "[警告] 回零后监控线程重启失败\n";
const char STR_ERR_HOME_JOINT[]    = "[错误] 单轴回零失败：%s\n";
const char STR_ERR_MOVE[]          = "[错误] 运动指令失败：%s\n";
const char STR_ERR_ENABLE[]        = "[错误] 使能失败：%s\n";
const char STR_ERR_DISABLE[]       = "[错误] 失能失败：%s\n";
const char STR_ERR_MASK_JOINT[]    = "[错误] %s关节%d失败：%s\n";
const char STR_ERR_MASK[]          = "[错误] %s失败：%s\n";
const char STR_OP_MASK[]           = "屏蔽";
const char STR_OP_UNMASK[]         = "恢复";
const char STR_ERR_TORQUE[]        = "[错误] 力矩碰撞诊断失败：%s\n";
const char STR_WARN_TORQUE_MON[]   = "[警告] 诊断后监控线程重启失败\n";
const char STR_CALIB_DISABLED[]    = "单关节调试功能未启用\n";
const char STR_WARN_UNKNOWN[]      = "[警告] 未知命令，输入 help 查看帮助\n";
const char STR_EXIT[]              = "已退出。\n";

/* ---- cmd_status ---- */
const char STR_STATUS_REFRESH[]    = "持续刷新状态中（按任意键退出）...\n\n";
const char STR_STATUS_MASKED[]     = "关节:%d,状态:已屏蔽\n";
const char STR_STATUS_OFFLINE[]    = "关节:%d,状态:离线\n";
const char STR_STATUS_ONLINE[]     = "关节:%d,状态:在线,字:0x%06X,步长:%d,角度:%.2f,电流:%d,速度:%d,报警:%s,IN0:%d,IN1:%d,标志:%s\n";
const char STR_STATUS_STOPPED[]    = "\n状态刷新已停止。\n";
const char STR_FLAG_INPOS[]        = "到位 ";
const char STR_FLAG_SOFT_NEG[]     = "负限位 ";
const char STR_FLAG_SOFT_POS[]     = "正限位 ";
const char STR_FLAG_HOMED[]        = "原点 ";
const char STR_FLAG_ENABLE[]       = "使能 ";
const char STR_FLAG_ALARM[]        = "报警!";
const char STR_DASH[]              = "—";

/* ---- cmd_scan ---- */
const char STR_SCAN_TITLE[]        = "总线电机扫描（关节 1..6）...\n\n";
const char STR_SCAN_MASKED[]       = "  关节 %d: 已屏蔽（跳过）\n";
const char STR_SCAN_ONLINE[]       = "  关节 %d: 在线\n";
const char STR_SCAN_OFFLINE[]      = "  关节 %d: 离线（无响应）\n";
const char STR_SCAN_RESULT[]       = "\n扫描结果: %d/6 在线";
const char STR_SCAN_ONLINE_JOINTS[]= "，在线关节 ";
const char STR_SCAN_MASKED_COUNT[] = "，%d 个关节被屏蔽";

/* ---- cmd_diag ---- */
const char STR_DIAG_TITLE[]        = "\n回零诊断读数（闭环判据）\n";
const char STR_DIAG_ENC_CFG[]      = "  配置细分 ENCODER_STEPS_PER_REV = %d\n\n";
const char STR_DIAG_HEADER[]       = "  %-6s %-15s %-9s %-11s %-10s %-11s %-8s %s\n";
const char STR_DIAG_COL_JOINT[]    = "关节";
const char STR_DIAG_COL_SUBDIV[]   = "细分(实际/配置)";
const char STR_DIAG_COL_ENC[]      = "编码器线数";
const char STR_DIAG_COL_SPD[]      = "实际速度rpm";
const char STR_DIAG_COL_ERR[]      = "位置偏差";
const char STR_DIAG_COL_POS[]      = "位置(步)";
const char STR_DIAG_COL_CUR[]      = "电流mA";
const char STR_DIAG_COL_STAT[]     = "状态字";
const char STR_DIAG_SEP[]          = "  ------ --------------- --------- ----------- ---------- ----------- -------- --------\n";
const char STR_DIAG_MASKED[]       = "  %-6d 已屏蔽\n";
const char STR_DIAG_OFFLINE[]      = "  %-6d 离线\n";
const char STR_DIAG_ROW[]          = "  %-6s %-15s %-9s %-11s %-10s %-11s %-8s 0x%06X\n";
const char STR_DIAG_DELTA[]        = "        └ Δ位置=%+d 步, Δ偏差=%+d 步（%dms 内）\n";
const char STR_DIAG_BUS_TITLE[]    = "\n  总线测速（关节%d，各 20 次）：\n";
const char STR_DIAG_BUS_POS[]      = "    位置+状态合并读  %u ms / 20 次 = %.1f ms/次\n";
const char STR_DIAG_BUS_CUR[]      = "    电流读            %u ms / 20 次 = %.1f ms/次\n";
const char STR_DIAG_BUS_PERIOD[]   = "    堵转轮询周期 ≈ %.1f ms（2 事务/轮）\n";
const char STR_DIAG_JUDGE_TITLE[]  = "\n  判定提示：\n";
const char STR_DIAG_JUDGE_SLIP[]   = "    实际速度≈设定速度 且 电流高、Δ位置≈额定步数 → 电机轴仍在转 = 打滑/跳齿（机械问题）\n";
const char STR_DIAG_JUDGE_STALL[]  = "    实际速度≈0、Δ位置≈0 且 Δ偏差持续累积        → 命令走电机不转 = 真堵转（判据侧可解）\n";
const char STR_DIAG_SUBDIV_OK[]    = "%d 一致";
const char STR_DIAG_SUBDIV_MISMATCH[] = "%d/%d 不符";
const char STR_DIAG_SUBDIV_FAIL[]  = "读失败";
