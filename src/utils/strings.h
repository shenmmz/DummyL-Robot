/*
 * strings.h —— main.c 用户可见字符串字面量集中管理
 * 把所有 printf 文案 / 提示符 / 状态格式串外置到此处，
 * 便于统一维护与多语言替换，main.c 只引用常量。
 */
#ifndef DUMMYL_UTILS_STRINGS_H
#define DUMMYL_UTILS_STRINGS_H

/* ---- 启动横幅 / 串口来源 ---- */
extern const char STR_BANNER[];
extern const char STR_HELP_HINT[];
extern const char STR_INI_PATH[];
extern const char STR_DEFAULT_PORT[];
extern const char STR_SRC_INI[];
extern const char STR_SRC_DEFAULT[];

/* ---- 串口探测 / 选择 ---- */
extern const char STR_ERR_NO_PORT[];
extern const char STR_MULTI_PORT[];
extern const char STR_PORT_ITEM[];
extern const char STR_SELECT_PORT[];
extern const char STR_ERR_BAD_SELECT[];
extern const char STR_USE_PORT[];

/* ---- 初始化 / 监控 ---- */
extern const char STR_ERR_INIT[];
extern const char STR_WARN_MON_CREATE[];
extern const char STR_WARN_MON_START[];

/* ---- 交互循环 / 命令反馈 ---- */
extern const char STR_PROMPT[];
extern const char STR_ERR_HOME[];
extern const char STR_WARN_HOME_MON[];
extern const char STR_ERR_HOME_JOINT[];
extern const char STR_ERR_MOVE[];
extern const char STR_ERR_ENABLE[];
extern const char STR_ERR_DISABLE[];
extern const char STR_ERR_MASK_JOINT[];
extern const char STR_ERR_MASK[];
extern const char STR_OP_MASK[];
extern const char STR_OP_UNMASK[];
extern const char STR_ERR_TORQUE[];
extern const char STR_WARN_TORQUE_MON[];
extern const char STR_CALIB_DISABLED[];
extern const char STR_WARN_UNKNOWN[];
extern const char STR_EXIT[];

/* ---- cmd_status ---- */
extern const char STR_STATUS_REFRESH[];
extern const char STR_STATUS_MASKED[];
extern const char STR_STATUS_OFFLINE[];
extern const char STR_STATUS_ONLINE[];
extern const char STR_STATUS_STOPPED[];
extern const char STR_FLAG_INPOS[];
extern const char STR_FLAG_SOFT_NEG[];
extern const char STR_FLAG_SOFT_POS[];
extern const char STR_FLAG_HOMED[];
extern const char STR_FLAG_ENABLE[];
extern const char STR_FLAG_ALARM[];
extern const char STR_DASH[];

/* ---- cmd_scan ---- */
extern const char STR_SCAN_TITLE[];
extern const char STR_SCAN_MASKED[];
extern const char STR_SCAN_ONLINE[];
extern const char STR_SCAN_OFFLINE[];
extern const char STR_SCAN_RESULT[];
extern const char STR_SCAN_ONLINE_JOINTS[];
extern const char STR_SCAN_MASKED_COUNT[];

/* ---- cmd_diag ---- */
extern const char STR_DIAG_TITLE[];
extern const char STR_DIAG_ENC_CFG[];
extern const char STR_DIAG_HEADER[];
extern const char STR_DIAG_COL_JOINT[];
extern const char STR_DIAG_COL_SUBDIV[];
extern const char STR_DIAG_COL_ENC[];
extern const char STR_DIAG_COL_SPD[];
extern const char STR_DIAG_COL_ERR[];
extern const char STR_DIAG_COL_POS[];
extern const char STR_DIAG_COL_CUR[];
extern const char STR_DIAG_COL_STAT[];
extern const char STR_DIAG_SEP[];
extern const char STR_DIAG_MASKED[];
extern const char STR_DIAG_OFFLINE[];
extern const char STR_DIAG_ROW[];
extern const char STR_DIAG_DELTA[];
extern const char STR_DIAG_BUS_TITLE[];
extern const char STR_DIAG_BUS_POS[];
extern const char STR_DIAG_BUS_CUR[];
extern const char STR_DIAG_BUS_PERIOD[];
extern const char STR_DIAG_JUDGE_TITLE[];
extern const char STR_DIAG_JUDGE_SLIP[];
extern const char STR_DIAG_JUDGE_STALL[];
extern const char STR_DIAG_SUBDIV_OK[];
extern const char STR_DIAG_SUBDIV_MISMATCH[];
extern const char STR_DIAG_SUBDIV_FAIL[];

#endif /* DUMMYL_UTILS_STRINGS_H */
