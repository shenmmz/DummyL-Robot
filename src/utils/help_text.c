/*
 * help_text.c —— CLI 命令帮助文本（"指令"说明）集中维护
 * 把所有命令说明单独成文件，便于统一编辑与查阅；
 * 由 cmd_print_help() 输出，main 的 help 命令调用。
 */
#include "utils/help_text.h"

#include <stdio.h>

static const char HELP_TEXT[] =
    "可用命令:\n"
    "  home                  回零（全轴）\n"
    "  home:N                仅单独回零关节 N，堵转后自动到该轴配置角\n"
    "                         (例 home:1 单独验证 1 轴，与其它轴无关)\n"
    "  homej:N:ANGLE         关节 N 回零后自动运动到 ANGLE 度 (N=1..6)\n"
    "  homej:N:ANGLE:SPEED   指定速度 (rpm)\n"
    "  movej:N:ANGLE         关节 N 绝对运动到 ANGLE 度 (N=1..6)\n"
    "  movej:N:ANGLE:SPEED   指定速度 (rpm)\n"
    "  enable:N              使能关节 N\n"
    "  disable:N             失能关节 N\n"
    "  status                查询所有关节状态\n"
    "  mask:N[,M,...]        屏蔽关节（可批量，例 mask:2,3,4）\n"
    "  unmask:N[,M,...]      恢复关节（可批量，例 unmask:2,3,4）\n"
    "  scan                  扫描总线电机\n"
    "  calib                 单关节手动调试\n"
    "  diag [N]              回零诊断：细分/编码器线数/实际速度/位置偏差\n"
    "                         (例 diag:2 只看 2 轴；省略 N 则全轴)\n"
    "  torque:N:L            力矩碰撞回原点诊断：关节 N 以等级 L(0~255) 试撞，\n"
    "                         每 200ms 打印 状态字/电流/位置，用于观察到位信号\n"
    "  help                  帮助\n"
    "  exit                  退出\n";

void cmd_print_help(void)
{
    fputs(HELP_TEXT, stdout);
}
