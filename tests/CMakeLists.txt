# ============================================================
# 离线单元测试（CTest 注册）
# ------------------------------------------------------------
# 仅链接 utils / kinematics 静态库：不含串口、总线与硬件依赖，
# 可在无机械臂的机器上直接运行：
#   ctest --test-dir build --output-on-failure
# ============================================================

# FK：位姿矩阵合法性 + J1 结构对称性 + 零位包络
add_executable(test_dh_fk test_dh_fk.c)
target_link_libraries(test_dh_fk PRIVATE kinematics)
target_include_directories(test_dh_fk PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR})
target_compile_options(test_dh_fk PRIVATE -Wall -Wextra)
add_test(NAME dh_fk COMMAND test_dh_fk)

# IK：FK→IK→FK 闭环（2000 组随机关节角，固定种子可复现）
add_executable(test_ik_roundtrip test_ik_roundtrip.c)
target_link_libraries(test_ik_roundtrip PRIVATE kinematics)
target_include_directories(test_ik_roundtrip PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR})
target_compile_options(test_ik_roundtrip PRIVATE -Wall -Wextra)
add_test(NAME ik_roundtrip COMMAND test_ik_roundtrip)

# CLI 解析：fk / ik 新命令 + 既有命令回归
add_executable(test_cmd_parser test_cmd_parser.c)
target_link_libraries(test_cmd_parser PRIVATE utils)
target_include_directories(test_cmd_parser PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR})
target_compile_options(test_cmd_parser PRIVATE -Wall -Wextra)
add_test(NAME cmd_parser COMMAND test_cmd_parser)

# 零点标定：电机角 <-> 机械角 换算往返一致 + 回零实测基准
add_executable(test_joint_zero test_joint_zero.c)
target_link_libraries(test_joint_zero PRIVATE kinematics)
target_include_directories(test_joint_zero PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR})
target_compile_options(test_joint_zero PRIVATE -Wall -Wextra)
add_test(NAME joint_zero COMMAND test_joint_zero)
