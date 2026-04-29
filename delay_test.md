现在这套“延时节点”是单次链路探针，不是 CSV 日志。它用共享内存 `single_shot_probe::TraceData` 记录几个关键时间戳，最后由 `motor_test_node` 打印 `[Probe][Summary]`。

**链路顺序**

1. `motor_test_node` 单次发布 `/joint_command`
   - 时间戳：`topic_pub_ns`
   - 位置：[main.cpp](</d:/SBdesktop/移动运控update/sb_new_update/428/sb_new_tor/WK_ZY/src/motor_test_pkg/src/main.cpp:84>)

2. `motor_node` 收到 `/joint_command`
   - 时间戳：`motor_node_rx_ns`
   - 位置：[motorComm.cpp](</d:/SBdesktop/移动运控update/sb_new_update/428/sb_new_tor/WK_ZY/src/motor_communication_pkg/src/motor_comm/motorComm.cpp:84>)

3. EtherCAT 任务进入发送回调
   - 时间戳：`ec_send_ns`
   - 位置：[main.cpp](</d:/SBdesktop/移动运控update/sb_new_update/428/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/Ti5-Encos-3master/main.cpp:18>)
   - 三个主站都接了 send callback：[main.cpp](</d:/SBdesktop/移动运控update/sb_new_update/428/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/Ti5-Encos-3master/main.cpp:205>)

4. EtherCAT 任务进入接收回调
   - 时间戳：`ec_receive_ns`
   - 位置：[master_program.cpp](</d:/SBdesktop/移动运控update/sb_new_update/428/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/Ti5-Encos-3master/master_program.cpp:52>)
   - 接在 receive callback 里：[master_program.cpp](</d:/SBdesktop/移动运控update/sb_new_update/428/sb_new_tor/yunkongzhongyi/NIIC-EtherCAT-Demo-4.1.3-NSPIC-R006NP_-U22-NECRO/NIIC-EtherCAT-Demo-4.1.3-NECRO/examples/Ti5-Encos-3master/master_program.cpp:272>)

5. `motor_node` 的 IPC 客户端收到服务端回包
   - 时间戳：`ipc_feedback_rx_ns`
   - 位置：[IPCComm.cpp](</d:/SBdesktop/移动运控update/sb_new_update/428/sb_new_tor/WK_ZY/src/motor_communication_pkg/src/motor_comm/IPCComm.cpp:336>)

6. `motor_node` 发布 `/joint_states_comm`
   - 时间戳：`state_pub_ns`
   - 位置：[motorComm.cpp](</d:/SBdesktop/移动运控update/sb_new_update/428/sb_new_tor/WK_ZY/src/motor_communication_pkg/src/motor_comm/motorComm.cpp:59>)

7. `motor_test_node` 收到 `/joint_states_comm`
   - 时间戳：`state_seen_ns`
   - 位置：[main.cpp](</d:/SBdesktop/移动运控update/sb_new_update/428/sb_new_tor/WK_ZY/src/motor_test_pkg/src/main.cpp:100>)

**怎么看延时**

最终打印逻辑在 [single_shot_probe.h](</d:/SBdesktop/移动运控update/sb_new_update/428/sb_new_tor/probe/single_shot_probe.h:132>)。

重点看这些 Summary 行：

- ROS 发布到 `motor_node` 收到：
  `topic_pub_ns -> motor_node_rx_ns`

- `motor_node` 收到指令到 EtherCAT 发送回调：
  `motor_node_rx_ns -> ec_send_ns`

- 主从站发送/接收链路：
  `ec_send_ns -> ec_receive_ns`

  这表示 EtherCAT send callback 到 receive callback 的时间。它不是“单个从站内部响应时间”，而是这一轮 EtherCAT 任务从下发到收到反馈的链路时间，包含总线周期、主站任务调度、从站处理和回读。

- EtherCAT 接收回调到 `motor_node` IPC 客户端拿到反馈：
  `ec_receive_ns -> ipc_feedback_rx_ns`

  这段就是你现在最接近“服务端反馈经进程间通信回到 motor_node”的时间。

- IPC 客户端拿到反馈到 ROS 发布状态：
  `ipc_feedback_rx_ns -> state_pub_ns`

- ROS 状态发布到测试节点看到：
  `state_pub_ns -> state_seen_ns`

- 全链路：
  `topic_pub_ns -> state_seen_ns`

当前默认测试目标在 [motor_test_pkg/src/main.cpp](</d:/SBdesktop/移动运控update/sb_new_update/428/sb_new_tor/WK_ZY/src/motor_test_pkg/src/main.cpp:16>)：`ROS joint index=0`、`master=0`、`motor=0`、目标位置 `-0.2`。如果要测其他主站/电机，需要同步改这几个常量，并确保 ROS joint index 和 `Motor_Index_Switch` 映射对应。