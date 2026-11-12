# 接入你自己的 AutoAim / ROS 2 工作区

P1SIM Foundation 提供虚拟场景与普通 ROS 2 输入/输出，**不提供其他团队的算法源码、模型权重、私有硬件驱动或比赛 Scorer**。它不会修改使用者的原始仓库。

## 已知平台接口

- 传感器：`/image_raw`、`/camera_info`、`/tf`、`/tf_static`、`/imu/data`；
- 云台状态：`/joint_states`，采用仿真时钟；指令映射见 [接口合同](INTERFACES.md)；
- 原生程序可在容器 Shell 运行普通 `ros2 launch`。使用者自行将 Camera 节点切为外部仿真相机，并处理自己的 launch 配置；
- 默认 `P1SIM_GIMBAL_TOPIC=/armor_planner/cmd_gimbal`，如果自己的消息接口不是 `rm_interfaces/msg/GimbalCmd`，必须自行适配或禁用该可选桥接；
- `P1SIM_SCENE=autoaim` 是开发用单目标场景；可调可见性、云台与时序的 Extension API 见 [Foundation V1](FOUNDATION.md)。

## 最小成功标准

1. 自己的工作区可以以 ROS Humble ABI 在容器内 source；算法节点真正启动，启动日志没有缺包或模型路径错误。
2. `/clock` 与 `/image_raw` 推进，Image / CameraInfo 的时戳和 TF 符合算法预期。
3. 自己的 Detector→Tracker→Planner 有真实消息；若启用云台桥接，JointState / TF 随命令变化，并能在同源时间轴核对。
4. `dev reset` 后算法按新 epoch 重新初始化；`dev down` 后容器退出，个人工作区、日志和录包不被删除。

**重要限制：** 默认云台是运动学开发模型，不是标定过的真实电机。开发场景不发射弹丸；不能把 Fire 字段、屏幕上的 HIT/MISS 或稳定跟踪直接算成算法物理命中率。

对于官方开发路径与独立科学评价，使用 `T0–T2.5` 做低成本软件/模型回归；普通虚拟实车联调用 `T3`，要求独立物理真值与评分才升级 `T4`。这些分层是测试选择，不是普通使用者必须先申请的注册手续。

若需要第一版目前无法表达的新传感器、目标运动或控制原语，优先通过场景 JSON/插件提供，只有确实无法表达跨用户共性能力时才升级 Foundation 核心接口。
