# P1SIM Foundation 1.0：从下载到第一次运行

本教程面向第一次使用 P1SIM 的算法开发者。**普通开发无需申请仿真任务、科学资格或实验预算。** 默认运行的是 CPU-headless 的 ROS 2/Gazebo 开发环境，不包含私有自瞄源码、权重、比赛场景 mesh、真实弹道/物理评分器。

## 1. 电脑准备

推荐 Ubuntu 22.04 x86_64，已安装 Docker Engine 和 Docker Compose v2，用户有 Docker 使用权限，Python 3.10+、4 GB 内存及至少 6 GB 可用磁盘空间；首次构建需要网络。宿主机无需安装 ROS 2/Gazebo，GPU 非必需。Windows/macOS、ARM 和远程 DDS 尚未做完整验收。

打开终端检查：

```bash
uname -m
docker version
docker compose version
python3 --version
```

如果 Docker 权限不足，请按 [Docker 官方 Linux 安装与权限文档](https://docs.docker.com/engine/install/) 配置；P1SIM 不会自动执行 `sudo`、添加用户到特权组或修改系统权限。

## 2. 克隆、构建、检查

```bash
git clone https://github.com/tianxiansheng166/P1SIM.git
cd P1SIM
sha256sum -c SHA256SUMS
./p1sim dev bootstrap
./p1sim dev doctor
```

首次 `bootstrap` 会通过官方 Ubuntu/ROS 软件包仓库**在本机创建容器**，可能需要较长时间；这不是从私人 GHCR 拉取镜像。如果上游下载困难，构建前可设置：

```bash
export P1SIM_UBUNTU_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu
export P1SIM_ROS_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
./p1sim dev bootstrap
```

镜像配置会使用已固定的 ROS 基础镜像 digest；apt 软件包修订号会随构建时间改变，**并非逐字节确定性镜像**。请保留自己的构建版本和镜像 digest。

## 3. 启动虚拟机器人

```bash
./p1sim dev up
./p1sim dev status
./p1sim dev run -- python3 /opt/p1sim/examples/smoke.py
./p1sim dev run -- ros2 launch /opt/p1sim/examples/consumer.launch.py
```

打开浏览器访问 **http://127.0.0.1:8765** 查看实时健康状态。再开一个终端可执行：

```bash
./p1sim dev run -- ros2 topic list
./p1sim dev run -- ros2 topic hz /image_raw
./p1sim dev logs
```

这个通用场景可用于 ROS 消息、相机、TF、JointState、云台运动与程序运行检查，**并不是完整的比赛机器人仿真或科学评分**。

## 4. 挂载自己的自瞄程序

先在 Ubuntu 22.04 / ROS Humble 兼容的环境里自行构建你的代码。源码、模型权重留在你自己的电脑，不会被 P1SIM 发布或上传。

```bash
./p1sim dev down
export P1SIM_USER_WORKSPACE=/你的/ROS工作区绝对路径
export P1SIM_WORKSPACE_SETUP=/user_ws/install/setup.bash
export P1SIM_SCENE=autoaim
./p1sim dev up
./p1sim dev shell
# 容器内：ros2 launch <你的bringup包> <你的launch文件> use_sim_time:=true
```

工作区以**只读**方式挂入 `/user_ws`；编译产物须预先准备，实验数据存储在 `/data`。请确保自己的 launch 使用模拟相机输入而不是打开实体相机或串口，开启 `use_sim_time` 并正确匹配 Image、CameraInfo、TF 和 Planner 指令话题。

完整适配说明见 [接入自己的 AutoAim](USER_AUTOAIM.md)。没有合法模型权重或依赖时，P1SIM 不会替你补齐或自动下载私有文件。

## 5. 记录、重置与重复实验

```bash
./p1sim dev record session-1   # Ctrl-C 结束记录
./p1sim dev reset              # 时间与场景一并重置
./p1sim dev down
```

日志与 rosbag 默认保存在 `~/.local/state/p1sim-developer`。Reset 会让仿真时间从零重新开始，因此算法节点和录包器最好重新启动，以免混合两个时间 epoch。

Foundation 1.0 支持有界、可机器调用的 Batch：

```bash
./p1sim dev batch --config configs/extensions.json --seed 42 --output ./results/first-batch
python3 examples/batch_caller.py --output ./results/three-runs
```

Batch 生成的是平台执行收据与观测数据，**不是**带独立物理真值的科学 ResultPack。正式算法 A/B、统一 TestSpec 与高成本共享仿真，由测试工程体系在其自己的后端进行。

## 6. 常见问题

| 现象 | 检查步骤 |
|---|---|
| Docker 权限失败 | 先独立运行 `docker info`，按官方文档处理权限；不要修改 P1SIM 脚本绕过权限 |
| `doctor` 报运行时缺失 | 完成 `bootstrap`；查看 `./p1sim dev logs` |
| 8765 端口被占 | `./p1sim dev down`，或为并行实例设置独立 `P1SIM_STATUS_PORT` 和 `P1SIM_PROJECT` |
| 相机消息没有数据 | 检查 `dev status`、`/clock`、`/image_raw` 和 ROS/Gazebo 日志 |
| 自瞄源码找不到包 | 确认工作区已构建、`P1SIM_WORKSPACE_SETUP` 是容器内真实路径；更改后先 `down` 再 `up` |
| 自瞄模型缺失 | 由使用者自行取得合法模型，并在自己的工作区提供路径；发行包不捆绑权重 |
| 云台指令没有响应 | 校验消息 ABI、`/armor_planner/cmd_gimbal` 话题、`control` 位和 TF；参考接口文档 |
| 没有 HIT/MISS 分数 | 正常：本开发环境不内置 Fire 或科学 Scorer，不应制造命中率 |
| 需要 Gazebo 实验科研真值 | 使用独立测试工程/科学仿真，不将开发运行日志直接视为物理结论 |

首次安装无法完成时，请记录系统、Docker/Compose 版本、`dev doctor` 的错误和**去除私密字段后的**日志再提交 issue。不要上传私人源码、权重、访问令牌、完整环境变量或未审查的 rosbag。

## 发布边界

本公开包为 **Foundation 1.0 Developer Preview（源码本地构建）**。单机软件/消费者能力已有项目验收；尚未拿到独立第二台普通用户机器的完整下载→安装→自瞄工作区闭环验收，因此不宣称 Production GA、真实硬件或比赛准确度。

如果从 GitHub Release 下载 `.tar.gz`，解压后**不需要存在 `.git`**：平台会验证包内 `SHA256SUMS` 再生成来源内容身份。不要在解压目录直接修改运行时代码；需要修改源码请改用 `git clone`，让 Git 正确记录修改状态。
