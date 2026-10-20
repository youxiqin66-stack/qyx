# VLM Servo — 阶段一 / 二 / 三

- **阶段一**：自然语言 → Qwen3-VL(开放词汇) → 2D bbox → OpenCV 实时画框。
- **阶段二**：bbox + D455 对齐深度 + 相机内参 → 相机系 3D XYZ。
- **阶段三**：bbox → 掩码 → FoundationPose → 目标 6D Pose（相机系），可选经 ZeroMQ 喂 ROS2。

阶段三接入的是你已有的 `FoundationPose/ros_demo.py` 那套链路，**不修改你 FoundationPose/ 里任何文件**；唯一变化是把 `ros_demo.py` 里的 `get_mask()`（YOLO+SAM）换成 **VLM 的 bbox**，`register`/`track_one`/`send_pose` 全按你原代码原样保留。

---

## 1. 项目目录结构

```
vlm_servo/
├── main.py                  # 阶段一+二入口：RGB -> VLM -> bbox -> 画框 + XYZ
├── main_pose.py             # 阶段三入口：VLM bbox -> FoundationPose -> 6D Pose (+可选 ZeroMQ)
├── config.py / config.yaml  # 集中配置（相机 / VLM / 深度 / FoundationPose / 显示）
├── camera.py                # 相机抽象：D455（彩色+对齐深度+内参）/ 静态图兜底
├── depth3d.py               # 阶段二：bbox + 深度 + 内参 -> 相机系 XYZ
├── mask_gen.py              # 阶段三：bbox -> 掩码（rect / sam）
├── foundationpose_bridge.py # 阶段三：封装 FoundationPose register/track/draw
├── pose_pub.py              # 阶段三：ZeroMQ 位姿发布（格式同 ros_demo.py，只发位姿）
├── visualizer.py            # OpenCV 画 bbox + XYZ + 顶部信息条 + FPS
├── run.sh / requirements.txt
└── vlm/                     # VLM 抽象层（你已有的包，不改）
    ├── __init__.py          # build_vlm() 工厂：qwen / mock
    ├── base.py              # BaseVLM + Detection + VisionResult
    ├── qwen_vl.py           # Qwen3-VL（DashScope API，不占显存）
    └── mock_vlm.py          # 模拟 VLM（离线测试）
```

## 2. 每个文件作用

| 文件 | 作用 | 阶段 |
|------|------|------|
| `main.py` | 三线程主循环：抓帧 + 终端输入 + 显示，画 bbox + XYZ | 一二 |
| `main_pose.py` | VLM bbox → 掩码 → FP register/track → 6D Pose，画 3D 框 + 可选 ZeroMQ | 三 |
| `camera.py` | `RealsenseCamera`(彩色+对齐深度+内参) / `ImageCamera`(静态图+合成深度) | 一二三 |
| `depth3d.py` | `pixel_to_xyz` / `bbox_to_xyz`：像素+深度+内参 → XYZ | 二 |
| `mask_gen.py` | `mask_from_bbox`：bbox → 物体掩码（rect/sam） | 三 |
| `foundationpose_bridge.py` | `PoseEstimator`：加载网格、register/track/draw、内参矩阵转换、目标名匹配网格 | 三 |
| `pose_pub.py` | `PosePublisher`：ZeroMQ 发布位姿（同 ros_demo.py 格式，只发位姿） | 三 |
| `visualizer.py` | `draw_detections`(bbox+XYZ) / `draw_hud` / `FpsCounter` | 一二三 |
| `config.py`/`config.yaml` | 集中配置 | 一二三 |
| `vlm/` 全部 | VLM 抽象 + Qwen3-VL(API) + mock | 一（已有，未改） |

## 3. 数据流

```
RealSense D455 (pyrealsense2, rs.align(color))
        │  后台线程持续抓帧
        ▼
latest_pair() -> (color BGR, depth 米) 同帧成对
        │  终端输入自然语言
        ▼
vlm.detect(color, instruction) ──► VisionResult{detections[].bbox}
        │
        ├─ 阶段二：bbox_to_xyz(bbox, depth, K) ──► 相机系 XYZ(米)
        │
        └─ 阶段三：mask_from_bbox(bbox) ──► FoundationPose register(首帧)
                    └─ track_one(连续) ──► 4x4 pose（物体在相机系）
                                          └─ 画 3D 框 + 坐标轴
                                          └─ (可选) ZeroMQ ──► foundationpose_node.py
                                                              ──► /object_pose, TF
```

## 4. RealSense D455 的 topic

阶段一/二/三**都不用 ROS2 topic**，直接用 `pyrealsense2` 取 `rs.stream.color`(bgr8) + `rs.stream.depth`(z16) 并 `rs.align(color)`，与你 `camera_module.py` 一致。原因：`realsense2_camera` 未装、`cv_bridge` 与你 NumPy 2.2.6 有 ABI 冲突。

阶段三如果要 RViz2 显示相机 RGB/Depth/PointCloud/TF，需单独装 realsense-ros，其默认 topic 见下方（供后续参考）：

| topic | 类型 | 内容 |
|-------|------|------|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RGB |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 深度(16UC1, mm) |
| `/camera/camera/depth/color/points` | `sensor_msgs/PointCloud2` | 彩色点云 |
| TF | `camera_link → camera_color_optical_frame` 等 | 相机坐标系 |

## 5. VLM 输入 / 输出

- 输入：BGR 图 `(H,W,3)` + 自然语言 `instruction`。
- 输出：`VisionResult{detections}`，`Detection{target, bbox, ...}`。**只做视觉定位，不做意图/动作分类。**
- `qwen_vl.py` 走 DashScope 百炼 OpenAI 兼容 API，**不占本地显存**（不与 FoundationPose 抢 6GB）。

## 6. bbox 数据格式

原图像素坐标 `(x1,y1,x2,y2)`，整数，左上→右下，原点在图像左上。Qwen 原生 0~1000 归一化坐标由 `qwen_vl.py::_to_bbox()` 转回像素。

## 7. 阶段二：bbox → 相机系 XYZ

```
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth          # 米
```

- 内参取设备彩色流内参；抗噪取 bbox 中心区域深度中位数（`depth.center_ratio`）。
- 坐标系 = `camera_color_optical_frame`（+X右 +Y下 +Z前）。

## 8. 阶段三：bbox → FoundationPose → 6D Pose

### 8.1 与 ros_demo.py 的关系（重点）

你 `FoundationPose/ros_demo.py` 的流程是：

```
Camera -> get_mask(color) [YOLO 检测 + SAM 分割]
       -> est.register(K, color, depth, ob_mask=mask)
       -> est.track_one(rgb, depth, K)
       -> send_pose() -> ZeroMQ JSON -> foundationpose_node.py -> ROS2
```

我的阶段三在 `vlm_servo/` 里复刻了这条链路，**唯一替换**：`get_mask()` → `VLM bbox` + `mask_gen.mask_from_bbox()`。`register`/`track_one` 的参数、位姿格式、ZeroMQ 消息字段与你原代码完全一致。

### 8.2 网格（mesh）映射

FoundationPose 是**基于模型**的：每个物体需要一个网格（.obj）。VLM 负责开放词汇地“找出”目标，然后把目标名映射到网格。在 `config.yaml` 的 `foundationpose.meshes` 里配置（关键词子串匹配，匹配不到用 `default_mesh`）：

```yaml
foundationpose:
  meshes:
    "bottle": "demo_prb_data/test1/mesh/bottle_red.obj"
    "瓶子":  "demo_prb_data/test1/mesh/bottle_red.obj"
    "triangle": "demo_prb_data/test_redtriangle/mesh/red_triangle.obj"
    "三角":  "demo_prb_data/test_redtriangle/mesh/red_triangle.obj"
  default_mesh: "demo_prb_data/test1/mesh/bottle_red.obj"
```

> 这是模型位姿估计的固有属性：想抓某个新物体，需要先有它的 mesh（或后续用重建方式生成）。VLM 解决“识别什么”，mesh 解决“怎么摆”。

### 8.3 掩码（mask）

`mask_gen.py` 两种模式（`foundationpose.mask_mode`）：

- `rect`（默认）：直接把 bbox 内部标为 True。零额外显存/依赖，适合瓶子/三角等饱满物体；细长物体（笔）会带入背景，可用 sam 更准。
- `sam`：用 segment_anything 以 bbox 为 prompt 分割，更准但占显存（6GB 下与 FoundationPose 同跑要留意）。

### 8.4 位姿格式

`register`/`track_one` 返回 **4×4 齐次矩阵**，物体「原始建模坐标系」在相机坐标系下的位姿，单位米，父坐标系 = `camera_color_optical_frame`（与你 `POSE_TRANSFORM_GUIDE.md` 一致）。阶段四再经手眼标定（眼在手外，demo.launch.py 静态 TF）转到机械臂基座。

### 8.5 输出到 ROS2（可选）

`config.yaml` 里 `foundationpose.publish_zmq: true` 时，会用 `pose_pub.py` 把 6D Pose 按 `ros_demo.py` 的消息格式发到 `tcp://*:5555`，你现有的 `foundationpose_node.py` 会收到并发布：

- `/object_pose`（PoseStamped）
- TF：`camera_frame → object`

> 注意：Python 侧已**不再发 `intent` 字段**，所以 `foundationpose_node.py` 的 `publish_intent(message.get('intent'))` 会拿到 `None` → 编码成 `-1` 发到 `/object_intent`。`action_dispatcher.cpp` 对 `-1` 直接忽略（`action_dispatcher.cpp:202`），不会误触发机械臂。如需彻底不发，把 `foundationpose_node.py` 里那行 `self.publish_intent(...)` 删掉即可。

> 提醒：你 `foundationpose_node.py` 的 `camera_frame` 参数默认是 `camera_frame`（文档里写的是 `camera_color_optical_frame`）。启动 ROS2 节点时要把这个参数设成与你的 TF 树一致的名字，否则 RViz2 里位姿挂错坐标系。

### 8.6 意图/动作路由（已移除）

Python 视觉侧的意图分类和动作路由**已整体移除**，现在只输出目标的 6D 位姿：

- VLM 只做视觉定位，`VisionResult` 里没有 `intent` 字段；`intent.py` 已删除。
- `pose_pub.py` 的 ZeroMQ 消息不再带 `intent` 字段。
- 动作路由（approach / grind / grasp）如果要恢复，应在 ROS 侧（`action_dispatcher`）根据别的信号自行判断，或在 Python 侧重新引入意图分类。

原始的意图分类设计记录在这里，供以后恢复时参考：

| 类别 | 关键词 | 细粒度 intent | 执行 |
|------|--------|--------------|------|
| 伺服/加工 | 加工 / 打磨 / 靠近 / 跟随 | approach(0) / grind(1) | `demo.launch.py` + `action_dispatcher`（靠近=上方规划；打磨/跟随=连续伺服） |
| 抓取 | 抓取 / 拿给我 / 取 | grasp(2) | `action_dispatcher` 的 `doGrasp` 规划（只跑位姿、不做夹爪硬件） |

（**跟随/跟踪** 当时归一化到 grind 共用连续伺服链路；"靠近" 走 approach。）

### 8.7 动机械臂前的两道确认（阶段四，未实现）

以下设计记录在此，**当前 Python 侧未实现**（`main_pose.py` 只发位姿，不发任何命令）：

```
输入指令 -> VLM -> FoundationPose 注册/跟踪（位姿照常显示）
   ↓
问「是否规划路径? [y/n]」──否→ 取消（只识别/显示，机械臂不动）
   ↓ 是
发真实 intent -> action_dispatcher 规划 + RViz 显示轨迹
   ↓
问「是否启动? [y/n]」──否→ 取消执行
   ↓ 是
发 execute 命令 -> 机械臂执行
```

- 原设计里**确认前** Python 侧只发 `intent="unknown"`，确认后才发真实 intent；意图移除后这条链路要重新设计。
- **执行确认链路**（原设计）：`main_pose.py` → ZeroMQ `{"command":"execute"}` → `foundationpose_node.py` → `/action_dispatcher/execute`（std_srvs/Trigger）。
- **视觉伺服多一步**（原设计）：`doGrind` 先离散规划「初始位姿对齐」到 `T_BASE_OBJ * T_OBJ_TCP`，`/execute` 执行对齐到位后自动切速度控制器 + 启动连续伺服。

## 9. 依赖

用你已有的 `FoundationPose` conda 环境（已装 pyrealsense2/opencv/openai/numpy/pyyaml/torch/pytorch3d/nvdiffrast/trimesh/zmq）。一般无需再装：

```bash
/home/qyx/anaconda3/envs/FoundationPose/bin/pip install -r requirements.txt
```

`requirements.txt`：opencv-python、pyrealsense2、openai、numpy、pyyaml。阶段三的 zmq/trimesh/nvdiffrast 已在 FoundationPose 环境里。

## 10. 如何启动

**阶段一+二**：`./run.sh`（或 `.../bin/python main.py`）

**阶段三**（先接入 FoundationPose，只在本机显示 6D Pose）：

```bash
cd /home/qyx/LLM/code/vlm_servo
/home/qyx/anaconda3/envs/FoundationPose/bin/python main_pose.py
```

启动后在终端输入 `抓红色的瓶子` 或 `找到三角形`，程序会：VLM 出 bbox → 生成掩码 → FoundationPose 首帧注册 → 连续跟踪 → 画面画 3D 框 + 坐标轴。

**阶段三 + 喂 ROS2**：

```bash
# 1) ROS2 侧（另一个终端，系统 python / ROS2 环境）
ros2 run foundationpose_ros2 foundationpose_node --ros-args -p camera_frame:=camera_color_optical_frame
# 2) 本机侧：把 config.yaml 里 foundationpose.publish_zmq 改成 true，再跑 main_pose.py
```

退出：终端输 `q`/`quit`/`exit`，或窗口内按 `q`。

## 11. 测试步骤

| 步骤 | 目的 | 做法 |
|------|------|------|
| ① 离线画框+XYZ | 阶段一二 | `camera.type: image` + `vlm.type: mock`，跑 `main.py` |
| ② 真相机+mock | 阶段一二 | `camera.type: realsense` + `vlm.type: mock`，跑 `main.py` |
| ③ 全链路 VLM | 阶段一二 | `vlm.type: qwen`，跑 `main.py` |
| ④ 离线阶段三 | 验证掩码/网格匹配（不联网） | `camera.type: image` + `vlm.type: mock` + `mask_mode: rect`，跑 `main_pose.py` |
| ⑤ 真相机+真VLM | 阶段三最终 | `camera.type: realsense` + `vlm.type: qwen`，跑 `main_pose.py` |

单文件自测（不联网、不启动相机）：

```bash
cd /home/qyx/LLM/code/vlm_servo
P=/home/qyx/anaconda3/envs/FoundationPose/bin/python
$P -c "import main_pose, mask_gen, foundationpose_bridge, pose_pub; print('import ok')"
$P -c "from foundationpose_bridge import K_dict_to_matrix, match_mesh; print(K_dict_to_matrix({'fx':385.3,'fy':384.9,'cx':322.4,'cy':243.5}))"
```

## 12. 常见错误排查

| 现象 | 原因 | 处理 |
|------|------|------|
| **`未找到匹配目标`，但目标明明在画面里** | **送模型的图太小，小目标分不到足够 patch** | 保持 `vlm.min_image_size: 1280`（长边小于它就放大再送）。640x480 直送时 45px 高的三角块认不出，放大到 1280x960 立刻能认 |
| 相机没画面 | D455 没插好 | `lsusb` 看 `8086:0b5c`；重插 USB3.0 |
| `找不到网格 / NO MESH` | 目标名没匹配到 mesh | 在 `config.yaml foundationpose.meshes` 加关键词 |
| FoundationPose 加载慢/卡 | 首次加载权重 | 正常，等几秒到几十秒 |
| 注册后跟踪漂移/乱转 | bbox 掩码含背景过多 | 换 `mask_mode: sam`；或调小 `depth.center_ratio` |
| `框内无有效深度` | 目标太近/太远/反光 | 调 `depth.min_depth/max_depth` |
| 显存不足 (CUDA OOM) | 6GB 下 FoundationPose+其它同跑 | 关掉多余 GPU 进程；VLM 走 API 不占显存；别用 sam |
| RViz 里位姿错位 | `camera_frame` 参数与实际 TF 树不一致 | 启动节点时 `-p camera_frame:=你的相机帧名` |
| `/object_intent` 一直是 -1 | Python 侧已不发 intent（见 8.5 的注意） | 正常现象，`action_dispatcher` 会忽略 -1；不想要这条日志就删掉 `foundationpose_node.py` 里的 `publish_intent` 调用 |
| `import cv_bridge` 报 NumPy 错 | NumPy 2.2.6 与 cv_bridge ABI 冲突 | 本工程不 import cv_bridge，忽略；勿降级 numpy |

## 13. 与 FoundationPose 解耦说明

- 阶段三在 `vlm_servo/` 里通过 `sys.path` 临时加入 `FoundationPose/` 目录后 `import estimater/Utils`，**只读复用、不改动**你 FoundationPose/ 里的任何文件。
- 换 VLM：写一个继承 `BaseVLM` 的类，在 `vlm/__init__.py::build_vlm()` 加分支即可。
- **阶段四接口**：`PoseEstimator` 输出的 4×4 pose（相机系）已就绪；接 MoveIt2 时，把 pose 经你已有的手眼标定转到机械臂基座即可。（意图字段已移除，动作路由需另行设计。）
