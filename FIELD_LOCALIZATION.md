# 全场定位系统（Limelight + 里程计融合）使用说明

本目录是在 `FtcRobotController` 官方 SDK（12.0.0）之上、**不依赖任何第三方库**（无 Pedro Pathing / FTCLib / Road Runner）实现的一套完整全场定位 + 自动程序。它对应 FatDragon2026 里 Limelight 用法的"完整版"：FatDragon 只做到"读出 AprilTag 的 ID"，这里补齐了从图像到场地坐标、再到自动行驶的整条链路。

---

## 一、与原项目（FatDragon2026）的对应关系

| FatDragon2026 | 本项目 | 说明 |
| --- | --- | --- |
| `subsystems/vision/AutoApriltag`（`Limelight3A` + `getFiducialResults()` 取 ID） | `localization/LimelightVision` | 同样的相机生命周期与轮询方式，但把每个 tag 的**几何量、时间戳、延迟**都解析出来，且永不阻塞主循环 |
| `subsystems/vision/AutoAlign`（空壳，仅有 `Pose2D -> 速度/角度` 两个返回常量的函数） | `localization/VisionEstimator` + `drive/PoseFollower` | 真正的位姿解算与"对准到某个场地坐标"的运动控制 |
| Pedro Pathing `Follower`（读 Pinpoint，`getPose()`） | `localization/Odometry` + `localization/FieldLocalizer` | 用四个驱动电机编码器 + 内置 IMU 做航迹推算，再用视觉修正；接口仍是"给我一个 pose" |
| `FollowerBuilder.pinpointLocalizer()` | `Odometry` | 如果你的车上有 Pinpoint / OTOS，只需替换 `Odometry` 内部实现，其余代码不用动 |
| `opmodes/autos/AutoTemplate`（`SequentialCommandGroup` + `AutoDriveCommand`） | `auto/FieldAutoBase` + `auto/AutoSteps` | 用更轻量的 Step 链替代 command 框架，且每一步都跑在**融合后的场地坐标**上 |
| `tests/MyLocalizationTest`（Dashboard 画轨迹） | `teleop/LocalizationTeleOp` | 不用 Dashboard，直接在 Driver Station 上看融合位姿、里程计位姿与二者的漂移差 |

---

## 二、坐标约定（全系统统一）

* 单位：**英寸（inch）** 与 **弧度**。只有 Limelight 层会碰到米，进入本系统前全部换算掉。
* 场地坐标原点在场地**角点**，`+X` 沿一条边，`+Y` 沿另一条边，整个场地为 `[0,144] × [0,144]`。
* 朝向 `heading` 从 `+X` 轴起算，**逆时针为正**（数学正方向），与 `atan2` 一致。
* 蓝方 = 红方坐标做镜像：`x -> 144 - x`，`heading -> 180° - heading`。所以自动程序只写一套红方位姿。

---

## 三、代码结构

```
teamcode/
├── localization/
│   ├── FieldPose.java          场地位姿（不可变）+ 角度归一化 / 插值 / 镜像
│   ├── FieldConstants.java     场地尺寸、边界判定、红蓝方枚举
│   ├── AprilTagMap.java        ★ AprilTag 场地坐标表（每个赛季必须改这里）
│   ├── VisionConstants.java    ★ 视觉与融合的全部可调参数
│   ├── LimelightVision.java    Limelight 封装：帧快照 / 时间戳 / 延迟 / 朝向注入
│   ├── VisionEstimator.java    多 tag 二维解算（MegaTag 思路，得到的坐标系由你掌控）
│   ├── Odometry.java           编码器 + IMU 航迹推算（可替换为 Pinpoint）
│   └── FieldLocalizer.java     ★ 融合核心：预测 + 延迟补偿 + 残差门限 + 加权修正
├── drive/
│   ├── DriveConstants.java     ★ 电机名、轮径、编码器线数、各种符号与限速
│   ├── RobotHardware.java      统一取硬件，缺件降级不崩
│   ├── MecanumDrive.java       四轮麦轮运动学（机器人系 / 场地系两种指令）
│   └── PoseFollower.java       "开到某个场地坐标"的梯形速度 + 朝向 PID 控制
├── auto/
│   ├── AutoStep.java           自动程序的最小执行单元
│   ├── AutoSteps.java          DriveTo / Action / Wait / VisionSettle 四种积木
│   ├── AutoPoses.java          ★ 航点坐标（红方系）
│   ├── FieldAutoBase.java      ★ 自动主流程（初始化锁定位姿 → 任务链 → telemetry）
│   ├── FieldAutoRed.java       Auto: Field Tour (Red)
│   └── FieldAutoBlue.java      Auto: Field Tour (Blue)
└── teleop/
    └── LocalizationTeleOp.java ★ 先跑这个验证定位，再跑自动
```

带 ★ 的是你实际需要改的文件，其余是实现细节。

---

## 四、上机前必须做的标定（15 分钟）

### 1. 硬件配置（Driver Station → Configure Robot）

| 名称 | 类型 | 是否必需 |
| --- | --- | --- |
| `fl` `fr` `bl` `br` | 驱动电机（麦轮） | 必需 |
| `imu` | Control Hub / Expansion Hub 内置 IMU | 强烈建议 |
| `limelight` | Limelight 3A | 必需 |
| `shooter` / `intake` / `feeder` | 电机 / 电机 / 舵机 | 可选，没有就跳过对应动作 |

> 电机名不一致就改 `DriveConstants` 里的常量，或反过来改配置名。

### 2. 编码器与 IMU 符号（决定定位能不能用）

跑 `Localization TeleOp`，用手推车：

* 向车头方向推 → `pose` 的 X 增大；左右推 → Y 增大；
* 逆时针转车 → `heading` 增大。

哪一项反了就改：

* 前后计数相反 → 改 `DriveConstants.ENCODER_SIGN_*`（按轮微调）；
* 朝向相反 → 改 `DriveConstants.IMU_HEADING_SIGN`（+1 / −1）。

### 3. AprilTag 场地坐标表（BIOBUZZ 2026-2027）

赛规（手册第 9 节）已确认的部分，代码里已按此实现：

* 家族 **36h11**，共 **16 个 tag，ID 30–45**，每组 4 个：
  * `30–33` 观众侧红色蜂房；`34–37` 观众侧蓝色蜂房
  * `38–41` 观众对侧红色蜂房；`42–45` 观众对侧蓝色蜂房
* 每个 tag **装在蜂房底部、朝下对着地垫，底边朝向场地中心**
* 一组内 4 个 tag 的中心距为 `6.5 in` / `7.0 in` / `6.5 in`，蜂房面上还有 3.25 in 方形参考孔

`AprilTagMap.java` 已经把 16 个 tag 用「4 个圆心 + 间距」自动生成，**你只需要填 4 组锚点**：

```java
public static double AUDIENCE_RED_X = 36.0;   // 该组 4 个 tag 中心点的 x（英寸）
public static double AUDIENCE_RED_Y = 12.0;   // y
public static double AUDIENCE_RED_YAW_DEG = 90.0;  // 底边指向（朝场地中心）
```

四个锚点分别是 `AUDIENCE_RED` / `AUDIENCE_BLUE` / `FAR_RED` / `FAR_BLUE`。现在的数值是对称估计值，**必须用卷尺实测**：量每个蜂房 4 个 tag 的中心点到两条场地边的距离即可。

> 蜂房在 144×144 场地上的具体坐标取决于场地图（图 9-8 之类的俯视图），手册这两页没有给出。数据一给，16 个坐标立刻自动生成。

### 3b. 相机必须“抬头看”

因为 tag 朝下，**相机必须朝上安装（或大仰角）**才能看到蜂房底部。三个后果：

* 相机 pitch 一定要在 Limelight 网页端标定好，否则距离会系统性偏差；
* `VisionConstants.USE_TAG_HEADING` 已设为 `false`：这种角度下算出来的 tag 朝向是相机俯仰，不是车的朝向，用它会把位姿慢慢拧歪。**朝向完全交给 IMU**；
* 位置解算仍然准确（只用 tag 在相机坐标系里的 x/y 投影）。

### 4. 视觉横向坐标符号

把一个 tag 放在车前偏左的位置：

* 若 telemetry 的横向值为正（`ROBOT_SPACE_LEFT_SIGN = +1` 正确）→ 不用动；
* 若为负 → 改成 `-1`。

判断依据：放偏左却算出偏右，位置解算会**系统性**偏 2 倍横向距离。

---

## 五、参数怎么调

| 现象 | 调哪里 |
| --- | --- |
| 停车时位姿在 1~2 inch 内轻微跳动 | `VisionConstants.TRANSLATION_GAIN_*` 调小（0.2/0.4） |
| 走一段路后视觉"拉不回来" | 增益调大（0.5/0.8），同时确认 `AprilTagMap` 准确 |
| 视觉经常被"position jump"拒绝 | `MAX_TRANSLATION_RESIDUAL_IN` 适当放大；先排查编码器符号 |
| 自动转弯过冲 / 抖动 | `PATH_HEADING_KP` 降一点，`ROTATION_SCALE` 调到实际值 |
| 自动到点太慢 / 太急 | `PATH_MAX_SPEED_IN_PER_SEC`、`PATH_MAX_ACCEL_IN_PER_SEC2` |
| 停车精度不够 | `PATH_POSITION_TOLERANCE_IN` 收紧到 0.8，同时提高视觉增益 |

---

## 六、融合算法在做什么（为什么不是"直接用相机位姿"）

```
编码器+IMU ──► Odometry ──► 增量位移 ──► 预测位姿 ──┐
                                                   ├──► 融合位姿 ──► 路径跟随 ──► 电机
Limelight ──► Frame ──► VisionEstimator ──► 观测 ──┤
                     │                             │
                     └── 时间戳 ──► 位姿历史 ──► 延迟补偿 ──► 残差门限
```

三个关键细节，缺一个就不是融合：

1. **延迟补偿**：相机帧是 30~80 ms 前的画面。融合时回查"拍下这张图那一刻"的预测位姿来做比较，而不是拿"现在的位姿"比较；否则车一动，视觉就会和自己打架，产生持续抖动。
2. **残差门限**：观测和预测差太多（>24 inch）、或几个 tag 之间互相矛盾（>18 inch）、或朝向差太多（>30°）时整帧丢弃并计数。这样单个误识别的 tag 不会把自动程序送到场地外面。第一帧例外：允许一次大范围"吸附"，用来替代人工设置起始位姿。
3. **加权修正**：每一帧只把位姿往观测方向拉一部分（单 tag 0.35 / 多 tag 0.65，再按距离衰减），输出连续，不会给电机一个阶跃指令。

朝向以陀螺仪为主，视觉只做小幅修正（≤2.5°/帧）：因为 tag 朝向约定一旦弄错，会变成恒定的角度偏差，如果给大权重就会让车自己转起来。

---

## 七、自动程序

`Auto: Field Tour (Red)` / `(Blue)`：

1. **init 阶段**持续解算（车不要动），第一帧有效 tag 直接替换掉配置的起始位姿；
2. 按 <kbd>Play</kbd> 后先 `anchor()`：把航迹推算归零到刚确认的位姿；
3. 任务链：`VisionSettle → 开到得分点 → 转轮/送料 → 开吸球去取球（边开边吸）→ 回得分点 → …→ 停靠`；
4. 每个航点都用融合位姿闭环，滑了、被撞了，下一个视觉帧就会拉回来；
5. telemetry 实时显示当前步骤、位姿、tag 数、残差、拒绝帧数。

`AutoPoses.java` 里是**示例航点**（故意横穿全场来检验定位）。改成你赛季的真实分数点即可，蓝方自动镜像，无需第二套坐标。

---

## 八、已知限制 / 下一步

* **单 tag 时朝向不可靠**：所以朝向以 IMU 为主；如果陀螺仪漂移明显，可以开启 `HEADING_GAIN` 并确认 tag 朝向约定正确。
* **没有障碍物建模**：路径是点到点直线，场地有队友时需要自行加避让航点。
* **如果想要更高级**：把 `Odometry` 换成 Pinpoint / 三轮全向轮里程计；或在 `FieldLocalizer` 里用 UKF/粒子滤波替换互补式加权。接口都是 `update()` + `getPose()`，其余层不用改。
* **Limelight 的 MegaTag2**：`LimelightVision` 每帧都把融合朝向注入 `updateRobotOrientation()`，因此相机端的 MegaTag2 也是可用的；但它的位姿坐标系取决于上传到相机的 field map，本系统故意不依赖它，以保证坐标完全自控。
