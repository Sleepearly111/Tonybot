# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Developer Identity

本项目（Tonybot CRAIC 2026 比赛机器人）有两个开发者，**每次对话开始时先确认当前是谁**，再按对应角色工作。

### 同学A（PlatformIO）

- **身份**：主程，ESP32 端负责人
- **工具**：VS Code + PlatformIO
- **负责目录**：`Esp32/`、`Esp32S3/`、`tools/`
- **权限**：可以创建/修改/删除 `Esp32/` 和 `Esp32S3/` 下的任何文件；可以在项目根目录创建调试工具和脚本；可以修改 CLAUDE.md 和 README.md
- **不能动**：`PillarNavigator/` 和 `ESP32S3Cam/`（同学B 的 Arduino IDE 原版代码）
- **技术栈**：C++ (Arduino framework, ESP32, FreeRTOS), Python (PC 端调试工具)

### 同学B（Arduino IDE）

- **身份**：LiDAR 算法负责人（终点区定位、识别区导航、物体识别）
- **工具**：Arduino IDE
- **负责目录**：`LiDARTest/`（LiDAR 模块开发+测试）、`PillarNavigator/`（已完成，仅保留）、`ESP32S3Cam/`（已完成，仅保留）
- **权限**：可以创建/修改/删除 `LiDARTest/` 下的文件；可以在项目根目录创建 Arduino 测试项目
- **不能动**：`Esp32/` 和 `Esp32S3/`（同学A 的 PlatformIO 代码）
- **技术栈**：C++ (Arduino framework, ESP32), Arduino 库管理
- **注意**：不熟悉 `pio` 命令，所有编译上传在 Arduino IDE 里完成；写好的 LiDAR 算法最终由 A 集成到 `Esp32/src/`

### 共享资源

- `CLAUDE.md`、`README.md` — 两人都可以更新
- `tools/lidar_viewer.py` — PC 端 LiDAR 可视化工具，两人通用
- LiDAR 流式协议 (`SCAN ... END`) — 跨平台，`lidar_viewer.py` 解析
- I2C 协议 (ESP32-S3 ↔ ESP32) — A 的 `Esp32/src/hw_esp32cam_ctl.*` 和 B 的 S3 代码之间的接口

### B 的 LiDAR 开发环境 (`LiDARTest/`)

```
LiDARTest/
├── LiDARTest.ino            ← setup/loop，调各模块
├── lidar_common.h/.cpp      ← 雷达初始化 + 数据采集 (g_map[360])
├── lidar_endzone.h/.cpp     ← 终点区定位 [TODO: B]
├── lidar_navigate.h/.cpp    ← 识别区导航 [TODO: B]
├── lidar_classify.h/.cpp    ← 物体形状识别 [Done, 待实测调参]
├── RPLidar.h + .cpp         ← RoboPeak 雷达驱动
└── inc/                     ← 底层协议头文件
```

B 的 loop() 里注释掉模块调用即可测试：依次取消 `lidar_isAtEndZone()`、`lidar_nav_toPlatform()`、`classifyObject()` 的注释。

## Commands

This is a **PlatformIO** project with two independent targets: `Esp32/` (main controller) and `Esp32S3/` (vision coprocessor).

```bash
# Build (open the project folder first, then run inside it)
pio run                          # compile
pio run -t clean                 # clean build cache
pio run -t upload                # build + upload to board
pio device monitor               # serial monitor (115200 baud)

# Test (test/ dirs are empty scaffolding — no tests yet)
pio test                         # run all tests
```

All commands run from inside the project directory (`Esp32/` or `Esp32S3/`). In VS Code, use the PlatformIO toolbar: **✓** = Build, **→** = Upload, **🔌** = Serial Monitor.

### Branch workflow

- `develop` — active development
- `main` — tested, competition-ready code
- New features → branch from `develop`, PR back to `develop`

## Current Status (2026-06-11)

**分工**：A 负责比赛状态机 + `Esp32/` PlatformIO 集成；B 负责 `LiDARTest/` 中三个 LiDAR 模块的算法开发

**B 进度**：雷达硬件已接通，`lidar_classify.cpp` 算法已从 A 移植，`lidar_endzone.cpp` 和 `lidar_navigate.cpp` 待 B 实现

### 场地布局

```
总宽 = 2100mm，总深 = 4400mm，所有 500 宽区域均居中

起点区(600×500) → 通道一(500×500) → 避障区(2100×1300) → 通道二(500×500)
  → 大终点区(600×500) → 空区(500×500) → 置物台(1800×400, 高300)

小起点: 200×200 居中于大起点区
小终点: 200×200 居中于大终点区
置物台上: 3物体(球φ200-300 / 正方体300³ / 圆柱φ300×300)，间距300mm
避障区内: 2根圆柱障碍物(红/蓝各一，直径300，高300)
标签: A4纸印"球体""正方体""圆柱体"，大小180mm，终点区展示
标记区: 3个150×150胶带框，距置物台200mm

识别区 = 通道二(500) + 大终点区(600) + 空区(500) + 置物台(400) = 2000mm深
```

### 传感器约束

- **LiDAR 平面 > 300mm**：全场只能看到置物台上的 3 个物体
- 柱子高 300mm → LiDAR 平面以下，扫不到（仅相机看颜色）
- 置物台高 300mm → LiDAR 扫不到台面
- 地上胶带/标记区(150×150) → 相机和 LiDAR 都看不到

### 导航策略

**核心思路：3 个物体从头到尾在 LiDAR 视野中，作为全程导航信标；终点区汉字标签决定识别目标**

| 阶段 | 传感器 | 方法 |
|------|--------|------|
| 通道一穿越 | IMU + LiDAR | 航向保持 + 3物体星座居中 |
| 避障区绕柱 (Task 1) | 相机 | HeadTracker + ObjectFollower + CirclePillar（S型绕行，左右方向裁判现场指定） |
| 找通道二入口 | LiDAR | 原地旋转扫描 3 物体方向 |
| 通道二穿越 | LiDAR | 3物体星座居中导航 |
| 终点区标签识别 | 相机(S3 AI) | 到终点区暂停 → S3识别A4纸汉字标签（球体/正方体/圆柱体）→ 确定需要识别的物体 |
| 物体定位 (Task 2) | LiDAR | 根据标签结果，classifyObjectAt() 多次投票识别台上对应物体 |
| 标记区导航+播报 (Task 3) | LiDAR + TTS | 以物体为参照接近到 200mm → ShapeVoicePlayer 播报3次（"我是Tonybot，我识别到的是XXX，XXX，XXX"） |

### 已知问题

| Issue | Severity | Status |
|-------|----------|--------|
| CirclePillar CIRCLING→DONE 无自动退出条件 | 🟡 已知 | 待修复 |
| action group 不一致: ObjectFollower 用 25 forward, RobotTracking 用 21 | 🟡 已知 | 待场地确认 |

---

## Project Context

**第二十八届中国机器人及人工智能大赛 - 具身智能任务赛（线下家庭服务场景）**。机器人从起点出发自主完成 3 个连续任务（避障→识别→播报），全程无遥控。

**Hardware:** ESP32 (main controller) + ESP32-S3 (vision coprocessor), Lobot serial bus servos, A1M8 RPLIDAR, QMI8658 IMU, TTS/ASR I2C voice modules.

**Tech stack:** C++ (Arduino framework for ESP32), FreeRTOS, I2C, MadgwickAHRS, TFLite.

## Core Constraints

1. **全程自主运行** — 不能遥控。所有决策必须在板载代码中完成。
2. **限时** — Task 1 最多 5 分钟；Task 2+3 总共最多 5 分钟。
3. **碰撞扣 5 分/次** — 持续碰撞同一障碍物 → 任务直接失败。
4. **压线扣 5 分/次** — 一直压线 → 任务直接失败。
5. **识别/播报错误扣 10 分/个**。
6. **Code rules:** Don't delete `PillarNavigator/` or `ESP32S3Cam/` (classmate's Arduino IDE originals). Action group IDs must match hardware. No `delay()` blocking in main loop (use non-blocking timers).

## Architecture

### Dual-chip I2C communication

```
ESP32-S3 (slave 0x52)                  ESP32 (master)
  ├─ Ov2640 camera                     ├─ Serial2 → Lobot bus servos
  ├─ TFLite AI inference               ├─ GPIO4   → PWM head servo
  ├─ HSV color detection               ├─ UART    → A1M8 RPLIDAR
  └─ I2C registers:                    ├─ I2C     → QMI8658 IMU
       0x00 = red block data           ├─ I2C     → TTS module (0x40)
       0x01 = green block data         └─ I2C     → ASR module (0x34)
       0x02 = blue block data
       0x10 = AI label result (0x11=球体, 0x12=正方体, 0x13=圆柱体, 0=none)
       0x20 = AI shape result  (1=球体, 2=正方体, 3=圆柱体, 0=none)
```

### Source layout

```
Esp32/src/              Esp32S3/src/
├── main.cpp            ├── main.cpp
├── globals.cpp         ├── camera_setting.*
├── base_config.h       ├── VisionAI.*
├── hw_esp32cam_ctl.*   ├── iic_data_send.*
├── voice/              ├── color_detection.*
│   ├── HWSensor.*      └── who_ai_utils.*
│   └── ShapeVoicePlayer.*
├── lidar/
│   └── LidarDriver.*       ← A1M8 360°扫描 + 形状分类
├── fall/
│   ├── TumbleGuard.*
│   └── FallDetector.*
├── tracking/
│   ├── HeadTracker.*       ← 头部舵机追踪色块
│   ├── ObjectFollower.*    ← 跟随色块状态机
│   ├── CirclePillar.*      ← 绕柱导航
│   └── RobotTracking.*     ← PID 视觉追踪
├── LobotServoCtl/
│   └── LobotServoController.cpp  ← 串口舵机驱动
├── PwmServo/
│   └── Servo.cpp                ← PWM 舵机驱动
└── IMU/
```

### Module status

| Module | Status | What it does |
|--------|--------|-------------|
| **LidarDriver** | ✅ Done | A1M8 360° scanning, obstacle detection, shape classification (sphere/cube/cylinder) |
| **IMU (QMI8658+Madgwick)** | ✅ Done | 6-axis orientation, roll/pitch output |
| **FallDetector / TumbleGuard** | ✅ Done | Two implementations of IMU-based fall detection + auto-stand action groups |
| **HeadTracker** | ✅ Done | Head servo tracks color target via I2C data from S3 |
| **ObjectFollower** | ✅ Done | State machine to follow color target (turn → forward → arrived) |
| **CirclePillar** | ✅ Done | Pillar navigation (left/right circling modes) |
| **RobotTracking** | ✅ Done | PID visual tracking (head servo + leg action groups) |
| **TTS/ASR + ShapeVoicePlayer** | ✅ Done | Voice synthesis + recognition; announces object name 3× |
| **ESP32Cam I2C** | ✅ Done | I2C master comms with S3 slave |
| **ESP32-S3 Vision** | ✅ Done | Camera + HSV detection + AI inference (汉字分类: 球体/正方体/圆柱体) |
| **Servo drivers** | ✅ Done | PWM (LEDC) + Lobot serial bus servo protocols |

## Gap Analysis

### A 待办 (Esp32/ PlatformIO)

| Priority | 模块 | 说明 |
|----------|------|------|
| 🔴 P0 | **比赛状态机** | `main.cpp`: Task1(相机绕柱) → Task2(读I2C标签→调B的LiDAR算法识别) → Task3(标记区+播报)，带超时保护 |
| 🔴 P0 | **集成B的LiDAR算法** | B 在 `LiDARTest/` 写好后，A 移植到 `Esp32/src/lidar/` 或调用 B 的接口 |
| 🟡 P1 | **CirclePillar DONE** | 加自动退出条件 |
| 🟡 P1 | **碰撞规避** | 实时 LiDAR 前方监测，碰撞前刹车 |
| 🟢 P2 | **扣分计数** | 运行时记录碰撞/压线次数 |

### B 待办 (LiDARTest/ Arduino IDE)

| Priority | 模块 | 文件 | 说明 |
|----------|------|------|------|
| 🔴 P0 | **终点区定位** | `lidar_endzone.cpp` | 判断穿过通道二后到达终点区 |
| 🔴 P0 | **识别区导航** | `lidar_navigate.cpp` | 以3物体为信标导航到置物台前200mm |
| 🟡 P1 | **物体识别实测** | `lidar_classify.cpp` | 用可视化工具实测调阈值 |
| 🟡 P1 | **3物体检测** | `lidar_navigate.cpp` 里 `lidar_findObjects()` | 在g_map[]中找出3个物体 |

### Arena constants reference

```cpp
// arena.h — all in mm
#define ARENA_TOTAL_DEPTH  4400  // 600+500+1300+500+600+500+400
#define ARENA_TOTAL_WIDTH  2100
#define NARROW_ZONE_WIDTH  500
#define OBSTACLE_ZONE_DEPTH 1300
#define RECOG_ZONE_DEPTH   2000  // channel2+end_zone+gnd_gap+platform
#define CHANNEL_LENGTH     500
#define PLATFORM_WIDTH     1800
#define PLATFORM_DEPTH     400
#define PLATFORM_HEIGHT    300
#define OBJECT_SPACING     300   // center-to-center between objects on platform
#define MARKER_OFFSET      200   // distance from platform to marker boxes
#define MARKER_SIZE        150   // marker box side length
#define START_BOX_SIZE     200   // small start/end box
#define LARGE_ZONE_DEPTH   600   // large start/end zone depth
#define LIDAR_SCAN_HEIGHT  300   // LiDAR plane is above this → only sees objects on platform
```

### Key function reference

```cpp
// ============ A: Esp32/src/lidar/LidarDriver (PlatformIO) ============
lidar.begin(rx, tx, motorPin);
lidar.update();                           // 每帧一个扫描点
lidar.isScanComplete();
lidar.printScanStream();                  // 流式输出 SCAN...END（给可视化工具）
lidar.findObjectInSector(start, end, maxDist);
lidar.classifyObjectAt(angleDeg);         // 1=sphere 2=cube 3=cylinder
lidar.sectorMin(centerDeg, widthDeg);
lidar.distanceAt(deg);

// ============ B: LiDARTest/ (Arduino IDE) ============
lidar_init();                             // 初始化雷达 + 开始扫描
lidar_update();                           // 每帧一个点 → g_map[360]
lidar_streamScan();                       // 流式输出
lidar_isAtEndZone();                      // 终点区判定 [TODO]
lidar_nav_toPlatform();                   // 导航到置物台前 [TODO]
lidar_findObjects(outAngles);             // 找3个物体 [TODO]
classifyObject(angleDeg, &info);          // 形状识别 [Done]

// ============ 可视化 ============
// python tools/lidar_viewer.py COM3           实时显示
// python tools/lidar_viewer.py COM3 --record  录制日志

// ============ S3 AI label recognition ============
cam.begin();
uint8_t label = cam.readAILabel();        // 0x11=球体 0x12=正方体 0x13=圆柱体

// ============ Tracking ============
circlePillar_init() / _update() / _isDone();
objectFollower_init() / _update();
tracker_set_color(TRACK_COLOR_RED|TRACK_COLOR_BLUE);

// ============ Voice ============
ShapeVoicePlayer player;
player.playSphere(); player.playCube(); player.playCylinder();

// ============ IMU ============
imu.begin();
imu.get_angle(&roll, &pitch);
imu.get_yaw();

// ============ WDT-safe delay ============
safeDelay(ms);

// ============ Servo ============
Controller.runActionGroup(groupId, times);
// 101=back-stand  102=front-stand  11=slide-left  12=slide-right
// 23=turn-left    24=turn-right    25=forward     0=stand
```
