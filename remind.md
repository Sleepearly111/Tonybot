# 今晚教B的内容

## 1. 跑起来（3分钟）

- Arduino IDE 打开 `LiDARTest/LiDARTest.ino`，上传
- 关串口监视器，终端跑：`python tools/lidar_viewer.py COM3`
- 拿盒子在雷达前晃，确认绿点出现

## 2. g_map[] 是什么（5分钟）

```
g_map[0]   = 前方 0° 距离(mm)，0 = 没东西
g_map[90]  = 右
g_map[180] = 后
g_map[270] = 左
```

可视化里的绿弧 = g_map 某一段角度。让他指认。

## 3. 第一个 TODO：终点区定位（7分钟）

打开 `lidar_endzone.cpp`，讲逻辑：

> 通道二是 500×500 窄口，穿过后正前方变空旷。
> 同时 3 个物体在 2~3 米外，稳定出现在前方 ±30°。
>
> 判断条件：
> - 前方 0° 距离 > 600mm（穿过窄口了）
> - 前方 ±60° 能数出 3 段连续有效点（物体）

不给代码，给思路让他自己写。

## B 的优先级

| 模块 | 文件 | 状态 |
|------|------|------|
| 终点区定位 | `lidar_endzone.cpp` | 今晚开始 |
| 物体检测 | `lidar_navigate.cpp` → `lidar_findObjects()` | 第二个做 |
| 识别区导航 | `lidar_navigate.cpp` → `lidar_nav_toPlatform()` | 第三个做 |
| 物体识别实测 | `lidar_classify.cpp` | 已写好，有空调参 |

## 之后要教的

- 物体检测：怎么在 g_map[360] 里把 3 个物体分出来
- 导航：3 个物体的中心角度 → 转弯量，平均距离 → 前进量
- 调阈值：用可视化看 classify 的结果对不对
