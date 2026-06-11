#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

#include <Arduino.h>
#include <RPLidar.h>

#define LIDAR_MAP_SIZE  360  // 1° 分辨率

class LidarDriver {
public:
    LidarDriver();
    ~LidarDriver();

    // === 初始化 ===
    // rxPin: ESP32 接收引脚（接 A1M8 TX）
    // txPin: ESP32 发送引脚（接 A1M8 RX）
    // motorPin: 电机 PWM 控制引脚
    bool begin(int rxPin, int txPin, int motorPin);
    void end();

    // === 主循环 ===
    // 每帧处理一个扫描点，返回 true 表示有新数据
    bool update();

    // === 扫描状态 ===
    // 是否完成了一圈完整的 360° 扫描
    bool isScanComplete();
    void resetScanFlag();
    bool isConnected() { return m_connected; }

    // === 数据读取（单位：mm）===
    // 指定角度（0~359°）的距离值，0=无数据
    float distanceAt(int deg) const;

    // 在角度区间 [startDeg, endDeg] 中找最远/最近的方向角
    // 返回值：角度（度），-1 表示无有效数据
    int  findFarthest(int startDeg, int endDeg, int stepDeg = 1) const;
    int  findNearest(int startDeg, int endDeg, int stepDeg = 1) const;

    // 扇形区域统计数据
    float sectorMin(int centerDeg, int widthDeg) const;
    float sectorMax(int centerDeg, int widthDeg) const;
    float sectorAvg(int centerDeg, int widthDeg) const;

    // 该扇形区域是否空旷（所有距离 > thresholdMM）
    bool  isSectorClear(int centerDeg, int widthDeg, float thresholdMM) const;

    // === 调试输出 ===
    void printSummary();
    void printPolarPlot();
    // 流式输出完整一圈扫描数据（给 PC 端 lidar_viewer.py 读取）
    // 格式: SCAN <ts> <count>\n<angle> <dist> <quality>\n... END
    void printScanStream();

    // === 形状识别（激光雷达方案）===
    // 在扇形区域中找到最近的物体，返回其中心角度（-1=无物体）
    int  findObjectInSector(int startDeg, int endDeg, float maxDistMm = 2000) const;

    // 物体分析结果
    struct ShapeInfo {
        int     centerAngle;    // 物体中心角度
        int     angularSpan;    // 物体占据的度数
        float   minDist;        // 最近距离 (mm)
        float   maxDist;        // 最远距离 (mm) — 物体表面范围内的
        float   distStddev;     // 距离标准差 — 衡量表面平坦度
        bool    sharpEdges;     // 边缘是否陡峭（直角边）
        int     shapeType;      // 0=未知, 1=球体, 2=正方体, 3=圆柱体
    };

    // 分析指定角度物体的形状
    // 返回 shapType，同时填充 info 结构体用于调试
    int  classifyObjectAt(int angleDeg, ShapeInfo* info = nullptr);

    // === 裸数据访问 ===
    const float*   getMap()    const { return m_map; }
    const uint8_t* getQuality()const { return m_quality; }
    uint32_t getPointCount() const { return m_pointCount; }

private:
    RPLidar     m_lidar;
    int         m_motorPin;
    float       m_map[LIDAR_MAP_SIZE];       // 距离 (mm)
    uint8_t     m_quality[LIDAR_MAP_SIZE];   // 信号质量 0~255
    unsigned long m_lastPointMs;
    bool        m_scanReady;
    bool        m_connected;
    int         m_lastAngle;
    uint32_t    m_pointCount;                // 统计
};

#endif
