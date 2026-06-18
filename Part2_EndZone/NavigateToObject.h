/*
 * NavigateToObject.h — 文字驱动导航
 *
 * 流程:
 *   第1轮: PARALLEL → WAIT_TEXT → CLASSIFY → FIND_TARGET → SLIDE → APPROACH → VOICE → TURN_HEAD
 *   第2轮: WAIT_TEXT → HEAD_BACK → CLASSIFY → FIND_TARGET → SLIDE → APPROACH → VOICE → TURN_HEAD
 *   第3轮: WAIT_TEXT → HEAD_BACK → CLASSIFY → FIND_TARGET → SLIDE → APPROACH → VOICE → DONE
 *
 *   第1轮头正前(90°)等文字, 之后偏开(0°)等文字, 识别后回正(90°)再导航
 */
#ifndef NAVIGATE_TO_OBJECT_H
#define NAVIGATE_TO_OBJECT_H

#include "lidar_common.h"

enum NTO_Phase {
    NTO_GOTO_FIRST,
    NTO_PARALLEL,      // 调平行
    NTO_WAIT_TEXT,     // 等文字
    NTO_HEAD_BACK,     // 头回正
    NTO_CLASSIFY,      // 分类记录三个位置形状
    NTO_SUMMARY,       // 从左到右播报一遍三个物体
    NTO_FIND_TARGET,   // 查表找目标位置
    NTO_SLIDE,         // 侧滑+数跌落
    NTO_APPROACH,      // 直走靠近
    NTO_VOICE,         // 播报
    NTO_TURN_HEAD,     // 偏头等下一个文字
    NTO_DONE           // 全部完成
};

void nto_reset();
NTO_Phase nto_phase();
NavCmd nto_update();

void nto_textReceived(int shape);
void nto_headTurnDone();
void nto_headBackDone();
void nto_summaryDone();
void nto_getObjects(int angles[3], int shapes[3]);
int  nto_currentShape();

#endif
