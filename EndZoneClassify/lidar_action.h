/*
 * lidar_action.h — 舵机动作执行
 */
#ifndef LIDAR_ACTION_H
#define LIDAR_ACTION_H

#include "lidar_common.h"

void action_init();
void action_execute(const NavCmd& cmd);
void action_stand();
void action_wait(unsigned long ms);

#endif
