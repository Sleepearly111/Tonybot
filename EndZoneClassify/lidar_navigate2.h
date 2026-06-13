/*
 * lidar_navigate2.h — 极简导航：对准 → 直走 → 停下
 */
#ifndef LIDAR_NAVIGATE2_H
#define LIDAR_NAVIGATE2_H

#include "lidar_common.h"

enum Nav2Phase {
    N2_STABLE,
    N2_TURN,
    N2_GO,
    N2_ARRIVED
};

void nav2_reset();
Nav2Phase nav2_phase();
NavCmd nav2_update();

#endif
