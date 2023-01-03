/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-Cyphal/graphs/contributors.
 */

#ifndef NODE_INFO_H_
#define NODE_INFO_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <107-Arduino-Cyphal.h>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static NodeInfo node_info
(
    /* uavcan.node.Version.1.0 protocol_version */
    1, 0,
    /* uavcan.node.Version.1.0 hardware_version */
    1, 0,
    /* uavcan.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
    0,
    /* saturated uint8[16] unique_id */
    OpenCyphalUniqueId(),
    /* saturated uint8[<=50] name */
    "generationmake.NeoPixelNode"
);

#endif /* NODE_INFO_H_ */
