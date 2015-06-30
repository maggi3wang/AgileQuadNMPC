/****************************************************************************
 *
 * AUTHOR:  Ross Allen
 * STARTED: June 29, 2015
 * 
 * NOTES:
 *      - This is a test uORB topic that compiles segments of
 *          mavlink_msg_traj_seg.
 *
 ****************************************************************************/

/**
 * @file trajectory_spline.h
 */

#ifndef TOPIC_TRAJECTORY_SPLINE_H_
#define TOPIC_TRAJECTORY_SPLINE_H_

//~ #include <stdint.h>
//~ #include <stdbool.h>
#include "../uORB.h"
//~ #include <vector>
#include <vector>
#include "trajectory_segment.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * 
 */
struct trajectory_spline_s {
    std::vector<trajectory_segment_s> segs; // segments of spline
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(trajectory_spline);

#endif
