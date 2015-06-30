/****************************************************************************
 *
 * AUTHOR:  Ross Allen
 * STARTED: June 29, 2015
 * 
 * NOTES:
 *      - This is a test uORB topic that corresponds to 
 *          mavlink_msg_traj_seg.
 *      - Follow on from poly_coefs
 *
 ****************************************************************************/

/**
 * @file trajectory_segment.h
 */

#ifndef TOPIC_TRAJECTORY_SEGMENT_H_
#define TOPIC_TRAJECTORY_SEGMENT_H_

//~ #include <stdint.h>
//~ #include <stdbool.h>
//~ #include "../uORB.h"
#include <vector>

/**
 * @addtogroup topics
 * @{
 */

/**
 * 
 */
struct trajectory_segment_s {
	float Tdel;     // time length of segment
    std::vector<float> xCoefs;  // coefficients for x polynomial
    std::vector<float> yCoefs;  // coefficients for y polynomial
    std::vector<float> zCoefs;  // coefficients for z polynomial
    std::vector<float> yawCoefs;  // coefficients for yaw polynomial
    uint8_t nSeg;   // total number of segments in spline
    uint8_t curSeg;     // current segment in spline
};

/**
 * @}
 */

/* register this as object request broker structure */
//~ ORB_DECLARE(trajectory_segment);

#endif
