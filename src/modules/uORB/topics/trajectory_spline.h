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

#include "../uORB.h"

#define MAX_TRAJ_SEGS 5

/**
 * @addtogroup topics
 * @{
 */
 
struct __EXPORT trajectory_segment_s {
	float Tdel;     // time length of segment
    uint8_t nSeg;   // total number of segments in spline
    uint8_t curSeg;     // current segment in spline
    float xCoefs[10];  // coefficients for x polynomial
    float yCoefs[10];  // coefficients for y polynomial
    float zCoefs[10];  // coefficients for z polynomial
    float yawCoefs[10];  // coefficients for yaw polynomial
    
    // Constructor
    trajectory_segment_s():
        Tdel(0.0f),
        nSeg(0),
        curSeg(0),
        xCoefs{0},
        yCoefs{0},
        zCoefs{0},
        yawCoefs{0}
    {
    }
};

/**
 * 
 */
struct __EXPORT trajectory_spline_s {
    trajectory_segment_s segArr[MAX_TRAJ_SEGS]; // segments of spline, max 5
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(trajectory_spline);

#endif
