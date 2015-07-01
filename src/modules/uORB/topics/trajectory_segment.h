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

//~ #ifndef TOPIC_TRAJECTORY_SEGMENT_H_
//~ #define TOPIC_TRAJECTORY_SEGMENT_H_

//~ #include <stdint.h>
//~ #include <stdbool.h>
//~ #include "../uORB.h"
//~ #include <vector>

/**
 * @addtogroup topics
 * @{
 */

/**
 * 
 */
//~ struct trajectory_segment_s {
	//~ float Tdel;     // time length of segment
    //~ uint8_t nSeg;   // total number of segments in spline
    //~ uint8_t curSeg;     // current segment in spline
    //~ float xCoefs[10];  // coefficients for x polynomial
    //~ float yCoefs[10];  // coefficients for y polynomial
    //~ float zCoefs[10];  // coefficients for z polynomial
    //~ float yawCoefs[10];  // coefficients for yaw polynomial
    //~ 
    //~ // Constructor
    //~ trajectory_segment_s():
        //~ Tdel(0.0f),
        //~ nSeg(0),
        //~ curSeg(0),
        //~ xCoefs{0},
        //~ yCoefs{0},
        //~ zCoefs{0},
        //~ yawCoefs{0}
    //~ {
    //~ }
//~ };

/**
 * @}
 */

/* register this as object request broker structure */
//~ ORB_DECLARE(trajectory_segment);

//~ #endif
