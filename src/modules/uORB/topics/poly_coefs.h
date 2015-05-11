/****************************************************************************
 *
 * AUTHOR:  Ross Allen
 * STARTED: May 11, 2015
 * 
 * NOTES:
 *      - This is a test uORB topic that corresponds to 
 *          mavlink_msg_poly_coefs.
 *      - Should be better developed later
 *
 ****************************************************************************/

/**
 * @file poly_coefs.h
 * Definition of the raw VICON Motion Capture position
 */

#ifndef TOPIC_POLY_COEFS_H_
#define TOPIC_POLY_COEFS_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * 
 */
struct poly_coefs_s {
	float del_t;			/**< length of time polynomial trajectory is valid */

    float p_0; ///< Zero order coefficient.
    float p_1; ///< First order coefficient.
    float p_2; ///< Second order coefficient.
    float p_3; ///< Third order coefficient.
    float p_4; ///< Fourth order coefficient.
    float p_5; ///< Fifth order coefficient.
    float p_6; ///< Sixth order coefficient.
    float p_7; ///< Seventh order coefficient.
    float p_8; ///< Eigth order coefficient.
    float p_9; ///< Ninth order coefficient.


};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(poly_coefs);

#endif
