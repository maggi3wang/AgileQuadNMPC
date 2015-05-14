/****************************************************************************
 *
 *  AUTHOR: @author Ross Allen
 *  STARTED: May 13, 2015
 *
 *
 ****************************************************************************/

/**
 * @file vehicle_velocity_feed_forward.h
 * Definition of the velocity feed forward uORB topic.
 */

#ifndef TOPIC_VEHICLE_VELOCITY_FEED_FORWARD_H_
#define TOPIC_VEHICLE_VELOCITY_FEED_FORWARD_H_

#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct vehicle_velocity_feed_forward_s {
	float vx;		/**< in m/s NED			  		*/
	float vy;		/**< in m/s NED			  		*/
	float vz;		/**< in m/s NED			  		*/
}; /**< Velocity feed forward term in NED frame */

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_velocity_feed_forward);

#endif
