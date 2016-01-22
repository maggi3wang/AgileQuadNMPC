/****************************************************************************
 *
 *  AUTHOR: @author Ross Allen <ross.allen@stanford.edu>
 *  STARTED: Jan 2016
 *
 *
 ****************************************************************************/

/**
 * @file obs_repel_force_ned.h
 * Contains repulsive force in NED coords due to proximity and relative velocity
 * of obstacles
 */

#ifndef TOPIC_OBS_REPEL_FORCE_NED_H_
#define TOPIC_OBS_REPEL_FORCE_NED_H_


#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct obs_repel_force_ned_s {
	uint64_t timestamp;		/**< in microseconds since system start, is set whenever the writing thread stores new data */
	
	float Fx;			/**< [m] x-axis repulsive force in world NED coords */
	float Fy;			/**< [m] y-axis repulsive force in world NED coords */
	float Fz;			/**< [m] z-axis repulsive force in world NED coords */
}; 


/* register this as object request broker structure */
ORB_DECLARE(obs_repel_force_ned);

#endif
