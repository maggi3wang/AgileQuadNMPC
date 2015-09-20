/****************************************************************************
 *
 *  AUTHOR: @author Ross Allen <ross.allen@stanford.edu>
 *  STARTED: Sept 20, 2015
 *
 *
 ****************************************************************************/

/**
 * @file trajectory_nominal_values.h
 * Contains nominal state and input values for a trajectory spline
 */

#ifndef TOPIC_TRAJECTORY_NOMINAL_VALUES_H_
#define TOPIC_TRAJECTORY_NOMINAL_VALUES_H_

#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct trajectory_nominal_values_s {
	uint64_t timestamp;		/**< in microseconds since system start, is set whenever the writing thread stores new data */
	
	float x;			/**< [m] x-pos in world NED coords */
	float y;			/**< [m] y-pos in world NED coords */
	float z;			/**< [m] z-pos in world NED coords */
	float vx;		/**< [m/s] x-vel in world NED coords */
	float vy;		/**< [m/s] y-vel in world NED coords */
	float vz;		/**< [m/s] z-vel in world NED coords */
	float phi;		/**< [rad] roll of body wrt world */
	float theta;	/**< [rad] pitch of body wrt world */
	float psi;		/**< [rad] yaw of body wrt world */
	float p;		/**< [rad/s] angular velocity of body x-axis in body coords */
	float q;		/**< [rad/s] angular velocity of body x-axis in body coords */
	float r;		/**< [rad/s] angular velocity of body x-axis in body coords */
}; 

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(trajectory_nominal_values);

#endif
