/**
 * @file mc_pos_control_params.h
 * Multicopter trajectory controller parameters for ASL testing.
 *
 * Autonomous Systems Laboratory
 * Stanford Aeronautics & Astronautics
 * Sep 18, 2015
 *
 * @author Ross Allen <rallen10@stanford.edu>
 */
 
/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_XY_POS 0.25f

/**
 * Proportional gain for vertical position error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_Z_POS 0.25f

/**
 * Proportional gain for horizontal velocity error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_XY_VEL 1.0f

/**
 * Proportional gain for vertical velocity error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_Z_VEL 1.0f

/**
 * Proportional gain for roll, pitch angular error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_RP_ANG 0.01f

/**
 * Proportional gain for yaw angular error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_YAW_ANG 0.01f

/**
 * Proportional gain for roll, pitch angular velocity error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_RP_OMG 0.01f

/**
 * Proportional gain for yaw angular velocity error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_YAW_OMG 0.01f

/**
 * Minimum thrust
 *
 * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 1.0
 */
#define TRAJ_PARAMS_THR_MIN 0.1f

/**
 * Maximum thrust
 *
 * Limit max allowed thrust.
 *
 * @min 0.0
 * @max 1.0
 */
#define TRAJ_PARAMS_THR_MAX 1.0f

/**
 * Maximum horizontal velocity
 *
 * Maximum horizontal velocity in AUTO mode and endpoint for position stabilized mode (POSCTRL).
 *
 * @unit m/s
 * @min 0.0
 */
#define TRAJ_PARAMS_XY_VEL_MAX 5.0f

/**
 * Maximum vertical velocity
 *
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
 *
 * @unit m/s
 * @min 0.0
 */
#define TRAJ_PARAMS_Z_VEL_MAX 2.0f

/**
 * Maximum tilt angle in air
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 */
#define TRAJ_PARAMS_TILTMAX_AIR 45.0f

/**
 * Maximum tilt during landing
 *
 * Limits maximum tilt angle on landing.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 */
#define TRAJ_PARAMS_TILTMAX_LND 15.0f

/**
 * Landing descend rate
 *
 * @unit m/s
 * @min 0.0
 */
#define TRAJ_PARAMS_LAND_SPEED 1.0f
