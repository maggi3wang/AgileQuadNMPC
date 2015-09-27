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
//~ #define TRAJ_GAINS_XY_POS 2.0f
#define TRAJ_GAINS_XY_POS 0.0f

/**
 * Proportional gain for vertical position error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_Z_POS 1.0f
#define TRAJ_GAINS_Z_POS 0.0f

/**
 * Proportional gain for horizontal velocity error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_XY_VEL 1.0f
#define TRAJ_GAINS_XY_VEL 0.0f

/**
 * Proportional gain for vertical velocity error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_Z_VEL 1.0f
#define TRAJ_GAINS_Z_VEL 0.0f

/**
 * Proportional gain for roll, pitch angular error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_RP_ANG 0.0f

/**
 * Proportional gain for roll, pitch angular velocity error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_RP_OMG 0.5f

/**
 * Proportional gain for yaw angular error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_YAW_ANG 1.0f
#define TRAJ_GAINS_YAW_ANG 0.0f


/**
 * Proportional gain for yaw angular velocity error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_YAW_OMG 0.1f

/**
 * Minimum thrust
 *
 * Minimum vertical acceleration due to thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
 * 
 * @unit m/s^2
 * @min 0.0
 */
#define TRAJ_PARAMS_VERT_ACC_MIN 1.0f

/**
 * Maximum thrust
 *
 * Maximum vertical acceleration due to thrust. I
 *
 * @unit m/s^2
 */
#define TRAJ_PARAMS_VERT_ACC_MAX 30.0f

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
 * Threshold for gimbal lock safety check
 *
 * @unit unitless
 * @min 0.0
 */
#define TRAJ_PARAMS_GIMBAL_LOCK 0.05f

/**
 * Threshold for free fall
 * 
 * Max downward acceleration becomes: 9.81 - FREEFALL_THRESHOLD
 *
 * @unit m/s^2
 * @min 0.0
 */
#define TRAJ_PARAMS_FREEFALL_THRESHOLD 1.0f

/**
 * Conversion from thrust force to throttle
 *
 * @unit 1/N
 * @min 0.0
 */
//~ #define TRAJ_PARAMS_THROTTLE_PER_THRUST 0.0599f
#define TRAJ_PARAMS_THROTTLE_PER_THRUST 0.06f

/**
 * Maximum Throttle Value
 *
 * @unit 1/N
 * @min 0.0
 */
#define TRAJ_PARAMS_THROTTLE_MAX 1.0f
