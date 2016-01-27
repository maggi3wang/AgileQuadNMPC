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
 * Integral gain for roll, pitch angular error
 * 
 * @min 0.0
 */
#define TRAJ_GAINS_RP_ANG_INT 0.0f
 
 /**
 * Proportional gain for roll, pitch angular error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_RP_ANG 0.4f
//~ #define TRAJ_GAINS_RP_ANG 0.075f
#define TRAJ_GAINS_RP_ANG 0.4f

/**
 * Proportional gain for roll, pitch angular velocity error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_RP_OMG 0.1f
#define TRAJ_GAINS_RP_OMG 0.08f

/**
 * Derivative gain for roll, pitch angular velocity error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_RP_OMG_DER 0.001f

/**
 * Integral gain for yaw angular error
 * 
 * @min 0.0
 */
/* NOTE: observed bad behavior even at very low gains. Should be zero or removed all together */
#define TRAJ_GAINS_YAW_ANG_INT 0.0f

/**
 * Proportional gain for yaw angular error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_YAW_ANG 1.5f
#define TRAJ_GAINS_YAW_ANG 0.5f


/**
 * Proportional gain for yaw angular velocity error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_YAW_OMG 0.5f
#define TRAJ_GAINS_YAW_OMG 0.25f

/**
 * Derivative gain for yaw angular velocity error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_YAW_OMG_DER 0.0f

/**
 * Integral gain for horizontal position error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_XY_POS_INT 0.5f
#define TRAJ_GAINS_XY_POS_INT 0.0f
 
/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_XY_POS 2.0f
#define TRAJ_GAINS_XY_POS 2.0f

/**
 * Proportional gain for horizontal velocity error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_XY_VEL 4.0f
#define TRAJ_GAINS_XY_VEL 4.0f

/**
 * Integral gain for vertical position error
 *
 * @min 0.0
 */
#define TRAJ_GAINS_Z_POS_INT 0.1f

/**
 * Proportional gain for vertical position error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_Z_POS 4.0f
#define TRAJ_GAINS_Z_POS 4.0f

/**
 * Proportional gain for vertical velocity error
 *
 * @min 0.0
 */
//~ #define TRAJ_GAINS_Z_VEL 3.0f
#define TRAJ_GAINS_Z_VEL 3.0f

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
 * Saturation limit for xy position error integrator
 * 
 * @unit m*sec (?)
 */
 #define TRAJ_PARAMS_XY_INT_LIMIT 10.0f
 
 /**
 * Saturation limit for xy position error integrator
 * 
 * @unit m*sec (?)
 */
 #define TRAJ_PARAMS_Z_INT_LIMIT 5.0f

/**
 * Saturation limit for roll, pitch angular error integrator
 * 
 * @unit rad*sec (?)
 */
 #define TRAJ_PARAMS_RP_INT_LIMIT 0.3f

/**
 * Saturation limit for yaw angular error integrator
 * 
 * @unit rad*sec (?)
 */
 #define TRAJ_PARAMS_YAW_INT_LIMIT 0.3f
 
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
#define TRAJ_PARAMS_THROTTLE_PER_THRUST 0.059f

/**
 * Maximum Throttle Value
 *
 * @unit unitless
 * @min 0.0
 */
#define TRAJ_PARAMS_THROTTLE_MAX 1.0f

/**
 * Maximum Throttle Value
 *
 * @unit unitless
 * @min 0.0
 */
#define TRAJ_PARAMS_THROTTLE_MIN 0.01f

/**
 * Smoothing factor for low pass filter for throttle mapping
 * 
 * @unit unitless
 * @min 0.0
 * @max 1.0
 */
#define THROTTLE_FILTER_SMOOTHING 0.01f
