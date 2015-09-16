/**
 * @file mc_pos_control_asl_params.h
 * Multicopter position controller parameters for ASL testing.
 * Modified mc_pos_control_params.c
 *
 * Autonomous Systems Laboratory
 * Stanford Aeronautics & Astronautics
 * Jan 27, 2015
 *
 * @author Ross Allen <rallen10@stanford.edu>
 */
 
 #define USE_ASL_PARAMS true
 
/**
 * Minimum thrust
 *
 * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @min 0.0
 * @max 1.0
 */
#define ASL_PARAMS_THR_MIN 0.1f

/**
 * Maximum thrust
 *
 * Limit max allowed thrust.
 *
 * @min 0.0
 * @max 1.0
 */
#define ASL_PARAMS_THR_MAX 1.0f

/**
 * Proportional gain for vertical position error
 *
 * @min 0.0
 */
#define ASL_PARAMS_Z_P 1.0f

/**
 * Proportional gain for vertical velocity error
 *
 * @min 0.0
 */
#define ASL_PARAMS_Z_VEL_P 0.1f

/**
 * Integral gain for vertical velocity error
 *
 * Non zero value allows hovering thrust estimation on stabilized or autonomous takeoff.
 *
 * @min 0.0
 */
#define ASL_PARAMS_Z_VEL_I 0.02f

/**
 * Differential gain for vertical velocity error
 *
 * @min 0.0
 */
#define ASL_PARAMS_Z_VEL_D 0.0f

/**
 * Maximum vertical velocity
 *
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
 *
 * @unit m/s
 * @min 0.0
 */
#define ASL_PARAMS_Z_VEL_MAX 5.0f

/**
 * Vertical velocity feed forward
 *
 * Feed forward weight for altitude control in stabilized modes (ALTCTRL, POSCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 */
#define ASL_PARAMS_Z_FF 0.5f

/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 */
#define ASL_PARAMS_XY_P 2.0f

/**
 * Proportional gain for horizontal velocity error
 *
 * @min 0.0
 */
#define ASL_PARAMS_XY_VEL_P 0.1f

/**
 * Integral gain for horizontal velocity error
 *
 * Non-zero value allows to resist wind.
 *
 * @min 0.0
 */
#define ASL_PARAMS_XY_VEL_I 0.02f

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 */
#define ASL_PARAMS_XY_VEL_D 0.01f

/**
 * Maximum horizontal velocity
 *
 * Maximum horizontal velocity in AUTO mode and endpoint for position stabilized mode (POSCTRL).
 *
 * @unit m/s
 * @min 0.0
 */
#define ASL_PARAMS_XY_VEL_MAX 5.0f

/**
 * Horizontal velocity feed forward
 *
 * Feed forward weight for position control in position control mode (POSCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 */
#define ASL_PARAMS_XY_FF 0.5f

/**
 * Maximum tilt angle in air
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 */
#define ASL_PARAMS_TILTMAX_AIR 45.0f

/**
 * Maximum tilt during landing
 *
 * Limits maximum tilt angle on landing.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 */
#define ASL_PARAMS_TILTMAX_LND 15.0f

/**
 * Landing descend rate
 *
 * @unit m/s
 * @min 0.0
 */
#define ASL_PARAMS_LAND_SPEED 1.0f
