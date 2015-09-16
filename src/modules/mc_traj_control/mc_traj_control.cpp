/****************************************************************************
 *
 *   Copyright (C) 2015 Ross Allen. All rights reserved.
 *   Author: @author Ross Allen <ross.allen@stanford.edu>
 *
 *
 ****************************************************************************/

/**
 * @file mc_traj_control_main.c
 * Feedforward/feedback controller for trajectory tracking of a multicopter
 */
 
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
 
extern "C" __EXPORT int mc_traj_control_main(int argc, char *argv[]);
 
int mc_traj_control_main(int argc, char *argv[])
{
	printf("Hello Sky!\n");
	return OK;
}
