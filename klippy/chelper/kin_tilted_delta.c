// Tilted Delta kinematics stepper pulse time generation
//
// Copyright (C) 2026-2037  Kevin?  Aaron Birenboim? <aaron@boim.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

//struct delta_stepper {
//    struct stepper_kinematics sk;
//    double arm2, tower_x, tower_y; };

struct tilted_delta_stepper {
    struct stepper_kinematics sk;
    double arm2;
    double base_x, base_y;  // tower location at z=0  
  double dir_x, dir_y, dir_z;    // direction vector for each tower, [0,0,1] for ideal classic delta
};

static double
tilted_delta_stepper_calc_position(struct stepper_kinematics *sk,
				   struct move *m
				   , double move_time)
{
    static double prev_t=-9e9;    // move_time at previouis call
    static struct coord xyz;       // coord at prev_t
    struct tilted_delta_stepper *ds = container_of(sk, struct tilted_delta_stepper, sk);

    if (move_time != prev_t)
      {
        xyz = move_get_coord(m, move_time);
	prev_t = move_time;
      }

    // for quadratic, a*d^2 + b*d + c = 0, a==1==|dir|
    double qx = xyz.x - ds->base_x;
    double qy = xyz.y - ds->base_y;
    double b = 2 * (ds->dir_z * xyz.z -
		    ds->dir_x * qx -
		    ds->dir_y * qy );
    double c = qx*qx + qy*qy + xyz.z * xyz.z - ds->arm2;
    double disc = b*b - 4.0 * c;
    if (disc < 0) return(-99.9);
    // I believe that for valid pose, answer is never b-sqrt(disc)
    return( (sqrt(disc)-b)*0.5 );
}

struct stepper_kinematics * __visible
tilted_delta_stepper_alloc(double arm, double x0, double y0,
		    double tilt_x, double tilt_y)
{
    struct tilted_delta_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));
    ds->sk.calc_position_cb = tilted_delta_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z;
    ds->arm2 = arm * arm;
    ds->base_x = x0;
    ds->base_y = y0;
    ds->dir_x = tilt_x;
    ds->dir_y = tilt_y;
    double z2 = 1.0 - tilt_x*tilt_x - tilt_y*tilt_y;
    if (z2 > 0.0) ds->dir_z = sqrt(z2); // else throw?
    return &ds->sk;
}
