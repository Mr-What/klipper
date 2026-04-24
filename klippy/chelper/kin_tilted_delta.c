// Tilted Delta kinematics stepper pulse time generation
//
// Copyright (C) 2026-2037  Kevin?  Aaron Birenboim? <aaron@boim.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h>   // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset, strcmp
#include <stdio.h>  // fprintf
#include "compiler.h"  // __visible
#include "itersolve.h" // struct stepper_kinematics
#ifndef __TEST__
#include "trapq.h"     // move_get_coord
#endif

struct tilted_delta_stepper {
    struct stepper_kinematics sk;
    double arm2;
    double base_x, base_y;  // tower location at z=0  
    double dir_x, dir_y, dir_z;    // direction vector for each tower, [0,0,1] for ideal classic delta
};

static void updateLog(double bx, double c, struct coord *pos)
{
  static FILE *f = NULL; 
  static long callCount = 0;
  //static long updateCount = 0;
  //static double abc[3];
  //static char msg[2][128];
  //static int imsg = 0;
  int i;

  callCount++;

  // which tower is this?
  // hackey, but works for initial development
  i = (bx < -10.) ? 0 : ((bx > 10.)? 1 : 2);
  //abc[i] = c;

  if (f == NULL) f = fopen("/tmp/td.log","w");
  if (f == NULL) f = stderr;

  //sprintf(msg[imsg],"cart=[%.3lf,%.3lf,%.3lf]; act=[%.3lf,%.3lf,%.3lf];",
	//	pos->x, pos->y, pos->z,
	//	abc[0], abc[1], abc[2]);
  /*
  sprintf(msg[imsg],"cart=[%.3lf,%.3lf,%.3lf]; act(%d)=%.3lf;",
		pos->x, pos->y, pos->z,	i, abc[i]);
  if (strcmp(msg[0],msg[1]) != 0) // not the same as previous report
    {
      fprintf(f,"%ld %s\n",callCount, msg[imsg]);
      imsg = imsg ? 0 : 1;  // toggle active buffer
      updateCount++;
      fflush(f);
    }
  */
  fprintf(f,"cart(k,:)=[%.3f,%.3f,%.6f];iact(k)=%d;act(k)=%.6f;k=k+1;\n",
        pos->x, pos->y, pos->z, i, c);
}

double __visible
tilted_delta_stepper_calc_position(struct stepper_kinematics *sk,
				   struct move *m,
				   double move_time)
{
    struct tilted_delta_stepper *ds = container_of(sk, struct tilted_delta_stepper, sk);
    struct coord xyz;      // coord at prev_t

    // do NOT try to cache.  caused errors.  I don't know why.
    // perhaps with iteration, xyz is changing for a given time?
    xyz = move_get_coord(m, move_time);

    // for quadratic, a*d^2 + b*d + c = 0, a==1==|dir|
    double qx = xyz.x - ds->base_x;
    double qy = xyz.y - ds->base_y;
    double b = 2 * (-ds->dir_z * xyz.z -
                     ds->dir_x * qx -
                     ds->dir_y * qy );
    double c = qx*qx + qy*qy + xyz.z * xyz.z - ds->arm2;
    double disc = b*b - 4.0 * c;
    if (disc < 0) {
      fprintf(stderr,"tilted_delta : [%.1lf,%.1lf,%.1lf] not reachable\n",
	      xyz.x,xyz.y,xyz.z);
      return(0.);
    }
    // I believe that for valid pose, answer is never b-sqrt(disc)
    c = 0.5 * (sqrt(disc)-b);

    //updateLog(ds->base_x, c, &xyz);
    return(c);
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
    if (z2 > 0.0) ds->dir_z = sqrt(z2);
    else
      {
        ds->dir_z = 1.;
	      fprintf(stderr,"FATAL: tilted_delta tower direction non-physical.\n\tTilt [%.3f, %.3f] should be two components of a UNIT vector!",ds->dir_x, ds->dir_y);
      }
    fprintf(stderr,"arm2=%.3lf; base=[%.3lf,%.3lf]; tilt=[%.5lf,%.5lf,%.5lf]; z2=%.5lf\n",
	    ds->arm2,
	    ds->base_x, ds->base_y,
	    ds->dir_x,  ds->dir_y, ds->dir_z, z2);
    return &ds->sk;
}
