
struct coord {
    union {
        struct {
            double x, y, z;
        };
        double axis[3];
    };
};

// spoof rest of klipper
//static struct coord move_get_coord(struct move *m, double move_time)
struct coord move_get_coord(void *m, double move_time)
{
  struct coord xyz;
  xyz.x=0;
  xyz.y=0;
  xyz.z = move_time;
  return xyz;
}

// I can't use the library due to tight coupled dependencies,
// so I'm just testing source 
// make kin_tilted_delta.c skip trapq.h which is spoofed here
#define __TEST__
#include "kin_tilted_delta.c"


int main (int argc, char *argv[])
{
  struct stepper_kinematics *a, *b, *c;

  a = tilted_delta_stepper_alloc(287., -138.564, -80.,  0.19481, 0.11248);
  b = tilted_delta_stepper_alloc(287.,  138.564, -80., -0.19481, 0.11248);
  c = tilted_delta_stepper_alloc(287.,    0.   , 160.,  0.     ,-0.22495);
  
  for (int k =1; k < 1000; k++)
    {
      double ta,tb,tc,t;
      struct coord xyz;

      t = k * 0.2;
      ta=tilted_delta_stepper_calc_position(a,NULL,t);
      tb=tilted_delta_stepper_calc_position(b,NULL,t);
      tc=tilted_delta_stepper_calc_position(c,NULL,t);
      xyz = move_get_coord(NULL, t);
      printf("xyz(%d,:)=[%.3f,%.3f,%.3f];  abc(%d,:)=[%.6f,%.6f,%.6f];\n",
	     k, xyz.x,xyz.y,xyz.z, k, ta,tb,tc);
    }
  return 0;
}
