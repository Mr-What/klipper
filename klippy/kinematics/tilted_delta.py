# Code for handling the kinematics of linear delta robots, which may have tilted towers
#
# Copyright (C) 2026-2031  Aaron Birenboim <aaron@boim.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import stepper, mathutil
# remove the following imports eventually:
import logging
import numpy as np

# Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
#SLOW_RATIO = 3.

class TiltedDeltaKinematics:
    def __init__(self, toolhead, config):
        self.config = config;
        self.printer = config.get_printer()
        self.logger = logging # self.printer.get_logger()
        
        # Setup tower rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abc']
        rail_a = stepper.LookupMultiRail(
            stepper_configs[0], need_position_minmax = False)
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.LookupMultiRail(
            stepper_configs[1], need_position_minmax = False,
            default_position_endstop=a_endstop)
        rail_c = stepper.LookupMultiRail(
            stepper_configs[2], need_position_minmax = False,
            default_position_endstop=a_endstop)
        self.rails = [rail_a, rail_b, rail_c]
        self.position_endstops = [ a_endstop,
                         rail_b.get_homing_info().position_endstop,
                         rail_c.get_homing_info().position_endstop]

        # we may want to check rotation_distance for stretched belts:
        #self.rotation_distances = [s['rotation_distance'] for s in stepper_configs]
        self.rotation_distances = [config.getfloat('stepper_' + a,
                                                   'rotation_distance', fallback=40.0)
                                   for a in "abc"]

        # retrieve settings from [printer] section
        #cfile = self.printer.lookup_object('configfile')
        #printer_cfg = cfile.getsection('printer')
        printer_cfg = config.getsection('printer')
        
        # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                          above=0., maxval=self.max_accel)

        print_radius = printer_cfg.getfloat('print_radius', 110., above=0.)

        # === parameters in for tilted_delta which are similar to classic linear delta
        #     many of which are similar to (linear) delta.
        #     Many are the same as classical, we have different parameters
        #     for each tower, instead of asserting an ideal uniform build
        self.delta_angles = self.getvector('delta_angles', (210., 330., 90.))

        # radii at BASE of towers A, B, C
        self.delta_radius = self.getvector('delta_radius', [180., 180., 180.])

        # length of arm for towers A, B, C
        self.arm_lengths  = self.getvector('arm_lengths', [287., 287., 287])

        # endstops handled in [stepper_?] section.
        
        #arm_length_a = stepper_configs[0].getfloat('arm_length', above=radius)
        #self.abs_endstops = [(rail.get_homing_info().position_endstop
        #                      + math.sqrt(arm2 - radius**2))
        #                     for rail, arm2 in zip(self.rails, self.arm2)]
        # Determine tower locations in cartesian space

        ## position in mm from base when carriage hits endstop.
        ##    usually a bit shorter than measured because of effector offset
        # I think this should be in [stepper_?] section
        #self.endstop_distances = config.getfloats('endstop_distances', [500., 500., 500.], count=3)
        
        # --- Parameters describing the tilt of towers in tilted_delta
        # angle, in degrees, of tower tilt toward center
        #       0's for linear delta, 13 for prototype tetrahedral delta
        self.tilt_radial = self.getvector('tilt_radial', [0.,0.,0.])
        
        # angle, in degrees, of tower to the left of center
        #       when looking from tower towards center, any non-0 is an error
        self.tilt_tangential = self.getvector('tilt_tangential', [0.,0.,0.])
        
        # kinematics assert base is z=0 plane, add this AFTER
        # cartesian coords to adjust for thinner print bed, or plate on top of bed.
        #self.z_offset = config.getfloat('z_offset',0.)

        #for r, a, t in zip(self.rails, self.arm2, self.towers):
        #    r.setup_itersolve('delta_stepper_alloc', a, t[0], t[1])
        
        #for s in self.get_steppers():
        #    s.set_trapq(toolhead.get_trapq())
        # Setup boundary checks
        self.need_home = True
        #self.limit_xy2 = -1.
        endstop_pos = [rail.get_homing_info().position_endstop
                       for rail in self.rails]
        self.max_z = min(endstop_pos)

        #self.min_z = config.getfloat('minimum_z_position', 0, maxval=self.max_z)
        #self.limit_z = min([ep - arm
        #                    for ep, arm in zip(self.abs_endstops, arm_lengths)])
        #self.set_position([0., 0., 0.], "")

        # --- Derived/calculated parameters (computed on init)
        self._compute_derived_parameters()

        # more parameters, dependant on derived parameters
        apos = (self.max_z,) * 3
        self.home_position = self._actuator_to_cartesian(apos)

        #for r, a, t in zip(self.rails, self.arm2, self.towers):
        #    r.setup_itersolve('delta_stepper_alloc', a, t[0], t[1])

        # set up chelper inverse kinematics, and provide necessary parameters
        for i in range(3) :
            arm = self.arm_lengths[i]
            bx = self.base[i,0]
            by = self.base[i,1]
            tx = self.tilt[i,0]
            ty = self.tilt[i,1]
            self.rails[i].setup_itersolve('tilted_delta_stepper_alloc',
                                          arm, bx, by, tx, ty)

    # --------------------------------------------- derived parameters
    def _compute_derived_parameters(self):
        """Compute derived geometry from base parameters"""
        #deg2rad = 3.1415926535898 / 180.0

        self.base = np.zeros((3,3))  # rows are locations of tower bases
        self.tilt = np.zeros((3,3))  # rows are unit direction vector of tower, pointing up from base
        v = np.array([0.,0.,0.])
        for i in range(3):
            v = np.array([ math.cos(np.radians(self.delta_angles[i])),
                           math.sin(np.radians(self.delta_angles[i])), 0.])
            self.base[i,:] = self.delta_radius[i] * v

            # tilt is a set of unit vectors describing the direction
            # of each tower.  Ideal linear delta has tilts of [0,0,1]
            # for tetrahedral, these tilts will tip significantly towards the origin
            z = self.tilt_radial[i]
            t = self.tilt_tangential[i]
            rHat = -v
            tHat = np.array([-rHat[1], rHat[0], 0.])
            sz = math.sin(np.radians(z))
            st = math.sin(np.radians(t))
            self.tilt[i,:] = sz * rHat + st * tHat;
            self.tilt[i,2] = math.sqrt(1. - sz*sz - st*st)

        self.logger.info("# [tilted_delta] kinematic parameters :")
        self.logger.info("arm_lengths:  %.3f, %.3f, %.3f",
                         self.arm_lengths[0], 
                         self.arm_lengths[1], 
                         self.arm_lengths[2])
        self.logger.info("delta_radius: %.3f, %.3f, %.3f",
                         self.delta_radius[0],
                         self.delta_radius[1],
                         self.delta_radius[2])
        self.logger.info("delta_angles: %.3f, %.3f, %.3f",
                         self.delta_angles[0],
                         self.delta_angles[1],
                         self.delta_angles[2])
        self.logger.info("tilt_radial:     %.6f, %.6f, %.6f",
                         self.tilt_radial[0],
                         self.tilt_radial[1],
                         self.tilt_radial[2])
        self.logger.info("tilt_tangential: %.6f, %.6f, %.6f",
                         self.tilt_tangential[0],
                         self.tilt_tangential[1],
                         self.tilt_tangential[2])
        self.logger.info("base = [[%8.3f, %8.3f, %8.3f],\n\t[%8.3f, %8.3f, %8.3f],\n\t[%8.3f, %8.3f, %8.3f]]",
                         self.base[0,0], self.base[0,1], self.base[0,2],
                         self.base[1,0], self.base[1,1], self.base[1,2],
                         self.base[2,0], self.base[2,1], self.base[2,2])
        self.logger.info("tilt = [[%9.6f, %9.6f, %9.6f],\n\t[%9.6f, %9.6f, %9.6f],\n\t[%9.6f, %9.6f, %9.6f]]",
                         self.tilt[0,0], self.tilt[0,1], self.tilt[0,2],
                         self.tilt[1,0], self.tilt[1,1], self.tilt[1,2],
                         self.tilt[2,0], self.tilt[2,1], self.tilt[2,2])

    def getvector(self, name, default_value):
        vlen = len(default_value);
        v = self.config.getfloatlist(name, default_value, count=vlen)
        if len(v) != vlen:
            self.logger.warning("Parameter %s is not the expected %d element vector.",
                        name,vlen)
            if vlen == 3:
                self.logger.warning("\tUsing default : %g, %g, %g",
                                    default_value[0],
                                    default_value[1],
                                    default_value[2])
                
            return default_value
        return v

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def _actuator_to_cartesian(self, spos):
        #sphere_coords = [(t[0], t[1], sp) for t, sp in zip(self.towers, spos)]
        #return mathutil.trilateration(sphere_coords, self.arm2)
        cp = self.base;       # carriage positions in cartesian
        for i in range(3):
            cp[i,:] += self.tilt[i,:] * spos[i];
        #cp = self.base + (self.tilt .* (spos' * ones(1,3)))
        vAB = cp[1,:] - cp[0,:]
        vAC = cp[2,:] - cp[0,:]
        baseLen = [np.linalg.norm(cp[1,:] - cp[2,:]),
                   np.linalg.norm(vAC),
                   np.linalg.norm(vAB)]

        # find the position of the apex of a tetrahedron, with
        # given base lengths, and arm_lengths which go from
        # the base to the apex
        apex = self.getTetraCoords(baseLen,self.arm_lengths)

        # convert from effector coords back to tower
        xHat = vAB / baseLen[2]
        xA = np.dot(xHat,vAC)
        origin = cp[0,:] + xA*xHat
        yC = cp[2,:]-origin
        yHat = yC/np.linalg.norm(yC)
        zHat = -np.cross(xHat,yHat)
        q = origin + apex[0]*xHat + apex[1]*yHat + apex[2]*zHat
        return q

    @staticmethod
    def getTetraCoords(baseLen, twrLen):
        aa, bb, cc = baseLen
        a2 = aa*aa
        b2 = bb*bb
        c2 = cc*cc
        rA, rB, rC = twrLen
        rA2 = rA*rA
        rB2 = rB*rB
        rC2 = rC*rC
        xB = (cc + (a2-b2)/cc)/2
        xB2 = xB*xB
        yC2 = a2 - xB*xB
        yC = math.sqrt(yC2)
        x1 = (rA2-rB2)/(2*cc) + xB - (cc/2)
        y1 = (rB2-rC2+yC2+xB*(2*x1-xB))/(2*yC)
        z1 = math.sqrt(rC2 - x1*x1 - ((y1-yC)**2))
        apex = [x1,y1,z1]
        return apex
        # check.. all err should be zero
        #A0=[xB-cc,0,0];
        #B0=[xB   ,0,0];
        #C0=[0,yC,0];
        #err = [norm(A0-B0)-cc,norm(A0-C0)-bb,norm(B0-C0)-aa,...
        #       norm(apex-A0)-twrLen(1),...
        #       norm(apex-B0)-twrLen(2),...
        #       norm(apex-C0)-twrLen(3)];
        #if sum(abs(err)) >0
        #disp(err);
        #end
        
    def calc_position(self, stepper_positions):
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self._actuator_to_cartesian(spos)
    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        #self.limit_xy2 = -1.
        if homing_axes == "xyz":
            self.need_home = False
    def clear_homing_state(self, clear_axes):
        # Clearing homing state for each axis individually is not implemented
        if clear_axes:
            #self.limit_xy2 = -1
            self.need_home = True
    def home(self, homing_state):
        # All axes are homed simultaneously
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        #forcepos[2] = -1.5 * math.sqrt(max(self.arm2)-self.max_xy2)
        homing_state.home_rails(self.rails, forcepos, self.home_position)
    def check_move(self, move):
        end_pos = move.end_pos
        #end_xy2 = end_pos[0]**2 + end_pos[1]**2
        #if end_xy2 <= self.limit_xy2 and not move.axes_d[2]:
        #    # Normal XY move
        #    return
        if self.need_home:
            raise move.move_error("Must home first")
        #end_z = end_pos[2]
        #limit_xy2 = self.max_xy2
        #if end_z > self.limit_z:
        #    above_z_limit = end_z - self.limit_z
        #    allowed_radius = self.radius - math.sqrt(
        #        self.min_arm2 - (self.min_arm_length - above_z_limit)**2
        #    )
        #    limit_xy2 = min(limit_xy2, allowed_radius**2)
        if end_xy2 > limit_xy2 or end_z > self.max_z or end_z < self.min_z:
            # Move out of range - verify not a homing move
            if (end_pos[:2] != self.home_position[:2]
                or end_z < self.min_z or end_z > self.home_position[2]):
                raise move.move_error()
            limit_xy2 = -1.
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
            limit_xy2 = -1.
        # Limit the speed/accel of this move if is is at the extreme
        # end of the build envelope
        extreme_xy2 = max(end_xy2, move.start_pos[0]**2 + move.start_pos[1]**2)
        if extreme_xy2 > self.slow_xy2:
            r = 0.5
            if extreme_xy2 > self.very_slow_xy2:
                r = 0.25
            move.limit_speed(self.max_velocity * r, self.max_accel * r)
            limit_xy2 = -1.
        self.limit_xy2 = min(limit_xy2, self.slow_xy2)
    def get_status(self, eventtime):
        return {
            'homed_axes': '' if self.need_home else 'xyz'
            #'axis_minimum': self.axes_min,
            #'axis_maximum': self.axes_max,
            #'cone_start_z': self.limit_z,
        }

def load_kinematics(toolhead, config):
    return TiltedDeltaKinematics(toolhead, config)
