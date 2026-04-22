# Code for handling the kinematics of linear delta robots, which may have tilted towers
#
# Copyright (C) 2026-2031  Aaron Birenboim <aaron@boim.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import stepper, mathutil
# remove the following imports eventually:
import logging
import time
from datetime import datetime
import numpy as np

# Slow moves once the ratio of tower to XY movement exceeds SLOW_RATIO
#SLOW_RATIO = 3.

class TiltedDeltaKinematics:
    def __init__(self, toolhead, config):
        self.config = config;
        self.printer = config.get_printer()
        #self.logger = self.printer.get_logger()  # logging for system logger
        self.logger = logging.getLogger(__name__) # should get directed to klippy.log

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
        self.rotation_distances = [
            config.getsection('stepper_' + a).getfloat('rotation_distance', 40.0)
            for a in 'abc']


        # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                          above=0., maxval=self.max_accel)

        # retrieve settings from [printer] section
        printer_cfg = config.getsection('printer')

        print_radius = printer_cfg.getfloat('print_radius', 110., above=0.)

        # === parameters in for tilted_delta which are similar to classic linear delta
        #     many of which are similar to (linear) delta.
        #     Many are the same as classical, we have different parameters
        #     for each tower, instead of asserting an ideal uniform build
        self.delta_angles = self.getvector('delta_angles', (210., 330., 90.))

        # radii at BASE of towers A, B, C
        self.delta_radius = self.getvector('delta_radius', (180., 180., 180.))

        # length of arm for towers A, B, C
        self.arm_lengths  = self.getvector('arm_lengths', (287., 287., 287))

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
        
        # Setup boundary checks
        self.need_home = True
        #self.limit_xy2 = -1.

        # --- Derived/calculated parameters (computed on init)
        self._compute_derived_parameters()

        # --- more parameters, dependant on derived parameters

        endstop_pos = [rail.get_homing_info().position_endstop
                       for rail in self.rails]
        endpos = min(endstop_pos)
        self.min_stepper = config.getfloat('minimum_stepper_position', 0, maxval=endpos-100)
        
        apos = (endpos-3,) * 3  # highest safe actuator position
        xyzHi = self._actuator_to_cartesian(apos)

        # go to here after homing?
        self.home_position = tuple(0., 0., round(xyzHi[2]))

        # update apos to desired home position instead of highest safe stepper position
        apos = self._cart2twr(self.home_position) 
        self._log("actuator position at home [%.3f,%.3f,%.3f]",
                  apos[0],apos[1],apos[2])
        self._log("home_position=[%.3f,%.3f,%.3f]",
                  self.home_position[0],
                  self.home_position[1],
                  self.home_position[2])

        self.init_time = time.monotonic() # need this set a little early for diagnostic init logs
        
        # set up chelper inverse kinematics, and provide necessary parameters
        for i in range(3) :
            arm = self.arm_lengths[i]
            bx = self.base[i,0]
            by = self.base[i,1]
            tx = self.tilt[i,0]
            ty = self.tilt[i,1]
            self._log("rail(%d) %.3fmm arm base=[%.3f,%.3f] tilt=[%.3f,%.3f]",
                      i,arm,bx,by,tx,ty)
            self.rails[i].setup_itersolve('tilted_delta_stepper_alloc',
                                          arm, bx, by, tx, ty)

        tq = toolhead.get_trapq()
        for s in self.get_steppers():
            s.set_trapq(tq)

        # not working for unknown reason.
        # complains about motor_off signature, but it is as documented.
        #config.get_printer().register_event_handler(
        #    "stepper_enable:motor_off", self.motor_off)

        #self.max_z = min([rail.get_homing_info().position_endstop
        #                  for rail in self.rails])
        #self.min_z = config.getfloat('minimum_stepper_position', 0,
        #                             maxval=self.max_z)
        
        #self.limit_z = min([ep - arm for ep, arm in zip(self.abs_endstops, arm_lengths)])
        #def ratio_to_xy(ratio):
        #    return (ratio * math.sqrt(min_arm_length**2 / (ratio**2 + 1.)
        #                              - half_min_step_dist**2)
        #            + half_min_step_dist - radius)
        #self.slow_xy2 = ratio_to_xy(SLOW_RATIO)**2
        #self.very_slow_xy2 = ratio_to_xy(2. * SLOW_RATIO)**2
        #self.max_xy2 = min(print_radius, min_arm_length - radius,
        #                   ratio_to_xy(4. * SLOW_RATIO))**2
        #self.axes_min = toolhead.Coord((-max_xy, -max_xy, self.min_z))
        #self.axes_max = toolhead.Coord((max_xy, max_xy, self.max_z))
        self.set_position([0., 0., 0.], "")

        self._log("init complete at %s", self._timestamp())
        
    @staticmethod
    def _timestamp():
        return datetime.now().strftime("%m/%d/%Y %H:%M:%S")

    # use this for informational logs whenever possible:
    def _log(self, fmt, *args):
        et = time.monotonic() - self.init_time
        self.logger.info("tilted_delta %.3f : " + fmt, et, *args)
        
    # --------------------------------------------- derived parameters
    def _compute_derived_parameters(self):
        self.base = np.zeros((3,3), dtype=float)  # rows are locations of tower bases
        self.tilt = np.zeros((3,3), dtype=float)  # rows are unit direction vector of tower, pointing up from base
        #v = np.array([0,0,0])
        for i in range(3):
            v = np.array([ math.cos(np.radians(self.delta_angles[i])),
                           math.sin(np.radians(self.delta_angles[i])), 0.], dtype=float)
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
        self.logger.info("tilt = [[%9.6f, %9.6f, %9.6f],\n\t[%9.6f, %9.6f, %9.6f],\n\t[%9.6f, %9.6f, %9.6f]]",
                         self.tilt[0,0], self.tilt[0,1], self.tilt[0,2],
                         self.tilt[1,0], self.tilt[1,1], self.tilt[1,2],
                         self.tilt[2,0], self.tilt[2,1], self.tilt[2,2])
        self._logBase()

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
        cp = np.array(self.base, copy=True)       # carriage positions in cartesian
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
        apex = self._getTetraCoords(baseLen,self.arm_lengths)

        # convert from effector coords back to tower
        xHat = vAB / baseLen[2]
        xA = np.dot(xHat,vAC)
        origin = cp[0,:] + xA*xHat
        yC = cp[2,:]-origin
        yHat = yC/np.linalg.norm(yC)
        zHat = -np.cross(xHat,yHat)
        q = origin + apex[0]*xHat + apex[1]*yHat + apex[2]*zHat
        self._log("\tabc=[%.3f,%.3f,%.3f]\n\t\t\txyz=[%.3f,%.3f,%.3f]",
                         spos[0],spos[1],spos[2],
                         q[0],q[1],q[2])
        return tuple(float(v) for v in q)

    @staticmethod
    def _getTetraCoords(baseLen, twrLen):
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

    # wrapper to _actuator_to_cartesian(), but gets stepper position by rail name
    def calc_position(self, stepper_positions):
        self._log("calc_position()")
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        xyz = self._actuator_to_cartesian(spos)
        #xyz[2] = -xyz[2]  # try flipping Z to match command.  had -Z problem, did nothing?!??
        return xyz

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        if homing_axes == "xyz":
            self.need_home = False
        self._log("set_position([%.3f,%.3f,%.3f])",
                  newpos[0], newpos[1], newpos[2])

    def clear_homing_state(self, clear_axes):
        # Clearing homing state for each axis individually is not implemented
        if clear_axes:
            #self.limit_xy2 = -1
            self.need_home = True

    def motor_off(self, eventTime):
        self.need_home = True
        
    def home(self, homing_state):
        self._log("home([%.3f,%.3f,%.3f])",
                  self.home_position[0],
                  self.home_position[1],
                  self.home_position[2])
        # All axes are homed simultaneously
        homing_state.set_axes([0, 1, 2])
        
        # assume we are start homing from here (cartesian)
        forcepos = [0.,0.,self.home_position[2]/5]

        # move here after all homing endstops were triggered
        homepos = [0.,0.,self.home_position[2]]
        
        self._log("homing rails from [%.0f,%.0f,%.0f] to [%.0f,%.0f,%.0f]",
                  forcepos[0],forcepos[1],forcepos[2],
                  homepos[0],  homepos[1], homepos[2])
        homing_state.home_rails(self.rails, forcepos, homepos)
        self._log("home() complete.")
        
    def check_move(self, move):
        end_pos = [move.end_pos[0], move.end_pos[1], move.end_pos[2]]  # last element is extruder position, E, and we don't care
        self._log("check_move() end_pos=[%.3f,%.3f,%.3f]%d axes_d=%s need_home=%s",
                  end_pos[0],end_pos[1],end_pos[2],len(end_pos), 
                  getattr(move,"axes_d", None),
                  self.need_home)

        return # debug.  no checking.
    
        if self.need_home:
            self._log("rejecting move, need home first");
            #raise move.move_error("Must home first")
            return  # debugging ONLY!  move anyway to test steppers

        if not self._check_envelope(end_pos):
            raise move.move_error("outside print envelope")

    def _logBase(self):
        self.logger.info("   base=[[%8.3f,%8.3f,%8.3f],",
                         self.base[0,0],
                         self.base[0,1],
                         self.base[0,2])
        self.logger.info("         [%8.3f,%8.3f,%8.3f],",
                         self.base[1,0],
                         self.base[1,1],
                         self.base[1,2])
        self.logger.info("         [%8.3f,%8.3f,%8.3f]]",
                         self.base[2,0],
                         self.base[2,1],
                         self.base[2,2])
        
    def _check_envelope(self, pos):
        self._log("_check_envelope(%.3f,%.3f,%.3f)",pos[0],pos[1],pos[2])
        twr = self._cart2twr(pos)
        if twr is None:  # error code.  out of envelope
            self._log("ERROR : not reachable")
            return False
        self._log("\ttwr=[%.3f,%.3f,%.3f]",twr[0], twr[1], twr[2])

        for i in range(3):
            z = twr[i] * self.tilt[i,2] # z(mm) for arm at twr[i] from base
            z -= pos[2] # how far from effector_z to top of arm
            z = z / self.arm_lengths[i] # sin of arm angle
            if (z < .1) or (z > .99):  # arm angle too extreme
                self._log("ERROR : arm angle %.1f too extreme",
                          math.degrees(math.asin(z)))
                return False   
        return True

    # this should do the same thing as chelper tilted_delta_calc_position()
    # but it will be a challenge to call that from here, because of
    # the move stuff.  tight coupling.
    # for now, I'm just duplicating the math in tilted_delta_calc_position().
    # perhaps we could have tilted_delta_calc_position call a sub-function
    # passing xyz (instead of move time), and we could call that from python.
    def _cart2twr(self, xyz):
        self._log("_cart2twr([%.3f,%.3f,%.3f])",
                  xyz[0],xyz[1],xyz[2])
        twr = [0.,0.,0.]
        for i in range(3):
            d = self._towerDistance(self.base[i,:], self.tilt[i,:],
                               self.arm_lengths[i],xyz)
            self._log("\ttower %d distance %.3f",i,d)
            if d == 0:
                return None
            
            twr[i] = d

        return twr
        
    @staticmethod
    def _towerDistance(p0, vHat, r, q):
        # for quadratic, a*d^2 + b*d + c = 0, a==1,
        q = np.asarray(q, dtype=float)
        b = 2 * np.dot(vHat,p0-q)
        dq = q - p0
        c = np.dot(dq,dq) - r*r
        disc = b*b - 4*c
        if (disc < 0):
            return 0
        d = (-b + math.sqrt(disc))/2
        return d

    def get_status(self, eventtime):
        return {
            'homed_axes': '' if self.need_home else 'xyz'
        }

def load_kinematics(toolhead, config):
    return TiltedDeltaKinematics(toolhead, config)
