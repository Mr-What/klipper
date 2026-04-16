# Tilted Delta branch of klipper

This file, README_tilted_delta.md, is unique to an experimental
tilted_delta kinematics for the [klipper project](https://github.com/Klipper3d/klipper).

The intent of this klipper fork is to maintain the smallest
change possible to klipper master, to support use of the
`tilted_delta` kinematic model.

If you are seeing this, you have most likely checked out the
[`tilted-delta-kinematics-dev` branch](https://github.com/Mr-What/klipper/tree/tilted-delta-kinematics-dev) to the [Mr-What fork](https://github.com/Mr-What/klipper/tree/master) of the
[klipper master](https://github.com/Klipper3d/klipper).

The master at [`Mr-What/klipper`](https://github.com/Mr-What/klipper)  is intended to stay up to date with stable versions of klipper, no changes.
The additional `tilted_delta` capabilities are maintained in the
[`tilted-delta-kinematics-dev` branch](https://github.com/Mr-What/klipper/tree/tilted-delta-kinematics-dev).

At this time, the only differences are as follows:

<DL><DT><code>klippy/chelper/__init__.py</code></DT>
<DD>Add entry point definitions for <code>kin_tilted_delta</code></DD>
<DT><code>klippy/chelper/kin_tilted_delta.c</code></DT>
<DD>fast inverse kinematics implementation</DD>
<DT><code>klippy/kinematics/tilted_delta.py</code></DT>
<DD>Core tilted_delta model implementation</DD>
</DL>

Calibration procedures for the tilted_delta model are provided
outside of kilpper.  See https://github.com/Mr-What/Tetra3D
for details.

The initial focus of the new tilted_delta kinematic model
was to support tetrahedral shaped delta printers.
However, if you assert that the towers are all perpendicular to the
bed, tilt angles all zero, you have a traditional delta model.
That is why the term 'tetra' is frequently used for the project.
The name `tilted_delta` was created for klipper, since this name
seems to be more in line with coding and naming conventions used in klipper.

The tilted delta supports many more parameters than a traditional delta.
Most users will not use the entire parameter space.  For example,
the tilted delta model defines a different length for each effector arm.
It is expected that most users will set `arm_lengths[3]` to all the same value.
Only if you expect that your kinematics have an artifact that might be
explained by arms of different lengths would you set these values to different
numbers.

An example of the unique parameters of the tilted_delta are:
```
   [printer]
      kinematics: tilted_delta
      delta_angles: 210., 330., 90.   # angle to tower A,B,C base
      delta_radius: 160., 160., 160.  # radii at BASE of towers A, B, C (mm)
      arm_lengths: 287., 287., 287.   # length of arm for towers A, B, C (mm)
      tilt_radial: 13.,13.,13.  # tower tilt towards center (deg)
      tilt_tangential: 0.,0.,0. # tower tilt to side (deg)
```
A traditional linear delta will set all of the `tilt_` parameters to zero.

Since arms are usually built with tight machine tolerances, most users
will use the same `arm_lengths` value for each tower.

The `delta_angles` and `delta_radius` can be set for each individual tower.
This is intended to be used for calibration, when the implemented
towers are not exactly at the intended position.

Calibration precedures in the [Tetra3D project](https://github.com/Mr-What/Tetra3D) can be used to estimate the values of all of these parameters, plus
the stepper `position_endstop` and `rotation_distance`.
Setting `rotation_distance` different from the designed value would
be very rare, and usually to compensate for belt stretch.
Calibration procedures, using bed probes, and measurements of calibration prints are relatively easy to create.

Convergence on a very large parameter set to be calibrated is not assured,
so in practical use, you would calibrate a few parameters at a time.
Update the values in the klipper `printer.cfg` if the calibrated
values improve the print accuracy.

For a final check, you could estimate a larger number of parameters, but define a smaller search space.

