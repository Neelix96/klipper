# Code for handling the kinematics of polar robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper


def distance_line_to_point(p1, p2, p=(0,0), margin=0) -> tuple:
    """
    Calculates the distance between a point and a line segment defined by two points.

    Args:
        p (list): The coordinates of the point [x, y].
        p1 (list): The coordinates of the first endpoint of the line segment [x1, y1].
        p2 (list): The coordinates of the second endpoint of the line segment [x2, y2].

    Returns:
        float: The distance between the point and the line segment.
    """
    x, y = p
    x1, y1 = p1
    x2, y2 = p2

    # Calculate the vectors AB and AP
    ab_x = x2 - x1
    ab_y = y2 - y1
    ap_x = x - x1
    ap_y = y - y1

    # Calculate the dot product of AB and AP
    ab_ap_dot_product = ab_x * ap_x + ab_y * ap_y

    # Calculate the length of vector AB
    ab_length = math.sqrt(ab_x ** 2 + ab_y ** 2)
    point_within_boundry = False

    # Check if the projected point lies on the bounded line segment
    if ab_ap_dot_product <= -0.1:
        # The projected point is outside the segment before the starting point
        dist = math.sqrt(ap_x ** 2 + ap_y ** 2)
        projected_point = (0, 0)
    elif ab_ap_dot_product >= ab_length ** 2:
        # The projected point is outside the segment after the end point
        dist = math.sqrt((x - x2) ** 2 + (y - y2) ** 2)
        projected_point = (0, 0)
    else:
        # The projected point is inside the bounded line segment
        # Calculate the distance using the cross product
        dist = abs(ab_x * ap_y - ab_y * ap_x) / ab_length
        point_within_boundry = True

        # Calculate the position along the segment as a percentage
        position_along_segment = ab_ap_dot_product / (ab_length ** 2)
        # Determine the coordinates of the projected point
        projected_x = x1 + position_along_segment * ab_x
        projected_y = y1 + position_along_segment * ab_y
        projected_point = (projected_x, projected_y)

    angle = math.degrees(math.atan2(ab_y, ab_x))

    return dist, angle, point_within_boundry, projected_point

class PolarKinematics:
    def __init__(self, toolhead, config):
        # Setup axis steppers
        stepper_bed = stepper.PrinterStepper(config.getsection('stepper_bed'),
                                             units_in_radians=True)
        rail_arm = stepper.PrinterRail(config.getsection('stepper_arm'))
        rail_z = stepper.LookupMultiRail(config.getsection('stepper_z'))
        stepper_bed.setup_itersolve('polar_stepper_alloc', b'a')
        rail_arm.setup_itersolve('polar_stepper_alloc', b'r')
        rail_z.setup_itersolve('cartesian_stepper_alloc', b'z')
        self.rails = [rail_arm, rail_z]
        self.steppers = [stepper_bed] + [ s for r in self.rails
                                          for s in r.get_steppers() ]
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        # Setup boundary checks
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity, above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', self.max_accel, above=0., maxval=self.max_accel)
        self.critical_radius = config.getfloat(
            'critical_radius', above=0.)
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.
        max_xy = self.rails[0].get_range()[1]
        min_z, max_z = self.rails[1].get_range()
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, min_z, 0.)
        self.axes_max = toolhead.Coord(max_xy, max_xy, max_z, 0.)
    def get_steppers(self):
        return list(self.steppers)
    def calc_position(self, stepper_positions):
        bed_angle = stepper_positions[self.steppers[0].get_name()]
        arm_pos = stepper_positions[self.rails[0].get_name()]
        z_pos = stepper_positions[self.rails[1].get_name()]
        return [math.cos(bed_angle) * arm_pos, math.sin(bed_angle) * arm_pos,
                z_pos]
    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
        if 2 in homing_axes:
            self.limit_z = self.rails[1].get_range()
        if 0 in homing_axes and 1 in homing_axes:
            self.limit_xy2 = self.rails[0].get_range()[1]**2
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limit_z = (1.0, -1.0)
    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        if axis == 0:
            homepos[1] = 0.
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= hi.position_endstop - position_min
        else:
            forcepos[axis] += position_max - hi.position_endstop
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        # Always home XY together
        homing_axes = homing_state.get_axes()
        home_xy = 0 in homing_axes or 1 in homing_axes
        home_z = 2 in homing_axes
        updated_axes = []
        if home_xy:
            updated_axes = [0, 1]
        if home_z:
            updated_axes.append(2)
        homing_state.set_axes(updated_axes)
        # Do actual homing
        if home_xy:
            self._home_axis(homing_state, 0, self.rails[0])
        if home_z:
            self._home_axis(homing_state, 2, self.rails[1])
    def _motor_off(self, print_time):
        self.limit_z = (1.0, -1.0)
        self.limit_xy2 = -1.
    def check_move(self, move):
        end_pos = move.end_pos
        xy2 = end_pos[0]**2 + end_pos[1]**2
        if xy2 > self.limit_xy2:
            if self.limit_xy2 < 0.:
                raise move.move_error("Must home axis first")
            raise move.move_error()
        if move.axes_d[2]:
            if end_pos[2] < self.limit_z[0] or end_pos[2] > self.limit_z[1]:
                if self.limit_z[0] > self.limit_z[1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
            # Move with Z - update velocity and accel for slower Z axis
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
        if move.axes_d[0] or move.axes_d[1]:
            min_dist, angle, point_within_boundary, v_max_point = distance_line_to_point(move.start_pos[0:2], move.end_pos[0:2])
            if min_dist <= self.critical_radius and point_within_boundary:
                if min_dist != 0:
                    scale_radius = min_dist/self.critical_radius
                    scale_angle = abs(1.0 - (abs(180.0 - angle if angle > 90.0 else angle) / 90.0))  # From Marlin
                    move.limit_speed((self.max_velocity/100) ** (scale_angle * scale_radius),
                                     (self.max_accel/100) ** (scale_angle * scale_radius))
                    logging.info("Vel: %s, Acc: %s", self.max_velocity ** (scale_angle * scale_radius), self.max_accel ** (scale_angle * scale_radius))

    def get_status(self, eventtime):
        xy_home = "xy" if self.limit_xy2 >= 0. else ""
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            'homed_axes': xy_home + z_home,
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return PolarKinematics(toolhead, config)
