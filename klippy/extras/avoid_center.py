# Exclude moves toward and inside objects
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
# Copyright (C) 2021  Troy Jacobson <troy.d.jacobson@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import json
import math


def distance_line_to_point(p1, p2, p):
    """
    Calculates the distance between a point and a line segment defined by two points.

    Args:
        p (list): The coordinates of the point [x, y].
        p1 (list): The coordinates of the first endpoint of the line segment [x1, y1].
        p2 (list): The coordinates of the second endpoint of the line segment [x2, y2].

    Returns:
        float: The distance between the point and the line segment.
        tuple: nearest point

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

    # Check if the projected point lies on the bounded line segment
    if ab_ap_dot_product <= 0:
        # The projected point is outside the segment before the starting point
        distance = math.sqrt(ap_x ** 2 + ap_y ** 2)
        nearest_point = (x1, y2)
        contact_position = 0
    elif ab_ap_dot_product >= ab_length ** 2:
        # The projected point is outside the segment after the end point
        distance = math.sqrt((x - x2) ** 2 + (y - y2) ** 2)
        nearest_point = (x2, y2)
        contact_position = 1
    else:
        # The projected point is inside the bounded line segment
        # Calculate the distance using the cross product
        distance = abs(ab_x * ap_y - ab_y * ap_x) / ab_length
        # Calculate the position along the segment as a percentage
        position_along_segment = ab_ap_dot_product / (ab_length ** 2)
        # Determine the coordinates of the projected point
        projected_x = x1 + position_along_segment * ab_x
        projected_y = y1 + position_along_segment * ab_y
        nearest_point = (projected_x, projected_y)
        contact_position = 2

    return distance, nearest_point, contact_position


def intersection_vector_circle(p1: tuple, p2: tuple, center: tuple, radius: float, all_points=False) -> list:
    """
    The intersection_vector_circle function calculates the intersection of the vector between two points with a circle.

    If all_points is set to True, it will return both intersections (if they exist). If not, it will only return one.

    :param p1: tuple: Define the first point of the line
    :param p2: tuple: Determine the end point of the vector
    :param center: tuple: Define the center of the circle
    :param radius: float: Set the radius of the circle
    :param all_points: Determine whether to return one or two points
    :return: The intersection point between a line and a circle
    """
    if p1 == p2:
        return []
    a = (p2[0]-p1[0])**2 + (p2[1]-p1[1])**2
    b = 2*(p2[0]-p1[0])*(p1[0]-center[0]) + 2*(p2[1]-p1[1])*(p1[1]-center[1])
    c = (p1[0]-center[0])**2 + (p1[1]-center[1])**2 - radius**2

    if c <= 0:  # P1 in circle
        if all_points:
            return list(p1), list(p2)
        return list(p1)
    if (b**2 - 4*a*c) <= 0:
        return []     #TODO better handling of this case (what is this case?)

    if all_points:
        s1 = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)
        s2 = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        x1 = (p2[0] - p1[0]) * s1 + p1[0]
        y1 = (p2[1] - p1[1]) * s1 + p1[1]
        x2 = (p2[0] - p1[0]) * s2 + p1[0]
        y2 = (p2[1] - p1[1]) * s2 + p1[1]
    else:
        s = 2*c/(-b + math.sqrt(b**2-(4*a*c)))
        x = (p2[0]-p1[0])*s + p1[0]
        y = (p2[1] - p1[1]) * s + p1[1]

    if all_points:
        return [x1, y1], [x2, y2]
    return [x, y]


def generate_arc_points(radius, point1, point2, angle_step_deg):
    # Compute angles of the two points
    theta1 = math.atan2(point1[1], point1[0])
    theta2 = math.atan2(point2[1], point2[0])

    # Ensure angles are in the range [0, 2*pi]
    if theta1 < 0:
        theta1 += 2 * math.pi
    if theta2 < 0:
        theta2 += 2 * math.pi

    # Calculate angular differences
    clockwise_diff = (theta1 - theta2) % (2 * math.pi)
    counterclockwise_diff = (theta2 - theta1) % (2 * math.pi)

    # Choose the smaller arc
    if clockwise_diff < counterclockwise_diff:
        # Clockwise arc
        step = -math.radians(angle_step_deg)
        angles = []
        current_angle = theta1
        while current_angle > theta2:
            angles.append(current_angle)
            current_angle += step
        angles.append(theta2)  # Ensure the last point is included
    else:
        # Counterclockwise arc
        step = math.radians(angle_step_deg)
        angles = []
        current_angle = theta1
        while current_angle < theta2:
            angles.append(current_angle)
            current_angle += step
        angles.append(theta2)  # Ensure the last point is included

    # Convert angles to points on the circle
    points = [(radius * math.cos(angle), radius * math.sin(angle)) for angle in angles]
    return points


class AvoidCenter:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.printer.register_event_handler('klippy:connect',
                                        self._handle_connect)
        self.printer.register_event_handler("virtual_sdcard:reset_file",
                                            self._reset_file)
        self.next_transform = None
        self.last_position_extruded = [0., 0., 0., 0.]
        self.last_position_excluded = [0., 0., 0., 0.]

        self.min_radius = 0.1   # mm
        self.radius_speed = 1   # mm/s
        self.circle_steps = 20  # in °

    def _register_transform(self):
        if self.next_transform is None:
            tuning_tower = self.printer.lookup_object('tuning_tower')
            if tuning_tower.is_active():
                logging.info('The Avoid Center move transform is not being '
                    'loaded due to Tuning tower being Active')
                return

            self.next_transform = self.gcode_move.set_move_transform(self, force=True)
            self.extrusion_offsets = {}
            self.max_position_extruded = 0
            self.max_position_excluded = 0
            self.extruder_adj = 0
            self.initial_extrusion_moves = 5
            self.last_position = [0., 0., 0., 0.]

            self.get_position()
            self.last_position_extruded[:] = self.last_position
            self.last_position_excluded[:] = self.last_position

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')

    def _unregister_transform(self):
        if self.next_transform:
            tuning_tower = self.printer.lookup_object('tuning_tower')
            if tuning_tower.is_active():
                logging.error('The Exclude Object move transform was not '
                    'unregistered because it is not at the head of the '
                    'transform chain.')
                return

            self.gcode_move.set_move_transform(self.next_transform, force=True)
            self.next_transform = None
            self.gcode_move.reset_last_position()

    def _reset_file(self):
        # self._reset_state()
        self._unregister_transform()

    def _get_extrusion_offsets(self):
        offset = self.extrusion_offsets.get(
            self.toolhead.get_extruder().get_name())
        if offset is None:
            offset = [0., 0., 0., 0.]
            self.extrusion_offsets[self.toolhead.get_extruder().get_name()] = \
                offset
        return offset

    def get_position(self):
        offset = self._get_extrusion_offsets()
        pos = self.next_transform.get_position()
        for i in range(4):
            self.last_position[i] = pos[i] + offset[i]
        return list(self.last_position)

    def _normal_move(self, newpos, speed):
        #offset = self._get_extrusion_offsets()

        #if self.initial_extrusion_moves > 0 and \
        #    self.last_position[3] != newpos[3]:
        #    # Since the transform is not loaded until there is a request to
        #    # exclude an object, the transform needs to track a few extrusions
        #    # to get the state of the extruder
        #    self.initial_extrusion_moves -= 1

        #self.last_position[:] = newpos
        #self.last_position_extruded[:] = self.last_position
        #self.max_position_extruded = max(self.max_position_extruded, newpos[3])

        ## These next few conditionals handle the moves immediately after leaving
        ## and excluded object.  The toolhead is at the end of the last printed
        ## object and the gcode is at the end of the last excluded object.
        ##
        ## Ideally, there will be Z and E moves right away to adjust any offsets
        ## before moving away from the last position.  Any remaining corrections
        ## will be made on the firs XY move.
        #if (offset[0] != 0 or offset[1] != 0) and \
        #    (newpos[0] != self.last_position_excluded[0] or \
        #    newpos[1] != self.last_position_excluded[1]):
        #    offset[0] = 0
        #    offset[1] = 0
        #    offset[2] = 0
        #    offset[3] += self.extruder_adj
        #    self.extruder_adj = 0

        #if offset[2] != 0 and newpos[2] != self.last_position_excluded[2]:
        #    offset[2] = 0

        #if self.extruder_adj != 0 and \
        #    newpos[3] != self.last_position_excluded[3]:
        #    offset[3] += self.extruder_adj
        #    self.extruder_adj = 0

        #tx_pos = newpos[:]
        #for i in range(4):
        #    tx_pos[i] = newpos[i] - offset[i]
        #self.next_transform.move(tx_pos, speed)
        self.next_transform.move(newpos, speed)

    # def _ignore_move(self, newpos, speed):
    #     offset = self._get_extrusion_offsets()
    #     for i in range(3):
    #         offset[i] = newpos[i] - self.last_position_extruded[i]
    #     offset[3] = offset[3] + newpos[3] - self.last_position[3]
    #     self.last_position[:] = newpos
    #     self.last_position_excluded[:] =self.last_position
    #     self.max_position_excluded = max(self.max_position_excluded, newpos[3])

    def _move_into_circle(self, newpos, speed):
        self._normal_move(newpos, speed)
        #TODO: Add extrusion compensation similiar to _ignore_move

    def _move_from_excluded_region(self, newpos, speed):
        #TODO: Add extrusion compensation similiar to ExludeObject
        self._normal_move(newpos, speed)

        # This adjustment value is used to compensate for any retraction
        # differences between the last object printed and excluded one.
        #self.extruder_adj = self.max_position_excluded \
        #    - self.last_position_excluded[3] \
        #    - (self.max_position_extruded - self.last_position_extruded[3])
        #self._normal_move(newpos, speed)

    def _move_on_circle(self, _start, _end):
        #TODO add extruder offset compensation
        _points = generate_arc_points(self.min_radius, _start[0:2],
                                      _end[0:2], self.circle_steps)
        for point in _points:
            self._normal_move((point[0], point[1], _end[2], _end[3]),
                              self.radius_speed)
        self._normal_move(_end, self.radius_speed)

    def move(self, newpos, speed):
        # check min distance to circle
        start_pos = self.get_position()
        end_pos = newpos
        min_dist, nearest_point, contact_state = distance_line_to_point(start_pos[0:2], end_pos[0:2], [0, 0])

        if min_dist >= 0.2:  # Outside radius
            self._normal_move(newpos, speed)
        else:  # Move hits circle
            if contact_state == 0:  # STARTS
                col_point_1 = intersection_vector_circle(tuple(start_pos[0:2]), end_pos[0:2],
                                                       (0,0), self.min_radius)
                self._move_on_circle(start_pos, col_point_1)
                self._normal_move(col_point_1, end_pos)
            elif contact_state == 1:  # ENDS
                col_point_1 = intersection_vector_circle(tuple(start_pos[0:2]), end_pos[0:2],
                                                       (0, 0), self.min_radius)
                adj_pos = newpos
                adj_pos[0] = col_point_1[0]
                adj_pos[1] = col_point_1[1]
                self._move_into_circle(adj_pos, speed)
            elif contact_state == 2:  # THROUGH
                col_point_1, col_point_2 = intersection_vector_circle(tuple(start_pos[0:2]), end_pos[0:2],
                                                       (0, 0), self.min_radius, True)
                adj_pos = newpos
                adj_pos[0] = col_point_1[0]
                adj_pos[1] = col_point_1[1]
                self._move_into_circle(adj_pos, speed)

                self._move_on_circle(col_point_1, col_point_2)
                self._move_from_excluded_region(col_point_2, end_pos)

    def cmd_AVOID_CENTER(self, gcmd):
        self._register_transform()
        gcmd.respond_info("Center avoidance successfully loaded")


def load_config(config):
    return AvoidCenter(config)

