# Exclude moves toward and inside objects
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
# Copyright (C) 2021  Troy Jacobson <troy.d.jacobson@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import json
import math


def distance(p1: tuple, p2: tuple) -> float:
    """Calculates distances between 2 points"""
    return math.sqrt(((p2[0] - p1[0]) ** 2) + ((p2[1] - p1[1]) ** 2))


def distance_line_to_point(p1, p2, p) -> float:
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

    # Check if the projected point lies on the bounded line segment
    if ab_ap_dot_product <= 0:
        # The projected point is outside the segment before the starting point
        distance = math.sqrt(ap_x ** 2 + ap_y ** 2)
    elif ab_ap_dot_product >= ab_length ** 2:
        # The projected point is outside the segment after the end point
        distance = math.sqrt((x - x2) ** 2 + (y - y2) ** 2)
    else:
        # The projected point is inside the bounded line segment
        # Calculate the distance using the cross product
        distance = abs(ab_x * ap_y - ab_y * ap_x) / ab_length

    return distance


def calc_object_max_radius(obj) -> float:
    """
    The calc_object_max_radius function takes in a single object and returns the maximum radius of that object.

    The function does this by first finding the center of the object, then calculating the distance between each point
    in its polygon and that center. The largest distance is returned as an output.

    :param obj: Pass in the object dictionary
    :return: The maximum distance between the center of an object and any point on its polygon
    """
    center = obj["center"]  # list: [x, y]
    radius = 0
    for point in obj["polygon"]:
        dis = distance(center, point)
        if dis > radius:
            radius = dis
    return radius


def point_in_circle(p1: tuple, center: tuple, radius: float) -> bool:
    return (p1[0]-center[0])**2 + (p1[1]-center[1])**2 <= radius**2


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


def rotate(origin, point, angle) -> list:
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    Returns:
        list - list with x, y coordinates
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return [qx, qy]


def shortest_path_around_circle(p_in: tuple, p_out: tuple, center) -> list:
    points_per_arc = 3
    points = []

    # calc arc angle
    x1 = p_in[0] - center[0]
    y1 = p_in[1] - center[1]
    x2 = p_out[0] - center[0]
    y2 = p_out[1] - center[1]
    arc_angle = math.atan2(x1 * y2 - y1 * x2, x1 * x2 + y1 * y2)  # -180 -> 180 | >0 -> anticlockwise, <0 -> clockwise

    step = arc_angle / (points_per_arc + 1)
    for i in range(1, points_per_arc + 1):
        points.append(rotate((0, 0), (p_in[0], p_in[1]), step * i))     # make p_in assignment nicer
        #TODO make points 4 item lists and lerp between input and output point values of item 3 and 4
    return points


class ExcludeObject:
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

        #TODO read that from config
        self.avoid_objects = True   # activates routing around objects

        self._reset_state()
        self.gcode.register_command(
            'EXCLUDE_OBJECT_START', self.cmd_EXCLUDE_OBJECT_START,
            desc=self.cmd_EXCLUDE_OBJECT_START_help)
        self.gcode.register_command(
            'EXCLUDE_OBJECT_END', self.cmd_EXCLUDE_OBJECT_END,
            desc=self.cmd_EXCLUDE_OBJECT_END_help)
        self.gcode.register_command(
            'EXCLUDE_OBJECT', self.cmd_EXCLUDE_OBJECT,
            desc=self.cmd_EXCLUDE_OBJECT_help)
        self.gcode.register_command(
            'EXCLUDE_OBJECT_DEFINE', self.cmd_EXCLUDE_OBJECT_DEFINE,
            desc=self.cmd_EXCLUDE_OBJECT_DEFINE_help)

    def _register_transform(self):
        if self.next_transform is None:
            tuning_tower = self.printer.lookup_object('tuning_tower')
            if tuning_tower.is_active():
                logging.info('The ExcludeObject move transform is not being '
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

    def _reset_state(self):
        self.objects = []
        self.excluded_objects = []
        self.current_object = None
        self.in_excluded_region = False

    def _reset_file(self):
        self._reset_state()
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
        offset = self._get_extrusion_offsets()

        if self.initial_extrusion_moves > 0 and \
            self.last_position[3] != newpos[3]:
            # Since the transform is not loaded until there is a request to
            # exclude an object, the transform needs to track a few extrusions
            # to get the state of the extruder
            self.initial_extrusion_moves -= 1

        self.last_position[:] = newpos
        self.last_position_extruded[:] = self.last_position
        self.max_position_extruded = max(self.max_position_extruded, newpos[3])

        # These next few conditionals handle the moves immediately after leaving
        # and excluded object.  The toolhead is at the end of the last printed
        # object and the gcode is at the end of the last excluded object.
        #
        # Ideally, there will be Z and E moves right away to adjust any offsets
        # before moving away from the last position.  Any remaining corrections
        # will be made on the firs XY move.
        if (offset[0] != 0 or offset[1] != 0) and \
            (newpos[0] != self.last_position_excluded[0] or \
            newpos[1] != self.last_position_excluded[1]):
            offset[0] = 0
            offset[1] = 0
            offset[2] = 0
            offset[3] += self.extruder_adj
            self.extruder_adj = 0

        if offset[2] != 0 and newpos[2] != self.last_position_excluded[2]:
            offset[2] = 0

        if self.extruder_adj != 0 and \
            newpos[3] != self.last_position_excluded[3]:
            offset[3] += self.extruder_adj
            self.extruder_adj = 0

        tx_pos = newpos[:]
        for i in range(4):
            tx_pos[i] = newpos[i] - offset[i]
        self.next_transform.move(tx_pos, speed)

    def _ignore_move(self, newpos, speed):
        offset = self._get_extrusion_offsets()
        for i in range(3):
            offset[i] = newpos[i] - self.last_position_extruded[i]
        offset[3] = offset[3] + newpos[3] - self.last_position[3]
        self.last_position[:] = newpos
        self.last_position_excluded[:] =self.last_position
        self.max_position_excluded = max(self.max_position_excluded, newpos[3])

    def _move_into_excluded_region(self, newpos, speed):
        self.in_excluded_region = True
        self._ignore_move(newpos, speed)

    def _move_from_excluded_region(self, newpos, speed):
        self.in_excluded_region = False

        # This adjustment value is used to compensate for any retraction
        # differences between the last object printed and excluded one.
        self.extruder_adj = self.max_position_excluded \
            - self.last_position_excluded[3] \
            - (self.max_position_extruded - self.last_position_extruded[3])
        self._normal_move(newpos, speed)

    def _test_in_excluded_region(self):
        # Inside cancelled object
        return self.current_object in self.excluded_objects \
            and self.initial_extrusion_moves == 0

    def _test_crossing_excluded_region(self, newpos: tuple) -> list:
        """
        The function is used to check if the robot will cross an excluded region.

        It does this by checking if the distance between a line connecting the start and end position of a movement
        and any point in an excluded region is less than that regions radius. If it is, then we know that there will be
        an intersection between our path and the excluded region.

        :param self: Make the method belong to the class
        :param newpos: tuple: Find the new position of the robot
        :return: A list of excluded objects
        """
        excluded_objects = []
        for object_name in self.excluded_objects:
            # find excluded object
            ex_object = None
            for obj in self.objects:
                if obj["name"] == object_name:
                    ex_object = obj
                    break

            # # find center + max circle distance
            center = ex_object["center"]  # list: [x, y]
            radius = calc_object_max_radius(ex_object)

            # check for rough interference
            start_pos = tuple(self.get_position()[0:2])
            end_pos = newpos[0:2]
            min_dist = distance_line_to_point(start_pos, end_pos, center)

            if min_dist < radius:
                excluded_objects.append(ex_object)
                # TODO: refine calculation to use the exact polygon

        return excluded_objects

    def get_status(self, eventtime=None):
        status = {
            "objects": self.objects,
            "excluded_objects": self.excluded_objects,
            "current_object": self.current_object
        }
        return status

    def move(self, newpos, speed):
        move_in_excluded_region = self._test_in_excluded_region()
        self.last_speed = speed

        if move_in_excluded_region:
            if self.in_excluded_region:
                self._ignore_move(newpos, speed)
            else:
                self._move_into_excluded_region(newpos, speed)
        else:
            if self.in_excluded_region:
                self._move_from_excluded_region(newpos, speed)
            else:
                if self.avoid_objects:
                    excluded_objects = self._test_crossing_excluded_region(newpos)
                    if excluded_objects:  # evasive_maneuvers
                        #TODO handle more than one excluded object
                        radius = calc_object_max_radius(excluded_objects[0])
                        if radius is not None:
                            # handle "Endpoint is in circle"
                            if point_in_circle(newpos, excluded_objects[0]["center"], radius):
                                p_inter = intersection_vector_circle(tuple(self.get_position()[0:2]),
                                                                     newpos,
                                                                     excluded_objects[0]["center"],
                                                                     radius)
                                p_inter.append(newpos[2])   #TODO put this into point generation
                                p_inter.append(newpos[3])   #TODO put this into point generation

                                if not p_inter:  # p_inter generation failed, move along
                                    p_inter = newpos
                                self._normal_move(p_inter, speed)
                            # handle "Passing through region"
                            else:
                                #TODO calculate the polygon value, not the circle
                                intersections = intersection_vector_circle(tuple(self.get_position()[0:2]),
                                                                           newpos,
                                                                           excluded_objects[0]["center"],
                                                                           radius,
                                                                           all_points=True)
                                if intersections:
                                    p_inter_1, p_inter_2 = intersections
                                    p_inter_1.append(newpos[2])  # TODO put this into point generation + handle extrus.
                                    p_inter_1.append(newpos[3])  # TODO put this into point generation + handle extrus.
                                    p_inter_2.append(newpos[2])  # TODO put this into point generation + handle extrus.
                                    p_inter_2.append(newpos[3])  # TODO put this into point generation + handle extrus.

                                    self._normal_move(p_inter_1, speed)

                                    # go around region
                                    points = shortest_path_around_circle(p_inter_1,
                                                                         p_inter_2,
                                                                         excluded_objects[0]["center"])
                                    for point in points:
                                        target = point
                                        target.append(newpos[2])
                                        target.append(newpos[3])
                                        self._normal_move(target, speed)
                                # finish with original finish
                                self._normal_move(newpos, speed)

                    else:
                        self._normal_move(newpos, speed)
                else:
                    self._normal_move(newpos, speed)

    cmd_EXCLUDE_OBJECT_START_help = "Marks the beginning the current object" \
                                    " as labeled"
    def cmd_EXCLUDE_OBJECT_START(self, gcmd):
        name = gcmd.get('NAME').upper()
        if not any(obj["name"] == name for obj in self.objects):
            self._add_object_definition({"name": name})
        self.current_object = name
        self.was_excluded_at_start = self._test_in_excluded_region()

    cmd_EXCLUDE_OBJECT_END_help = "Marks the end the current object"
    def cmd_EXCLUDE_OBJECT_END(self, gcmd):
        if self.current_object == None and self.next_transform:
            gcmd.respond_info("EXCLUDE_OBJECT_END called, but no object is"
                              " currently active")
            return
        name = gcmd.get('NAME', default=None)
        if name != None and name.upper() != self.current_object:
            gcmd.respond_info("EXCLUDE_OBJECT_END NAME=%s does not match the"
                              " current object NAME=%s" %
                              (name.upper(), self.current_object))

        self.current_object = None

    cmd_EXCLUDE_OBJECT_help = "Cancel moves inside a specified objects"
    def cmd_EXCLUDE_OBJECT(self, gcmd):
        reset = gcmd.get('RESET', None)
        current = gcmd.get('CURRENT', None)
        name = gcmd.get('NAME', '').upper()

        if reset:
            if name:
                self._unexclude_object(name)

            else:
                self.excluded_objects = []

        elif name:
            if name.upper() not in self.excluded_objects:
                self._exclude_object(name.upper())

        elif current:
            if not self.current_object:
                gcmd.respond_error('There is no current object to cancel')

            else:
                self._exclude_object(self.current_object)

        else:
            self._list_excluded_objects(gcmd)

    cmd_EXCLUDE_OBJECT_DEFINE_help = "Provides a summary of an object"
    def cmd_EXCLUDE_OBJECT_DEFINE(self, gcmd):
        reset = gcmd.get('RESET', None)
        name = gcmd.get('NAME', '').upper()

        if reset:
            self._reset_file()

        elif name:
            parameters = gcmd.get_command_parameters().copy()
            parameters.pop('NAME')
            center = parameters.pop('CENTER', None)
            polygon = parameters.pop('POLYGON', None)

            obj = {"name": name.upper()}
            obj.update(parameters)

            if center != None:
                obj['center'] = json.loads('[%s]' % center)

            if polygon != None:
                obj['polygon'] = json.loads(polygon)

            self._add_object_definition(obj)

        else:
            self._list_objects(gcmd)

    def _add_object_definition(self, definition):
        self.objects = sorted(self.objects + [definition],
                              key=lambda o: o["name"])

    def _exclude_object(self, name):
        self._register_transform()
        self.gcode.respond_info('Excluding object {}'.format(name.upper()))
        if name not in self.excluded_objects:
            self.excluded_objects = sorted(self.excluded_objects + [name])

    def _unexclude_object(self, name):
        self.gcode.respond_info('Unexcluding object {}'.format(name.upper()))
        if name in self.excluded_objects:
            excluded_objects = list(self.excluded_objects)
            excluded_objects.remove(name)
            self.excluded_objects = sorted(excluded_objects)

    def _list_objects(self, gcmd):
        if gcmd.get('JSON', None) is not None:
            object_list = json.dumps(self.objects)
        else:
            object_list = " ".join(obj['name'] for obj in self.objects)
        gcmd.respond_info('Known objects: {}'.format(object_list))

    def _list_excluded_objects(self, gcmd):
        object_list = " ".join(self.excluded_objects)
        gcmd.respond_info('Excluded objects: {}'.format(object_list))

def load_config(config):
    return ExcludeObject(config)
