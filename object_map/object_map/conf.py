#!/usr/bin/env python
"""
 Copyright (c) 2018 Intel Corporation

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
"""
import rclpy
from geometry_msgs.msg import Point
from object_map_msgs.msg import ObjectInfo


class Param(object):
    """
    parameters
    """
    def __init__(self, logger):        
        self._logger = logger

        # init parameters instead of yaml file
        self._min_diameter = 0.2
        self._max_diameter = 0.4
        self._max_distance = 8.0
        self._ref_camera_id = "rgbd_cam"
        self._ref_map_id = "map"
        self._default_distance_tolerance = 1.0
        self._default_probability_tolerance = 0.5
        self._object_map_file = "/tmp/object_map.yaml"
        self._object_type_dict = {  'chair':          {'probability': 0.2, 'distance_tolerance': 0.8},
                                    'charger_pile':   {'probability': 0.2, 'distance_tolerance': 0.6},
                                    'car':            {'probability': 0.2, 'distance_tolerance': 0.4},
                                    'toy':            {'probability': 0.2, 'distance_tolerance': 0.4},
                                    'person':         {'probability': 0.2, 'distance_tolerance': 0.4},
                                    'aeroplane':      {'probability': 0.2, 'distance_tolerance': 0.4},
                                    'bird':           {'probability': 0.2, 'distance_tolerance': 0.4}
                                 }
        self._object_type_list = self._object_type_dict.keys()

        self._config_from_param()


    def _config_from_param(self):
        
        # Todo
        # Read configure from rosparam
        # command line parameters and parameters from a yaml file is not ready in ros2
        # unti next release in summer 2018
        
        self._logger.info("Init Param()")

    @property
    def min_diameter(self):
        """
        get min diameter
        """
        return self._min_diameter

    @property
    def max_diameter(self):
        """
        get max diameter
        """
        return self._max_diameter

    @property
    def max_distance(self):
        """
        get max distance
        """
        return self._max_distance

    @property
    def ref_camera_id(self):
        """
        get reference camera id
        """
        return self._ref_camera_id

    @property
    def ref_map_id(self):
        """
        get reference map id
        """
        return self._ref_map_id

    @property
    def default_distance_tolerance(self):
        """
        get default distance tolerance
        """
        return self._default_distance_tolerance

    @property
    def default_probability_tolerance(self):
        """
        get default probability tolerance
        """
        return self._default_probability_tolerance

    @property
    def object_map_file(self):
        """
        get object map file
        """
        return self._object_map_file

    @property
    def object_type_list(self):
        """
        get supported object types
        """
        return self._object_type_list

    def get_object_distance_tolerance(self, object_type):
        """
        get distance tolerance of selected object
        """
        if not self._object_type_dict:
            return self.default_distance_tolerance

        if 'distance_tolerance' in self._object_type_dict[object_type]:
            return self._object_type_dict[object_type]['distance_tolerance']
        return self.default_distance_tolerance

    def get_object_probability_tolerance(self, object_type):
        """
        get probability tolerance of selected object
        """
        if not self._object_type_dict:
            return self.default_probability_tolerance

        if 'probability' in self._object_type_dict[object_type]:
            return self._object_type_dict[object_type]["probability"]
        return self.default_probability_tolerance

