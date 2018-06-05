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
import os
import copy
import rclpy
import yaml
import time
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from object_map_msgs.msg import ObjectInfo


class ObjectMarker(object):
    """
    ObjectMarker class
    """
    def __init__(self, node):
        self._node = node
        self._node.get_logger().info('Init ObjectMarker()')

        # Initial default marker properties
        self.marker = Marker()
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.scale.x = 0.0
        self.marker.scale.y = 0.0
        self.marker.scale.z = 0.0

        line_color = ColorRGBA()
        line_color.r = 1.0
        line_color.g = 1.0
        line_color.b = 0.0
        line_color.a = 1.0
        self.marker.colors.append(line_color)
        self.marker.colors.append(line_color)

        # Marker topic on Rviz
        self._markers_pub = self._node.create_publisher(MarkerArray, '/object_map/Markers')

    def save_map(self, map_file, object_infos):
        """
        save object_infos to map file
        """
        self._node.get_logger().info('Save map to ' + str(map_file))
        if map_file is None:
            self._node.get_logger().warn('save_map: map is null, stop to save map')
            return

        map_folder = os.path.split(map_file)[0]
        if not os.access(map_folder, os.W_OK):
            self._node.get_logger().warn('save_map: has no permission save to ' + str(map_file))
            return

        self._node.get_logger().info('Save map to ' + str(map_file))
        self._node.get_logger().info('object_infos: ' + str(object_infos))
        with open(map_file, 'w') as tmp_file:
            marker_dict_list = []
            for marker in object_infos:
                marker_dict = {'id': marker.id, 'type': marker.type, 'scale':
                               [marker.scale.x, marker.scale.y, marker.scale.z],
                               'pose': [float(marker.pose.orientation.w),
                                        float(marker.pose.position.x),
                                        float(marker.pose.position.y),
                                        float(marker.pose.position.z)],
                               'points': [float(marker.points[0].x),
                                          float(marker.points[0].y),
                                          float(marker.points[0].z),
                                          float(marker.points[1].x),
                                          float(marker.points[1].y),
                                          float(marker.points[1].z)]}
                marker_dict_list.append(marker_dict)
            tmp_file.write(yaml.dump_all(marker_dict_list))

    def load_map(self, map_file):
        """
        Load object maps from a exist yaml file
        No detection or exception handling for FileNotExist exception
        """
        object_infos = []
        if map_file is None:
            self._node.get_logger().warn('load_map: fail to load map file')
            return object_infos

        if not os.path.exists(map_file):
            self._node.get_logger().info('load_map: Map not exist: ' + str(map_file))
            return object_infos

        self._node.get_logger().info('Load map: ' + str(map_file))
        with open(map_file, 'r') as tmp_file:
            for i in yaml.load_all(tmp_file):
                try:
                    object_info = ObjectInfo()
                    object_info.id = i['id']
                    object_info.type = i['type']
                    object_info.scale.x = i['scale'][0]
                    object_info.scale.y = i['scale'][1]
                    object_info.scale.z = i['scale'][2]
                    object_info.pose.orientation.w = i['pose'][0]
                    object_info.pose.position.x = i['pose'][1]
                    object_info.pose.position.y = i['pose'][2]
                    object_info.pose.position.z = i['pose'][3]

                    point1 = Point()
                    point1.x = i['points'][0]
                    point1.y = i['points'][1]
                    point1.z = i['points'][2]
                    object_info.points.append(point1)

                    point2 = Point()
                    point2.x = i['points'][3]
                    point2.y = i['points'][4]
                    point2.z = i['points'][5]
                    object_info.points.append(point2)
                    object_infos.append(object_info)
                except KeyError:
                    self._node.get_logger().info('load_maps KeyError')
        return object_infos

    def display_in_rviz(self, frame_id, object_infos):
        """
        publish objects to marker topic and display in rviz
        """
        pub_markers = MarkerArray()
        for object_info in object_infos:
            marker = copy.deepcopy(self.marker)
            marker.header.stamp = Time()
            marker.header.frame_id = frame_id
            marker.action = Marker.ADD
            marker.type = Marker.CYLINDER
            marker.ns = "SematicSLAM_line"
            marker.scale.z = 0.01
            marker.color.a = 0.35

            marker.id = object_info.id
            marker.text = object_info.type
            #marker.points = object_info.points
            marker.scale.x = object_info.scale.x
            marker.scale.y = object_info.scale.x
            marker.scale.z = object_info.scale.y
            marker.pose.position.x = object_info.pose.position.x
            marker.pose.position.y = object_info.pose.position.y
            marker.pose.position.z = object_info.pose.position.z
            marker.pose.orientation.x = object_info.pose.orientation.x
            marker.pose.orientation.y = object_info.pose.orientation.y
            marker.pose.orientation.z = object_info.pose.orientation.z
            marker.pose.orientation.w = object_info.pose.orientation.w

            pub_markers.markers.append(marker)

        for object_info in object_infos:
            marker = copy.deepcopy(self.marker)
            marker.header.stamp = Time()
            marker.header.frame_id = frame_id
            marker.action = Marker.ADD
            marker.type = Marker.TEXT_VIEW_FACING
            marker.ns = "SematicSLAM_text"
            marker.scale.z = 0.3
            marker.color.a = 1.0

            marker.id = object_info.id
            marker.text = object_info.type
            #marker.points = object_info.points
            marker.scale.x = object_info.scale.x
            marker.scale.y = object_info.scale.x
            marker.scale.z = object_info.scale.y
            marker.pose.position.x = object_info.pose.position.x
            marker.pose.position.y = object_info.pose.position.y
            marker.pose.position.z = object_info.pose.position.z
            marker.pose.orientation.x = object_info.pose.orientation.x
            marker.pose.orientation.y = object_info.pose.orientation.y
            marker.pose.orientation.z = object_info.pose.orientation.z
            marker.pose.orientation.w = object_info.pose.orientation.w

            pub_markers.markers.append(marker)

        # Publish markers on Rviz
        self._node.get_logger().info('Publish markers to Topic: /object_map/Markers')
        self._markers_pub.publish(pub_markers)
