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
import math
import rclpy
import sys
from geometry_msgs.msg import Point
from object_map.object_marker import ObjectMarker
from object_map_msgs.msg import ObjectInfo


def main(args=None):
    """
    test to create map then load the map
    """
    if args is None:
        args = sys.argv

    rclpy.init(args=args)
    node = rclpy.create_node("test_conf")
    file_path = "/tmp/test.yaml"
    test_marker_number = 3      # Number of markers created in test function

    object_marker = ObjectMarker(node)

    print ("Create markers %d, and save their info to %s." % (test_marker_number, file_path))
    object_infos = fake_object_map(test_marker_number)
    object_marker.save_map(file_path, object_infos)          # save markers in the yaml file

    print ("Load marker information from %s and print their id." % file_path)
    object_infos = object_marker.load_map(file_path)  # Get current index and object_infos
    if object_infos is not None:
        for i in object_infos:
            print (i.id)
        print ("Please type \'cat %s\' to check detailed information." % file_path)
    else:
        print ("object_infos is None")

    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

def fake_object_map(test_marker_number):
    """
    fake the data for test
    """
    object_infos = []
    for i in range(0, test_marker_number):
        object_info = ObjectInfo()
        object_info.id = i
        object_info.type = "type.%d" % i
        object_info.scale.x = 0.1 * i
        object_info.scale.y = 0.1 * i
        object_info.scale.z = 0.1 * i
        object_info.pose.orientation.w = i / 2.0 / math.pi
        object_info.pose.position.x = float(i)
        object_info.pose.position.y = float(i)
        object_info.pose.position.z = float(i)
        point1 = Point()
        point1.x = float(i)
        point1.y = float(i)
        point1.z = float(i)
        object_info.points.append(point1)
        point2 = Point()
        point2.x = i + 1.0
        point2.y = i + 1.0
        point2.z = i + 1.0
        object_info.points.append(point2)
        object_infos.append(object_info)

    # print 'maps: %s' % object_infos
    return object_infos

if __name__ == '__main__':
    main()
