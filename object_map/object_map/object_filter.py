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
import message_filters

from object_analytics_msgs.msg import TrackedObjects
from object_analytics_msgs.msg import ObjectsInBoxes3D
from object_msgs.msg import ObjectsInBoxes

from object_map_msgs.msg import Objects, Object


class ObjectFilter(object):
    """
    receive multiple topics and fileter as one object topic
    """
    def __init__(self, node):

        self._logger = node.get_logger()
        self._node = node

        self._logger.info("Init ObjectFilter()")
        self._pub_object_topic = self._node.create_publisher(Objects, '/object_map/filtered_object')

        sub_detection = message_filters.Subscriber(self._node, ObjectsInBoxes, '/movidius_ncs_stream/detected_objects')
        sub_tracking = message_filters.Subscriber(self._node, TrackedObjects, '/object_analytics/tracking')
        sub_localization = message_filters.Subscriber(self._node, ObjectsInBoxes3D, '/object_analytics/localization')

        object_filter = message_filters.ApproximateTimeSynchronizer(
                        [sub_detection, sub_tracking, sub_localization], 10, 0.01, allow_headerless=False)

        object_filter.registerCallback(self._callback)

    def _callback(self, objects_in_boxes, tracked_objects, objects_in_boxes_3d):
        """
        when received msg from topic, execute callback
        """
        def overlap_ratio(obj_a, obj_b):
            x1 = obj_a.roi.x_offset
            y1 = obj_b.roi.y_offset
            width1 = obj_a.roi.width
            height1 = obj_a.roi.height
            x2 = obj_b.roi.x_offset
            y2 = obj_b.roi.y_offset
            width2 = obj_b.roi.width
            height2 = obj_b.roi.height
            endx = max(x1+width1,x2+width2)
            startx = min(x1,x2)
            width = width1+width2-(endx-startx)
            endy = max(y1+height1,y2+height2)
            starty = min(y1,y2)
            height = height1+height2-(endy-starty)
            if (width<=0 and height<=0):
                return 0
            else:
                Area = width*height
                Area1 = width1*height1
                Area2 = width2*height2
                ratio = Area /(Area1+Area2-Area)
            return ratio

        self._logger.info("Receiving Message Filter Message...")
        pub_objects = Objects()
        pub_objects.header.stamp = tracked_objects.header.stamp
        for tracked_object in tracked_objects.tracked_objects:
            filter_object = TrackObject()

            filter_object._track_id = tracked_object.id
            for object_inboxes_3d in objects_in_boxes_3d.objects_in_boxes:

                if overlap_ratio(tracked_object, object_inboxes_3d) > 0.6:
                    filter_object._min_point= object_inboxes_3d.min
                    filter_object._max_point= object_inboxes_3d.max

            for object_in_box in objects_in_boxes.objects_vector:
                if overlap_ratio(tracked_object, object_in_box) > 0.6:
                    filter_object._text = object_in_box.object.object_name
                    filter_object._probability = object_in_box.object.probability

            if filter_object.verify():
                self._logger.info("Appending filtered object...")
                self._logger.info("___ID:{0},probability:{1}, min:{2}, max:{3}, text:{4}".format(filter_object.track_id, \
                  filter_object.probability, filter_object.min_point, filter_object.max_point, filter_object.text))
                pub_objects.objects.append(filter_object.get_object())

        self._logger.info("Publishing filtered object ...")
        self._pub_object_topic.publish(pub_objects)


class TrackObject(object):
    """
    Track object API
    """
    def __init__(self):
        self._track_id = None
        self._probability = None
        self._min_point = None
        self._max_point = None
        self._text = None

    def verify(self):
        """
        verity
        """
        if self._track_id is not None and self._probability is not None and \
           self._max_point is not None and self._min_point is not None and self._text is not None:
            return True
        return False

    @property
    def track_id(self):
        """
        get track_id
        """
        return self._track_id

    @track_id.setter
    def track_id(self, value):
        """
        set track ID
        """
        self._track_id = value

    @property
    def probability(self):
        """
        get probability
        """
        return self._probability

    @probability.setter
    def probability(self, value):
        """
        set probability
        """
        self._probability = value

    @property
    def max_point(self):
        """
        get max_point
        """
        return self._max_point

    @max_point.setter
    def max_point(self, value):
        """
        set max point
        """
        self._max_point = value

    @property
    def min_point(self):
        """
        get min_point
        """
        return self._min_point

    @min_point.setter
    def min_point(self, value):
        """
        set min point
        """
        self._min_point = value

    @property
    def text(self):
        """
        get text
        """
        return self._text

    @text.setter
    def text(self, value):
        """
        set text
        """
        self._text = value

    def get_object(self):
        """
        get object
        """
        filter_object = Object()
        filter_object.id = self._track_id
        filter_object.name = self._text
        filter_object.probability = self._probability
        filter_object.min_point = self._min_point
        filter_object.max_point = self._max_point
        return filter_object
