# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
std_msgs/Header header               # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id                             # object ID useful in conjunction with the namespace for manipulating and deleting the object later
string type                          # Type of object
geometry_msgs/Pose pose              # Pose of the object
geometry_msgs/Vector3 scale          # Scale of the object 1,1,1 means default (usually 1 meter square)
geometry_msgs/Point[] points
