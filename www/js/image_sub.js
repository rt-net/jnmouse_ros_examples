// image_sub.js
//
// Copyright 2020-2021 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

var image_topic_l = "/csi_cam_0/image_raw";
var image_topic_r = "/csi_cam_1/image_raw";

document.write("<img class='camera_image' src='http://" + location.hostname + ":8080/stream?topic=" + image_topic_l + "&type=ros_compressed'></img>");
document.write("<img class='camera_image' src='http://" + location.hostname + ":8080/stream?topic=" + image_topic_r + "&type=ros_compressed'></img>");
