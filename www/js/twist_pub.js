// twist_pub.js
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
// 
// roslibjs公式サンプルを改変して作成しています
// http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality#CA-3bc839e9e2d3c2cac3f7fabffacfbda3d5c050ae_18
// ROS wiki by Open Robotics (CC BY 3.0)
// https://creativecommons.org/licenses/by/3.0/
//
// また以下のページを参考にしています
// https://gray-code.com/javascript/get-parameter-of-url/

var yaw = 0;
var vel = 0;

var url = new URL(window.location.href);
var params = url.searchParams;
var port_8080 = params.get('port_8080');
var port_9090 = params.get('port_9090');
console.log(port_8080)
console.log(port_9090)
if (port_8080 == null) port_8080 = location.hostname + ':8080'
if (port_9090 == null) port_9090 = 'ws://' + location.hostname + ':9090'

//ROS
var ros = new ROSLIB.Ros({
  url: port_9090
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

var cmd_vel = new ROSLIB.Topic({
  ros : ros,
  name : '/cmd_vel',
  messageType : 'geometry_msgs/Twist'
});

function twist_pub() {
  var twist = new ROSLIB.Message({
    linear : {
      x : vel,
      y : 0,
      z : 0
    },
    angular : {
      x : 0,
      y : 0,
      z : yaw
    }
  });

  cmd_vel.publish(twist);
}

setInterval(twist_pub, 100);
