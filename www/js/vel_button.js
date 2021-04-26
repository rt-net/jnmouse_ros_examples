// vel_button.js
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

// 並進速度 (m/s)
var linear_speed = 0.1;
// 角速度 (rad/s)
var angular_speed = 1;

const f_button = document.getElementById("forward");
const b_button = document.getElementById("backward");
const l_button = document.getElementById("left");
const r_button = document.getElementById("right");

function vel_button_state(e){
  vel = this.state;
  console.log("vel button state: " + this.state);
}

function rot_button_state(e){
  yaw = this.state;
  console.log("rot button state: " + this.state);
}

f_button.addEventListener("mousedown", {state: linear_speed, handleEvent: vel_button_state}, false);
f_button.addEventListener("touchstart", {state: linear_speed, handleEvent: vel_button_state}, false);
f_button.addEventListener("mouseup", {state: 0, handleEvent: vel_button_state}, false);
f_button.addEventListener("touchend", {state: 0, handleEvent: vel_button_state}, false);

b_button.addEventListener("mousedown", {state: -linear_speed, handleEvent: vel_button_state}, false);
b_button.addEventListener("touchstart", {state: -linear_speed, handleEvent: vel_button_state}, false);
b_button.addEventListener("mouseup", {state: 0, handleEvent: vel_button_state}, false);
b_button.addEventListener("touchend", {state: 0, handleEvent: vel_button_state}, false);

l_button.addEventListener("mousedown", {state: angular_speed, handleEvent: rot_button_state}, false);
l_button.addEventListener("touchstart", {state: angular_speed, handleEvent: rot_button_state}, false);
l_button.addEventListener("mouseup", {state: 0, handleEvent: rot_button_state}, false);
l_button.addEventListener("touchend", {state: 0, handleEvent: rot_button_state}, false);

r_button.addEventListener("mousedown", {state: -angular_speed, handleEvent: rot_button_state}, false);
r_button.addEventListener("touchstart", {state: -angular_speed, handleEvent: rot_button_state}, false);
r_button.addEventListener("mouseup", {state: 0, handleEvent: rot_button_state}, false);
r_button.addEventListener("touchend", {state: 0, handleEvent: rot_button_state}, false);
