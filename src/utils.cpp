/**
Copyright 2016 Open Source Robotics Foundation, Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
 */


/**
 * @file utils.cpp
 * @author Govind Ajith Kumar, Pradeep Gopal, Rajesh NS, Cheng, Dakota Abernathy
 * @copyright MIT License
 * @brief Implementing the publisher
 * This is the talker file for ROS subscriber-publisher example.
 */

/**
 *MIT License
 *Copyright (c) 2020 Govind Ajith Kumar, Pradeep Gopal, Rajesh NS, Cheng, Dakota Abernathy
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

#include "utils.h"

/**
 * Hash-map to get different model heights for different parts.
 * They were modified because each part sinks into the surface a bit,
 * in varying amounts.
 */
std::unordered_map<std::string, double> model_height = { {
    "piston_rod_part_red", 0.0065 },
    { "piston_rod_part_green", 0.0065 }, { "piston_rod_part_blue", 0.0065 }, {
        "pulley_part_red", 0.07 }, { "pulley_part_green", 0.07 }, {
        "pulley_part_blue", 0.07 }, { "gear_part_red", 0.012 }, {
        "gear_part_green", 0.012 }, { "gear_part_blue", 0.012 }, {
        "gasket_part_red", 0.02 }, { "gasket_part_green", 0.02 }, {
        "gasket_part_blue", 0.02 }, { "disk_part_red", 0.023 }, {
        "disk_part_green", 0.02 }, { "disk_part_blue", 0.023 } };
