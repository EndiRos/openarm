// Copyright 2025 Enactic, Inc.
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

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>

#include "../../damiao_motor/dm_motor.hpp"
#include "../../damiao_motor/dm_motor_device_collection.hpp"

namespace openarm::can::socket {

class GripperComponent : public damiao_motor::DMDeviceCollection {
public:
    GripperComponent(canbus::CANSocket& can_socket);
    ~GripperComponent() = default;

    void init_motor_device(damiao_motor::MotorType motor_type, uint32_t send_can_id,
                           uint32_t recv_can_id, bool use_fd);

    // Gripper-specific controls
    void open(double kp = 50.0, double kd = 1.0);
    void close(double kp = 50.0, double kd = 1.0);
    damiao_motor::Motor* get_motor() const { return motor_.get(); }

private:
    std::unique_ptr<damiao_motor::Motor> motor_;
    std::shared_ptr<damiao_motor::DMCANDevice> motor_device_;

    void set_position(double position, double kp = 50.0, double kd = 1.0);

    // The actual physical gripper uses a slider cranker-like mechanism, this mapping is an
    // approximation.
    double gripper_to_motor_position(double gripper_position) {
        // Map gripper position (0.0=closed, 1.0=open) to motor position
        return (gripper_position - gripper_open_position_) /
                   (gripper_closed_position_ - gripper_open_position_) *
                   (motor_closed_position_ - motor_open_position_) +
               motor_open_position_;
    }

    double motor_to_gripper_position(double motor_position) {
        // Map motor position back to gripper position (0.0=closed, 1.0=open)
        return (motor_position - motor_open_position_) /
                   (motor_closed_position_ - motor_open_position_) *
                   (gripper_closed_position_ - gripper_open_position_) +
               gripper_open_position_;
    }

    // Gripper configuration
    double gripper_open_position_ = 1.0;
    double gripper_closed_position_ = 0.0;
    double motor_open_position_ = -1.0472;  // 60 degrees
    double motor_closed_position_ = 0.0;
};

}  // namespace openarm::can::socket
