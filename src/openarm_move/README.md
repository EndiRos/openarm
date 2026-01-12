# OpenArm Move

## Description

`openarm_move` is a ROS 2 C++ package designed for motion control of OpenArm robotic arms using MoveIt. It provides a high-level interface for planning and executing movements in both unimanual mode (left or right arm) and bimanual mode (both arms simultaneously).

## Main Features

### 🎯 Motion Strategies

The package implements a strategy design pattern with two types of planning:

#### 1. **PTP (Point-to-Point)**
Direct movement between poses in joint space.

- ✅ Fast and efficient
- ✅ Avoids singularities
- ✅ Ideal for movements between defined poses
- ✅ Integrated gripper control

#### 2. **Cartesian** *(In development)*
Linear movement in Cartesian space.

- 📐 Predictable linear trajectories
- 📐 Precise end-effector control
- 📐 Intermediate waypoints

### 🤖 Operation Modes

#### **Unimanual**
- Individual left arm control
- Individual right arm control
- Independent gripper control

#### **Bimanual**
- Synchronized coordination of both arms
- Temporal interpolation between trajectories
- Delay compensation between arms

## Architecture

```
OpenArmMove
├── MotionStrategy (Abstract class)
│   ├── Ptp (Implementation)
│   └── Cartesian (Implementation)
└── Utils (Utilities)
```

### Main Components

#### `OpenArmMove`
Main class that encapsulates motion strategies.

```cpp
OpenArmMove move(node);
move.ptp->Left(target_poses, gripper_value);
move.ptp->Right(target_poses, gripper_value);
move.ptp->Bimanual(left_poses, right_poses, left_gripper, right_gripper);
```

#### `MotionStrategy`
Abstract interface that defines common methods:
- `PlanLeftArm()` - Left arm planning
- `PlanRightArm()` - Right arm planning
- `PlanBimanual()` - Bimanual planning
- `ExecuteLeft()` - Left arm execution
- `ExecuteRight()` - Right arm execution
- `ExecuteBimanual()` - Bimanual execution

#### `Ptp` (Point-to-Point)
Point-to-point motion implementation:

**Main methods:**
```cpp
// Unimanual movement with poses
void Left(const std::vector<geometry_msgs::msg::Pose>& target, float gripper);
void Right(const std::vector<geometry_msgs::msg::Pose>& target, float gripper);

// Movement to named poses
void Left(std::string pose_name);
void Right(std::string pose_name);

// Bimanual movement
void Bimanual(const std::vector<geometry_msgs::msg::Pose>& target_left,
              const std::vector<geometry_msgs::msg::Pose>& target_right,
              float gripper_l, float gripper_r);

void Bimanual(std::string pose_left, std::string pose_right);

// Gripper control
void OpenLeftGripper();
void CloseLeftGripper();
void OpenRightGripper();
void CloseRightGripper();
```

#### `Utils`
Utilidades para interactuar con el robot:
# openarm
```cpp
Utils utils(strategy);

// Información del robot
utils.PrintRobotInfo();

// Obtener poses actuales
auto current_left = utils.GetCurrentPoseLeft();
auto current_right = utils.GetCurrentPoseRight();

// Calcular punto de aproximación
auto approach = utils.AproachPoint(target_pose, 0.1); // 10cm antes

// Debug
utils.PrintPose("Target", pose);
```

## Instalación y Compilación

### Dependencias

- ROS 2 (Humble o superior)
- MoveIt 2
- `openarm_description` - Descripción URDF del robot
- `openarm_bimanual_moveit_config` - Configuración de MoveIt

### Compilación

```bash
cd ~/ros2_ws
colcon build --packages-select openarm_move
source install/setup.bash
```

## Usage

### 1. Launch the Node

```bash
ros2 launch openarm_move move.launch.py
```

This launch file:
- Loads the robot's URDF description (bimanual mode)
- Configures MoveIt with SRDF semantics
- Initializes inverse kinematics (IK)
- Sets `use_sim_time: true` for simulation

### 2. Code Example

```cpp
#include "openarm_move.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("my_application");
    
    // Initialize OpenArmMove
    OpenArmMove move(node);
    
    // --- Example 1: Simple single arm movement ---
    std::vector<geometry_msgs::msg::Pose> target_left;
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.3;
    pose.position.y = 0.2;
    pose.position.z = 0.4;
    pose.orientation.w = 1.0;
    target_left.push_back(pose);
    
    move.ptp->Left(target_left, 0.044); // Gripper open
    
    // --- Example 2: Movement to named pose ---
    move.ptp->Right("home");
    
    // --- Example 3: Bimanual movement ---
    move.ptp->Bimanual("home", "ready");
    
    // --- Example 4: Gripper control ---
    move.ptp->CloseLeftGripper();
    
    // --- Example 5: Using utilities ---
    auto current = move.utils.GetCurrentPoseLeft();
    move.utils.PrintRobotInfo();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 3. Gripper Values

```cpp
#define OPEN 0.044f   // Gripper open (44mm)
#define CLOSE 0.0f    // Gripper closed
```

## API Reference

### `Ptp` Class

| Method | Description | Parameters |
|--------|-------------|------------|
| `Left(poses, gripper)` | Moves left arm to target poses | `poses`: Pose vector, `gripper`: opening (0.0-0.044) |
| `Right(poses, gripper)` | Moves right arm to target poses | `poses`: Pose vector, `gripper`: opening (0.0-0.044) |
| `Left(pose_name)` | Moves left arm to named pose | `pose_name`: name defined in SRDF |
| `Right(pose_name)` | Moves right arm to named pose | `pose_name`: name defined in SRDF |
| `Bimanual(left, right, gl, gr)` | Coordinated bimanual movement | Poses and grippers for both arms |
| `Bimanual(left_name, right_name)` | Bimanual to named poses | SRDF pose names |
| `OpenLeftGripper()` | Opens left gripper | - |
| `CloseLeftGripper()` | Closes left gripper | - |
| `OpenRightGripper()` | Opens right gripper | - |
| `CloseRightGripper()` | Closes right gripper | - |

### `Utils` Class

| Method | Description | Return |
|--------|-------------|---------|
| `GetCurrentPoseLeft()` | Gets current left arm pose | `vector<Pose>` |
| `GetCurrentPoseRight()` | Gets current right arm pose | `vector<Pose>` |
| `AproachPoint(target, dist)` | Calculates approach point | `vector<Pose>` |
| `PrintRobotInfo()` | Prints robot information | void |
| `PrintPose(name, pose)` | Prints pose for debugging | void |

## Configuration

### Planning Parameters

MoveIt parameters are configured in `MotionStrategy`:

- `MaxVelocityScalingFactor`: Velocity scaling factor (0.0-1.0)
- `MaxAccelerationScalingFactor`: Acceleration scaling factor (0.0-1.0)
- `NumPlanningAttempts`: Number of planning attempts
- `PlanningTime`: Maximum planning time (seconds)

### Planning Groups

Defined in SRDF (`openarm_bimanual_moveit_config`):

- `left_arm` - Left arm (arm only)
- `right_arm` - Right arm (arm only)
- `left_gripper` - Left gripper
- `right_gripper` - Right gripper
- `left_arm_full` - Left arm with gripper
- `right_arm_full` - Right arm with gripper
- `bimanual` - Both complete arms

## Advanced Features

### Bimanual Synchronization

The `mergePlansWithInterpolation()` method synchronizes trajectories of both arms:

- Interpolates trajectory points for temporal alignment
- Supports optional delays between arms
- Configurable temporal resolution (`min_dt`)

### Virtual Offset Calculation

For control in virtual space (e.g., with teleoperation):

```cpp
void CalculateOffset(geometry_msgs::msg::Pose target_virtual);
void CalCurrentVirtual();
```

### Transformations

Conversion between coordinate systems:

```cpp
geometry_msgs::msg::Pose EigenToPos(Eigen::Isometry3d eigen);
std::vector<Pose> TF2Vector(const tf2_msgs::msg::TFMessage& tf);
```

## Troubleshooting

### Robot doesn't move
- Verify MoveIt is running: `ros2 topic list | grep move_group`
- Check that URDF is loaded correctly
- Review collisions in RViz

### Planning error
- Increase `NumPlanningAttempts` and `PlanningTime`
- Verify that target pose is reachable (IK solution)
- Review joint limits in URDF

### Bimanual movement not synchronized
- Adjust `min_dt` in `mergePlansWithInterpolation()`
- Verify that both trajectories have valid points

## Contributing

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for contribution guidelines.

## License

Apache License 2.0 - See [LICENSE](../../LICENSE)

## Author

Endika Etxebarrieta (eetxebarrieta@gmail.com)

## References

- [MoveIt 2 Documentation](https://moveit.ros.org/)
- [ROS 2 Documentation](https://docs.ros.org/)
- Based on SRI International examples (BSD License)
