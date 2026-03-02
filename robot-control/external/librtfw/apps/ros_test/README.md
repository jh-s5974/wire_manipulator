# ROS Test - RTFW + ROS2 Integration Test

This program tests the integration between the Real-Time Framework (RTFW) and ROS2, extending the `poc_app` example with ROS2 topic exposure. It runs the same core RT tasks as poc_app but exposes the internal RTFW data to ROS2 topics for external monitoring and control.

## Overview

`ros_test` runs the same task pipeline as poc_app:
- **SensorProducer** (1000Hz): Generates synthetic sensor data
- **TorqueController** (1000Hz): PD controller for joint torque commands
- **StateEstimator** (200Hz): Estimates robot state from sensor data
- **VisionProcessor** (30Hz): Generates vision object detections
- **GaitPlanner** (30Hz): Generates walking gait parameters
- **RosBridgeTask** (100Hz, Non-RT): Exposes RTFW data as ROS2 topics

## Dependencies

- RTFW core libraries
- ROS2 (Humble or later recommended)
- rclcpp, std_msgs

## Building

From the workspace root:

```bash
mkdir -p build && cd build
cmake ..
cmake --build . --target ros_test
```

## Running

Make sure ROS2 environment is sourced:

```bash
# Source ROS2 environment
source /opt/ros/<distro>/setup.bash

# Run in LIVE mode
./build/apps/ros_test/ros_test --live

# Run and record RTFW data to blackbox
./build/apps/ros_test/ros_test --record my_recording.bin
```

## Available ROS2 Topics

### Publishers (RTFW → ROS2)

All topics publish `std_msgs/Float64MultiArray`:

- `/sensors/high_freq` - 24 values: [joint_positions(12), joint_velocities(12)]
- `/robot/state` - 6 values: [center_of_mass(3), zero_moment_point(2), is_stable]
- `/motor/torques` - 12 values: [joint_torques]
- `/robot/gait` - 3 values: [step_length, step_height, step_frequency]
- `/vision/object` - 4 values: [object_id, position_x, position_y, position_z]

## Testing the Bridge

In separate terminals:

**Terminal 1**: Start ros_test
```bash
./build/apps/ros_test/ros_test --live
```

**Terminal 2**: Monitor ROS2 topics
```bash
# Watch all topics
ros2 topic list

# Monitor sensor data
ros2 topic echo /sensors/high_freq

# Monitor robot state
ros2 topic echo /robot/state
```

## Architecture

### Task Structure (same as poc_app)

```
SensorProducer (1000Hz)
    ↓ (sensors.high_freq)
    ├→ TorqueController (1000Hz) → motor.torques.target
    └→ StateEstimator (200Hz) ← robot.gait.planned
           ↓ (robot.state.estimated)
    GaitPlanner (30Hz) → robot.gait.planned
    
VisionProcessor (30Hz) → vision.object.detected

          ↓ (All data keys)
      RosBridgeTask (100Hz, Non-RT)
          ↓
      ROS2 Topics
```

### Core Execution Frequencies

- **1000Hz**: Sensor production and control (Core 7)
- **200Hz**: State estimation (Core 6)
- **30Hz**: Vision and gait planning (Core 5)
- **100Hz**: ROS2 bridge (Non-RT thread pool)

## Implementation Notes

1. **RosBridgeTask**: 
   - Runs as a Non-RT task to handle ROS2 async communication without affecting RT guarantees
   - Uses weak dependencies to avoid blocking the RT schedule
   - Calls `rclcpp::spin_some()` each iteration for responsive ROS2 processing

2. **Data Conversion**:
   - Internal RTFW struct → `Float64MultiArray` for ROS2 compatibility
   - Serializes arrays into a single flat list for simplicity

3. **Real-time Safety**:
   - RTFW data is captured by RT tasks in deterministic RT threads
   - Bridge reads updates asynchronously in Non-RT context
   - No cross-contamination between RT and Non-RT scheduling

## Comparing with poc_app

The core difference between ros_test and poc_app:
- **poc_app**: Focuses purely on RTFW data flow and RT scheduling
- **ros_test**: Same RT data flow, but also exposes data via ROS2 topics for external tools

All RT task implementations are identical; only the integration layer differs.

## Extending

To add more data producers/consumers or ROS2 topics:

1. Add task classes to `ros_test_tasks.h` (identical pattern to poc_app)
2. Register tasks in `main()` 
3. Update `RosBridgeTask::setup()` to declare dependencies
4. Add publisher creation in `RosBridgeTask::initialize()`
5. Add data publishing in `RosBridgeTask::execute()`

Example:
```cpp
// Add new data reader in RosBridgeTask
rtfw::rt::DataReader<MyDataType> my_reader_{"my.data.key", rtfw::rt::DependencyType::Weak};

// In execute(), publish when updated:
my_reader_.on_update([this](const MyDataType& data) {
    auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    // Convert data to msg...
    my_pub_->publish(std::move(msg));
});
```

