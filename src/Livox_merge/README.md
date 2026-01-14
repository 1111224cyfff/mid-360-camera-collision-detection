# Livox Multi-LiDAR and IMU Fusion Package

## Overview

This package provides real-time multi-LiDAR point cloud fusion and IMU data integration specifically designed for Livox sensors. It combines multiple LiDAR point clouds into a unified coordinate system and publishes time-sorted IMU data from multiple sensors.

## Features

- **Multi-LiDAR Fusion**: Combines point clouds from multiple Livox LiDAR sensors
- **Dual Format Output**: Publishes both PointCloud2 and Livox CustomMsg formats
- **IMU Integration**: Time-sorted IMU data from multiple sensors in unified coordinate system
- **Real-time Performance**: 20Hz synchronization with OpenMP acceleration
- **Automatic Detection**: Supports both PointCloud2 and Livox CustomMsg input formats
- **Coordinate Transformation**: Transforms all sensor data to a unified body frame

## Package Structure

```
livox_merge/
├── src/
│   └── MergeLidar.cpp          # Main fusion implementation
├── launch/
│   ├── livox_merge.launch      # Full system launch with RViz
│   └── livox_merge_simple.launch # Basic fusion only
├── scripts/
│   ├── test_fusion_system.py   # System monitoring script
│   └── test_imu_time_order.py  # IMU time order verification
├── config/
│   └── livox_merge_config.yaml # Configuration parameters
├── rviz/
│   └── livox_merge.rviz        # RViz visualization config
└── README.md
```

## Dependencies

- ROS (tested with ROS Melodic/Noetic)
- PCL (Point Cloud Library)
- Eigen3
- OpenMP
- livox_ros_driver
- sensor_msgs
- geometry_msgs

## Building

1. Clone this package to your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone <repository_url> livox_merge
```

2. Install dependencies:
```bash
rosdep install --from-paths . --ignore-src -r -y
```

3. Build the package:
```bash
cd ~/catkin_ws
catkin_make
# or
catkin build livox_merge
```

## Usage

### Basic Usage

1. Launch the basic fusion node:
```bash
roslaunch livox_merge livox_merge_simple.launch
```

2. Launch with full visualization:
```bash
roslaunch livox_merge livox_merge.launch
```

### Configuration

Edit the launch files to configure:

- **LiDAR Topics**: Input point cloud topics
- **IMU Topics**: Input IMU data topics
- **Extrinsics**: Transformation matrices for each sensor
- **Output Topics**: Remapped output topic names

Example configuration:
```xml
<rosparam param="lidar_topic">[
    "/livox/lidar_192_168_10_110",
    "/livox/lidar_192_168_11_125"
]</rosparam>

<rosparam param="imu_topic">[
    "/livox/imu_192_168_10_110", 
    "/livox/imu_192_168_11_125"
]</rosparam>
```

### Output Topics

- `/merged_pointcloud` - Merged point cloud (PointCloud2)
- `/merged_livox` - Merged point cloud (Livox CustomMsg)
- `/merged_imu` - Time-sorted IMU data (sensor_msgs/Imu)

### Monitoring

Use the provided monitoring scripts:

```bash
# Monitor overall system performance
rosrun livox_merge test_fusion_system.py

# Check IMU time ordering
rosrun livox_merge test_imu_time_order.py
```

## Coordinate Frames

- **Input**: Individual LiDAR and IMU coordinate frames
- **Output**: Unified "body" coordinate frame
- **Transformation**: 4x4 homogeneous transformation matrices

## Performance

- **Target Frequency**: 20Hz synchronization
- **Processing**: Multi-threaded with OpenMP
- **Buffer Management**: Automatic overflow protection
- **Memory**: Efficient point cloud handling

## Calibration

1. **LiDAR Extrinsics**: Define transformation from each LiDAR frame to body frame
2. **IMU Extrinsics**: Define transformation from each IMU frame to body frame
3. **Time Synchronization**: Ensure all sensors are time-synchronized

## Troubleshooting

### Common Issues

1. **No data received**: Check topic names and sensor connectivity
2. **Out-of-order IMU**: Verify time synchronization between sensors
3. **Poor performance**: Adjust sync_frequency or buffer sizes
4. **Memory issues**: Reduce buffer sizes or point cloud density

### Debug Commands

```bash
# Check topics
rostopic list | grep livox

# Monitor data rates
rostopic hz /livox_merge/merged_pointcloud

# Check message content
rostopic echo /livox_merge/merged_imu
```

## Example Launch Configuration

```xml
<launch>
    <node pkg="livox_merge" type="merge_lidar" name="livox_merge_node">
        <rosparam param="lidar_topic">
            ["/livox/lidar_1", "/livox/lidar_2"]
        </rosparam>
        <rosparam param="imu_topic">
            ["/livox/imu_1", "/livox/imu_2"]
        </rosparam>
        <!-- Add extrinsics here -->
    </node>
</launch>
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contact

For questions or support, please open an issue on the repository.
merge multiple livox lidar and IMU just like one lidar
