# TF Tree For `system_bringup/run.launch`

Generated on 2026-03-23 from the current launch chain and source code behavior.

## Active TF Tree

```text
map
└─ odom
   └─ base_link
      ├─ chassis_link
      │  ├─ imu_link
      │  ├─ velodyne
      │  └─ navsat_link
      └─ body
         └─ body_leveled
            └─ hikrobot_camera_leveled
```

## Edge Sources

1. `map -> odom`
   - Published by LIO-SAM IMU preintegration.
   - Current implementation publishes an identity transform.

2. `odom -> base_link`
   - Published by LIO-SAM map optimization.
   - In the current config, `lidarFrame = base_link`, so the odometry child frame is `base_link`.

3. `base_link -> chassis_link`
   - Fixed joint from the robot URDF through `robot_state_publisher`.

4. `chassis_link -> imu_link`
   - Fixed joint from the robot URDF.

5. `chassis_link -> velodyne`
   - Fixed joint from the robot URDF.

6. `chassis_link -> navsat_link`
   - Fixed joint from the robot URDF.

7. `base_link -> body`
   - Static identity transform from `run.launch`.
   - Used to declare merged Livox body frame aligned with `base_link`.

8. `body -> body_leveled`
   - Static transform published by `merged_pointcloud_leveler_node` after one-time IMU leveling.
   - This edge appears only after calibration completes.

9. `body_leveled -> hikrobot_camera_leveled`
   - Static transform published by `camera_frame_transform_node`.
   - It composes the camera extrinsic with the existing `body -> body_leveled` transform.

## Important Notes

- `livox_merge` sets message `frame_id` to `body`, but does not itself connect `body` into the TF tree.
- The connection from the LIO-SAM chain to the merged sensor chain is the static transform `base_link -> body` defined in `run.launch`.
- The camera input frame name is `hikrobot_camera`, but the currently published TF node is `hikrobot_camera_leveled`.
- `M-detector` and `dynamic_tracker` use TF and frame IDs for data processing, but they do not add new TF edges in this launch chain.

## Main Files Used

- `src/system_bringup/launch/run.launch`
- `src/LIO-SAM-MID360/config/paramsLivoxIMU.yaml`
- `src/LIO-SAM-MID360/src/imuPreintegration.cpp`
- `src/LIO-SAM-MID360/src/mapOptmization.cpp`
- `src/LIO-SAM-MID360/launch/include/config/robot.urdf.xacro`
- `src/Livox_merge/src/merged_pointcloud_leveler_node.cpp`
- `src/hikrobot_camera/src/camera_frame_transform_node.cpp`
