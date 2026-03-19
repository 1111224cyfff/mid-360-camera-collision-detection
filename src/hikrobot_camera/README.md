# HIKROBOT-MVS-CAMERA-ROS
The ros driver package of Hikvision Industrial Camera SDK. Support configuration parameters, the parameters have been optimized, and the photos have been transcoded to rgb format.
Please install mvs, https://blog.csdn.net/weixin_41965898/article/details/116801491

# Install
```
mkdir -p ~/ws_hikrobot_camera/src
git clone https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS.git ~/ws_hikrobot_camera/src/hikrobot_camera
cd ~/ws_hikrobot_camera
catkin_make
```
# launch run
```
source ./devel/setup.bash 
roslaunch hikrobot_camera hikrobot_camera.launch
```

# camera frame transform
Republish `/hikrobot_camera/rgb/compressed` to a new compressed-image topic with a new `frame_id`, and publish the corresponding static extrinsic transform.
Preferred launch:
```
source ./devel/setup.bash
roslaunch hikrobot_camera camera_frame_transform.launch
```
Enable from the camera launches with:
```
roslaunch hikrobot_camera hikrobot_camera_indoor.launch enable_camera_frame_transform:=true
```

# camera lidar alignment audit
Use this audit node to verify that the republished image frame, the leveled point cloud frame, and the runtime TF chain are mutually consistent.
It does not replace reprojection validation, but it catches the most common frame wiring mistakes immediately.

```
source ./devel/setup.bash
roslaunch hikrobot_camera camera_lidar_alignment_audit.launch
```

The audit checks:
- the image topic frame matches `output_frame`
- the point cloud topic frame matches `expected_reference_frame`
- TF exists from point cloud frame to image frame
- TF exists from `parent_frame` to `output_frame`
- the camera extrinsic transform has a valid rigid-body rotation matrix

Recommended defaults in this workspace:
- `image_topic=/hikrobot_camera/rgb/leveled/compressed`
- `pointcloud_topic=/merged_pointcloud_leveled`
- `parent_frame=body_leveled`
- `output_frame=hikrobot_camera_leveled`

If the audit passes, you have runtime evidence that the image and point cloud are at least attached to the same TF tree with the expected frame semantics. You should still run point-cloud-to-image reprojection to confirm that the extrinsic values themselves are geometrically correct.

# launch run
use rviz subscribe topic： /hikrobot_camera/rgb
```
source ./devel/setup.bash 
roslaunch hikrobot_camera hikrobot_camera_rviz.launch
```
