# Recording Camera Startup

The imitation recorder requires the wrist camera topic first:

- Required wrist topic: `/right_wrist_camera/image_raw`
- Optional wrist info topic: `/right_wrist_camera/camera_info`
- Optional head topic: `/head_camera/color/image_raw`
- Alternate RealSense topic seen on some setups: `/camera/camera/color/image_raw`

Camera defaults live in [configs/imitation/cameras.yaml](/home/arthur/humanoid/configs/imitation/cameras.yaml:1).

## List camera devices

```bash
v4l2-ctl --list-devices
ls -l /dev/video*
```

Use the correct `/dev/videoN` path for the wrist USB camera before launching the recorder.

## Start wrist USB camera only

```bash
ros2 launch robot_imitation_pipeline start_recording_cameras.launch.py \
  use_wrist_camera:=true \
  wrist_video_device:=/dev/video0 \
  use_head_camera:=false
```

The launch file prefers `v4l2_camera`, then `usb_cam`, and falls back to the local OpenCV publisher if neither ROS driver is installed.

## Start wrist + RealSense

```bash
ros2 launch robot_imitation_pipeline start_recording_cameras.launch.py \
  use_wrist_camera:=true \
  wrist_video_device:=/dev/video0 \
  use_head_camera:=true \
  head_camera_namespace:=head_camera
```

If `realsense2_camera` is installed and the device is available, the head camera node starts as an optional source.

## Check topics

```bash
ros2 topic list | grep image
ros2 topic hz /right_wrist_camera/image_raw
python3 tools/check_camera_topics.py
```

If the RealSense publishes on the alternate namespace, the diagnostic script also accepts `/camera/camera/color/image_raw`.

## RealSense failure

If the RealSense does not start, continue with the wrist camera only:

```bash
ros2 launch robot_imitation_pipeline start_recording_cameras.launch.py \
  use_wrist_camera:=true \
  wrist_video_device:=/dev/video0 \
  use_head_camera:=false
```

This keeps `/right_wrist_camera/image_raw` available so the recorder can still run.
