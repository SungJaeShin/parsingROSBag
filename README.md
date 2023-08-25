# parsing ROSBag files with sync data
### Input Data

(1) sensor_msgs::CameraInfo
- /camera/camera/color/camera_info
- /camera/camera/depth/camera_info
- /camera/camera/infra1/camera_info
- /camera/camera/infra2/camera_info <br>
  
(2) sensor_msgs::CompressedImage 
- /camera/camera/color/image_raw/compressed
- /camera/camera/infra1/image_rect_raw/compressed
- /camera/camera/infra2/image_rect_raw/compressed <br>

(3) sensor_msgs::Image
- /camera/camera/depth/image_rect_raw <br>

---
### Output Data
(1) sensor_msgs::Image
- /sync_color
- /sync_depth
- /sync_infra1
- /sync_infra2

(2) sensor_msgs::CameraInfo
- sync_infra1_info
- sync_infra2_info

---


