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

(2) Save Sync Color Image (.jpg)
- If you want to save .png extension image, please go to saveSyncImgs funtion and change .jpg to .png !!
- Default saved image is **3 FPS** in rosbag time !!
  - If you want to save more images, then please change double value in `time_diff` statement ! (in this code, setting 0.33)
  
---


