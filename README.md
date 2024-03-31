# Dependencies
```
sudo apt update
sudo apt install ros-<your_ros2_distro>-compressed-image-transport
```

## Study log
### 3/31 : about compressed images
- When save rosbag with mp4(1920x1080, 22.7MB),
    - rgb_raw: 1.6GB, rgb_compressed: 140.8MB, gray_image: 527.5MB, gray_compressed: 130MB
