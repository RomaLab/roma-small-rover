# Introduction

wheel-camera包的主要功能：使用opencv打开多个车轮相机录像

# Requirement

opencv库

# How to use

在工作空间下，

> source devel/setup.bash
>
> roslaunch wheel-camera cv_camera.launch

# Notice

launch文件配置参考：

```
        <param name = "cameraID" type="int" value = "0"/> <!--摄像头的ID，摄像头的固有标签-->
        <param name = "frameWidth" type="double" value = "320.0"/> <!--摄像头的宽度分辨率-->
        <param name = "frameHeight" type="double" value = "240.0"/> <!--摄像头的高度分辨率-->
        <param name = "frameFPS" type="double" value = "30.0"/> <!--摄像头的帧率-->
        <param name = "fileName" type="str" value = "/home/roma/roma-small-rover/src/wheel-camera/data/$(arg posture)"/> <!--视频数据存储位置，arg是调整文件的前缀，与工况相关联-->
```
