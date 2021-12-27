# Introduction

loadcell包的主要功能：读取传感器的数据，并转换为力传感数据，发布为话题，并记录csv文件

# Source code

src/loadcell-multi-pub.cppread.cpp 读取三维力传感器数据的脚本，需要参数输入
src/loadcell-pub1.cpp 可能是单轴力传感器

# How to use

在工作空间下，

> source devel/setup.bash
>
> roslaunch loadcell load-cell-4-input.launch


# Notice

如果使用的是对应的三维力传感器，相关的参数需要在launch文件中修改。

```
        <param name = "cellID" type="int" value = "1"/> <!--力传感器的ID，自定义即可-->
        <param name = "SN" type="int" value = "410183"/> <!--电桥的SN码，这个需要与电桥完全对应-->
        <param name = "kx" type="double" value = "1.023"/> <!--力传感器X轴灵敏度-->
        <param name = "ky" type="double" value = "0.946"/> <!--力传感器Y轴灵敏度-->
        <param name = "kz" type="double" value = "1.000"/> <!--力传感器Z轴灵敏度-->
        <param name = "fileName" type="str" value = "/home/roma/roma-small-rover/src/loadcell/data/$(arg posture)"/> <!--传感器数据存储位置，arg是调整文件的前缀，与工况相关联-->
```
