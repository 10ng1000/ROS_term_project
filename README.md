# README #

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=bitbucket_leggedrobotics/smb_common/master)](https://ci.leggedrobotics.com/job/bitbucket_leggedrobotics/job/smb_common/job/master/)

使用了SMB包

实现了: 
1. 基于激光雷达的避障.
2. ~~围绕物体做圆周运动~~.

## 运行方法

编译整个包后，运行
```
roslaunch smb_highlevel_controller smb_highlevel_controller.launch
```

## 节点函数说明

### readParameter
``` cpp
bool readParameters();
```
读参数文档
****
### scanCallback
``` cpp
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
```
订阅LaserScan话题的回调函数，这个函数中也执行了其他功能函数，比如`findPillar`，`startCircularMotion`，`obstacleAvoidance`。
****
### findPillar
``` cpp
void findPillar(float xpos, float ypos, float angle);
```
将小车向World中的Pillar运动。
****
### startCircularMotion
``` cpp
void startCircularMotion(float startAngle, float augular, float radius);
```
在回调函数中开启圆周运动功能。

### iniCircularMotion
``` cpp
void iniCircularMotion(float angle,float distance, bool &ini, float &fixedRadius);
```
初始化小车的圆周运动，使小车转向正确位置。
****
### ~~circularMotion~~
``` cpp
void circularMotion(float augular, float radius);
```
~~小车开始进行圆周运动。~~
****
### obstacleAvoidance
```cpp
void obstacleAvoidance(float distance, float disMinLeft, float disMinRight, float angleMinLeft, float angleMinRight);
```
在当前回调函数中开启避障功能。
