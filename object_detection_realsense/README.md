### Object and 3D pose detection with realsense

##### Start connection with the realsense D435 camera

```
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
```

##### Start the object recognition node
```
source install/setup.bash && rosrun object_detection main
```

##### Start rviz
```
rviz
```
Select `camera_link` as fixed frame.
Then add the elements to show the Camera output, a Pointcloud2 and the detected 3d Position as PointStamped and select the available topics.
Check the terminal where the `object_detection` node is running, to see which objects are detected.


Then the detections should look like this:

![](bottle.png?raw=true)
![](cup.png?raw=true)
![](keyboard.png?raw=true)
![](banana.png?raw=true)

