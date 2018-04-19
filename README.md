# manipulation
## Demo for opening door

Robot should be localized first w.r.t map - laucnh full-version rviz

```
viz_full
```
### Call handle & manipulation service server

```
roslaunch villa_manipulation open_door_task.launch
```

### Call waypoint navigation service server

```
roslaunch villa_navi_service waypoint.launch
```


## Call client for service

```
rosrun villa_manipulation nav_open_door_client.py
```




# Individual Service


Run handle_localization sensor 

```
roslaunch handle_detector localization_sensor.launch
```

Run ros service server to find door handle

```
rosrun handle_tracking scanner
```

Run ros service server for opening_door

```
rosrun villa_manipulation open_door_service.py
```

Run ros service client && opening door code

```
rosrun manipulation_tutorials open_hinged_door_final.py
```
