# manipulation
### Demo for opening door

Robot should be localized first w.r.t map - laucnh full-version rviz

```
viz_full
```

Run handle_localization sensor 

```
roslaunch handle_detector localization_sensor.launch
```

Run ros service server to find door handle

```
rosrun handle_tracking scanner
```

Run ros service client && opening door code

```
rosrun manipulation_tutorials open_hinged_door_final.py
```
