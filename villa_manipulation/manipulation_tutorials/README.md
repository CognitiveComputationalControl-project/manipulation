# manipulation_tutorials

This package is modified from the [hsrb\_controller\_tutorials](https://docs.hsr.io/manual_en/development/ros_interface.html) to test the out-of-box manipulation API for HSR.


## Pickup object sample from Kazuto

### Terminal 1

   ```bash
   $ roslaunch villa_manipulation hsrb_empty_world_with_small_objects.launch
   ```

### Terminal 2

   ```bash
   $ rosrun villa_manipulation pickup_bottle.py
   ```

## Open hinged door sample from Kazuto

### Terminal 1

   ```bash
   $ roslaunch villa_manipulation hsrb_empty_world_with_hinged_door.launch
   ```

### Terminal 2

   ```bash
   $ rosrun hvilla_manipulation open_hinged_door.py
   ```
   
## Gazed point fixing

This is a node for fixing gazed point of HSR.

### Usage

0. Turn off viewpoint_controller if necessary

    ```bash
    $ rosservice call /viewpoint_controller/stop
    ```

1. Execute a node

    ```bash
    $ rosrun villa_manipulation gazed_point_fixing_node
    ```

2. Set the target point from /odom frame(in another terminal)

    ```bash
    $ rostopic pub /gazed_point_fixing_node/target_point geometry_msgs/Point "x: 0.0
                                                                              y: 0.0
                                                                              z: 1.0"
    ```

3. Activate the function

    ```bash
    $ rostopic pub /gazed_point_fixing_node/activate std_msgs/Bool "data: true"
    ```

4. You can change the target point and activation status dynamically using above topics.
