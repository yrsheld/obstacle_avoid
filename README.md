# Obstacle Avoid

Train a robot (Turtlebot3) to avoid obstalces and navigate its way based on laser scan data.

## Requirement
### For turtlebot3_model
* turtlebot3_description
### For NN model
* tensorflow (version: 1.8.0 for Ubuntu 16.04)

## Tasks
### 1. Collect data
* Run gazebo. Can collect data in another world other than the final testing environment (i.e., `turtlebot3_stage_2.world`)
  ```
  roslaunch obstacle_avoid gazebo.launch
  ```
* Record laser scan data & command velocity and write to data/record.csv
  ```
  python data_recorder.py
  ```
* Control the robot manually and try to avoid obstacle along the way
  ```
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```

### 2. Train the motion controller
* Train NN model and save
  * input: laser scan data (90-dim) (45~135 degree)
  * output: velocity linear_x, angular_z
  ```
  python train.py
  ```

### 3. Apply the motion controller
* Run gazebo
  ```
  roslaunch obstacle_avoid gazebo.launch
  ```
* Run motion controller, which publishes to 'cmd_vel'
  ```
  roslaunch obstacle_avoid controller.launch
  ```
## Result
<img src=demo.gif height="300">