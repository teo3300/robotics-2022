# Referring to the tutorial on [wiki.ros.org](https://wiki.ros.org/rosbag/Tutorials/reading%20msgs%20from%20a%20bag%20file)
First step extract the topic from all rosbags to understand the data.

---

## Goals:
- [x] compute velocity
  - [x] write formulas
  - [x] adapt formulas to ticks
  - [x] compute v, $\omega$
  - [x] publish v, $\omega$ on `cmd_vel` ot type `geometry_msgs/TwistStamped`
- [x] compute odometry
  - [x] performs discrete integration with Euler and Runge-Kutta
  - [x] automatically set initial position
  - [x] publish on `odom` of type `nav_msgs/Odometry`
  - [x] broadcast TF `odom->base_link`
- [ ] calibrate parameters
- [ ] compute wheel control speeds
  - [x] reverse formulas
  - [x] apply inverse kinematic to `cmd_vel`
  - [x] create custom message
  - [x] public custom message on topic `wheels_rpm`
  - [ ] use `rqt_plot` or `plotjuggler` to check correct speed
- [ ] service to reset odometry to specified pose(x, y, $\theta$)
  - [ ] dynamic reconfigure to select integration method
- [ ] dynamic reconfigure to change integration method
  - [ ] enum with values: `Euler` and `RK` to specify method