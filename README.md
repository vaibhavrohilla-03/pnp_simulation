# UR5 PnP Simulation MUJOCO

## PnP check list

- [x] set main simulation pipeline
- [x] write fk class and equate results with mj_forward
- [x] combine gripper and arm make it reproducible
- [x] write ik for loop drive arm through it
- [x] make  OSC class drive arm through it
- [ ] Make a parent actuator class for ik and osc, invoke that instead of making general if's
- [ ] implement factory class to instantiate classes at runtime
- [x] Implement Splines for trajectory
- [ ]  class to identify and cache  singularities at runtime 
- [x] generalize the project into namespaces, app, renderer etc ...
- [ ] make a class defining the robot model itself and methods to instantiate at runtime (I don't know if its necessary to implement this but looking to build this just to replace different grippers at runtime)
- [ ] a gripper controller class
- [ ] fix twist errors with respect to singularities
- [ ] implement a finite state machine (Trajectory manager for the distributing actions)
- [ ] have to implement smooth interpolation too for rotations - slerp 
- [ ] path planner implementation