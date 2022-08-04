
# Motor_controller node for the Ethercat application

------------

### basic information

1.The code is written in the ros neotic environment(20.04)

http://wiki.ros.org/noetic/Installation/Ubuntu

### How to launch

 Every command should be implemented in the Super user status

 0. ifconfig -> to find your device and change the name of the device in the "node_motor_controller.hpp"
 1. sudo su
 2. roscd motor_controller
 3. rosrun motor_controller motor_controller

### Debugging

-1 slave founded! -> This means your slave has not been found

 1 slave founded! -> This means your slave has been found -> You are ready to Go


