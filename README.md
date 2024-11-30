Added docker for simulating a robotic dog from the Volt brothers. 
Have created during 3 dayas hackaton so maybe there are a lot of bugs... 

Change path in 77 line in Dockerfile.

Make ``` xhost +local:docker ``` for gui

Make ```  docker build ~/path_to_project/mors_sim -t noetic ``` for build

Make ``` docker run -it --rm --env="DISPLAY" --volume="/home/path_to_project/mors_sim/:/home/path_to_project/mors_sim/"  -v /tmp/.X11-unix:/tmp/.X11-unix noetic:latest ``` for start

Have tested in ubuntu 24:
``` roslaunch mors bringup_sim.launch ```

![image](https://github.com/user-attachments/assets/178f15e5-892c-4d68-92f5-f0831d643206)

``` rosservice call /robot_action "cmd: 6.0" ```

![image](https://github.com/user-attachments/assets/e130d4cf-44c2-448d-b326-e3dc1561e177)

original pkg:

https://github.com/voltdog/mors_base

https://github.com/voltdog/mors_pc

https://github.com/loco-3d/whole_body_state_msgs.git
