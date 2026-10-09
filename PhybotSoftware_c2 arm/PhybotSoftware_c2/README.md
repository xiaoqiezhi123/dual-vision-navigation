# PhybotSofware

Version 0.0.2  

1. put all sensor file into "device" file
2. adjust the code structure， datapackage save all model and data.
3. use tinyfsm change state.
4. add minimal example.
5. use yaml replace Qjson
6. put all thirdparty library in "ThridParty"
7. use pinocchio and create model from urdf directly


Version 0.0.3 
1. add tinyfsm example in "StateMachine"
2. add "RobotStart",  include "realrobot", "mujoco_sim" now
3. add auto_build.sh
4. add spdlog

Version 0.0.4
1. add phybot xml model 


Version 0.0.5
1. add stand wbc mujoco sim
2. add hunter robot model
3. edit statemchine use map


Version 0.0.6
1. add ros in cmake and package.xml


Version 0.0.7
1. add a mjcf/xml robot model with simplified collision mesh


Version 0.1.0
1. add phybot wbc stand sim
2. add state estimator
3. edit state machine


Version 0.1.1
1. add some wbc task
2. correct kalman bug
3. add yaml config of wbc


Version 0.1.2
1. use statemachine choose wbc


Version 0.1.3
1. edit xml to add friction

Version 0.1.4
1. add BasicFunction
2. add GaitPlanner
3. add some wbc task
4. add some function of state estimator

Version 0.1.5
1. add running gait
2. add joystick
3. change urdf and xml for arm
4. add GaitPlanner of the arm

Version 0.1.6
1. add libtorch ans install.sh

Version 0.2.0
1. add gazebo sim

Version 0.2.1
1. change robot model

Version 0.2.2
1. add realRobot test


Version 1.0.0

1. realrobot test ok
2. optimize fsm


Version 1.0.1

1. delete webots and other


Version 1.0.2

1. add yaw friction cone 

Version 1.0.3

1. edit forceref task

Version 1.0.4

1. update real robot params


Version 1.0.5

1. update gazebo_sim
2. add rl_deploy


Version 2.0.0

1. rl_deploy ok
2. every part has different pd


Version 2.0.1

1. optimize statemachine


Version 2.1.0

1. update libtorch 
2. only has  realrobot and mujoco_sim

Version 2.1.1

1. update RL_deploy 