# UIO_CACC
 Estimate Cycber-attack for CACC
https://youtu.be/CjGddM6lK9Y
![fig1](carla_d.png)

CARLA simulation using PYTHON :

We use version CARLA_0.9.13 !!!!! 
Should check how to run first in CARLA docs !!!!

You should copy the file to same folder of your CARLA folder :
 - custom_agent.py + custom_controller.py + custom_local_planner.py in the ...\CARLA_0.9.13\WindowsNoEditor\PythonAPI\carla\agents\navigation
 - agent_control.py  in ...\CARLA_0.9.13\WindowsNoEditor\PythonAPI\examples



HOW to run :
    - Open executable file Carla.exe (or ubuntu)
    - RUN "set SCENARIO_RUNNER_ROOT=D:\PHD\CARLA_0.9.13\scenario_runner" in one terminal
    - RUN : """python scenario_runner.py --scenario FollowLeadingVehicle_1 --reloadWorld""" in the same terminal
    - RUN : """ python agent_control.py """ to run the simulation 

HOW to plot the results :

 - Use file "data_carla.m" 



