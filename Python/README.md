# UIO_CACC
 Estimate Cycber-attack for CACC
https://youtu.be/CjGddM6lK9Y
![fig1](carla_d.png)

CARLA simulation using PYTHON :


We use version CARLA_0.9.13 !!!!! <br>
Should check how to run first in CARLA docs !!!!<br>

You should copy the file to same folder of your CARLA folder : <br>
 - custom_agent.py + custom_controller.py + custom_local_planner.py in the ...\CARLA_0.9.13\WindowsNoEditor\PythonAPI\carla\agents\navigation <br>
 - agent_control.py  in ...\CARLA_0.9.13\WindowsNoEditor\PythonAPI\examples
 - misc.py  in ...\CARLA_0.9.13\WindowsNoEditor\PythonAPI\carla\agents\tools


HOW to run : <br>
    - Open executable file Carla.exe (or ubuntu)<br>
    - RUN "set SCENARIO_RUNNER_ROOT=D:\PHD\CARLA_0.9.13\scenario_runner" in one terminal<br>
    - RUN : """python scenario_runner.py --scenario FollowLeadingVehicle_1 --reloadWorld""" in the same terminal<br>
    - RUN : """ python agent_control.py """ to run the simulation <br>

HOW to plot the results :

 - Use file "data_carla.m" 



