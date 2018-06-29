# ParrotDrone_actionsController1
A ROS package and simulation to taking pictures and going forward using actions communication. I use Gazebo, Parrot Drone model, ROS

For  first simulation you have to be in Robot Ignite Academy.

In console:\
	rosrun my_action_pkg ardrone_call_action_getstate_node.py\
then ardrone will start to fly and take photos each second

The second exercise is the Fibonacci Serie server and client:\
	rosrun my_Action_pkg fibonacci_action_server_node.py\
then:\
	rostopic pub /fibonacci_as/goal actionlib_tutorials/FibonacciActionGoal TAB-TAB

TAB-TAB button in your keyboard to autocompletition


