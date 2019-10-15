# Harmonious sampling

You can see information about this code through this webcite.(We uploaded our paper and video.) </br>
https://sglab.kaist.ac.kr/HarmoniousSampling/</br>

1. Install the dependencies.

<pre> MoveIt!, OpenRAVE, OMPL </pre> 

2. Setup your robot.

<pre> roslaunch moveit_setup_assistant setup_assistant.launch </pre>
  For example, we make three groups.
   - PLANNING_GROUP: base(3-DoF) + right_arm(7-DoF)
   - BASE_GROUP: base(3-DoF)
   - MANI_COLL_CHECK_GROUP: all links without rigth_arm(7-DoF). This is for collision checking to identify manipulation regions.
   - If you don't use PR2 robot, you have to add your robot to OpenRAVE. Change the "robot.xml" file in the "script" folder.

3. Execute the code.
  - Run the "move_group.launch" that is set up for your robot.
  - Run the python file "possible_confs.py": <pre> python possible_confs.py </pre>
    It needs a lot of time to construct the inverse kinematics and inverse reachability databases for your robot.
  - Run the planning code.
    <pre> rosrun harmonious_sampling main </pre>
    - You should change the "main.cpp" according to your settings. (e.g., bound setting, start configuration, target grasp pose ...)
    - You can change the scene. "Scene.cpp" is an example. So, you should change it.

4. You can change the parameters in the "HarmoniousParameters.cpp" file.
