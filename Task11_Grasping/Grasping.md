<h1 align="center"> Grasping </h1>

For grasping objects, there are several considerations, such as End Effector Design, Grasp generation, and Grasp execution. Grasping generally happens in 2 steps: first, you calculate a grasp for your object and then use some control approach to execute the grasp. Depending on your application, a simple or complex approach can be used for each.


## End Effector Design ##

There are many designs of rigid and compliant end effector designs, usually highly specialised for a particular task.  For example, a gripper designed for picking up fruit will likely be compliant and low force consisting of individual "fingers", compared to a vacuum suction end effector for picking up boxes with flat sides.  The mechanical design is a free choice, and should be informed by the tasks being undertaken.

The most common gripper design is the two-finger type, typically a parallel motion or simple "pinching" design.  Most commonly, two fingers move sympathetically so the object is centered within the jaws (see the image below of the Robotiq 2F series gripper). Another common design is the suction cup gripper and the three-finger gripper (image below). However, the gripper design is entirely open to customization.

<img title="Robotiq 2F Series Parallel Gripper"  src="../Images/EndEffectors/RobotiqParallelGripper.jpg"  width=40% height=auto>
<img title="Robotiq Vacuum Grippers"  src="../Images/EndEffectors/suction_gripper.png"  width=40% height=auto>
<img title="Robotiq 3-Finger Adaptive Robot Gripper"  src="../Images/EndEffectors/3_finger_gripper.jpg"  width=40% height=auto>

## Grasping ##

Packages such as MoveIt have some built in [grasping utilities](https://moveit.picknik.ai/humble/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html) such as simple pick and place. These packages can do both the grasp generation and excution step but they may be limited in scope for your needs.
You can also separately calcualte your grasp then use MoveIt to control your robot to graps the objects. 
### Grasp Genreation ###
There are many apporches to genreate grasp that you can use to grasp your objects.For simple objects you may wish to just store a set of precalculate grasps for your objects then exucte one the grasps at run time when you discover the object. 
