<h1 align="center"> Grasping </h1>

## End Effector Design ##

There are many designs of rigid and compliant end effector designs, usually highly specialised for a particular task.  For example, a gripper designed for picking up fruit will likely be compliant and low force consisting of individual "fingers", compared to a vacuum suction end effector for picking up boxes with flat sides.  The mechanical design is a free choice, and should be informed by the tasks being undertaken.

The most common gripper design is the two finger type, typically a parallel motion or simple "pinching" design.  Most commonly, two fingers move sympathetically so the object is centered within the jaws (see the image below of the Robotiq 2F series gripper).  However, the design of gripper is entirely open for customisation.

<img title="Robotiq 2F Series Parallel Gripper"  src="../Images/EndEffectors/RobotiqParallelGripper.jpg"  width=40% height=auto>

## Grasping ##

Packages such as MoveIt have some built in [grasping utilities](https://moveit.picknik.ai/humble/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html) such as simple pick and place, but they may be limited in scope for your needs.

There are algorithms available for grasping techniques which you may choose to implement.