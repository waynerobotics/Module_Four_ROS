![Module_Four_ROS](https://github.com/hannab8/Module_Four_ROS/assets/83167499/0f166dc5-0e6f-44c5-ab6b-228bce9cabb9)
# Module Four: ROS

## Introduction
In this fourth module, you will be introduced to ROS (Robot Operating System). It's an open source framework that allows us to develop and control the robot using the tools and libraries that it provides. We will show you how to install ROS and begin to make use of it. Please be sure to follow the instructions as they are written. We want to make sure that you are using the same version we use here in the lab.

## ROS Tutorials
You will be directed to the ROS wiki site for this module. Please follow each of the tutorials listed below:

1. [Installing ROS](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
   * In the second part where you choose a "distro", type in **Noetic**.
2. [Create ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
3. [Build your ROS package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
4. [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
5. [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)

## Key Points

1. **Topics:** A communication channel used in the ROS framework to facilitate the exchange of data between different nodes in a robotic system. 
2. **Nodes:** Individual components that perform specific tasks. They communicate with each other by publishing and subscribing to topics.
   * **Publisher:** A node that wants to share some data and creates a topic and publishes messages to it.
   * **Subscriber:** A node that is interested in the type of data that is being published and can subscribe to the topic. They receive the messages published on the topic and can process the information as needed.
         
3. **Messages:**
4. **Packages:** This is a collection of different software components such as nodes, libraries, config files, etc. that are contained within your workspace.

## Example:

In this example we have a package dedicated to moving the robot. Within this package we have two nodes (a publisher and subscriber). This example also provides us with the details on how the nodes communicate with each other.

**Topic:** "/move_robot"

**Message Type:** "geometry_msgs/Twist"

**Publisher Node:** move_control

Description: This node controls how the robot moves using linear and angular velocity commands. It publishes "geometry_msgs/Twist" messages on the "/move_robot" topic.
```
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_control')
pub = rospy.Publisher('/move_robot', Twist, queue_size=10)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    move_cmd = Twist()
    move_cmd.linear.x = 0.2
    move_cmd.angular.z = 0.1
    pub.publish(move_cmd)
    rate.sleep()
```

**Subscriber Node:** movement_execution

Description: This node subscribes to the "/move_robot" topic and uses the messages it receives to control the robot's movement. 
```
import rospy
from geometry_msgs.msg import Twist

def movement_callback(data):
    linear_velocity = data.linear.x
    angular_velocity = data.angular.z

rospy.init_node('movement_execution')
rospy.Subscriber('/move_robot', Twist, movement_callback)
rospy.spin()
```

## Assignment (Originially written by Lloyd :robot:)

1.  Open your terminal:
      - Go to your workspace: cd ~/catkin_ws/src
      - Type in: git clone https://github.com/waynerobotics/Module_Four_ROS.git
      - Go to the cloned repository: cd Module_Four_ROS
      - Create new branch called: <YourName>_Module_Four
      - Checkout your new branch

2.  Open the turtle_go.cpp file (Module_Four_ROS/src/turtle_go.cpp)
      - Make the changes that the file is requesting you to make. (feel free to review to doc file within the package)
      - You can use the solution.cpp file for reference, but try to do it on your own as best as you can first.

3.  When you complete the task, go back to your terminal.
      - Go to your catkin_ws: cd ~/catkin_ws
      - Compile the program: catkin_make (fix any errors that may come up)
      - Open two more terminals (ctrl-t)
      - Terminal 1 - Run roscore: roscore (current terminal)
      - Terminal 2 - Run turtlesim: rosrun turtlesim turtlesim_node
      - Terminal 3 - Run program: rosrun Module_Four_ROS turtle_go

4.  Stage, commit, and push your changes.
      - Commit Message: "Completed Task Four "
  
5.  Go back to the GitHub website, then go to the settings tab and under 'Access' click on 'Collaborators'.
    - Click the green 'Add People' button
    - Type in: hannab8
    - Click 'Add Hanna to this repository'

6.  Click on the code tab once you've committed your changes, then click the green button labeled 'Compare & pull request'. 

7.  Fill in the pull request:
    - Title: Task Four
    - Comment: [ Please write a one-two sentence summary of what you did for this assignment ]
    - Assignees: [ Choose the team lead member who is overseeing modules ]

8.  And now you are done! :sparkles:

## References

Here you will find some additional information on ROS. If you have any suggestions on what should be added, please let a team lead know!

1. [Topics](http://wiki.ros.org/Topics)
2. [Nodes](http://wiki.ros.org/Nodes)
3. [Messages](http://wiki.ros.org/Messages)
4. [Command Cheat Sheet](https://mirror.umd.edu/roswiki/attachments/de/ROScheatsheet.pdf)
