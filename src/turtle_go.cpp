#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace std;

//TODO: Vary these values to see different behaviors from the turtle
double SPEED = 1;          //Meters per Second. Try from -3 to 3
double TURN_RATE = -1;      //radians per second.  Try from -3 to 3
int DURATION = 2;           //seconds to apply velocity command. Try from 1 to 5



int main(int argc, char **argv)
{
    //register node named turtle_go with roscore, then get a nodehandle
    ros::init(argc, argv, "turtle_go");
    ros::NodeHandle node;

//TODO:
    //Register our node with roscore as publisher of the message type
    //geometry_msgs::Twist under the topic name "turtle1/cmd_vel"
    //Notice that ROS convention has us using lower case letters and underscores for topic names
    ros::Publisher pubVelocity =
        node.advertise<ENTER MESSAGE TYPE HERE>("***ENTER TOPIC NAME IN THESE QUOTES", 1);


//TODO:
    //declare a ros message object of the type we need to publish and call it "cmdVel"
    //here this is just our variable name, and we use C++ naming convention
    ???? cmdVel

//TODO:
    //Look at the message type definition here: https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
    //notice in consists of two other messages (of type geometry_msgs/Vector3)
    //and that you can click on that type to see what those consist of
    //It's just a way to bundle speed commands in all three axes and turn rate commands about all 3 axes
//TODO:
    //initialize the data members in our variable to all zeroes    
    cmdVel.linear.x = ????;
    cmdVel.linear.y = ????;
    cmdVel.linear.z = ????;;
    cmdVel.angular.x = ????;;
    cmdVel.angular.y = ????;;
    cmdVel.angular.z = ????;;


    //set the frequency that you want the loop below to execute at
    ros::Rate loop_rate(10); //10 cycles per second

    //just an integer for counting cycle of the loop
    int i =0;

    //our loop rate is 10 times/second
    //lets loop 10*the number of seconds you want the turtle to go
    while (i++ < DURATION*10)
    {

        //Set our desired speed (AKA linear velocity). The x direction is forward on our robot, 
        //and our robot does not have a mechanism to move in the y (right/left) direction
        //nor the z (up/down) direction, so linear.y and linear.z are ignored
        //by both the turtle and our robot
        cmdVel.linear.x = ????;

        //Set our desired turn rate (also AKA angular velocity). 
        //The turtle and our ground robot can only turn about the z axis, that runs vertically. 
        //Both the turle and our robot ignore the values in the x (pitch forward/backward)
        //and the y (roll right/left) data members.
        cmdVel.angular.z = ????;

        //publish the message that we internally called cmdVel
        pubVelocity.publish(cmdVel);

        //We set the frequency for 10Hz, this sleeps as long as it
        //takes to keep that frequency
        loop_rate.sleep();
    }

    //it is good practice to explicitly finish with an "all stop" command
    //instead of relying on the other node to have a timeout built in
    cmdVel.linear.x = 0;
    cmdVel.angular.z = 0;
    pubVelocity.publish(cmdVel);

    //exit program (aka terminate node)
    return 0;
}