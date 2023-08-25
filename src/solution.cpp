#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace std;

double SPEED = 1;          //Meters per Second. Try from -3 to 3
double TURN_RATE = -1;      //radians per second.  Try from -3 to 3
int DURATION = 2;           //seconds to apply velocity command. Try from 1 to 5



int main(int argc, char **argv)
{
    //register node named turtle_go with roscore, then get a nodehandle
    ros::init(argc, argv, "turtle_go");
    ros::NodeHandle node;


    //Register our node with roscore as publisher of the message type
    //geometry_msgs::Twist under the topic name "turtle1/cmd_vel"
    //Notice that ROS convention has us using lower case letters and underscores for topic names
    ros::Publisher pubVelocity =
        node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);


    //declaring ros message object of the type  we need to publish
    //here this is just our variable name, and we use C++ naming convention
    geometry_msgs::Twist cmdVel;

    //initialize the data members in our variable to all zeroes
    cmdVel.linear.x = 0;
    cmdVel.linear.y = 0;
    cmdVel.linear.z = 0;
    cmdVel.angular.x = 0;
    cmdVel.angular.y = 0;
    cmdVel.angular.z = 0;



    //set the frequency that you want the loop below to execute at
    ros::Rate loop_rate(10); //10 cycles per second

    //just an integer for counting cycle of the loop
    int i =0;

    //our loop rate is 10 times/second, so if we oop 10 times our desired duration in seconds
    while (i++ < DURATION*10)
    {

        //Set our desired speed (AKA linear velocity). The x direction is forward on our robot, 
        //and our robot does not have a mechanism to move in the y (right/left) direction
        //nor the z (up/down) direction, so linear.y and linear.z are ignored
        //by both the turtle and our robot
        cmdVel.linear.x = SPEED;

        //Set our desired turn rate (also AKA angular velocity). 
        //The turtle and our ground robot can only turn about the z axis, that runs vertically. 
        //Both the turle and our robot ignore the values in the x (pitch forward/backward)
        //and the y (roll right/left) data members.
        cmdVel.angular.z = TURN_RATE;

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

    //exit program
    return 0;
}