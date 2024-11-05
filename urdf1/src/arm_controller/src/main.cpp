#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


int main(int argc, char** argv) {
   ros::init(argc, argv, "joint_state_publisher");
   ros::NodeHandle nh;
   ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
   ros::Rate loop_rate(10);


   while(ros::ok()){
       sensor_msgs::JointState joint_state;
       joint_state.header.stamp = ros::Time::now();
       joint_state.name.resize(3);
       joint_state.position.resize(joint_state.name.size());
       joint_state.name = {"Q_to_base", "A_to_base", "K_to_base"};


       joint_state.position = {0, 0, 0}; //in radians | hip, shoulder, elbow
       joint_state_pub.publish(joint_state);
       printf("joint state published\n");


       ros::spinOnce();
       loop_rate.sleep();
   }
   return 0;
}
