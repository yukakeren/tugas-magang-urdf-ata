#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Rate loop_rate(30); 

    std::vector<std::vector<double>> joint_positions = {
        //A
        {-1.57, -0.34, 0.27, 0.2},
        {-1.25, -1.19, 0.65, 0.46},
        {-1.00, -0.39, 0.27, 0.2},
        {-1.57, -0.48, 0.33, 0.14},
        {-1.00, -0.48, 0.33, 0.14},
        //T
        {-0.89, -0.78, 1.04, -0.39},
        {-0.89, -0.05, -0.72, 0.72},
        {-1.25, -0.05, -0.72, 0.72},
        {-0.50, -0.05, -0.72, 0.72},
        //A
        {-0.50, -0.34, 0.27, 0.2},
        {-0.25, -1.19, 0.65, 0.46},
        {-0.00, -0.39, 0.27, 0.2},
        {-0.50, -0.48, 0.33, 0.14},
        {-0.00, -0.48, 0.33, 0.14},
        //K
        {0, -0.52, 0.80, -0.31},
        {0, -0.05, -0.72, 0.72},
        {0, -0.21, -0.07, 0.29},
        {0.35, -0.05, -0.72, 0.72},
        {0, -0.21, -0.07, 0.29},
        {0.35, -0.52, 0.80, -0.31}
    };

    size_t current_position = 0; 
    size_t next_position = 1;     

    double interpolation_duration = 1.0;  
    double time_step = 0.0;  

    while (ros::ok()) {
        
        time_step += 1.0 / 30.0;  

        
        std::vector<double> current_pos = joint_positions[current_position];
        std::vector<double> target_pos = joint_positions[next_position];

        std::vector<double> interpolated_position(4);  
        for (size_t i = 0; i < 4; ++i) {
            
            interpolated_position[i] = current_pos[i] + (target_pos[i] - current_pos[i]) * (time_step / interpolation_duration);
        }

        if (time_step >= interpolation_duration) {
            time_step = 0.0;  
            current_position = next_position;
            next_position = (next_position + 1) % joint_positions.size();  
        }

        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = {"hip", "shoulder", "elbow", "wrist"};
        joint_state.position = interpolated_position;
        joint_state_pub.publish(joint_state);

        ROS_INFO_STREAM("Publishing joint state: "); 

        ros::spinOnce();
        loop_rate.sleep();  
    }

    return 0;
}