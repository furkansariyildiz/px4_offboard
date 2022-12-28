#include <px4_offboard/offboard.h>

using namespace std;
using namespace NHTechVision;

PX4OffBoard::PX4OffBoard(ros::NodeHandle& nh):
nh_(nh)
{
    nh_.param<float>("/px4_offboard/OffBoard/rate", _rate, 5.0);
    nh_.param<double>("/px4_offboard/DesiredPosition/Position/x", _desired_pose.pose.position.x, 15.0);
    
    _state_subscriber = nh_.subscribe("/mavros/state", 10, &PX4OffBoard::stateCallback, this);
    _desired_pose_subscriber = nh_.subscribe("/desired_pose", 10, &PX4OffBoard::desiredPoseCallback, this);
    _local_pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    _arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    _set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    _offboard_set_mode.request.custom_mode = "OFFBOARD";
    _arm_command.request.value = true;
    _desired_pose.pose.position.x = 0.0;
    _desired_pose.pose.position.y = 0.0;
}


void PX4OffBoard::stateCallback(const mavros_msgs::State::ConstPtr& message)
{
    _current_state = *message;
}

void PX4OffBoard::desiredPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
{
    _desired_pose = *message;
    ROS_INFO_STREAM("desired position" << _desired_pose);
    _local_pose_publisher.publish(_desired_pose);
}

void PX4OffBoard::waitFCUConnection()
{
    while(ros::ok() && !_current_state.connected)
    {
        ros::spinOnce();
        ros::Rate(_rate).sleep();
        ROS_INFO("%s Trying to connect FCU", LOG_ZONE_OFFBOARD);
    }

    ROS_INFO("%s FCU Connection is created.", LOG_ZONE_OFFBOARD);
}



void PX4OffBoard::sendDesiredPositionToMavros()
{
    for(int i=0; ros::ok() && i > 0; --i)
    {
        _local_pose_publisher.publish(_desired_pose);
        ros::spinOnce();
        ros::Rate(_rate).sleep();
    }
}

void PX4OffBoard::armDrone(ros::Time last_request)
{
    if(_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
        if(_set_mode_client.call(_offboard_set_mode) && _offboard_set_mode.response.mode_sent)
        {
            ROS_INFO("%s Offboard enabled.", LOG_ZONE_OFFBOARD);
        }
        last_request = ros::Time::now();
    }
    else
    {
        if(!_current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if(_arming_client.call(_arm_command) && _arm_command.response.success)
            {
                ROS_INFO("%s Drone has been armed.", LOG_ZONE_OFFBOARD);
            }
            last_request = ros::Time::now();
        }
    }
    
    _local_pose_publisher.publish(_desired_pose);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4_offboard");
    ros::NodeHandle node_handle;
    PX4OffBoard px4_offboard = PX4OffBoard(node_handle);
    px4_offboard.waitFCUConnection();
    px4_offboard.sendDesiredPositionToMavros();
    ros::Rate rate(px4_offboard._rate);
    ros::Time last_request = ros::Time::now();
    while(ros::ok())
    {
        px4_offboard.armDrone(last_request);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}


