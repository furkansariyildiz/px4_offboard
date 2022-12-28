#ifndef OFFBOARD_H
#define OFFBOARD_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#define LOG_ZONE_OFFBOARD " [ NHTechVision.OffBoard ] "


namespace NHTechVision
{
    class PX4OffBoard
    {
        private:
            ros::NodeHandle &nh_;
        public:
        /**
         * @brief PX4OffBoard Constructor: Defining ros_node, ros_subscribers, ros_publisher and class variables
         * @param node_handle: ros::NodeHandle
         * @details https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html
         * @return None
        */
            PX4OffBoard(ros::NodeHandle&);

            ros::Subscriber _state_subscriber;
            ros::Subscriber _desired_pose_subscriber;
            ros::Publisher _local_pose_publisher;
            ros::ServiceClient _arming_client;
            ros::ServiceClient _set_mode_client;
            
            /**
             * @brief '/mavros/state' topic Callback function with mavros_msgs::State type.
             * @param ConstPtr mavros_msgs::State::ConstPtr
             * @return None
            */
            void stateCallback(const mavros_msgs::State::ConstPtr&);

            /**
             * @brief '/desired_pose' topic Callback function with geometry_msgs::PoseStamped
             * @param ConstPtr geometry_msgs::PoseStamped::ConstPtr
             * @return None
            */
            void desiredPoseCallback(const geometry_msgs::PoseStamped::ConstPtr&);
            
            /**
             * @brief Function for waiting FCU Connection with void type
             * @param void None
             * @return None
            */
            void waitFCUConnection(void);

            /**
             * @brief Function for sending desired geometry_msgs::PoseStamped message to MavRos
             * @param void None
             * @return None
            */
            void sendDesiredPositionToMavros(void);

            /**
             * @brief Function for arming and changing state mode to OFFBOARD mode
             * @param last_request ros::Time
             * @return None
            */
            void armDrone(ros::Time);

            mavros_msgs::State _current_state;
            mavros_msgs::SetMode _offboard_set_mode;
            mavros_msgs::CommandBool _arm_command;
            geometry_msgs::PoseStamped _desired_pose;
            // ros::Time _last_request = ros::Time::now();
            float _rate;
            
    };
}

#endif