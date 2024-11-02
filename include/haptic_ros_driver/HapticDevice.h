#ifndef HAPTIC_DEVICE_H__
#define HAPTIC_DEVICE_H__

#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/thread/thread.hpp>
#include <mutex>
#include <vector>
#include <eigen3/Eigen/Dense>


class HapticDevice
{
	public:
        HapticDevice(ros::NodeHandle& node, float loop_rate, bool set_force);
        virtual ~HapticDevice();

        void PublishHapticData();
        void RegisterCallback();

        void GetHapticDataRun();
		
        void SetForce(Eigen::Vector3d force);

        void VerifyForceLimit(Eigen::Vector3d &force);
        void ForceCallback(const geometry_msgs::Vector3::ConstPtr &data);
        void ApplyReturnToOriginForce();


        void Start();

    protected:
       std::shared_ptr<boost::thread> dev_op_thread_;

	private:
       ros::NodeHandle nh_;
       ros::Rate loop_rate_;
       int device_count_;
       int dev_id_;
       bool set_force_;
       bool keep_alive_=false;
       bool device_enabled_ = false;
       std::mutex val_lock_;
       bool force_released_;

       ros::Publisher pose_pub_;
       ros::Publisher twist_pub_;
       ros::Publisher ori_encoder_pub_;
       ros::Publisher button_state_pub_;
       ros::Subscriber force_sub_;

       Eigen::Vector3d force_limit_;

       Eigen::Vector3d position_;
       Eigen::Matrix3d orientation_;
       Eigen::Vector3d ori_encoder_;
       Eigen::Vector3d force_;
       Eigen::Vector3d lin_vel_;
       Eigen::Vector3d ang_vel_;
       bool button0_state_=false;
};

#endif
