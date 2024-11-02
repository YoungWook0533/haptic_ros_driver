#include "haptic_ros_driver/HapticDevice.h"

// haptic device API
#include "haptic_ros_driver/dhdc.h"

HapticDevice::HapticDevice(ros::NodeHandle & node, float loop_rate, bool set_force): loop_rate_(loop_rate)
{
    nh_ = node;

    dev_id_ = -2; // we set a value not in API defined range
    device_enabled_ = -1;


    position_.setZero();
    orientation_.setIdentity();
    ori_encoder_.setZero();
    force_.setZero();
    lin_vel_.setZero();
    ang_vel_.setZero();
    button0_state_ = false;

    keep_alive_ = false;
    force_released_ = true;

    force_limit_ << 10.0, 10.0, 10.0;

    // connect to hardware device
    device_count_ = dhdGetDeviceCount();

    // we only accept one haptic device.
    if ( device_count_ >= 1) {
        dev_id_ = dhdOpenID(0); // if open failed, we will get -1, and sucess with 0.
        if ( dev_id_ < 0) {
            ROS_WARN("error: handler device: %s\n", dhdErrorGetLastStr());
            device_enabled_ = false;
            return;
        }
    } else {
        ROS_WARN("No handler device find! %s\n", dhdErrorGetLastStr());
        device_enabled_ = false;
        return;
    }

    set_force_ = set_force;
    if(set_force_)
    {
        // Enable force rendering on the haptic device.
        if(dhdEnableForce(DHD_ON) != 0)
        {
            ROS_INFO("error: failed to enable force rendering ( %s )", dhdErrorGetLastStr());
            device_enabled_ = false;
            return;
        }
    }

    device_enabled_ =true;
}


HapticDevice::~HapticDevice()
{
    dev_id_ = -1;
    device_count_ = 0;
    keep_alive_ = false;
    if (dev_op_thread_)
    {
        dev_op_thread_->join();
    }
    if (dhdClose() < 0)
    {
        ROS_WARN("error: failed to close the connection (%s)", dhdErrorGetLastStr());
        dhdSleep(2.0);
        return;
    }
}

void HapticDevice::PublishHapticData()
{
    Eigen::Quaterniond q(orientation_);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = ros::this_node::getName();
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = position_[0];
    pose.pose.position.y = position_[1];
    pose.pose.position.z = position_[2];
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    std_msgs::Float32MultiArray ori_encoder_msg;
    ori_encoder_msg.data.push_back(ori_encoder_(0));
    ori_encoder_msg.data.push_back(ori_encoder_(1));
    ori_encoder_msg.data.push_back(ori_encoder_(2));

    geometry_msgs::Twist twist;
    twist.linear.x = lin_vel_(0); 
    twist.linear.y = lin_vel_(1); 
    twist.linear.z = lin_vel_(2); 
    twist.angular.x = ang_vel_(0); 
    twist.angular.y = ang_vel_(1); 
    twist.angular.z = ang_vel_(2); 

    std_msgs::Int8MultiArray button_stat;
    button_stat.data.push_back(button0_state_);

    pose_pub_.publish(pose);
    ori_encoder_pub_.publish(ori_encoder_msg);
    twist_pub_.publish(twist);
    button_state_pub_.publish(button_stat);
}

void HapticDevice::RegisterCallback()
{
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/haptic/pose",1);
    ori_encoder_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/haptic/encoder_orientation", 1);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/haptic/twist",1);
    button_state_pub_ = nh_.advertise<std_msgs::Int8MultiArray>("/haptic/button_state", 1);
    force_sub_ = nh_.subscribe<geometry_msgs::Vector3>("/haptic/force",1, &HapticDevice::ForceCallback, this);
}

void HapticDevice::ForceCallback(const geometry_msgs::Vector3::ConstPtr &data)
{
    // wrapper force
    Eigen::Vector3d force(data->x, data->y, data->z);
    SetForce(force);
}

void HapticDevice::GetHapticDataRun()
{   // get and we will publish immediately


    double feed_force[3] = {0.0, 0.0, 0.0};
    double current_position[3] = {0.0, 0.0, 0.0};
    double current_orientation[3][3] = {{1.0, 0.0, 0.0},
                                        {0.0, 1.0, 0.0},
                                        {0.0, 0.0, 1.0}};
    double current_ori_encoder[3] = {0.0, 0.0, 0.0};
    double current_lin_vel[3] = {0.0, 0.0, 0.0};
    double current_ang_vel[3] = {0.0, 0.0, 0.0};

    while (ros::ok() && (keep_alive_ == true)) {

        if (device_count_ >= 1 && dev_id_ >= 0) {
                dhdGetPosition(&current_position[0], &current_position[1], &current_position[2]);
                position_(0) = current_position[0];
                position_(1) = current_position[1];
                position_(2) = current_position[2];

                dhdGetOrientationFrame(current_orientation);
                orientation_(0,0) = current_orientation[0][0];
                orientation_(0,1) = current_orientation[0][1];
                orientation_(0,2) = current_orientation[0][2];
                orientation_(1,0) = current_orientation[1][0];
                orientation_(1,1) = current_orientation[1][1];
                orientation_(1,2) = current_orientation[1][2];
                orientation_(2,0) = current_orientation[2][0];
                orientation_(2,1) = current_orientation[2][1];
                orientation_(2,2) = current_orientation[2][2];

                dhdGetOrientationRad(&current_ori_encoder[0], &current_ori_encoder[1], &current_ori_encoder[2]);
                ori_encoder_(0) = current_ori_encoder[0];
                ori_encoder_(1) = current_ori_encoder[1];
                ori_encoder_(2) = current_ori_encoder[2];

                dhdGetLinearVelocity(&current_lin_vel[0], &current_lin_vel[1], &current_lin_vel[2]);
                lin_vel_(0) = current_lin_vel[0];
                lin_vel_(1) = current_lin_vel[1];
                lin_vel_(2) = current_lin_vel[2];
                dhdGetAngularVelocityRad(&current_ang_vel[0], &current_ang_vel[1], &current_ang_vel[2]);
                ang_vel_(0) = current_ang_vel[0];
                ang_vel_(1) = current_ang_vel[1];
                ang_vel_(2) = current_ang_vel[2];

                button0_state_ = dhdGetButton(0, dev_id_);
        }

        PublishHapticData();
        ApplyReturnToOriginForce();

        // apply force
        if (set_force_) {
            val_lock_.lock();
            feed_force[0] = force_(0);
            feed_force[1] = force_(1);
            feed_force[2] = force_(2);
            int status = dhdSetForce(feed_force[0], feed_force[1], feed_force[2]);
            if(status != 0)
            {
                ROS_WARN("error: failed to set force (%s)", dhdErrorGetLastStr());
            }
            val_lock_.unlock();
        }

        loop_rate_.sleep();
    }

}

void HapticDevice::SetForce(Eigen::Vector3d force)
{
    if (set_force_)
    {
        val_lock_.lock();
        VerifyForceLimit(force);
        force_ = force;
        force_released_ = false;
        val_lock_.unlock();
    }
}



void HapticDevice::VerifyForceLimit(Eigen::Vector3d &force)
{
    for(size_t i = 0; i < 3; i++)
    {
        force(i) = std::min(force_limit_(i), std::max(-force_limit_(i), force(i)));
    }
}

void HapticDevice::ApplyReturnToOriginForce()
{
    if (!set_force_) return;

    double p_gain = 100.0;
    double d_gain = 5.0;

    val_lock_.lock();
    Eigen::Vector3d distance_to_origin = -position_;
    if(distance_to_origin.norm() < 0.01) p_gain *= 5.0;
    force_ = p_gain * distance_to_origin - d_gain * lin_vel_;
    val_lock_.unlock();
}



void HapticDevice::Start()
{   
    if (!device_enabled_)
    {
        return; 
    }
    

    RegisterCallback();
    ros::AsyncSpinner spinner(2);
    spinner.start();

    dev_op_thread_ = std::make_shared<boost::thread>(boost::bind(&HapticDevice::GetHapticDataRun, this));
    keep_alive_ = true;

    while (ros::ok() && (keep_alive_ == true)) {
        ros::Duration(0.0005).sleep();
    }

    keep_alive_ = false;
    spinner.stop();
}
