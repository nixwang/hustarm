/*
 * imu_calibrate.hpp
 *
 *  Created on: 2018年6月12日
 *      Author: hustac
 */

#pragma once

#include <array>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"

#include "arm_robot_msgs/Imu.h"
#include "arm_robot_msgs/MagneticField.h"

namespace hustac {

// acceleration model: a^o = T^aK^a(a^s - b^a + v^a)
// 理想值 = 坐标轴旋转 * 尺度因子 (测量值 - 零偏 + 白噪声)

// gyro model: g^o = T^gK^g(g^s - b^g + v^g)
// 理想值 = 坐标轴旋转 * 尺度因子 (测量值 - 零偏 + 白噪声)

    class BMX055Bridge {
    public:
        const std::string name_prefix;  // something like "camera"
    private:
        ros::NodeHandle &nh;
        ros::Subscriber sub_imu;
        ros::Subscriber sub_mag;
        ros::Subscriber sub_temp;
        ros::Publisher pub_imu;
        ros::Publisher pub_mag;
        ros::Publisher pub_temp;
        ros::ServiceServer srv_start_calibrate;
        ros::ServiceServer srv_is_calibrating;
        ros::ServiceServer srv_stop_calibrate;

        bool calibrating = false;
        ros::Time start_time;
        std::array<double, 3> sum_gyro;
        int count;
        int max_count;
        ros::Duration max_time;

        std::array<double, 9> gyro_trans;   // line major matrix
        std::array<double, 3> gyro_scale;
        std::array<double, 3> gyro_offset;

        std::array<double, 9> accel_trans;
        std::array<double, 3> accel_scale;
        std::array<double, 3> accel_offset;

        std::array<double, 9> mag_trans;
        std::array<double, 3> mag_scale;
        std::array<double, 3> mag_offset;

        struct CachedTrans {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Eigen::Matrix3d Ta;
            Eigen::Matrix3d Tg;
            Eigen::Matrix3d Tm;
        };

        CachedTrans *cached_trans;

        double temp_scale;
        double temp_offset;

        ros::Time last_read_param;
    public:
        BMX055Bridge(ros::NodeHandle &_nh, std::string _name_prefix) :
                nh(_nh), name_prefix(_name_prefix) {
            cached_trans = new CachedTrans;

            _reset_param();
            _read_param();
            _write_param();

            sub_imu = nh.subscribe<sensor_msgs::Imu>(name_prefix + "data_raw",
                                                     100, &BMX055Bridge::update_imu, this);
            sub_mag = nh.subscribe<arm_robot_msgs::MagneticField>(
                    name_prefix + "mag_raw", 100, &BMX055Bridge::update_mag, this);
            sub_temp = nh.subscribe<sensor_msgs::Temperature>(
                    name_prefix + "temp_raw", 100, &BMX055Bridge::update_temp,
                    this);

            pub_imu = nh.advertise<sensor_msgs::Imu>(name_prefix + "data", 100);
            pub_mag = nh.advertise<sensor_msgs::MagneticField>(name_prefix + "mag",
                                                               100);
            pub_temp = nh.advertise<sensor_msgs::Temperature>(name_prefix + "temp",
                                                              100);

            srv_start_calibrate = nh.advertiseService(
                    name_prefix + "start_calibrate",
                    &BMX055Bridge::_start_calibrate, this);
            srv_is_calibrating = nh.advertiseService(name_prefix + "is_calibrating",
                                                     &BMX055Bridge::_is_calibrating, this);
            srv_stop_calibrate = nh.advertiseService(name_prefix + "stop_calibrate",
                                                     &BMX055Bridge::_stop_calibrate, this);
        }

        BMX055Bridge(const BMX055Bridge &) = delete;

        ~BMX055Bridge() {
            delete cached_trans;
        }

        void _reset_param() {
            gyro_trans = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            gyro_scale = {1, 1, 1};
            gyro_offset = {0, 0, 0};
            accel_trans = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            accel_scale = {1, 1, 1};
            accel_offset = {0, 0, 0};
            mag_trans = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            mag_scale = {1, 1, 1};
            mag_offset = {0, 0, 0};
            temp_offset = 0;
            temp_scale = 1;

            cached_trans->Ta << 1, 0, 0, 0, 1, 0, 0, 0, 1;
            cached_trans->Tg << 1, 0, 0, 0, 1, 0, 0, 0, 1;
            cached_trans->Tm << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        }

        void _read_param() {

#define READ_PARAM_TO_ARRAY(scr, dst) \
        do {\
            std::vector<double> tmp;\
            if (nh.getParam(std::string("/camera/") + scr, tmp) && tmp.size() == dst.size()) {\
                std::copy_n(tmp.begin(), dst.size(), dst.begin());\
            }\
        } while (false)

            READ_PARAM_TO_ARRAY("gyro/trans", gyro_trans);
            READ_PARAM_TO_ARRAY("gyro/scale", gyro_scale);
            READ_PARAM_TO_ARRAY("gyro/offset", gyro_offset);

            READ_PARAM_TO_ARRAY("accel/trans", accel_trans);
            READ_PARAM_TO_ARRAY("accel/scale", accel_scale);
            READ_PARAM_TO_ARRAY("accel/offset", accel_offset);

            READ_PARAM_TO_ARRAY("mag/trans", mag_trans);
            READ_PARAM_TO_ARRAY("mag/scale", mag_scale);
            READ_PARAM_TO_ARRAY("mag/offset", mag_offset);

#undef READ_PARAM_TO_ARRAY

            nh.getParam(name_prefix + "temp/scale", temp_scale);
            nh.getParam(name_prefix + "temp/offset", temp_offset);

            // cache matrix
            cached_trans->Ta << accel_trans[0], accel_trans[1], accel_trans[2],
                    accel_trans[3], accel_trans[4], accel_trans[5],
                    accel_trans[6], accel_trans[7], accel_trans[8];
//            ROS_INFO_STREAM("T_accel: " << std::endl << cached_trans->Ta);

            cached_trans->Tg << gyro_trans[0], gyro_trans[1], gyro_trans[2],
                    gyro_trans[3], gyro_trans[4], gyro_trans[5],
                    gyro_trans[6], gyro_trans[7], gyro_trans[8];
//            ROS_INFO_STREAM("T_gyro: " << std::endl << cached_trans->Tg);

            cached_trans->Tm << mag_trans[0], mag_trans[1], mag_trans[2],
                    mag_trans[3], mag_trans[4], mag_trans[5],
                    mag_trans[6], mag_trans[7], mag_trans[8];

//            ROS_INFO_STREAM("T_mag: " << std::endl << cached_trans->Tm);

            last_read_param = ros::Time::now();
        }

        void _write_param() {
            nh.setParam(name_prefix + "gyro/trans", std::vector<double>(gyro_trans.begin(), gyro_trans.end()));
            nh.setParam(name_prefix + "gyro/scale", std::vector<double>(gyro_scale.begin(), gyro_scale.end()));
            nh.setParam(name_prefix + "gyro/offset", std::vector<double>(gyro_offset.begin(), gyro_offset.end()));

            nh.setParam(name_prefix + "accel/trans", std::vector<double>(accel_trans.begin(), accel_trans.end()));
            nh.setParam(name_prefix + "accel/scale", std::vector<double>(accel_scale.begin(), accel_scale.end()));
            nh.setParam(name_prefix + "accel/offset", std::vector<double>(accel_offset.begin(), accel_offset.end()));

            nh.setParam(name_prefix + "mag/trans", std::vector<double>(mag_trans.begin(), mag_trans.end()));
            nh.setParam(name_prefix + "mag/scale", std::vector<double>(mag_scale.begin(), mag_scale.end()));
            nh.setParam(name_prefix + "mag/offset", std::vector<double>(mag_offset.begin(), mag_offset.end()));

            nh.setParam(name_prefix + "temp/offset", temp_offset);
            nh.setParam(name_prefix + "temp/scale", temp_scale);
        }

        bool _start_calibrate(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response) {
            if (calibrating) {
                response.success = true;
                response.message = "already started";
            } else {
                start_calibrate(20000, ros::Duration(20, 0));
                response.success = true;
            }
            return true;
        }

        void start_calibrate(int _max_count, const ros::Duration &_max_time) {
            max_count = _max_count;
            max_time = _max_time;
            count = 0;
            sum_gyro = {0, 0, 0};
            start_time = ros::Time::now();
            calibrating = true;
            ROS_INFO_STREAM("start calibration " << name_prefix);
        }

        bool _stop_calibrate(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response) {
            if (calibrating) {
                stop_calibrate();
                response.success = true;
            } else {
                response.success = true;
                response.message = "already stopped";
            }
            return true;
        }

        void stop_calibrate() {
            calibrating = false;
            for (int i = 0; i < 3; i++) {
                gyro_offset[i] = sum_gyro[i] / count;
            }
            _write_param();
            ROS_INFO_STREAM("calibration finished " << name_prefix
                                                    << "gyro offset: " << gyro_offset[0] << " " << gyro_offset[1] << " "
                                                    << gyro_offset[2]);
        }

        bool _is_calibrating(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response) {
            response.success = calibrating;
            return true;
        }

        bool is_calibrating() {
            return calibrating;
        }

        void update_imu(const sensor_msgs::Imu::ConstPtr &raw_msg) {
            if (calibrating) {
                sum_gyro[0] += raw_msg->angular_velocity.x;
                sum_gyro[1] += raw_msg->angular_velocity.y;
                sum_gyro[2] += raw_msg->angular_velocity.z;
                count++;
                if ((!max_time.isZero() && ros::Time::now() - start_time >= max_time)
                    || (max_count > 0 && count >= max_count)) {
                    stop_calibrate();
                }
            }
            // read param from server every second
            if (ros::Time::now() - last_read_param >= ros::Duration(1, 0)) {
                _read_param();
            }

            // construct msg
            sensor_msgs::Imu imu_msg;
            imu_msg.header = raw_msg->header;
            imu_msg.orientation_covariance[0] = -1;// orientation is unknown
            imu_msg.angular_velocity = raw_msg->angular_velocity;
            imu_msg.linear_acceleration = raw_msg->linear_acceleration;

            // apply calibration
            imu_msg.linear_acceleration.x = (imu_msg.linear_acceleration.x - accel_offset[0]) * accel_scale[0];
            imu_msg.linear_acceleration.y = (imu_msg.linear_acceleration.y - accel_offset[1]) * accel_scale[1];
            imu_msg.linear_acceleration.z = (imu_msg.linear_acceleration.z - accel_offset[2]) * accel_scale[2];
            Eigen::Vector3d ao;
            ao << imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z;
            Eigen::Vector3d ao_rect = cached_trans->Ta * ao;
            imu_msg.linear_acceleration.x = ao_rect(0);
            imu_msg.linear_acceleration.y = ao_rect(1);
            imu_msg.linear_acceleration.z = ao_rect(2);

            imu_msg.angular_velocity.x = (imu_msg.angular_velocity.x - gyro_offset[0]) * gyro_scale[0];
            imu_msg.angular_velocity.y = (imu_msg.angular_velocity.y - gyro_offset[1]) * gyro_scale[1];
            imu_msg.angular_velocity.z = (imu_msg.angular_velocity.z - gyro_offset[2]) * gyro_scale[2];
            Eigen::Vector3d go;
            go << imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z;
            Eigen::Vector3d go_rect = cached_trans->Tg * go;
            imu_msg.angular_velocity.x = go_rect(0);
            imu_msg.angular_velocity.y = go_rect(1);
            imu_msg.angular_velocity.z = go_rect(2);

            pub_imu.publish(imu_msg);
        }

        void update_mag(const arm_robot_msgs::MagneticField::ConstPtr &raw_msg) {
            // read param from server every second
            if (ros::Time::now() - last_read_param >= ros::Duration(1, 0)) {
                _read_param();
            }

            // construct msg
            sensor_msgs::MagneticField mag_msg;
            mag_msg.header = raw_msg->header;
            mag_msg.magnetic_field = raw_msg->magnetic_field;

            // apply calibration
            mag_msg.magnetic_field.x = (mag_msg.magnetic_field.x - mag_offset[0]) * mag_scale[0];
            mag_msg.magnetic_field.y = (mag_msg.magnetic_field.y - mag_offset[1]) * mag_scale[1];
            mag_msg.magnetic_field.z = (mag_msg.magnetic_field.z - mag_offset[2]) * mag_scale[2];
            Eigen::Vector3d mo;
            mo << mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z;
            Eigen::Vector3d mo_rect = cached_trans->Tm * mo;
            mag_msg.magnetic_field.x = mo_rect(0);
            mag_msg.magnetic_field.y = mo_rect(1);
            mag_msg.magnetic_field.z = mo_rect(2);

            pub_mag.publish(mag_msg);
        }

        void update_temp(const sensor_msgs::Temperature::ConstPtr &raw_msg) {
            // read param from server every second
            if (ros::Time::now() - last_read_param >= ros::Duration(1, 0)) {
                _read_param();
            }

            // construct msg
            sensor_msgs::Temperature temp_msg;
            temp_msg.header = raw_msg->header;
            temp_msg.temperature = raw_msg->temperature;

            // apply calibration
            temp_msg.temperature /= temp_scale;
            temp_msg.temperature -= temp_offset;

            pub_temp.publish(temp_msg);
        }
    };

}
