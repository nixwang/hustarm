#include <string>
#include <vector>
#include <iostream>
#include <functional>
#include <deque>
#include <fstream>

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <realsense_camera/GetIMUInfo.h>

template<typename T>
decltype(T::x) sum(T xyz) {
    return std::sqrt(xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z);
}

template <typename T>
Eigen::Vector4d xyz_to_vector4d(T& xyz) {
    Eigen::Vector4d vector;
    vector(0) = xyz.x;
    vector(1) = xyz.y;
    vector(2) = xyz.z;
    vector(3) = 1;
    return vector;
}

template <typename V, typename T>
void vector_to_xyz(V& vector, T& xyz) {
    xyz.x = vector(0);
    xyz.y = vector(1);
    xyz.z = vector(2);
}

template <typename T>
T rad_to_deg(T v) {
    v *= 180 / M_PI;
    return v;
}

struct Record {
    sensor_msgs::Imu raw_msg;
    sensor_msgs::Imu msg;
    ros::Time recv_time;
};

static std::deque<Record> records;

static Eigen::Matrix4d accel_trans;
static Eigen::Matrix4d gyro_trans;

/* Problem:
 * realsense_camera pkg will publish imu_msg on either accel update or gyro update.
 * Accel update rate: 250 Hz
 * Gyro update rate: 200 Hz
 * Mixed update rate: 400 - 450 Hz (timestamp of msg maybe not increases)
 * Solution:
 * realsense_camera pkg is modified to only publish imu_msg on gyro update (200 Hz)
 * Because gyro data is more important for Visual Inertial Odometer.
 * */
void on_imu(sensor_msgs::ImuConstPtr msg) {
    auto now = ros::Time::now();
    if (!records.empty()) {
        if (msg->header.seq != records.back().msg.header.seq + 1) {
            ROS_WARN_STREAM("bad header.seq!!! " << msg->header.seq - records.back().msg.header.seq);
        }
        if (msg->header.stamp <= records.back().msg.header.stamp) {
            ROS_WARN_STREAM(
                    "header.stamp not increase!!! " << (msg->header.stamp - records.back().msg.header.stamp).toSec());
        }
    }
    sensor_msgs::Imu calibrated;
    calibrated = *msg;

    Eigen::Vector4d accel = xyz_to_vector4d(msg->linear_acceleration);
    Eigen::Vector4d gyro = xyz_to_vector4d(msg->angular_velocity);
//    accel = accel_trans * accel;
//    gyro = gyro_trans * gyro;
    vector_to_xyz(accel, calibrated.linear_acceleration);
    vector_to_xyz(gyro, calibrated.angular_velocity);

//    ROS_INFO_STREAM("Time: " << calibrated.header.stamp);
//    ROS_INFO("Raw Accel: %6.3lf %6.3lf %6.3lf -> %6.3lf", msg->linear_acceleration.x, msg->linear_acceleration.y,
//             msg->linear_acceleration.z, sum(msg->linear_acceleration));
//    ROS_INFO("    Accel: %6.3lf %6.3lf %6.3lf -> %6.3lf", calibrated.linear_acceleration.x,
//             calibrated.linear_acceleration.y,
//             calibrated.linear_acceleration.z, sum(calibrated.linear_acceleration));
//    ROS_INFO("Raw Gyro : %6.3lf %6.3lf %6.3lf -> %6.3lf", msg->angular_velocity.x, msg->angular_velocity.y,
//             msg->angular_velocity.z,
//             std::abs(calibrated.angular_velocity.x) + std::abs(calibrated.angular_velocity.y) +
//             std::abs(calibrated.angular_velocity.z));
//    ROS_INFO("    Gyro : %6.3lf %6.3lf %6.3lf -> %6.3lf", calibrated.angular_velocity.x, calibrated.angular_velocity.y,
//             calibrated.angular_velocity.z,
//             std::abs(calibrated.angular_velocity.x) + std::abs(calibrated.angular_velocity.y) +
//             std::abs(calibrated.angular_velocity.z));

    Record record{.raw_msg = *msg, .msg = calibrated, .recv_time = now};
    records.push_back(record);
    while (msg->header.stamp - records.front().msg.header.stamp >= ros::Duration(1)) {
        records.pop_front();
    }
}

void save_csv() {
    if (records.empty())
        return;
    std::string filepath = "/home/hustac/imu_record.csv";
    std::ofstream ofs(filepath, std::ios::out | std::ios::binary);
    ofs << "stamp,recv_time" << std::endl;
    for (const auto &record : records) {
        ofs << (record.msg.header.stamp - records.front().msg.header.stamp).toSec() << ',';
        ofs << (record.recv_time - records.front().recv_time).toSec() << std::endl;
    }
    ROS_INFO_STREAM("Save to " << filepath);
}

static int count_print = 0;

void on_timer(const ros::TimerEvent) {
    double dt_sum = 0;
    double dt_max = std::numeric_limits<double>::min();
    double dt_min = std::numeric_limits<double>::max();
    double latency_sum = 0;
    double latency_max = std::numeric_limits<double>::min();
    double latency_min = std::numeric_limits<double>::max();
    Eigen::Vector4d raw_accel_sum;
    Eigen::Vector4d raw_gyro_sum;
    Eigen::Vector4d accel_sum;
    Eigen::Vector4d gyro_sum;
    const sensor_msgs::Imu *p_last = std::nullptr_t();
    for (const auto &msg : records) {
        if (p_last) {
            auto dt = (msg.msg.header.stamp - p_last->header.stamp).toSec();
            dt_sum += dt;
            dt_max = std::max(dt_max, dt);
            dt_min = std::min(dt_min, dt);
        }
        auto latency = (msg.recv_time - msg.msg.header.stamp).toSec();
        latency_sum += latency;
        latency_min = std::min(latency_min, latency);
        latency_max = std::max(latency_max, latency);

        raw_accel_sum += xyz_to_vector4d(msg.raw_msg.linear_acceleration);
        raw_gyro_sum += xyz_to_vector4d(msg.raw_msg.angular_velocity);
        accel_sum += xyz_to_vector4d(msg.msg.linear_acceleration);
        gyro_sum += xyz_to_vector4d(msg.msg.angular_velocity);

        p_last = &msg.msg;
    }

    Eigen::Vector3d raw_accel_avg = raw_accel_sum.block(0, 0, 3, 1) / records.size();
    Eigen::Vector3d raw_gyro_avg = raw_gyro_sum.block(0, 0, 3, 1) / records.size();
    Eigen::Vector3d accel_avg = accel_sum.block(0, 0, 3, 1) / records.size();
    Eigen::Vector3d gyro_avg = gyro_sum.block(0, 0, 3, 1) / records.size();

    ROS_INFO("Freq: %d Hz dt_avg=%.3lf dt_max=%.3lf dt_min=%.3lf", (int) records.size(),
             dt_sum / (records.size() - 1) * 1000,
             dt_max * 1000, dt_min * 1000);
    ROS_INFO("Latency: avg=%.3lf max=%.3lf min=%.3lf", latency_sum / records.size(), latency_max, latency_min);
    ROS_INFO_STREAM("Raw Accel: " << raw_accel_avg.transpose() << " norm=" << raw_accel_avg.norm());
    ROS_INFO_STREAM("    Accel: " << accel_avg.transpose() << " norm=" << accel_avg.norm());
    Eigen::Vector3d raw_gyro_avg_deg = rad_to_deg(raw_gyro_avg);
    Eigen::Vector3d gyro_avg_deg = rad_to_deg(gyro_avg);
    ROS_INFO_STREAM("Raw Gyro : " << raw_gyro_avg_deg.transpose());
    ROS_INFO_STREAM("    Gyro : " << gyro_avg_deg.transpose());

    if (count_print == 1)
        save_csv();
    count_print++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "diag_imu");

    ros::NodeHandle nh("msg_bridge");

    auto sub_imu = nh.subscribe<sensor_msgs::Imu>("/camera/imu/data", 100, on_imu);
    auto print_timer = nh.createTimer(ros::Duration(1), on_timer);

    auto client_imu_info = nh.serviceClient<realsense_camera::GetIMUInfo>("/camera/driver/get_imu_info", false);
    {
        ROS_INFO_STREAM("Get IMU Info...");
        realsense_camera::GetIMUInfoRequest req;
        realsense_camera::GetIMUInfoResponse resp;
        while (!client_imu_info.call(req, resp)) {
        }
        {
            std::ostringstream oss;
            for (auto &v : resp.accel.data) {
                oss << v << ',';
            }
            ROS_INFO_STREAM("accel.data: " << oss.str());
        }
        {
            std::ostringstream oss;
            for (auto &v : resp.accel.noise_variances) {
                oss << v << ',';
            }
            ROS_INFO_STREAM("accel.noise_variances: " << oss.str());
        }
        {
            std::ostringstream oss;
            for (auto &v : resp.accel.bias_variances) {
                oss << v << ',';
            }
            ROS_INFO_STREAM("accel.bias_variances: " << oss.str());
        }
        {
            std::ostringstream oss;
            for (auto &v : resp.gyro.data) {
                oss << v << ',';
            }
            ROS_INFO_STREAM("gyro.data: " << oss.str());
        }
        {
            std::ostringstream oss;
            for (auto &v : resp.gyro.noise_variances) {
                oss << v << ',';
            }
            ROS_INFO_STREAM("gyro.noise_variances: " << oss.str());
        }
        {
            std::ostringstream oss;
            for (auto &v : resp.gyro.bias_variances) {
                oss << v << ',';
            }
            ROS_INFO_STREAM("gyro.bias_variances: " << oss.str());
        }

        for (int c = 0; c < 3; c++) {
            for (int r = 0; r < 4; r++) {
                int i = c * 4 + r;
                accel_trans(c, r) = resp.accel.data[i];
                gyro_trans(c, r) = resp.gyro.data[i];
            }
        }
        accel_trans(3, 3) = 1;
        gyro_trans(3, 3) = 1;
    }

    ROS_INFO_STREAM("Waiting for IMU msg...");
    print_timer.start();
    ros::spin();

    return 0;
}

