//
// Created by hustac on 18-10-22.
//

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

namespace hustac {

class JoyAxisControl {
public:
    static constexpr double speedup_threshold_ = 0.3;
    static constexpr double speedup_acceleration_ = 0.3;    // percent of scale /sec
    static constexpr double speedup_deceleration_ = 2.0;    // percent of scale /sec
    static constexpr double speedup_max = 1.0;     // percent of scale
    static constexpr double joy_timeout_ = 0.5;

    int index_;
    double scale_ = 0;
    double integral_ = 0;
    ros::Time prev_output_;
    sensor_msgs::Joy prev_joy_;

    JoyAxisControl(int index) : index_(index), prev_output_(ros::Time::now()) {}

    void scale(double v) {
        scale_ = v;
    }

    void on_joy(const sensor_msgs::Joy &joy) {
        prev_joy_ = joy;
    }

    double output() {
        double axis_value;
        if (prev_joy_.header.stamp == ros::Time() ||
            ros::Time::now() - prev_joy_.header.stamp >= ros::Duration(joy_timeout_)) {
            axis_value = 0;
        } else {
            axis_value = prev_joy_.axes[index_];
        }

        auto now = ros::Time::now();
        double dur = (now - prev_output_).toSec();

        if (dur >= 0) {
            prev_output_ = now;
            if (axis_value >= speedup_threshold_) {
                if (integral_ >= 0) {
                    integral_ += speedup_acceleration_ * dur;
                } else {
                    integral_ = 0;
//                    if (std::abs(integral_) / speedup_deceleration_ < dur) {
//                        integral_ = speedup_acceleration_ * (dur - std::abs(integral_) / speedup_deceleration_);
//                    } else {
//                        integral_ += speedup_deceleration_ * dur;
//                    }
                }
            } else if (axis_value <= -speedup_threshold_) {
                if (integral_ <= 0) {
                    integral_ -= speedup_acceleration_ * dur;
                } else {
                    integral_ = 0;
//                    if (std::abs(integral_) / speedup_deceleration_ < dur) {
//                        integral_ = -speedup_acceleration_ * (dur - std::abs(integral_) / speedup_deceleration_);
//                    } else {
//                        integral_ -= speedup_deceleration_ * dur;
//                    }
                }
            } else if (integral_ > 0) {
                integral_ -= speedup_deceleration_ * dur;
                integral_ = std::max(0.0, integral_);
            } else if (integral_ < 0) {
                integral_ += speedup_deceleration_ * dur;
                integral_ = std::min(0.0, integral_);
            }
        }

        integral_ = std::max(integral_, -speedup_max);
        integral_ = std::min(integral_, speedup_max);

        double precent = axis_value + integral_;
        double output = scale_ * precent;
        return output;
    }

};

constexpr double JoyAxisControl::speedup_threshold_;
constexpr double JoyAxisControl::speedup_acceleration_;
constexpr double JoyAxisControl::speedup_deceleration_;
constexpr double JoyAxisControl::speedup_max;
constexpr double JoyAxisControl::joy_timeout_;

class JoyStepControl {
public:
    double step_length_;
    const std::pair<double, double> output_range_;
    double value_ = 0;

    sensor_msgs::Joy prev_joy_;
    bool prev_inc_pressed_ = false;
    bool prev_dec_pressed_ = false;
    std::function<bool(const sensor_msgs::Joy &joy)> func_is_inc_pressed_;
    std::function<bool(const sensor_msgs::Joy &joy)> func_is_dec_pressed_;

    JoyStepControl(double step, std::pair<double, double> output_range,
                   decltype(func_is_inc_pressed_) func_is_inc_pressed,
                   decltype(func_is_dec_pressed_) func_is_dec_pressed)
            : step_length_(step),
              output_range_(std::move(output_range)),
              func_is_inc_pressed_(std::move(func_is_inc_pressed)),
              func_is_dec_pressed_(std::move(func_is_dec_pressed)) {
        if (step_length_ <= 0)
            throw std::runtime_error("assert step_length > 0");
        if (output_range_.first > 0 || output_range_.second < 0)
            throw std::runtime_error("assert output_range.first <= 0 <= output_range.second");
    }

    void on_joy(const sensor_msgs::Joy &joy) {
        bool inc_pressed = func_is_inc_pressed_(joy);
        bool dec_pressed = func_is_dec_pressed_(joy);
        if (!prev_inc_pressed_ && inc_pressed) {
            // inc pushed
            if (value_ >= -step_length_ && value_ < 0)
                value_ = 0;
            else
                value_ += step_length_;
            value_ = std::min(output_range_.second, value_);
            ROS_INFO("Button inc pressed, value = %.3f", value_);
        } else if (prev_inc_pressed_ && !inc_pressed) {
            // inc released
        }
        if (!prev_dec_pressed_ && dec_pressed) {
            // dec pushed
            if (value_ <= step_length_ && value_ > 0)
                value_ = 0;
            else
                value_ -= step_length_;
            value_ = std::max(output_range_.first, value_);
            ROS_INFO("Button dec pressed, value = %.3f", value_);
        } else if (prev_dec_pressed_ && !dec_pressed) {
            // dec released
        }
        prev_joy_ = joy;
        prev_inc_pressed_ = inc_pressed;
        prev_dec_pressed_ = dec_pressed;
    }

    void value(double v) {
        if (!std::isfinite(v))
            throw std::runtime_error("value is not finite");
        v = std::max(output_range_.first, v);
        v = std::min(output_range_.second, v);
        value_ = v;
    }

    double value() {
        return value_;
    }
};

class TeleopJoy {
protected:
    static constexpr int default_axis_pan_x = 1;
    static constexpr int default_axis_pan_y = 2;
    static constexpr int default_axis_car_x = 3;
    static constexpr int default_axis_car_y = 0;
    static constexpr int default_axis_yaw = 6;
    static constexpr int default_axis_pitch = 7;
    static constexpr double default_scale_pan_x = 0.2;
    static constexpr double default_scale_pan_y = 0.2;
    static constexpr double default_scale_car_x = 0.2;
    static constexpr double default_scale_car_y = 25 * M_PI / 180;

    ros::NodeHandle nh_;

    std::shared_ptr<JoyAxisControl> axis_pan_x;
    std::shared_ptr<JoyAxisControl> axis_pan_y;
    std::shared_ptr<JoyAxisControl> axis_car_x;
    std::shared_ptr<JoyAxisControl> axis_car_y;

    int axis_yaw = -1;
    int axis_pitch = -1;
    std::shared_ptr<JoyStepControl> button_yaw;
    std::shared_ptr<JoyStepControl> button_pitch;

    ros::Subscriber sub_joy_;

    geometry_msgs::Twist twist_;
    ros::Time prev_time_print_twist_;
    geometry_msgs::Twist prev_printed_twist_;

    ros::Publisher pub_twist_;
    ros::Publisher pub_yaw;
    ros::Publisher pub_pitch;
    ros::Timer timer_pub_;

public:
    TeleopJoy();

private:
    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy);

    void publish_twist();
    void publish_gaze();

    void timer_pub_callback(const ros::TimerEvent &event);

    static bool is_axis_inc(const sensor_msgs::Joy &joy, int axis_index) {
        return joy.axes[axis_index] > 0.9;
    }

    static bool is_axis_dec(const sensor_msgs::Joy &joy, int axis_index) {
        return joy.axes[axis_index] < -0.9;
    }
};

constexpr int TeleopJoy::default_axis_pan_x;
constexpr int TeleopJoy::default_axis_pan_y;
constexpr int TeleopJoy::default_axis_car_x;
constexpr int TeleopJoy::default_axis_car_y;
constexpr int TeleopJoy::default_axis_yaw;
constexpr int TeleopJoy::default_axis_pitch;
constexpr double TeleopJoy::default_scale_pan_x;
constexpr double TeleopJoy::default_scale_pan_y;
constexpr double TeleopJoy::default_scale_car_x;
constexpr double TeleopJoy::default_scale_car_y;


TeleopJoy::TeleopJoy() :
        pub_twist_(nh_.advertise<geometry_msgs::Twist>("/base/cmd_vel", 1)),
        pub_yaw(nh_.advertise<std_msgs::Float64>("/gaze/single_joint_position/direct/1", 1)),
        pub_pitch(nh_.advertise<std_msgs::Float64>("/gaze/single_joint_position/direct/2", 1)),
        sub_joy_(nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joy_callback, this)),
        timer_pub_(nh_.createTimer(ros::Duration(0.01), &TeleopJoy::timer_pub_callback, this)) {

    axis_pan_x = std::make_shared<JoyAxisControl>(nh_.param("axis_pan_x", default_axis_pan_x));
    axis_pan_y = std::make_shared<JoyAxisControl>(nh_.param("axis_pan_y", default_axis_pan_y));
    axis_car_x = std::make_shared<JoyAxisControl>(nh_.param("axis_car_x", default_axis_car_x));
    axis_car_y = std::make_shared<JoyAxisControl>(nh_.param("axis_car_y", default_axis_car_y));
    axis_pan_x->scale(nh_.param("scale_pan_x", default_scale_pan_x));
    axis_pan_y->scale(nh_.param("scale_pan_y", default_scale_pan_y));
    axis_car_x->scale(nh_.param("scale_car_x", default_scale_car_x));
    axis_car_y->scale(nh_.param("scale_car_y", default_scale_car_y));

    axis_yaw = nh_.param("axis_yaw", default_axis_yaw);
    axis_pitch = nh_.param("axis_pitch", default_axis_pitch);

    button_yaw = std::make_shared<JoyStepControl>(0.1, std::make_pair(-2.87, 2.87),
                                                  std::bind(is_axis_inc, std::placeholders::_1, axis_yaw),
                                                  std::bind(is_axis_dec, std::placeholders::_1, axis_yaw));
    button_pitch = std::make_shared<JoyStepControl>(0.1, std::make_pair(-1.2, 1.4),
                                                  std::bind(is_axis_inc, std::placeholders::_1, axis_pitch),
                                                  std::bind(is_axis_dec, std::placeholders::_1, axis_pitch));

    timer_pub_.start();

    ROS_INFO("Twist=(%.2f m, %.2f m, %.2f deg)/s", twist_.linear.x, twist_.linear.y,
             twist_.angular.z / M_PI * 180);
}

void TeleopJoy::joy_callback(const sensor_msgs::Joy::ConstPtr &joy) {
    axis_pan_x->on_joy(*joy);
    axis_pan_y->on_joy(*joy);
    axis_car_x->on_joy(*joy);
    axis_car_y->on_joy(*joy);
    publish_twist();

    button_yaw->on_joy(*joy);
    button_pitch->on_joy(*joy);
    publish_gaze();
}

void TeleopJoy::timer_pub_callback(const ros::TimerEvent &event) {
    publish_twist();
    publish_gaze();

    if ((prev_printed_twist_.linear.x != twist_.linear.x || prev_printed_twist_.linear.y != twist_.linear.y ||
         prev_printed_twist_.angular.z != twist_.angular.z) &&
        ros::Time::now() - prev_time_print_twist_ >= ros::Duration(0.2)) {
        ROS_INFO("Twist=(%.2f m, %.2f m, %.2f deg)/s", twist_.linear.x, twist_.linear.y,
                 twist_.angular.z / M_PI * 180);
        prev_printed_twist_ = twist_;
        prev_time_print_twist_ = ros::Time::now();
    }
}

void TeleopJoy::publish_twist() {
    twist_.linear.x = axis_pan_x->output() + axis_car_x->output();
    twist_.linear.y = axis_pan_y->output();
    twist_.angular.z = axis_car_y->output();

    pub_twist_.publish(twist_);
}

void TeleopJoy::publish_gaze() {
    std_msgs::Float64 msg;
    msg.data = button_yaw->value();
    pub_yaw.publish(msg);
    msg.data = button_pitch->value();
    pub_pitch.publish(msg);
}

}

using namespace hustac;

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_ctrl");
    TeleopJoy teleop_turtle;
    ros::spin();
}
