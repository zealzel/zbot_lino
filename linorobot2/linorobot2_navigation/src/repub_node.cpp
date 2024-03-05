#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class RepubNode : public rclcpp::Node {
  public:
    RepubNode() : Node("repub_node"), msg_ready_(false) {
        pub_range1_sensor_ = this->create_publisher<sensor_msgs::msg::Range>("/range1_sensor", 10);
        sub_range1_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/range1/data", 10,
            std::bind(&RepubNode::range1_callback, this, std::placeholders::_1));

        pub_range2_sensor_ = this->create_publisher<sensor_msgs::msg::Range>("/range2_sensor", 10);
        sub_range2_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/range2/data", 10,
            std::bind(&RepubNode::range2_callback, this, std::placeholders::_1));

        pub_range3_sensor_ = this->create_publisher<sensor_msgs::msg::Range>("/range3_sensor", 10);
        sub_range3_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/range3/data", 10,
            std::bind(&RepubNode::range3_callback, this, std::placeholders::_1));

        pub_range4_sensor_ = this->create_publisher<sensor_msgs::msg::Range>("/range4_sensor", 10);
        sub_range4_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/range4/data", 10,
            std::bind(&RepubNode::range4_callback, this, std::placeholders::_1));

        // pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        //     "/cmd_vel", 10, std::bind(&RepubNode::cmd_vel_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&RepubNode::timer_callback, this));
    }

  private:
    void range1_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
        last_range1_msg_ = *msg;
        // fix the costmap-not-clear problem. ref: https://blog.csdn.net/xiekaikaibing/article/details/80096033
        if (last_range1_msg_.range > last_range1_msg_.max_range)
            last_range1_msg_.range = last_range1_msg_.max_range;
        msg_ready_ = true;
    }

    void range2_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
        last_range2_msg_ = *msg;
        if (last_range2_msg_.range > last_range2_msg_.max_range)
            last_range2_msg_.range = last_range2_msg_.max_range;
        msg_ready_ = true;
    }

    void range3_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
        last_range3_msg_ = *msg;
        if (last_range3_msg_.range > last_range3_msg_.max_range)
            last_range3_msg_.range = last_range3_msg_.max_range;
        msg_ready_ = true;
    }

    void range4_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
        last_range4_msg_ = *msg;
        if (last_range4_msg_.range > last_range4_msg_.max_range)
            last_range4_msg_.range = last_range4_msg_.max_range;
        msg_ready_ = true;
    }

    // void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    //     last_cmd_vel_msg_ = *msg;
    //     msg_ready_ = true;
    // }

    void timer_callback() {
        if (msg_ready_) {
            pub_range1_sensor_->publish(last_range1_msg_);
            pub_range2_sensor_->publish(last_range2_msg_);
            pub_range3_sensor_->publish(last_range3_msg_);
            pub_range4_sensor_->publish(last_range4_msg_);
            // pub_cmd_vel_->publish(last_cmd_vel_msg_);
            msg_ready_ = false;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_range1_sensor_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_range1_;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_range2_sensor_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_range2_;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_range3_sensor_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_range3_;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_range4_sensor_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_range4_;

    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Range last_range1_msg_;
    sensor_msgs::msg::Range last_range2_msg_;
    sensor_msgs::msg::Range last_range3_msg_;
    sensor_msgs::msg::Range last_range4_msg_;
    // geometry_msgs::msg::Twist last_cmd_vel_msg_;
    bool msg_ready_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RepubNode>());
    rclcpp::shutdown();
    return 0;
}
