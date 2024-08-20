#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

class DockPosePublisher : public rclcpp::Node {
  public:
    DockPosePublisher() : Node("dock_pose_publisher") {
        subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "apriltag_detections", 10,
            std::bind(&DockPosePublisher::detection_callback, this, std::placeholders::_1));

        publisher_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", 10);

        use_first_detection_ = this->declare_parameter<bool>("use_first_detection", false);
        dock_tag_family_ = this->declare_parameter<std::string>("dock_tag_family", "tag36h11");
        dock_tag_ids_ = this->declare_parameter<std::vector<int64_t>>("dock_tag_ids", {10, 20, 30});
    }

  private:
    void detection_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
        auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();

        // RCLCPP_INFO(this->get_logger(), "detection_callback!");

        for (const auto& detection : msg->detections) {
            if (!use_first_detection_) {
                if (std::find(dock_tag_ids_.begin(), dock_tag_ids_.end(), detection.id) !=
                    dock_tag_ids_.end()) {
                    pose->header = msg->header;
                    pose->pose = detection.pose.pose.pose;
                    // RCLCPP_INFO(this->get_logger(), "(%f,%f,%f)\n", pose->pose.position.x,
                    //             pose->pose.position.y, pose->pose.position.z);
                    publisher_->publish(*pose);
                    return;
                }
            } else {
                pose->header = msg->header;
                pose->pose = msg->detections[0].pose.pose.pose;
                publisher_->publish(*pose);
                return;
            }
        }
    }

    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;

    bool use_first_detection_;
    std::string dock_tag_family_;
    std::vector<int64_t> dock_tag_ids_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DockPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
