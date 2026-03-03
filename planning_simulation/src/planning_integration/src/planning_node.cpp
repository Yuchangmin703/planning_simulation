#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>
#include <vector>

class SmoothPlanner : public rclcpp::Node {
public:
    SmoothPlanner() : Node("smooth_planner"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        pub_path_ = create_publisher<nav_msgs::msg::Path>("/planning/smooth_path", 10);
        pub_history_ = create_publisher<nav_msgs::msg::Path>("/planning/history_path", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&SmoothPlanner::plan, this));
        history_path_.header.frame_id = "map";
    }
private:
    void plan() {
        geometry_msgs::msg::TransformStamped tf;
        try { tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero); }
        catch (const tf2::TransformException & ex) { return; }

        double x = tf.transform.translation.x, y = tf.transform.translation.y;
        double current_R = std::hypot(x, y), current_angle = std::atan2(y, x);

        // 1. 주행 기록: 최근 1초(20개 점)만 유지 로직
        geometry_msgs::msg::PoseStamped curr_p;
        curr_p.header = tf.header;
        curr_p.pose.position.x = x; curr_p.pose.position.y = y;
        history_path_.header.stamp = tf.header.stamp;
        history_path_.poses.push_back(curr_p);

        if(history_path_.poses.size() > 20) {
            history_path_.poses.erase(history_path_.poses.begin());
        }
        pub_history_->publish(history_path_);

        // 2. 미래 경로 생성
        double dist_to_obs = std::hypot(x - 0.0, y - 5.0);
        double target_R = (dist_to_obs < 3.0) ? 5.8 : 5.0;

        auto path = nav_msgs::msg::Path();
        path.header = tf.header;
        for (double a = 0; a < 1.5; a += 0.1) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            double future_angle = current_angle + a;
            double smooth_step = 1.0 / (1.0 + std::exp(-7.0 * (a - 0.5)));
            double r = current_R + (target_R - current_R) * smooth_step;
            ps.pose.position.x = r * std::cos(future_angle);
            ps.pose.position.y = r * std::sin(future_angle);
            path.poses.push_back(ps);
        }
        pub_path_->publish(path);
    }
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_, pub_history_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path history_path_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmoothPlanner>());
    rclcpp::shutdown();
    return 0;
}
