#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"

#include "perception/msg/lanes.hpp" 

enum class BehaviorState { KEEP_LANE_CRUISE, KEEP_LANE_CAUTION, LANE_CHANGE, EMERGENCY_BRAKE };
enum class Lane { LEFT = 0, CENTER = 1, RIGHT = 2, UNKNOWN = -1 }; 

struct VehicleInfo {
    double lane_obs_dist[3];
    double lane_obs_speed[3];

    double ego_speed;     
    Lane   current_lane;  

    std::vector<geometry_msgs::msg::Point> left_line;
    std::vector<geometry_msgs::msg::Point> center_line; 
    std::vector<geometry_msgs::msg::Point> right_line;
};

struct DecisionResult {
    BehaviorState state;        
    Lane target_lane;           
    double target_speed;
};

class DecisionMaker {
private:
    double current_target_v_ = 0.0;
    bool is_tja_stopped_ = false; 

public:
   DecisionResult decide(const VehicleInfo& v) {
        DecisionResult result;
        int curr = static_cast<int>(v.current_lane);
        if (curr < 0 || curr > 2) curr = 1; 
        
        result.state = BehaviorState::KEEP_LANE_CRUISE;
        result.target_lane = static_cast<Lane>(curr);
        
        double MAX_CRUISE_SPEED = 4.0; 
        double my_front_dist = v.lane_obs_dist[curr];
        double my_front_speed = v.lane_obs_speed[curr];
        
        double STANDSTILL_GAP = 1.5; 
        double safe_dist = STANDSTILL_GAP + v.ego_speed * 1.0; 
        double raw_target_v = MAX_CRUISE_SPEED;
        
        if (my_front_dist < safe_dist * 2.0) {
            result.state = BehaviorState::KEEP_LANE_CAUTION;
            double dist_err = my_front_dist - safe_dist;
            
            if (dist_err < 0) {
                raw_target_v = std::clamp(my_front_speed + (dist_err * 0.5), 0.0, std::max(0.0, my_front_speed - 0.5));
            } else {
                raw_target_v = std::clamp(my_front_speed + (dist_err * 0.5), 0.0, MAX_CRUISE_SPEED);
            }
        }

        if (v.ego_speed < 0.2 && my_front_dist < 3.0 && my_front_speed < 0.2) {
            is_tja_stopped_ = true; 
        }
        
        if (is_tja_stopped_) {
            if (my_front_dist > 3.5 || my_front_speed > 1.0) {
                is_tja_stopped_ = false; 
            } else {
                raw_target_v = 0.0; 
                result.state = BehaviorState::KEEP_LANE_CAUTION;
            }
        }

        bool want_to_change = (my_front_dist < 15.0) && (my_front_speed < MAX_CRUISE_SPEED * 0.8) && !is_tja_stopped_;
        
        if (want_to_change) {
            int best_lane = curr;
            double max_dist = my_front_dist;
            
            for (int i = 0; i < 3; ++i) {
                if (i == curr) continue;
                if (v.lane_obs_dist[i] > max_dist + 4.0) {
                    best_lane = i;
                    max_dist = v.lane_obs_dist[i];
                }
            }

            if (best_lane != curr) {
                int next_lane = curr + (best_lane > curr ? 1 : -1);
                double gap_dist = v.lane_obs_dist[next_lane];
                double gap_speed = v.lane_obs_speed[next_lane];
                double req_gap = std::max(2.0, v.ego_speed * 0.7); 
                
                if (gap_dist > req_gap) {
                    result.state = BehaviorState::LANE_CHANGE;
                    result.target_lane = static_cast<Lane>(next_lane);
                    raw_target_v = std::clamp(v.ego_speed + 0.5, 2.0, MAX_CRUISE_SPEED); 
                } else {
                    double slide_speed = std::max(0.0, gap_speed - 0.5);
                    if (gap_speed < 0.2) slide_speed = 0.0; 
                    raw_target_v = std::min(raw_target_v, slide_speed);
                }
            }
        }

        if (my_front_dist < 1.2) {
            result.state = BehaviorState::EMERGENCY_BRAKE;
            raw_target_v = 0.0;
            current_target_v_ = 0.0; 
        }

        if (result.state != BehaviorState::EMERGENCY_BRAKE) {
            current_target_v_ = (0.2 * raw_target_v) + (0.8 * current_target_v_);
        }

        result.target_speed = current_target_v_;
        return result;
    }
};

class PlanningNode : public rclcpp::Node {
private:
    VehicleInfo myCar;
    DecisionMaker brain;

    // 🌟 추가된 메모리 변수들
    double mem_obs_dist_[3] = {20.0, 20.0, 20.0};
    double mem_obs_speed_[3] = {0.0, 0.0, 0.0};
    int mem_timer_[3] = {0, 0, 0}; // 기억 유지 시간 (단위: 틱)

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_objects_;
    rclcpp::Subscription<perception::msg::Lanes>::SharedPtr sub_lanes_; 
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_ego_speed_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr viz_path_pub_; 
    
    rclcpp::TimerBase::SharedPtr timer_;

    void calculate_velocity_profile(nav_msgs::msg::Path &path, double max_v) {
        int n = path.poses.size();
        if (n < 2) return;
        
        double decel_limit = 1.0; 
        path.poses[n-1].pose.position.z = std::min(max_v, path.poses[n-1].pose.position.z);
        for (int i = n - 2; i >= 0; --i) {
            double dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
            double dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
            double dist = std::hypot(dx, dy);
            double v_next = path.poses[i+1].pose.position.z;
            double safe_v = std::sqrt(v_next * v_next + 2.0 * decel_limit * dist);
            path.poses[i].pose.position.z = std::min(max_v, safe_v);
        }
    }

public:
    PlanningNode() : Node("planning_node") {
        myCar.ego_speed = 0.0; 
        myCar.current_lane = Lane::CENTER; 
        reset_sensor_data();

        sub_objects_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/perception/tracked_objects", 10, std::bind(&PlanningNode::objects_callback, this, std::placeholders::_1));

        sub_lanes_ = this->create_subscription<perception::msg::Lanes>(
            "/perception/lane/lanes", 10, std::bind(&PlanningNode::lanes_callback, this, std::placeholders::_1));

        sub_ego_speed_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ego_speed", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { this->myCar.ego_speed = msg->data; });

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/local_path", 10);
        viz_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/viz_path", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PlanningNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "🚀 [센서 깜빡임 픽스] 고스트 메모리 로직 가동! 대참사 완벽 방어!");
    }

private:
    void reset_sensor_data() {
        for(int i=0; i<3; i++) {
            myCar.lane_obs_dist[i] = 20.0;
            myCar.lane_obs_speed[i] = 0.0;
        }
    }

    void lanes_callback(const perception::msg::Lanes::SharedPtr msg) {
        if (msg->lanes.empty()) return;
        
        if (msg->lanes.size() >= 3) {
            myCar.left_line = msg->lanes[0].points;
            myCar.center_line = msg->lanes[1].points;
            myCar.right_line = msg->lanes[2].points;
        } else {
            myCar.center_line = msg->lanes[0].points;
            myCar.left_line = msg->lanes[0].points;
            myCar.right_line = msg->lanes[0].points;
        }
    }

    void objects_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        reset_sensor_data();
        
        // 🌟 1. 센서가 아무것도 못 봤더라도, 기존 기억을 복원
        for(int i=0; i<3; i++) {
            if (mem_timer_[i] > 0) {
                myCar.lane_obs_dist[i] = mem_obs_dist_[i];
                myCar.lane_obs_speed[i] = mem_obs_speed_[i];
                mem_timer_[i]--; // 기억 시간 1틱 차감
            }
        }

        if (msg->poses.empty()) return;

        auto get_expected_y = [](const std::vector<geometry_msgs::msg::Point>& line, double x) {
            if (line.empty()) return 999.0;
            double min_dx = 999.0; double exp_y = 999.0;
            for (const auto& pt : line) {
                double dx = std::abs(pt.x - x);
                if (dx < min_dx) { min_dx = dx; exp_y = pt.y; }
            }
            return exp_y;
        };

        for (const auto& pose : msg->poses) {
            double obs_x = pose.position.x; 
            double obs_y = pose.position.y; 
            double obs_speed = pose.position.z; 

            double y_left = get_expected_y(myCar.left_line, obs_x);
            double y_center = get_expected_y(myCar.center_line, obs_x);
            double y_right = get_expected_y(myCar.right_line, obs_x);

            int obs_lane = -1;
            double d_left = std::abs(obs_y - y_left);
            double d_center = std::abs(obs_y - y_center);
            double d_right = std::abs(obs_y - y_right);
            
            double min_d = std::min({d_left, d_center, d_right});
            
            // 말씀하신 대로 차선 폭을 넓게 주셨다면 이 조건은 그대로 넉넉하게 적용됩니다.
            if (min_d < 4.5) { 
                if (min_d == d_left) obs_lane = 0;
                else if (min_d == d_center) obs_lane = 1;
                else if (min_d == d_right) obs_lane = 2;
            }

            if (obs_lane != -1) {
                double min_x = (obs_lane == static_cast<int>(myCar.current_lane)) ? -0.1 : -1.0;
                
                if (obs_x > min_x && obs_x < myCar.lane_obs_dist[obs_lane]) {
                    // 🌟 2. 센서가 물체를 잡았다면 값을 갱신하고, '기억(메모리)'을 10틱(약 1초)으로 재충전함
                    myCar.lane_obs_dist[obs_lane] = obs_x;
                    myCar.lane_obs_speed[obs_lane] = obs_speed;
                    
                    mem_obs_dist_[obs_lane] = obs_x;
                    mem_obs_speed_[obs_lane] = obs_speed;
                    mem_timer_[obs_lane] = 10; 
                }
            }
        }
    }

    void timer_callback() {
        DecisionResult decision = brain.decide(myCar);
        
        if (decision.state == BehaviorState::LANE_CHANGE) {
            myCar.current_lane = decision.target_lane;
        }

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "base_link"; 
        path_msg.header.stamp = this->get_clock()->now();

        std::vector<geometry_msgs::msg::Point>* target_line = nullptr;
        if (myCar.current_lane == Lane::LEFT) target_line = &myCar.left_line;
        else if (myCar.current_lane == Lane::CENTER) target_line = &myCar.center_line;
        else if (myCar.current_lane == Lane::RIGHT) target_line = &myCar.right_line;

        if (target_line == nullptr || target_line->empty()) {
            for (double x = 0; x <= 2.0; x += 0.2) { 
                geometry_msgs::msg::PoseStamped p;
                p.header = path_msg.header; p.pose.position.x = x; p.pose.position.y = 0.0; p.pose.position.z = 0.0;
                path_msg.poses.push_back(p);
            }
        } else {
            for (size_t i = 0; i < target_line->size(); ++i) {
                geometry_msgs::msg::PoseStamped p;
                p.header = path_msg.header;
                p.pose.position.x = (*target_line)[i].x;
                p.pose.position.y = (*target_line)[i].y; 
                p.pose.position.z = decision.target_speed; 
                path_msg.poses.push_back(p);
            }
        }

        if (path_msg.poses.size() > 1) {
            for (size_t i = 0; i < path_msg.poses.size(); ++i) {
                if (i < path_msg.poses.size() - 1) {
                    double dx = path_msg.poses[i+1].pose.position.x - path_msg.poses[i].pose.position.x;
                    double dy = path_msg.poses[i+1].pose.position.y - path_msg.poses[i].pose.position.y;
                    double yaw = std::atan2(dy, dx);
                    path_msg.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
                    path_msg.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
                } else {
                    path_msg.poses[i].pose.orientation = path_msg.poses[i-1].pose.orientation;
                }
            }
        }

        if (!path_msg.poses.empty()) {
            calculate_velocity_profile(path_msg, decision.target_speed);
            
            path_pub_->publish(path_msg);

            nav_msgs::msg::Path viz_path = path_msg;
            for (auto& p : viz_path.poses) {
                p.pose.position.z = 0.0; 
            }
            
            if (viz_path.poses.front().pose.position.x > 0.1) {
                geometry_msgs::msg::PoseStamped origin = viz_path.poses.front();
                origin.pose.position.x = 0.0;
                origin.pose.position.y = 0.0; 
                viz_path.poses.insert(viz_path.poses.begin(), origin);
            }
            
            viz_path_pub_->publish(viz_path);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
