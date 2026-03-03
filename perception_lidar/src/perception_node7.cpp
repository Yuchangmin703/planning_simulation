#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

struct TrackedObject {
    int id;
    float x, y;
    float vx, vy;
    int age;
    int miss_count;
};

class PerceptionNode8 : public rclcpp::Node {
public:
    PerceptionNode8() : Node("perception_node8"), next_id_(0) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&PerceptionNode8::lidar_callback, this, std::placeholders::_1));
        
        pub_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/perception/tracked_objects", 10);
        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/object_markers", 10);

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Perception Node 8: 데드존 필터 및 20cm 후처리 병합 적용 완료");
    }

private:
    const float ALPHA_POS = 0.85f; 
    const float ALPHA_VEL = 0.15f; 
    const float DEADZONE_DIST = 0.01f;  
    const float ZERO_CLAMPING = 0.05f;  
    
    // [추가] 30cm 크기의 차량이 쪼개지는 것을 막기 위한 20cm 병합 임계값
    const float MERGE_THRESHOLD = 0.30f; // 후처리 병합 임계값 [단위=m]

    float get_adaptive_threshold(float dist) {
        if (dist < 1.5f) return 0.18f;
        return 0.28f;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        rclcpp::Time curr_time = this->now();
        double dt = (curr_time - last_time_).seconds();
        if (dt <= 0 || dt > 0.5) { last_time_ = curr_time; return; }

        // [수정] 바로 current_clusters로 보내지 않고 raw_clusters에 임시 저장
        std::vector<std::pair<float, float>> raw_clusters;
        std::vector<std::pair<float, float>> current_group;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            
            if (std::isnan(r) || std::isinf(r) || r <= 0.1f) {
                continue; 
            }

            float angle = msg->angle_min + i * msg->angle_increment;
            
            if (angle < -M_PI / 2.0f || angle > M_PI / 2.0f) {
                continue;
            }

            float pt_x = r * std::cos(angle);
            float pt_y = r * std::sin(angle);

            if (pt_x > 3.0f || pt_y < -1.0f || pt_y > 1.0f) {
                continue;
            }

            std::pair<float, float> current_pt = {pt_x, pt_y};

            if (current_group.empty()) {
                current_group.push_back(current_pt);
            } else {
                auto& last_pt = current_group.back();
                float dist = std::sqrt(std::pow(current_pt.first - last_pt.first, 2) + 
                                       std::pow(current_pt.second - last_pt.second, 2));
                
                float threshold = get_adaptive_threshold(r);

                if (dist < threshold) {
                    current_group.push_back(current_pt);
                } else {
                    if (current_group.size() >= 4) {
                        float sum_x = 0, sum_y = 0;
                        for (const auto& p : current_group) {
                            sum_x += p.first; sum_y += p.second;
                        }
                        raw_clusters.push_back({sum_x / current_group.size(), sum_y / current_group.size()});
                    }
                    current_group.clear();
                    current_group.push_back(current_pt);
                }
            }
        }

        if (current_group.size() >= 4) {
            float sum_x = 0, sum_y = 0;
            for (const auto& p : current_group) {
                sum_x += p.first; sum_y += p.second;
            }
            raw_clusters.push_back({sum_x / current_group.size(), sum_y / current_group.size()});
        }

        // [추가] 후처리 병합 로직 (Post-process Centroid Merging)
        std::vector<std::pair<float, float>> current_clusters;
        std::vector<bool> merged(raw_clusters.size(), false);

        for (size_t i = 0; i < raw_clusters.size(); ++i) {
            if (merged[i]) continue;
            float sum_x = raw_clusters[i].first;
            float sum_y = raw_clusters[i].second;
            int count = 1;

            for (size_t j = i + 1; j < raw_clusters.size(); ++j) {
                if (merged[j]) continue;
                
                // 두 클러스터 중심 간의 거리 계산
                float d = std::sqrt(std::pow(raw_clusters[i].first - raw_clusters[j].first, 2) + 
                                   std::pow(raw_clusters[i].second - raw_clusters[j].second, 2));
                
                // 중심 간 거리가 20cm 이내면 하나의 물체로 병합
                if (d < MERGE_THRESHOLD) {
                    sum_x += raw_clusters[j].first;
                    sum_y += raw_clusters[j].second;
                    count++;
                    merged[j] = true;
                }
            }
            // 병합된 좌표들의 평균 위치를 최종 클러스터로 등록
            current_clusters.push_back({sum_x / count, sum_y / count});
        }

        update_tracking(current_clusters, dt);
        publish_data(curr_time);
        last_time_ = curr_time;
    }

    void update_tracking(const std::vector<std::pair<float, float>>& clusters, double dt) {
        std::vector<bool> matched(clusters.size(), false);
        
        for (auto& obj : tracked_objects_) {
            float min_d = 0.5f; 
            int best_idx = -1;
            
            for (size_t i = 0; i < clusters.size(); ++i) {
                if (matched[i]) continue;
                float d = std::sqrt(std::pow(obj.x - clusters[i].first, 2) + std::pow(obj.y - clusters[i].second, 2));
                if (d < min_d) { min_d = d; best_idx = i; }
            }
            
            if (best_idx != -1) {
                float dx = clusters[best_idx].first - obj.x;
                float dy = clusters[best_idx].second - obj.y;
                float dist_diff = std::sqrt(dx * dx + dy * dy);

                float raw_vx = 0.0f;
                float raw_vy = 0.0f;

                if (dist_diff > DEADZONE_DIST) {
                    raw_vx = dx / dt;
                    raw_vy = dy / dt;
                }

                obj.vx = (1.0f - ALPHA_VEL) * obj.vx + ALPHA_VEL * raw_vx;
                obj.vy = (1.0f - ALPHA_VEL) * obj.vy + ALPHA_VEL * raw_vy;

                if (std::sqrt(obj.vx * obj.vx + obj.vy * obj.vy) < ZERO_CLAMPING) {
                    obj.vx = 0.0f;
                    obj.vy = 0.0f;
                }

                obj.x = (1.0f - ALPHA_POS) * obj.x + ALPHA_POS * clusters[best_idx].first;
                obj.y = (1.0f - ALPHA_POS) * obj.y + ALPHA_POS * clusters[best_idx].second;

                obj.age++; obj.miss_count = 0; matched[best_idx] = true;
            } else { 
                obj.miss_count++; 
            }
        }
        
        for (size_t i = 0; i < clusters.size(); ++i) {
            if (!matched[i]) tracked_objects_.push_back({next_id_++, clusters[i].first, clusters[i].second, 0, 0, 1, 0});
        }
        tracked_objects_.erase(std::remove_if(tracked_objects_.begin(), tracked_objects_.end(),
            [](const TrackedObject& o){ return o.miss_count > 3; }), tracked_objects_.end());
    }

    void publish_data(const rclcpp::Time& t) {
        geometry_msgs::msg::PoseArray pose_array;
        visualization_msgs::msg::MarkerArray marker_array;
        pose_array.header.frame_id = "laser";
        pose_array.header.stamp = t;

        visualization_msgs::msg::Marker delete_all;
        delete_all.header.frame_id = "laser";
        delete_all.header.stamp = t;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);

        for (const auto& obj : tracked_objects_) {
            if (obj.age < 3 || obj.miss_count > 0) continue;

            float speed = std::sqrt(obj.vx * obj.vx + obj.vy * obj.vy);
            float yaw = std::atan2(obj.vy, obj.vx);

            geometry_msgs::msg::Pose data_pose;
            data_pose.position.x = obj.x;
            data_pose.position.y = obj.y;
            data_pose.position.z = speed; 
            data_pose.orientation.x = static_cast<double>(obj.id); 
            data_pose.orientation.z = std::sin(yaw / 2.0);
            data_pose.orientation.w = std::cos(yaw / 2.0);
            pose_array.poses.push_back(data_pose);

            auto create_marker = [&](std::string ns, int id_offset, int type) {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = "laser"; 
                m.header.stamp = t;
                m.ns = ns; 
                m.id = obj.id + id_offset; 
                m.type = type; 
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x = obj.x; 
                m.pose.position.y = obj.y;
                m.pose.position.z = 0.0;
                m.pose.orientation.w = 1.0;
                m.lifetime = rclcpp::Duration::from_seconds(0.2); 
                return m;
            };

            auto arrow = create_marker("velocity", 2000, visualization_msgs::msg::Marker::ARROW);
            arrow.pose.orientation.z = std::sin(yaw / 2.0); 
            arrow.pose.orientation.w = std::cos(yaw / 2.0);
            arrow.scale.x = std::max(0.01f, std::min(speed * 0.3f, 0.7f)); 
            arrow.scale.y = 0.05; arrow.scale.z = 0.05;
            arrow.color.r = 1.0; arrow.color.a = 1.0;
            marker_array.markers.push_back(arrow);

            auto cube = create_marker("boxes", 0, visualization_msgs::msg::Marker::CUBE);
            cube.pose.position.z = 0.1; 
            cube.scale.x = 0.2; cube.scale.y = 0.2; cube.scale.z = 0.2;
            cube.color.g = 1.0; cube.color.a = 0.6;
            marker_array.markers.push_back(cube);

            auto text = create_marker("ids", 1000, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            text.pose.position.z = 0.4; 
            text.scale.z = 0.25;
            text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
            text.text = std::to_string(obj.id);
            marker_array.markers.push_back(text);
        }
        pub_poses_->publish(pose_array);
        pub_markers_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_poses_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    std::vector<TrackedObject> tracked_objects_;
    rclcpp::Time last_time_;
    int next_id_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode8>());
    rclcpp::shutdown();
    return 0;
}
