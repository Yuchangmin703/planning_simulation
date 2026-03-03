#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import random
import cv2
import numpy as np
from cv_bridge import CvBridge

from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

# 🌟 추가: perception 메시지 임포트
from perception.msg import Lanes, Lane

class CircuitSilSim(Node):
    def __init__(self):
        super().__init__('circuit_sil_sim')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_bev = self.create_publisher(Image, '/perception/bev/image', 10)
        self.pub_ego_speed = self.create_publisher(Float32, '/ego_speed', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/world/npc_markers', 10)
        self.pub_track = self.create_publisher(MarkerArray, '/world/track', 10)
        
        # 🌟 추가: 완벽한 정답(Ground Truth) 차선 데이터를 플래닝 노드에 직접 쏴주는 퍼블리셔
        self.pub_gt_lanes = self.create_publisher(Lanes, '/perception/lane/lanes', 10)
        
        self.sub_path = self.create_subscription(Path, '/planning/local_path', self.path_cb, 10)
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.033, self.run_step)
        
        self.R_CENTER = 10.0
        self.LANE_WIDTH = 0.3
        self.LANE_RADII = [self.R_CENTER - self.LANE_WIDTH, self.R_CENTER, self.R_CENTER + self.LANE_WIDTH]
        
        self.ego_v = 1.0
        self.ego_x = self.R_CENTER
        self.ego_y = 0.0
        self.ego_yaw = math.pi/2
        
        self.target_yaw_rate = 0.0
        self.last_path_time = 0.0
        
        self.npcs = []
        for lane_idx in range(3):
            for i in range(4):
                self.npcs.append({
                    'lane': lane_idx,
                    'angle': (i * (math.pi / 2)) + random.uniform(-0.2, 0.2),
                    'v': random.uniform(0.5, 0.8),
                    'width': random.uniform(0.15, 0.25),
                    'length': random.uniform(0.15, 0.25)
                })

    def path_cb(self, msg):
        if len(msg.poses) > 3:
            target = msg.poses[-1].pose.position
            for p in msg.poses:
                if p.pose.position.x > 1.0:
                    target = p.pose.position
                    break
                    
            tx = target.x
            ty = target.y
            Ld = math.hypot(tx, ty)
            
            if Ld > 0.1:
                alpha = math.atan2(ty, tx)
                self.target_yaw_rate = (2.0 * self.ego_v * math.sin(alpha)) / Ld
            else:
                self.target_yaw_rate = 0.0
                
            self.ego_v = target.z
            self.last_path_time = self.get_clock().now().nanoseconds / 1e9

    def run_step(self):
        dt = 0.033
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if current_time - self.last_path_time < 0.5:
            self.ego_yaw += self.target_yaw_rate * dt
            self.ego_x += self.ego_v * math.cos(self.ego_yaw) * dt
            self.ego_y += self.ego_v * math.sin(self.ego_yaw) * dt
        else:
            current_angle = math.atan2(self.ego_y, self.ego_x)
            current_angle += (1.0 / self.R_CENTER) * dt
            self.ego_x = self.R_CENTER * math.cos(current_angle)
            self.ego_y = self.R_CENTER * math.sin(current_angle)
            self.ego_yaw = current_angle + math.pi/2
            
        for npc in self.npcs:
            R = self.LANE_RADII[npc['lane']]
            npc['angle'] = (npc['angle'] + (npc['v'] / R) * dt) % (2 * math.pi)
            
        self.publish_tf()
        self.publish_track()
        self.pub_ego_speed.publish(Float32(data=self.ego_v))
        self.publish_fake_lidar()
        self.publish_fake_bev_image()
        self.publish_npc_markers()
        
        # 🌟 추가: 매 스텝마다 완벽한 차선 궤적을 계산해서 퍼블리시
        self.publish_gt_lanes()

    def publish_gt_lanes(self):
        msg = Lanes()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # 맵의 중심(0,0)을 기준으로 한 차량의 현재 각도
        current_angle = math.atan2(self.ego_y, self.ego_x)

        # 3개의 차선(안쪽, 중간, 바깥쪽)에 대해 모두 좌표 생성
        for R in self.LANE_RADII:
            lane = Lane()
            
            # 차량 위치부터 전방 약 10m 앞까지 0.5m 간격으로 점을 생성
            for d in range(0, 20):
                dist = d * 0.5
                theta = current_angle + (dist / R)

                # 1. 맵 기준 절대 좌표 (Map Frame)
                gx = R * math.cos(theta)
                gy = R * math.sin(theta)

                # 2. 차량 기준 상대 좌표로 변환 (Base_link Frame)
                dx = gx - self.ego_x
                dy = gy - self.ego_y

                local_x = dx * math.cos(-self.ego_yaw) - dy * math.sin(-self.ego_yaw)
                local_y = dx * math.sin(-self.ego_yaw) + dy * math.cos(-self.ego_yaw)

                # 차량보다 뒤에 있는 점은 버림
                if local_x >= -0.5:
                    pt = Point()
                    pt.x = float(local_x)
                    pt.y = float(local_y)
                    pt.z = 0.0
                    lane.points.append(pt)

            msg.lanes.append(lane)

        self.pub_gt_lanes.publish(msg)

    def publish_track(self):
        ma = MarkerArray()
        for i, R in enumerate(self.LANE_RADII):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'track'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.02
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.5
            for a in range(0, 361, 5):
                rad = math.radians(a)
                m.points.append(Point(x=R*math.cos(rad), y=R*math.sin(rad), z=0.0))
            ma.markers.append(m)
        self.pub_track.publish(ma)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.ego_x
        t.transform.translation.y = self.ego_y
        t.transform.rotation.z = math.sin(self.ego_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.ego_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

    def publish_fake_lidar(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = math.radians(1.0)
        scan.range_min = 0.1
        scan.range_max = 8.0
        
        num_rays = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        ranges = [float('inf')] * num_rays
        
        for npc in self.npcs:
            R = self.LANE_RADII[npc['lane']]
            nx = R * math.cos(npc['angle'])
            ny = R * math.sin(npc['angle'])
            
            dx, dy = nx - self.ego_x, ny - self.ego_y
            ex = dx * math.cos(-self.ego_yaw) - dy * math.sin(-self.ego_yaw)
            ey = dx * math.sin(-self.ego_yaw) + dy * math.cos(-self.ego_yaw)
            
            dist = math.hypot(ex, ey)
            angle = math.atan2(ey, ex)
            
            if scan.angle_min <= angle <= scan.angle_max and dist < scan.range_max:
                center_idx = int((angle - scan.angle_min) / scan.angle_increment)
                spread_idx = int((npc['width'] / dist) / scan.angle_increment) 
                
                for i in range(max(0, center_idx - spread_idx), min(num_rays, center_idx + spread_idx + 1)):
                    if dist < ranges[i]:
                        ranges[i] = dist
                        
        scan.ranges = ranges
        self.pub_scan.publish(scan)

    def publish_fake_bev_image(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        dx, dy = 0.0 - self.ego_x, 0.0 - self.ego_y
        cx = dx * math.cos(-self.ego_yaw) - dy * math.sin(-self.ego_yaw)
        cy = dx * math.sin(-self.ego_yaw) + dy * math.cos(-self.ego_yaw)
        
        mpp = 0.005
        img_cx = int(320 - cy / mpp)
        img_cy = int(480 - cx / mpp)
        
        r_outer = int((self.R_CENTER + self.LANE_WIDTH * 1.5) / mpp)
        cv2.circle(img, (img_cx, img_cy), r_outer, (0, 255, 255), 3) 
        
        r_mid1 = int((self.R_CENTER + self.LANE_WIDTH * 0.5) / mpp)
        r_mid2 = int((self.R_CENTER - self.LANE_WIDTH * 0.5) / mpp)
        cv2.circle(img, (img_cx, img_cy), r_mid1, (255, 255, 255), 2)
        cv2.circle(img, (img_cx, img_cy), r_mid2, (255, 255, 255), 2)
        
        r_inner = int((self.R_CENTER - self.LANE_WIDTH * 1.5) / mpp)
        cv2.circle(img, (img_cx, img_cy), r_inner, (255, 255, 255), 3)

        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.pub_bev.publish(msg)

    def publish_npc_markers(self):
        ma = MarkerArray()
        for i, npc in enumerate(self.npcs):
            R = self.LANE_RADII[npc['lane']]
            nx = R * math.cos(npc['angle'])
            ny = R * math.sin(npc['angle'])
            
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'npcs', i, Marker.CUBE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = nx, ny, 0.1
            yaw = npc['angle'] + math.pi/2
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            m.scale.x, m.scale.y, m.scale.z = npc['length'], npc['width'], 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.8
            ma.markers.append(m)
            
        m_ego = Marker()
        m_ego.header.frame_id = 'map' 
        m_ego.header.stamp = self.get_clock().now().to_msg()
        m_ego.ns, m_ego.id, m_ego.type, m_ego.action = 'ego', 99, Marker.CUBE, Marker.ADD
        m_ego.pose.position.x = self.ego_x
        m_ego.pose.position.y = self.ego_y
        m_ego.pose.position.z = 0.1
        m_ego.pose.orientation.z = math.sin(self.ego_yaw/2.0)
        m_ego.pose.orientation.w = math.cos(self.ego_yaw/2.0)
        m_ego.scale.x, m_ego.scale.y, m_ego.scale.z = 0.3, 0.2, 0.2
        m_ego.color.r, m_ego.color.g, m_ego.color.b, m_ego.color.a = 0.0, 1.0, 0.0, 1.0
        ma.markers.append(m_ego)
        
        self.pub_markers.publish(ma)

if __name__ == '__main__':
    rclpy.init()
    node = CircuitSilSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
