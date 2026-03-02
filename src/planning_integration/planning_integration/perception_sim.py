#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseArray, Pose, Point
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import math, time

class GlobalTrackSim(Node):
    def __init__(self):
        super().__init__('global_track_sim')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_track = self.create_publisher(MarkerArray, '/world/track', 10)
        self.pub_obs = self.create_publisher(PoseArray, '/world/obstacles', 10)
        self.create_timer(0.02, self.run) # 50Hz
        self.start_t = time.time()
        self.current_R = 5.0

    def run(self):
        t = time.time() - self.start_t
        angle = (t * 0.4) % (2 * math.pi)
        
        # 장애물 회피를 위한 반지름 보간
        target_R = 5.8 if abs(angle - math.pi/2) < 0.6 else 5.0
        self.current_R += (target_R - self.current_R) * 0.05
        
        now = self.get_clock().now().to_msg()
        x, y = self.current_R * math.cos(angle), self.current_R * math.sin(angle)
        yaw = angle + math.pi/2

        # TF 전송 (map -> base_link)
        t_fs = TransformStamped()
        t_fs.header.stamp, t_fs.header.frame_id, t_fs.child_frame_id = now, 'map', 'base_link'
        t_fs.transform.translation.x, t_fs.transform.translation.y = x, y
        t_fs.transform.rotation.z, t_fs.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.tf_broadcaster.sendTransform(t_fs)

        # 고정 트랙 및 장애물 발행
        ma = MarkerArray()
        for i, R in enumerate([5.0, 5.8]):
            m = Marker()
            m.header.frame_id, m.id, m.ns = 'map', i, 'tracks'
            m.type, m.action, m.scale.x = Marker.LINE_STRIP, Marker.ADD, 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 1.0, 0.2
            for a in range(0, 361, 10):
                rad = math.radians(a)
                m.points.append(Point(x=R*math.cos(rad), y=R*math.sin(rad), z=0.0))
            ma.markers.append(m)
        self.pub_track.publish(ma)

        obs = PoseArray()
        obs.header.frame_id = 'map'
        p = Pose(); p.position.x, p.position.y = 0.0, 5.0
        obs.poses.append(p)
        self.pub_obs.publish(obs)

if __name__ == '__main__':
    rclpy.init(); rclpy.spin(GlobalTrackSim()); rclpy.shutdown()
