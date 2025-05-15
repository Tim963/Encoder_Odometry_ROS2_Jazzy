import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from rclpy.time import Time
import math

class EncoderOdometry(Node):
    def __init__(self):
        super().__init__('encoder_odometry')
        self.robot_base_frame = "base_footprint" 
        # Korrigierte Parameter
        self.wheel_radius = 0.057 / 2.0  # 4.3cm Durchmesser → Radius (0.0215m)
        self.track_width = 0.89          # 15cm Spurweite
        self.wheel_joints = ['wheel_left_joint', 'wheel_right_joint']

        # Odometrie-Daten
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_positions = {}
        self.first_message = True

        # Subscriber und Publisher
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        self.odom_pub = self.create_publisher(Odometry, 'encoder_odometry', 10)

    def joint_state_callback(self, msg):
        try:
            left_idx = msg.name.index(self.wheel_joints[0])
            right_idx = msg.name.index(self.wheel_joints[1])
        except ValueError:
            self.get_logger().error("Joints not found!")
            return

        current_time = Time.from_msg(msg.header.stamp)
        
        if self.first_message:
            self.prev_positions = {
                'left': msg.position[left_idx],
                'right': msg.position[right_idx]
            }
            self.prev_time = current_time
            self.first_message = False
            return

        # Berechnung der Radbewegung
        delta_left = msg.position[left_idx] - self.prev_positions['left']
        delta_right = msg.position[right_idx] - self.prev_positions['right']
        
        delta_left_m = delta_left * self.wheel_radius
        delta_right_m = delta_right * self.wheel_radius
        
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        
        if dt <= 0:
            return

        # Odometrie-Berechnung
        delta_distance = (delta_left_m + delta_right_m) / 2.0
        delta_theta = (delta_right_m - delta_left_m) / self.track_width

        # Nur Position aktualisieren, wenn lineare Bewegung vorhanden
        self.yaw += delta_theta
        if abs(delta_distance) > 1e-6:
            self.x += 0.95 * delta_distance * math.cos((self.yaw*1.32))
            self.y += 0.95 * delta_distance * math.sin((self.yaw*1.32))

                    # Debug-Ausgabew
        self.get_logger().info(
            f"Position: x={self.x:.3f}m, y={self.y:.3f}m | "
            f"Yaw: {1.32*self.yaw:.3f}rad ({math.degrees(1.32*self.yaw):.1f}°) |",

            
            throttle_duration_sec=0.7  # Max. 10x pro Sekunde
        )
        
        # Yaw immer aktualisieren
        self.yaw += delta_theta
        
        # Quaternion aus Euler-Winkel
        #q = quaternion_from_euler(0, 0, (self.yaw)) original
        q = quaternion_from_euler(0, 0, (1.32*self.yaw))
        
        # Odometrie-Nachricht
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom_raw'
        odom_msg.child_frame_id = 'base_footprint'
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # Geschwindigkeiten
        odom_msg.twist.twist.linear.x = delta_distance / dt if dt > 0 else 0.0
        odom_msg.twist.twist.angular.z = delta_theta / dt if dt > 0 else 0.0

        self.odom_pub.publish(odom_msg)

        self.prev_positions = {
            'left': msg.position[left_idx],
            'right': msg.position[right_idx]
        }
        self.prev_time = current_time
       # yaw_deg = math.degrees(self.yaw)
      #  self.get_logger().info(f"Yaw: {1.2*self.yaw:.2f} rad ({yaw_deg:.1f}°)")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()