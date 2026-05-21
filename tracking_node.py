import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import lcm
import threading
from simulator_lcmt import simulator_lcmt  # Import the generated LCM message

class LCMToROS2(Node):
    def __init__(self):
        super().__init__('lcm_to_ros2')
        self.publisher = self.create_publisher(PoseStamped, 'simulator_pose', 10)

        # Shared variable for storing the latest LCM message
        self.latest_msg = None
        self.lock = threading.Lock()  # Ensure thread safety when updating the message

        # Initialize LCM subscriber
        self.lc = lcm.LCM()
        self.subscription = self.lc.subscribe("simulator_state", self.lcm_callback)

        # Run LCM handling in a separate daemon thread
        self.lcm_thread = threading.Thread(target=self.lcm_listen, daemon=True)
        self.lcm_thread.start()

        # ROS2 Timer (acts as the main thread function)
        self.timer = self.create_timer(0.1, self.publish_ros_msg)  # Publish every 100ms
        print("Initialization done")

    def lcm_callback(self, channel, data):
        """LCM subscriber callback: Store received data in self.latest_msg."""
        msg = simulator_lcmt.decode(data)
        print("Callback triggered")
        
        with self.lock:
            self.latest_msg = msg  # Store the latest LCM message safely

    def publish_ros_msg(self):
        """ROS 2 Timer: Read the latest LCM message and publish it."""
        with self.lock:
            if self.latest_msg is None:
                return  # No new data to publish yet
            
            msg = self.latest_msg  # Copy latest message
        
        # Create ROS PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"  # Assuming global frame is "world"

        # Set position (global location x, y, z)
        pose_msg.pose.position.x = float(msg.p[0])
        pose_msg.pose.position.y = float(msg.p[1])
        pose_msg.pose.position.z = float(msg.p[2])

        # Set orientation (global roll, pitch, yaw)

        pose_msg.pose.orientation.x = float(msg.quat[1])  # Quaternion representation
        pose_msg.pose.orientation.y = float(msg.quat[2])
        pose_msg.pose.orientation.z = float(msg.quat[3])
        pose_msg.pose.orientation.w = float(msg.quat[0])

        # Publish the message
        self.publisher.publish(pose_msg)
        self.get_logger().info(
            f"Published pose: position=({msg.p[0]:.3f}, {msg.p[1]:.3f}, {msg.p[2]:.3f}) "
            f"orientation(quat)=({msg.quat[0]:.3f}, {msg.quat[1]:.3f}, {msg.quat[2]:.3f}, {msg.quat[3]:.3f})"
        )

    def lcm_listen(self):
        """LCM loop to listen for messages continuously (daemon thread)."""
        while True:
            self.lc.handle()

def main(args=None):
    rclpy.init(args=args)
    node = LCMToROS2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
