import paho.mqtt.client as mqtt_client
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandLong
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import json
import numpy as np
from math import sqrt, atan2, cos, sin, radians

# --- Updated Imports for EKF Origin (using GeoPointStamped publisher method) ---
from geographic_msgs.msg import GeoPointStamped # This is the message type to publish
# Note: mavros_msgs.srv.SetGPSGlobalOrigin is no longer needed for this approach
# -----------------------------------------------------------------------------

# MQTT Configuration
broker_address = "localhost"
port = 1883
mqtt_topic_subscribe = "motive/data"

# ROS Node Settings
ros_node_name = "motive_mqtt_to_ros_publisher"
ros_frame_id = "world"  # Ensure this matches your MAVROS setu

# Follower Drone Specific Settings
FOLLOWER_DRONE_ID = 2   # Follower drone ID in Motive
LEADER_DRONE_ID = 1     # Leader Drone ID in Motive
FOLLOW_OFFSET_X = -1.0  # Meters: behind the leader
FOLLOW_OFFSET_Y = 0.0   # Meters: side offset
FOLLOW_OFFSET_Z = 0.0   # Meters: above the leader
POSITION_TOLERANCE = 0.3  # Meters: tolerance for waypoint reached (retained for speed logic)
HEADING_TOLERANCE = 0.01  # Radians: tolerance for heading (retained for speed logic)
SETPOINT_RATE = 10.0      # Hz: setpoint publishing frequency
SPEED_HIGH = 5.0          # m/s: high speed when far
SPEED_LOW = 2.0           # m/s: low speed when close
BREAKING_DIST = 5.0       # m: distance to start slowing down
STALE_DATA_TIMEOUT = 1.0  # s: timeout for stale leader data

# --- EKF Origin Coordinates (your provided location) ---
# For indoor OptiTrack, these coordinates are arbitrary but provide a fixed reference
# for the EKF's local NED frame. Altitude is also required.
EKF_ORIGIN_LATITUDE = 51.7693529
EKF_ORIGIN_LONGITUDE = 14.3232767
EKF_ORIGIN_ALTITUDE = 0.0 # Meters above sea level (or chosen arbitrary ground level)
# --------------------------------------------------------

class MotiveMqttToRosPublisher(Node):

    def __init__(self):
        super().__init__(ros_node_name)
        self.get_logger().info(f"ROS 2 node '{ros_node_name}' initialized.")

        # QoS for publishing
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.setpoint_raw_local_publisher_follower = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", qos)
        self.pose_publisher_follower_mocap = self.create_publisher(PoseStamped, "/mavros/mocap/pose", qos)

        # --- New: Publisher for setting EKF global origin via topic ---
        # MAVROS listens to this topic to set the EKF origin
        self.set_gps_origin_pub = self.create_publisher(
            GeoPointStamped,
            "/mavros/global_position/set_gp_origin", # Topic name is fixed by MAVROS
            1 # QoS depth for this specific topic
        )
        # -----------------------------------------------------------

        # Subscribers
        self.state_sub = self.create_subscription(State, "/mavros/state", self.state_callback, qos)
        self.follower_pose_sub = self.create_subscription(Odometry, "/mavros/global_position/local", self.follower_pose_callback, qos)

        # Service Clients (CommandLong is still used for speed commands)
        self.speed_client = self.create_client(CommandLong, "/mavros/cmd/command")
        # Note: self.set_origin_client (for SetGPSGlobalOrigin service) is removed

        # State Variables
        self.current_state = State()
        self.leader_pose = PoseStamped()
        self.follower_pose = PoseStamped()
        self.last_leader_time = self.get_clock().now().to_msg().sec
        self.is_guided = False
        self.current_speed = SPEED_HIGH
        self.origin_set = False # Flag to ensure origin is set only once successfully

        # Timer for setpoint publishing
        self.setpoint_timer = self.create_timer(1.0 / SETPOINT_RATE, self.publish_setpoint)

        # MQTT Setup
        self.mqtt_client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1)
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect

        try:
            self.mqtt_client.connect(broker_address, port, 60)
            self.get_logger().info("Connecting to MQTT Broker...")
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")
            self.destroy_node()

    def state_callback(self, msg):
        """
        Callback for the /mavros/state topic.
        Updates the drone's current state and checks if it's in GUIDED mode.
        Also attempts to set EKF origin when GUIDED mode is entered.
        """
        self.current_state = msg
        previous_is_guided = self.is_guided # Store previous state to detect mode change
        self.is_guided = (msg.mode == "GUIDED")

        if self.is_guided:
            self.get_logger().info("Follower in GUIDED mode, ready to follow.")
            # --- New: Attempt to set EKF origin when GUIDED mode is first entered ---
            if not self.origin_set: # Only attempt if not already set
                self.set_ekf_origin(EKF_ORIGIN_LATITUDE, EKF_ORIGIN_LONGITUDE, EKF_ORIGIN_ALTITUDE)
            # ---------------------------------------------------------------------
        elif previous_is_guided: # Log when exiting GUIDED mode
            self.get_logger().info("Follower exited GUIDED mode.")
        else:
            self.get_logger().debug("Follower not in GUIDED mode.")

    def follower_pose_callback(self, msg: Odometry):
        """
        Callback for the /mavros/global_position/local topic.
        Updates the follower's current position using Odometry message.
        """
        self.follower_pose.header = msg.header
        self.follower_pose.pose = msg.pose.pose

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """
        MQTT connection callback. Subscribes to the data topic upon successful connection.
        """
        if rc == 0:
            self.get_logger().info("Connected to MQTT Broker.(script 2)")
            client.subscribe(mqtt_topic_subscribe)
        else:
            self.get_logger().error(f"Failed to connect to MQTT Broker, return code {rc}")

    def _on_mqtt_message(self, client, userdata, msg):
        """
        MQTT message callback. Processes incoming Motive data.
        """
        try:
            data = json.loads(msg.payload.decode())
            if isinstance(data, list):
                for obj in data:
                    self._process_motive_object(obj)
            else:
                self._process_motive_object(data)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in MQTT payload.")
        except Exception as e:
            self.get_logger().error(f"Error in processing MQTT message: {e}")

    def _process_motive_object(self, obj_data):
        """
        Processes individual object data received from Motive via MQTT.
        Transforms coordinates and updates leader/follower poses.
        """
        try:
            object_id = obj_data.get('id', None)
            if object_id is None:
                self.get_logger().warn("Received object without 'id' field.")
                return

            pos = obj_data['position']
            rot = obj_data['rotation']

            # Coordinate transformation from Motive (Y-up, X-forward) to ROS (Z-up, X-forward/East, Y-North)
            # This is an example transformation, ensure it matches your specific Motive and MAVROS setup.
            motive_pos_x = float(pos['x'])
            motive_pos_y = float(pos['y'])
            motive_pos_z = float(pos['z'])
            motive_quat_x = float(rot['x'])
            motive_quat_y = float(rot['y'])
            motive_quat_z = float(rot['z'])
            motive_quat_w = float(rot['w'])

            # Example transformation: Motive Y -> ROS -X, Motive X -> ROS Y, Motive Z -> ROS Z
            ros_pos_x = -motive_pos_y
            ros_pos_y = motive_pos_x
            ros_pos_z = motive_pos_z
            # Quaternions also need corresponding transformation
            ros_quat_x = -motive_quat_y
            ros_quat_y = motive_quat_x
            ros_quat_z = motive_quat_z
            ros_quat_w = motive_quat_w

            # Create PoseStamped message for the current object
            current_object_pose = PoseStamped()
            current_object_pose.header.stamp = self.get_clock().now().to_msg()
            current_object_pose.header.frame_id = ros_frame_id
            current_object_pose.pose.position.x = ros_pos_x
            current_object_pose.pose.position.y = ros_pos_y
            current_object_pose.pose.position.z = ros_pos_z
            current_object_pose.pose.orientation.x = ros_quat_x
            current_object_pose.pose.orientation.y = ros_quat_y
            current_object_pose.pose.orientation.z = ros_quat_z
            current_object_pose.pose.orientation.w = ros_quat_w

            if object_id == LEADER_DRONE_ID:
                self.leader_pose = current_object_pose
                self.last_leader_time = self.get_clock().now().to_msg().sec
                self.get_logger().info(f"Leader (ID {LEADER_DRONE_ID}) position: X={ros_pos_x:.2f}, Y={ros_pos_y:.2f}, Z={ros_pos_z:.2f}")

            elif object_id == FOLLOWER_DRONE_ID:
                self.pose_publisher_follower_mocap.publish(current_object_pose)
                self.get_logger().info(f"Follower (ID {FOLLOWER_DRONE_ID}) mocap position: X={ros_pos_x:.2f}, Y={ros_pos_y:.2f}, Z={ros_pos_z:.2f}")

        except KeyError as e:
            self.get_logger().error(f"Missing key: {e} in object: {obj_data}")
        except Exception as e:
            self.get_logger().error(f"Failed to process object data for ID {object_id}: {e}")

    def _on_mqtt_disconnect(self, client, userdata, rc):
        """
        MQTT disconnection callback. Logs a warning.
        """
        self.get_logger().warn(f"MQTT disconnected with code: {rc}")

    def send_speed_cmd(self, speed):
        """
        Sends a speed command (MAV_CMD_DO_CHANGE_SPEED) to the drone via MAVROS service.
        """
        while not self.speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /mavros/cmd/command service...')
        
        req = CommandLong.Request()
        req.command = 178
        req.param1 = 1.0
        req.param2 = float(speed)
        req.param3 = -1.0
        req.param4 = 0.0

        future = self.speed_client.call_async(req)
        future.add_done_callback(self.speed_cmd_callback)
        self.get_logger().info(f"Requested speed: {speed} m/s")

    def speed_cmd_callback(self, future):
        """
        Callback for the speed command service response. Logs the response or any errors.
        """
        try:
            response = future.result()
            self.get_logger().info(f"Speed command response: {response}")
        except Exception as e:
            self.get_logger().error(f"Speed command service call failed: {e}")

    # --- Updated: Function to set EKF Global Origin using PUBLISHER ---
    def set_ekf_origin(self, lat, lon, alt):
        """
        Publishes a GeoPointStamped message to /mavros/global_position/set_gp_origin
        to set the EKF global origin. This is crucial for GPS-denied navigation.
        """
        if self.origin_set:
            self.get_logger().info("EKF origin already set, skipping.")
            return

        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ros_frame_id # Or another appropriate frame_id
        msg.position.latitude = lat
        msg.position.longitude = lon
        msg.position.altitude = alt # Altitude is required

        self.set_gps_origin_pub.publish(msg)
        self.get_logger().info(
            f"Published EKF origin to /mavros/global_position/set_gp_origin: "
            f"Lat: {lat}, Lon: {lon}, Alt: {alt}"
        )
        # Assuming successful publication means it's set for now.
        # A more robust check might involve monitoring EKF status, but this is a start.
        self.origin_set = True
    # -----------------------------------------------------------------

    def publish_setpoint(self):
        """
        Calculates and publishes the desired setpoint for the follower drone at a fixed rate.
        This is the core of the leader-follower logic.
        """
        # Only publish setpoints if the drone is in GUIDED mode
        if not self.is_guided:
            self.get_logger().info("Not in GUIDED mode, skipping setpoint publishing.")
            return

        # Check if leader data is too old. If so, hold position (by not publishing new setpoints).
        current_time = self.get_clock().now().to_msg().sec
        if current_time - self.last_leader_time > STALE_DATA_TIMEOUT:
            self.get_logger().warn("Leader data stale, holding position.")
            return

        # Ensure valid leader pose has been received
        if self.leader_pose.header.stamp.sec == 0 and self.leader_pose.header.stamp.nanosec == 0:
            self.get_logger().warn("No valid leader pose received yet.")
            return

        # Construct PositionTarget message
        setpoint_msg = PositionTarget()
        setpoint_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_msg.header.frame_id = ros_frame_id
        setpoint_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED # Local North-East-Down frame

        # Apply predefined offsets to the leader's current position to get the follower's target position
        setpoint_msg.position.x = self.leader_pose.pose.position.x + FOLLOW_OFFSET_X
        setpoint_msg.position.y = self.leader_pose.pose.position.y + FOLLOW_OFFSET_Y
        setpoint_msg.position.z = self.leader_pose.pose.position.z + FOLLOW_OFFSET_Z

        # Align the follower's yaw with the leader's yaw
        leader_quat = (
            self.leader_pose.pose.orientation.x,
            self.leader_pose.pose.orientation.y,
            self.leader_pose.pose.orientation.z,
            self.leader_pose.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(leader_quat) # Extract yaw from leader's quaternion
        setpoint_msg.yaw = yaw

        # Dynamic speed adjustment based on the distance to the target setpoint
        # This allows the drone to slow down as it approaches its desired follow position
        distance = sqrt(
            (self.follower_pose.pose.position.x - setpoint_msg.position.x)**2 +
            (self.follower_pose.pose.position.y - setpoint_msg.position.y)**2 +
            (self.follower_pose.pose.position.z - setpoint_msg.position.z)**2
        )
        if distance > BREAKING_DIST:
            new_speed = SPEED_HIGH
        elif distance > POSITION_TOLERANCE:
            # Linear interpolation for speed between SPEED_LOW and SPEED_HIGH
            new_speed = SPEED_LOW + (SPEED_HIGH - SPEED_LOW) * (distance - POSITION_TOLERANCE) / (BREAKING_DIST - POSITION_TOLERANCE)
        else:
            new_speed = SPEED_LOW

        # Only send a speed command if the new speed significantly differs from the current commanded speed
        if abs(new_speed - self.current_speed) > 0.1:
            self.send_speed_cmd(new_speed)
            self.current_speed = new_speed

        # Set type mask to ignore velocity, acceleration, and yaw rate,
        # indicating that only position and yaw are being commanded.
        setpoint_msg.type_mask = PositionTarget.IGNORE_VX | \
                                 PositionTarget.IGNORE_VY | \
                                 PositionTarget.IGNORE_VZ | \
                                 PositionTarget.IGNORE_AFX | \
                                 PositionTarget.IGNORE_AFY | \
                                 PositionTarget.IGNORE_AFZ | \
                                 PositionTarget.IGNORE_YAW_RATE

        self.setpoint_raw_local_publisher_follower.publish(setpoint_msg)
        self.get_logger().info( # Changed to info for better visibility
            f"Follower setpoint: X={setpoint_msg.position.x:.2f}, "
            f"Y={setpoint_msg.position.y:.2f}, Z={setpoint_msg.position.z:.2f}, "
            f"Yaw={setpoint_msg.yaw:.2f}, Speed={self.current_speed:.2f}"
        )

    def destroy_node(self):
        """ 
        Cleans up resources (MQTT client, ROS node) when the node is destroyed.
        """
        if self.mqtt_client:
            self.mqtt_client.loop_stop() # Stop the MQTT network lo
            self.mqtt_client.disconnect() # Disconnect from the MQTT broker
            self.get_logger().info("MQTT client disconnected.")
        super().destroy_node() # Call the base class's destroy_node method

def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    rclpy.init(args=args) # Initialize ROS 2
    node = MotiveMqttToRosPublisher() # Create an instance of the node
    try:
        rclpy.spin(node) # Keep the node alive and processing callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt.")
    finally:
        node.destroy_node() # Clean up resources
        rclpy.shutdown() # Shut down ROS 2

if __name__ == '__main__':
    main()
