#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from ublox_gps import UbloxGps
import serial
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from gps_interfaces.msg import CarverNavSatFix
from geometry_msgs.msg import  TwistWithCovarianceStamped


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
        self.gps = UbloxGps(self.port)
        
        # qos_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     depth=30
        # )

        self.nav_sat_pub = self.create_publisher(CarverNavSatFix, 'nav_sat_data',10)
        self.twist_cov_stamped = self.create_publisher(TwistWithCovarianceStamped, 'gnss_dopler_vel',10)
        # self.gps_pub = self.create_publisher(Float64MultiArray,"gps_data", 10)

        self.create_timer(0.05,self.timer_callback)

    def timer_callback(self):
        self.navsat_publisher()  
        self.velned_publisher()

        # self.gps_publisher()


    def navsat_publisher(self):
        try:
            # Create a new instance of CarverNavSatFix message
            gps_msg = CarverNavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()  # Current ROS time
            gps_msg.header.frame_id = "gps_frame"  # Set the frame of reference

            # Retrieve high-precision geographic coordinates and status from the GPS
            geo = self.gps.hp_geo_coords()
            status = self.gps.geo_coords()

            # Populate CarverNavSatFix message fields
            gps_msg.latitude = geo.lat + (geo.latHp * 1e-2)  # Combine standard and high-precision lat
            gps_msg.longitude = geo.lon + (geo.lonHp * 1e-2)  # Combine standard and high-precision lon
            gps_msg.altitude = geo.hMSL / 1000.0  # Convert altitude from mm to meters

            # Set the position covariance based on hAcc and vAcc
            hAcc = geo.hAcc / 1000.0  # Horizontal accuracy in meters
            vAcc = geo.vAcc / 1000.0  # Vertical accuracy in meters
            gps_msg.position_covariance = [
                hAcc ** 2, 0.0, 0.0,
                0.0, hAcc ** 2, 0.0,
                0.0, 0.0, vAcc ** 2
            ]

            # Set position covariance type (assuming approximation here)
            gps_msg.position_covariance_type = CarverNavSatFix.COVARIANCE_TYPE_APPROXIMATED

            # Set GNSS fix type in status based on the provided fixType
            gps_msg.status.status = status.fixType

            # Set other fields if available in CarverNavSatFix, such as time or other GNSS parameters

            # Publish the message
            self.nav_sat_pub.publish(gps_msg)
            self.get_logger().info(f"Published GPS data: Lat: {gps_msg.latitude}, Lon: {gps_msg.longitude}, Alt: {gps_msg.altitude}")
   


        except (ValueError, IOError) as err:
            self.get_logger().error(err)
    
    def velned_publisher(self):

        try:
            velned_msg = TwistWithCovarianceStamped()
            velned = self.gps.vel_sol_ned()

            # Set the header
            # velned_msg.header.stamp = self.get_clock().now().to_msg()  # ROS time
            velned_msg.header.frame_id = "velned"

            # Set the linear velocities (converted to m/s)
            velned_msg.twist.twist.linear.x = velned.velN / 100.0  # North velocity in m/s
            velned_msg.twist.twist.linear.y = velned.velE / 100.0  # East velocity in m/s
            velned_msg.twist.twist.linear.z = velned.velD / 100.0  # Down velocity in m/s

            # Covariance (not sure)

            self.twist_cov_stamped.publish(velned_msg)

        except (ValueError, IOError) as err:
            self.get_logger().error(err)


    def gps_publisher(self):
        try:
            msg = Float64MultiArray()
            geo = self.gps.hp_geo_coords()
            status = self.gps.geo_coords()
            velned = self.gps.vel_sol_ned()

            nav_sat_timestamp = self.get_clock().now().seconds_nanoseconds()
            nav_sat_timestamp_float = nav_sat_timestamp[0] + nav_sat_timestamp[1] * 1e-9  # Convert to seconds
            # Populate CarverNavSatFix message fields


            latitudeHp = geo.lat + (geo.latHp * 1e-2)  # Combine standard and high-precision lat
            longitudeHp = geo.lon + (geo.lonHp * 1e-2)  # Combine standard and high-precision lon
            altitudeHp = geo.hMSL / 1000.0  # Convert altitude from mm to meters

            # Set the position covariance based on hAcc and vAcc
            hAcc = geo.hAcc / 1000.0  # Horizontal accuracy in meters
            vAcc = geo.vAcc / 1000.0  # Vertical accuracy in meters
            position_covariance = [
                hAcc ** 2.0, 0.0, 0.0,
                0.0, hAcc ** 2.0, 0.0,
                0.0, 0.0, vAcc ** 2.0
            ]

            # Set position covariance type (assuming approximation here)
            position_covariance_type = float(CarverNavSatFix.COVARIANCE_TYPE_APPROXIMATED)

            # Set GNSS fix type in status based on the provided fixType
            status = float(status.fixType)


            # Set the linear velocities (converted to m/s)
            lin_vel_ned_x= velned.velN / 100.0  # North velocity in m/s
            lin_vel_ned_y = velned.velE / 100.0  # East velocity in m/s
            lin_vel_ned_z = velned.velD / 100.0  # Down velocity in m/s


            # Prepare Float64MultiArray message
           
            # Populate the data field, ensuring each value is a float
            msg.data = [
                nav_sat_timestamp_float,   # Timestamp as a float in seconds
                latitudeHp,                # Latitude (high precision)
                longitudeHp,               # Longitude (high precision)
                altitudeHp,                # Altitude in meters
                hAcc,                      # Horizontal accuracy in meters
                vAcc,                      # Vertical accuracy in meters
                position_covariance[0],    # Covariance XX
                position_covariance[4],    # Covariance YY
                position_covariance[8],    # Covariance ZZ    
                position_covariance_type,  # Position covariance type
                status,                  # GNSS fix type
                lin_vel_ned_x,             # North velocity in m/s
                lin_vel_ned_y,             # East velocity in m/s
                lin_vel_ned_z              # Down velocity in m/s
            ]
            self.gps_pub.publish(msg)

        except (ValueError, IOError) as err:
            self.get_logger().error(err)

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

