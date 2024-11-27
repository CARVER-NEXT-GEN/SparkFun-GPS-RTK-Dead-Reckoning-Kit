#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from ublox_gps import UbloxGps
import serial
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray,Float64MultiArray
# from gps_interfaces.msg import CarverNavSatFix
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import  TwistWithCovarianceStamped
import time


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
        self.gps = UbloxGps(self.port)
        if self.port.isOpen():
            print("Serial port is open and ready.")
        else:
            print("Serial port is not open.")
        self.nav_sat_pub = self.create_publisher(NavSatFix, 'nav_sat_data',10)
        self.create_timer(0.05,self.timer_callback)

        # For Fix-Position Covariance

        # try:
        #     self.cov = self.gps.nav_cov_matrice()
        #     self.p_NN = self.cov.posCovNN
        #     self.p_NE = self.cov.posCovNE
        #     self.p_ND = self.cov.posCovND
        #     self.p_EE = self.cov.posCovEE
        #     self.p_ED = self.cov.posCovED
        #     self.p_DD = self.cov.posCovDD      
        #     print(self.p_NN,self.p_NE,self.p_ND,self.p_EE,self.p_ED,self.p_DD,)
        # except Exception as err:
        #             self.get_logger().error(f"Unexpected error: {err}")

        self.gps_msg = NavSatFix()

    def timer_callback(self):
        self.navsat_publisher()

    def navsat_publisher(self):

        # For Periodic Position Covariance (still has an issue; Can't publish the data overtime)
        try:
            self.cov = self.gps.nav_cov_matrice()
            self.p_NN = self.cov.posCovNN
            self.p_NE = self.cov.posCovNE
            self.p_ND = self.cov.posCovND
            self.p_EE = self.cov.posCovEE
            self.p_ED = self.cov.posCovED
            self.p_DD = self.cov.posCovDD      
            # print(self.p_NN,self.p_NE,self.p_ND,self.p_EE,self.p_ED,self.p_DD,)
        except AttributeError as err:
            self.get_logger().error(f"AttributeError: {err}")
        except IOError as err:
            self.get_logger().error(f"I/O Error: {err}")
        except Exception as err:
            self.get_logger().error(f"Unexpected error: {err}")

       
        try:
            
            self.gps_msg.header.stamp = self.get_clock().now().to_msg()
            self.gps_msg.header.frame_id = "gps_frame"
            

            # Retrieve GPS data
            self.geo = self.gps.geo_coords()
            

            # Fill GPS message
            self.gps_msg.latitude = self.geo.lat
            self.gps_msg.longitude = self.geo.lon
            self.gps_msg.altitude = float(self.geo.height)

            # Map GNSS fix type
            self.gps_msg.status.status = int(self.geo.fixType)

            # Covariance matrix       
            self.gps_msg.position_covariance = [
                    self.p_NN, self.p_NE, -self.p_ND,
                    self.p_NE, self.p_EE, -self.p_ED,
                    -self.p_ND, -self.p_ED, self.p_DD
                ]
            

            self.gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            # Publish GPS message
            self.nav_sat_pub.publish(self.gps_msg)
            self.get_logger().info(f"Published GPS data: Lat: {self.gps_msg.latitude}, Lon: {self.gps_msg.longitude}, Alt: {self.gps_msg.altitude}")

        except AttributeError as err:
            self.get_logger().error(f"AttributeError: {err}")
        except IOError as err:
            self.get_logger().error(f"I/O Error: {err}")
        except Exception as err:
            self.get_logger().error(f"Unexpected error: {err}")

   
    
def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

