"""
A simple subcriber node that subscribes to the data topic, computes the difference

Author  : Aneesh Chodisetty
Date    : 07/13/2023
"""
import rclpy
from rclpy.node import Node
from gps_interfaces.msg import GPS
from std_msgs.msg import Float64

class GPSDataProcessor(Node):
    """
    The GPSDataProcessor class is responsible for subscribing to the data topic and computing the time difference.
    """
    
    __first_flag = True
    __previous_data = GPS()
    __current_data = GPS()

    def __init__(self, node_name: str = 'gps_data_processor') -> None:
        
        super().__init__(node_name, parameter_overrides = list())

        self.subscriber_ = self.create_subscription(GPS, 'data', self.data_process_callback, 10)
        self.diff_publisher_ = self.create_publisher(Float64, 'diff', 10)

    def data_process_callback(self, msg) -> None:
        if self.__first_flag:
            self.__first_flag = False
            self.__previous_data = msg
        else:
            self.__current_data = msg
            diff = Float64()
            diff.data = self.__current_data.time - self.__previous_data.time
            self.get_logger().info(f'The difference is : {diff.data}')
            self.__previous_data = self.__current_data
            self.diff_publisher_.publish(diff)
    
    def get_current_difference(self) -> float:
        return self.__current_data.time - self.__previous_data.time

def main(args=None):
    rclpy.init(args=args)
    
    gps_data_processor = GPSDataProcessor()
    rclpy.spin(gps_data_processor)
    
    gps_data_processor.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()