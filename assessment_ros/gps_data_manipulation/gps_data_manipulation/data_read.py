"""
A simple publisher node that reads and publishes the data from the CSV file.

Author  : Aneesh Chodisetty
Date    : 07/13/2023
"""

import rclpy
from rclpy.node import Node
from gps_interfaces.msg import GPS

import os
import csv
import datetime

class GPSDataReader(Node):
    """
    The GPSDataReader class is resposible for reading the CSV data and publishing it to the '/data' topic.
    """

    # Protected members
    _csv_data = list()

    def __init__(self, node_name: str = 'gps_data_reader', file_path: str = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))), 'data', 'GPS_Dataset_test.csv')) -> None:
        """
        The default constructor. Initializes the node.

        Args:
            node_name: The name of the reader node. Defaults to 'gps_data_reader'.
            file_path: The file path from which the GPS data is read. Defaults to the test file in the install folder.
        """
        super().__init__(node_name, parameter_overrides = list())

        self.declare_parameter("file_path", value = file_path)
        self.file_path_ = self.get_parameter("file_path").get_parameter_value().string_value

        self.publisher_ = self.create_publisher(GPS, 'data', 10)
        rate = 1/3.0 # 3 Hz
        self.timer_ = self.create_timer(rate, self.data_publisher_callback)

    def csv_reader(self) -> None:
        """
        A method to read the CSV file from the store file path.
        """
        first_flag = True

        with open(self.file_path_, 'r') as csv_file:
            csv_data = csv.reader(csv_file)
            for row in csv_data:
                if first_flag:
                    first_flag = False
                    continue
                self._csv_data.append(row[1:6]) # Storing only the necessary data.
        self.get_logger().info("Read the CSV file.")

    def set_csv_data(self, csv_data) -> None:
        """
        A method to manually set the csv_data.

        Args:
            csv_data: The CSV data to be stored.
        """         
        self._csv_data = csv_data

    def get_csv_data(self) -> list:
        """
        A method to return the stored CSV data.

        Returns:
            The stored CSV data.
        """
        return self._csv_data

    def set_file_path(self, file_path) -> None:
        """
        A method to set the file path of the CSV data. This path has to be set before spinning the node.

        Args:
            file_path: The CSV file path.
        """
        self.file_path_ = file_path

    def get_file_path(self) -> str:
        """
        A method to return the stored file path.

        Returns:
            The stored file path.
        """
        return self.file_path_

    def data_publisher_callback(self) -> None:
        """
        A callback method to publish the stored CSV data when available.
        """
        try:
            msg = GPS()
            current_data = self._csv_data.pop(0)

            # Converting the CSV data to float.
            msg.longitude = float(current_data[0])
            msg.latitute = float(current_data[1])
            msg.altitude = float(current_data[2])
            msg.time = datetime.datetime.strptime(current_data[3], "%Y-%m-%d %H:%M:%S+00:00").timestamp()
            msg.actual_speed = float(current_data[4])
            self.publisher_.publish(msg)
        except:
            self.get_logger().info("End of CSV data.")


def main(args=None):

    # Initializing the rclpy library.
    rclpy.init(args=args)
    
    # Creating a GPSDataReader Node.
    gps_data_reader = GPSDataReader()

    # Reading the data from the CSV file.
    gps_data_reader.csv_reader()

    # Spinning the node for the callback.
    rclpy.spin(gps_data_reader)
    
    gps_data_reader.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()