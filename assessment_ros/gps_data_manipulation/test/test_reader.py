import os
import sys
import time
import pytest
import unittest
import uuid # might have to remove this.

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions

import rclpy

from std_msgs.msg import Float64
from gps_interfaces.msg import GPS

# Launching the Nodes for testing
@pytest.mark.rostest
def generate_test_description():
    
    file_path = os.path.dirname(__file__)

    publisher_node = launch_ros.actions.Node(
        executable = sys.executable,
        arguments = [os.path.join(file_path, "..", "gps_data_manipulation", "data_read.py")],
        parameters = [{
            "file_path" : os.path.join(file_path, "..", "data", "GPS_Dataset_test.csv")
        }]
    )

    processor_node = launch_ros.actions.Node(
        executable = sys.executable,
        arguments = [os.path.join(file_path, "..", "gps_data_manipulation", "data_process.py")],
    )

    return (
        launch.LaunchDescription([
            publisher_node,
            processor_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'reader' : publisher_node,
            'processor' : processor_node
        }
    )

# Testing the Nodes
class TestReaderProcessor(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        """
        Initializing the ROS for the test node.
        """
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        """
        Shutting down the ROS.
        """
        rclpy.shutdown()

    def setUp(self) -> None:
        """
        Setting up a ROS node for each test case.
        """
        self.node = rclpy.create_node('test_reader_processor', parameter_overrides = list())

    def tearDown(self) -> None:
        """
        Destroying the created node after each test case.
        """
        self.node.destroy_node()

    def test_reader(self, reader, proc_output) -> None:

        msg_tx = list()
        subscriber_reader = self.node.create_subscription(
            GPS,
            'data',
            lambda msg : msg_tx.append(msg),
            10 
        )

        try:
            
            # Spinning to receive the data from teh 'data' topic.
            end_time = time.time() + 5

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec = 0.1)

            # Checking whether we received all the 5 messages.
            self.assertAlmostEqual(len(msg_tx), 5, 1)

            # Checking if the first message is equal to the one transmitted.
            self.assertAlmostEqual(msg_tx[0].longitude, -105.434187, delta = 0.5)

        finally:
            self.node.destroy_subscription(subscriber_reader)
