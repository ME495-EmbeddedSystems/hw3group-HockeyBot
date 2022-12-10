import unittest
import pytest
from launch import LaunchDescription
import launch
import launch_ros
import launch_ros.actions
from launch_ros.actions import Node
import launch_testing.actions
import launch_testing
import rclpy
import time
from geometry_msgs.msg import PointStamped
@pytest.mark.rostest
def generate_test_description():
    hockeybot = Node(package = 'hockeybot', 
                    executable='traj_calc')
    return (
        LaunchDescription([
            hockeybot,
            launch_testing.actions.ReadyToTest() 
            ]),
            {'main': hockeybot } # this is a way to pass the node action to the test case
            )
class TestMyTestCaseName(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Runs one time, when the testcase is loaded"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """ Runs one time, when testcase is unloaded"""
        rclpy.shutdown()

    def setUp(self):
        """Runs before every test"""
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        """Runs after every test"""
        self.node.destroy_node()

    def test_my_test(self):
        """In UnitTest, any function starting with "test" is run as a test"""
        msgs_rx = []

        self.sub = self.node.create_subscription(
            PointStamped,
            "/waypoint1",
            lambda msg: msgs_rx.append(msg),
            10
        )
        try:
            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            self.assertAlmostEqual(10 / len(msgs_rx), 0.1,  1)

        finally:
            self.tearDown()
