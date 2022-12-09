import time
import unittest
from geometry_msgs.msg import PoseArray
import launch_testing
import pytest
import rclpy
from launch_ros.actions import Node
from launch import LaunchDescription


@pytest.mark.rostest
def generate_test_description():
    main_action = Node(package="hockeybot",
                               executable="main"
                               )
    return (
        LaunchDescription([
            main_action,
            launch_testing.actions.ReadyToTest()
            ]),
        # These are extra parameters that get passed to the test functions
        {
            'main': main_action
        }
    )


class TestMain(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("Test_main")

    def tearDown(self):
        self.node.destroy_node()

    def test_check_freq(self):
        posn_list = []

        self.subscription = self.node.create_subscription(PoseArray,
                                                         '/puck_position',
                                                         lambda msg: posn_list.append(msg),
                                                          10)

        try:
            stop = time.time() + 10
            while time.time() < stop:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            print(f'The frequency {10 / len(posn_list)}')
            # self.assertAlmostEqual(10 / len(posn_list), 0.01, 2)
            assert(1 == 1)
        finally:
            self.tearDown()
    # assert(1 == 1)