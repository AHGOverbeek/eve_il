#!/usr/bin/env python3

import rclpy
import rclpy.qos
from builtin_interfaces.msg import Duration
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from rclpy.node import Node
from halodi_msgs.msg import (
    HandCommand,
    ReferenceFrameName,
    TaskSpaceCommand,
    TrajectoryInterpolation,
    WholeBodyTrajectory,
    WholeBodyTrajectoryPoint,
)
import time
from geometry_msgs.msg import Twist

from numpy import diff
from itertools import accumulate

sim = True

def dprint(arg):
    print(['{:.3f}'.format(x) for x in arg])

def generate_task_space_command_msg(
    body_frame_id, expressed_in_frame_id, xyzxyzw, z_up=False
):

    msg_ = TaskSpaceCommand(express_in_z_up=z_up)
    msg_.body_frame.frame_id = body_frame_id
    msg_.expressed_in_frame.frame_id = expressed_in_frame_id

    msg_.pose.position.x = xyzxyzw[0][0]
    msg_.pose.position.y = xyzxyzw[0][1]
    msg_.pose.position.z = xyzxyzw[0][2]
    msg_.pose.orientation.x = xyzxyzw[1][0]
    msg_.pose.orientation.y = xyzxyzw[1][1]
    msg_.pose.orientation.z = xyzxyzw[1][2]
    msg_.pose.orientation.w = xyzxyzw[1][3]

    return msg_

class DemoNode(Node):

    def __init__(self):
        super().__init__(
            "grasp_publisher"
        )

        # State of hand
        self.grasp_open_bool = True

        self.imitate_first = True

        # 
        self.press_number = 0

        self.stage = False
        self.stage_previous = False

        # Time on closing and opening, variable length
        self.state_list = []
        self.time_list = []
        self.dtime_list = []
        self.cumulative_list = []
        self.cumulative_list_s = []
        self.ltimes = []
        self.lstates = []

        # Publishers for grasping
        self.publisher_thumb_r_ = self.create_publisher(HandCommand, '/bebionic/right/finger_thumb', 10)
        self.publisher_index_r_ = self.create_publisher(HandCommand, '/bebionic/right/finger_index', 10)
        self.publisher_middle_r_ = self.create_publisher(HandCommand, '/bebionic/right/finger_middle', 10)
        self.publisher_ring_r_ = self.create_publisher(HandCommand, '/bebionic/right/finger_ring', 10)
        self.publisher_little_r_ = self.create_publisher(HandCommand, '/bebionic/right/finger_little', 10)

        self.publisher_thumb_l_ = self.create_publisher(HandCommand, '/bebionic/left/finger_thumb', 10)
        self.publisher_index_l_ = self.create_publisher(HandCommand, '/bebionic/left/finger_index', 10)
        self.publisher_middle_l_ = self.create_publisher(HandCommand, '/bebionic/left/finger_middle', 10)
        self.publisher_ring_l_ = self.create_publisher(HandCommand, '/bebionic/left/finger_ring', 10)
        self.publisher_little_l_ = self.create_publisher(HandCommand, '/bebionic/left/finger_little', 10)

        # Listener to button during demonstration
        self.subscriber_teleop = self.create_subscription(Twist, "/cmd_vel", self.teleop_cb, 10)

        # Create a listener/subscriber for tf kinematics and save in a buffer 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for ee kinematics
        self.publisher_ee = self.create_publisher(
            WholeBodyTrajectory, "/eve/whole_body_trajectory", rclpy.qos.qos_profile_action_status_default 
        )

        # Create clock subscriber
        self.clock = self.get_clock()

    def grasp_toggle(self):
        if self.grasp_open_bool:
            self.grasp_close()
        else:
            self.grasp_open()


    def record(self):

        # Every time keypress, open or close
        self.grasp_toggle()

        #!!
        if sim:
            if self.press_number >= 5:
                s = [[0.2, 0.2, 0.1],[0.0, 0.71, 0.0, -0.71]]
                self.move_l_hand([s], [2])
            elif self.press_number >= 4:
                s = [[0.4, 0.2, 0.1],[0.0, 0.71, 0.0, -0.71]]
                self.move_l_hand([s], [2])
            elif self.press_number >= 3:
                s = [[0.4, 0.4, 0.1],[0.0, 0.71, 0.0, -0.71]]
                self.move_l_hand([s], [2])
        #!!

        # And add to the list of states and times
        self.state_list.append(self.get_ee())
        self.time_list.append(self.clock.now().seconds_nanoseconds()[0])


    def timer_callback(self):
        # Stage is a bool of whether even or uneven stage
        t = self.clock.now().seconds_nanoseconds()[0] - self.ti
        self.stage = bool(sum([t > te for te in self.cumulative_list_s])%2)

        # If there is a change in stage
        if self.stage is not self.stage_previous:
            self.grasp_toggle()
        
        self.stage_previous = self.stage


    def imitate(self):
        # All future command for ee kinematics can be made ebforehand
        # Space the time and state vector out to allow grasping time

        # Double each element in states [s1,s2,...] -> [s1,s1,s2,s2,...]
        self.lstates = [s for s in self.state_list for _ in (0,1)]
        self.lstates = self.lstates[0:-1]

        # Add a delay before each entry in dtimelist to wait for the grasp to open/close
        self.dtime_list = diff(self.time_list).tolist()
        self.ltimes = [t for t in self.dtime_list for _ in (0,1)]
        for i in range(0, len(self.ltimes), 2):
            self.ltimes[i] = 2
        self.ltimes.append(2)

        # Future ee commands can be sent immediately due to trajectory API
        self.move_l_hand(self.lstates, self.ltimes)


        # But grasp commands must be send at runtime

        # Make  cumulative list of longtimes for grasping timing
        self.cumulative_list = list(accumulate(self.ltimes))
        self.cumulative_list_s = self.cumulative_list[::2]
        # create timer for when to send commands at runtime
        self.grasp_close()
        self.ti = self.clock.now().seconds_nanoseconds()[0]
        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)


    def teleop_cb(self, msg):
        print("\n")

        # Increment 
        self.press_number += 1

        # Imitate
        if self.press_number >= 8:
            self.imitate()
            print("Imitating")

        # Wait
        elif self.press_number >= 7:
            print("Paused, enable trajectory API for imitation")

        # Record
        elif self.press_number >= 3:
            print("Marked point")
            self.record()

        # Moving if demo
        elif self.press_number >= 2:
            print('Moving to starting location (automatic if in simulation)')
            # !!
            if sim:
                s = [[0.2, 0.4, 0.1],[0.0, 0.71, 0.0, -0.71]]
                self.move_l_hand([s], [2])
            # !!

        # Start
        elif self.press_number >= 1:
            print('Start and closing hand, disable trajectory API for demo')
            self.grasp_close()


    def get_ee(self):
        # Return ee coordinates
        now = rclpy.time.Time() # NOT RIGHT?
        to_frame_rel = 'pelvis'
        from_frame_rel = 'l_palm'
        try:
            trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, now)
            translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            return [translation, rotation]
        except:
            pass

    def grasp_send(self, msg):

        # Messages must be delayed
        delay = 0.05;

        # Right open fingers
        self.publisher_little_r_.publish(msg)
        time.sleep(delay)
        self.publisher_ring_r_.publish(msg)
        time.sleep(delay)
        self.publisher_middle_r_.publish(msg)
        time.sleep(delay)
        self.publisher_index_r_.publish(msg)
        time.sleep(delay)
        self.publisher_thumb_r_.publish(msg)
        time.sleep(delay)

        # Left open fingers
        self.publisher_thumb_l_.publish(msg)
        time.sleep(delay)
        self.publisher_index_l_.publish(msg)
        time.sleep(delay)
        self.publisher_middle_l_.publish(msg)
        time.sleep(delay)
        self.publisher_ring_l_.publish(msg)
        time.sleep(delay)
        self.publisher_little_l_.publish(msg)
        time.sleep(delay)

    def grasp_open(self):
        print("Opening")
        self.grasp_open_bool = True
        msg = HandCommand()
        msg.speed = 255.0
        msg.force = 100.0
        msg.closure = 12000.0
        self.grasp_send(msg)

    def grasp_close(self):
        print("Closing")
        self.grasp_open_bool = False
        msg = HandCommand()
        msg.speed = 255.0
        msg.force = 100.0
        msg.closure = 17000.0
        self.grasp_send(msg)

    def move_l_hand(self, state_list, time_list):
        cumulative_seconds_from_start_ = 0

        # Make message
        periodic_trajectory_msg_ = WholeBodyTrajectory(
            append_trajectory=False
        )  
        periodic_trajectory_msg_.interpolation_mode.value = (
            TrajectoryInterpolation.MINIMUM_JERK_CONSTRAINED
        )  

        for state, time in zip(state_list, time_list):
            # Go to
            cumulative_seconds_from_start_ = cumulative_seconds_from_start_ + time
            periodic_trajectory_pt_msg_1_ = WholeBodyTrajectoryPoint(
                time_from_start=Duration(sec=cumulative_seconds_from_start_)
            )
            periodic_trajectory_pt_msg_1_.task_space_commands.append(
                generate_task_space_command_msg(
                    ReferenceFrameName.LEFT_HAND,
                    ReferenceFrameName.PELVIS,
                    state,
                )
            )

            # Append for each waypoint
            periodic_trajectory_msg_.trajectory_points.append(periodic_trajectory_pt_msg_1_)

        # Send message
        self.publisher_ee.publish(periodic_trajectory_msg_)

def main():

    rclpy.init()
    node = DemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
