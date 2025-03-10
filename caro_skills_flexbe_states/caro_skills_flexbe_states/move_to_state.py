#!/usr/bin/env python

# Copyright 2023 Carologistics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This state navigates the robot to the given pose using NavigateToPose
messages.
"""

# import math

from rclpy.duration import Duration
# from geometry_msgs.msg import Quaternion
from transforms3d.euler import euler2quat

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from nav2_msgs.action import NavigateToPose

# import of required action
# This ExampleActionState is based on the standard action tutorials
#    https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html
#    https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
#    https://docs.ros2.org/latest/api/turtlesim/action/RotateAbsolute.html
# from turtlesim.action import RotateAbsolute


class MoveToState(EventState):
    """
    This state navigates the robot to the given pose using NavigateToPose messages

    Parameters
    -- timeout             Maximum time allowed (seconds)
    -- action_topic        Name of action to invoke

    Outputs
    <= pose_reached        Robot reached pose successful.
    <= failed              Failed for some reason.
    <= canceled            User canceled before completion.
    <= timeout             The action has timed out.

    User data
    ># frame_id            Frame of the goal pose
    ># target_x              X value of goal pose
    ># target_y              Y value of goal pose
    ># target_yaw            Yaw value of goal pose
    """

    def __init__(self, timeout, action_topic):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['pose_reached', 'failed', 'canceled', 'timeout'],
                         input_keys=['frame_id', 'target_x','target_y', 'target_yaw'],
                         output_keys=[])

        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._topic = action_topic

        # Create the action client when building the behavior.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior.
        ProxyActionClient.initialize(MoveToState._node)

        self._client = ProxyActionClient({self._topic: NavigateToPose},
                                         wait_duration=0.0)  # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._error = False
        self._return = None  # Retain return value in case the outcome is blocked by operator
        self._start_time = None

    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            return 'failed'

        if self._return is not None:
            # Return prior outcome in case transition is blocked by autonomy level
            return self._return

        # Check if the action has been finished
        if self._client.has_result(self._topic):
            _ = self._client.get_result(self._topic)  # The delta result value is not useful here
            #userdata.duration = self._node.get_clock().now() - self._start_time
            Logger.loginfo('Pose reached')
            self._return = 'pose_reached'
            return self._return

        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
            # Checking for timeout after we check for goal response
            self._return = 'timeout'
            return 'timeout'

        # If the action has not yet finished, no outcome will be returned and the state stays active.
        return None

    def on_enter(self, userdata):

        # make sure to reset the error state since a previous state execution might have failed
        self._error = False
        self._return = None

        if 'target_x' not in userdata:
            self._error = True
            Logger.logwarn("MoveTotState requires userdata.target_x key!")
            return
        
        if 'target_y' not in userdata:
            self._error = True
            Logger.logwarn("MoveTotState requires userdata.target_y key!")
            return
        
        if 'target_yaw' not in userdata:
            self._error = True
            Logger.logwarn("MoveTotState requires userdata.target_yaw key!")
            return

        # create goal msg
        goal = NavigateToPose.Goal()

        # Recording the start time to set rotation duration output
        self._start_time = self._node.get_clock().now()
        # goal.pose.header.stamp = self._start_time

        Logger.logwarn("frame_id type %s. Expects a string.", type(userdata.frame_id).__name__)
        Logger.logwarn("frame_id = %s.", userdata.frame_id)
        Logger.logwarn("target_x is %s. Expects an int or a float.", type(userdata.target_x).__name__)
        Logger.logwarn("target_x is %f.", userdata.target_x)
        Logger.logwarn("target_y is %s. Expects an int or a float.", type(userdata.target_y).__name__)
        Logger.logwarn("target_y is %f.", userdata.target_y)
        Logger.logwarn("target_yaw is %s. Expects an int or a float.", type(userdata.target_yaw).__name__)
        Logger.logwarn("target_yaw is %f.", userdata.target_yaw)


        if isinstance(userdata.frame_id, str):
            goal.pose.header.frame_id = userdata.frame_id
        else:
            self._error = True
            Logger.logwarn("Input is %s. Expects a string.", type(userdata.frame_id).__name__)
            return
        
        if isinstance(userdata.target_x, (float, int)):
            goal.pose.pose.position.x = userdata.target_x
        else:
            self._error = True
            Logger.logwarn("Input is %s. Expects an int or a float.", type(userdata.target_x).__name__)
            return

        if isinstance(userdata.target_y, (float, int)):
            goal.pose.pose.position.y = userdata.target_y
        else:
            self._error = True
            Logger.logwarn("Input is %s. Expects an int or a float.", type(userdata.target_y).__name__)
            return

        if isinstance(userdata.target_yaw, (float, int)):
            quat = euler2quat(0, 0, userdata.target_yaw)
            goal.pose.pose.orientation.x = quat[1]
            goal.pose.pose.orientation.y = quat[2]
            goal.pose.pose.orientation.z = quat[3]
            goal.pose.pose.orientation.w = quat[0]
            pass
        else:
            self._error = True
            Logger.logwarn("Input is %s. Expects an int or a float.", type(userdata.target_yaw).__name__)
            return

        # Send the goal.
        try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
        except Exception as exc:  # pylint: disable=W0703
            # Since a state failure not necessarily causes a behavior failure,
            # it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn(f"Failed to send the NavigateToPose command:\n  {type(exc)} - {exc}")
            self._error = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
