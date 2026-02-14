#!/usr/bin/env python

# Copyright 2024 Carologistics
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

"""Send a motor_move goal to navigate the robot to a target pose."""

from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from transforms3d.euler import euler2quat

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from motor_move_msgs.action import MotorMove


class MotorMoveState(EventState):
    """
    Send a MotorMove action goal to navigate the robot to a target pose.

    The action topic is constructed from the namespace as
    /{namespace}/motor_move_action.

    Parameters
    -- timeout             Maximum time allowed (seconds)

    Outputs
    <= reached             Robot reached the target pose.
    <= failed              The action server reported failure or the goal was rejected.
    <= canceled            The goal was canceled (e.g. by the operator).
    <= timeout             The action did not complete within the allowed time.

    User data
    ># namespace   string  Robot namespace (e.g. 'robotinobase3')
    ># frame       string  Target frame relative to namespace (e.g. 'base_link')
    ># target_x    float   X position of the goal pose
    ># target_y    float   Y position of the goal pose
    ># target_yaw  float   Yaw orientation of the goal pose (radians)
    #> distance    float   Last reported distance to the target
    """

    def __init__(self, timeout):
        super().__init__(
            outcomes=['reached', 'failed', 'canceled', 'timeout'],
            input_keys=['namespace', 'frame', 'target_x', 'target_y', 'target_yaw'],
            output_keys=['distance'])

        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._topic = None
        self._client = None

        self._error = False
        self._return = None
        self._start_time = None

    def on_enter(self, userdata):
        self._error = False
        self._return = None

        # -- validate inputs ------------------------------------------------
        required = {
            'namespace': str,
            'frame': str,
            'target_x': (float, int),
            'target_y': (float, int),
            'target_yaw': (float, int),
        }
        for key, expected_type in required.items():
            if key not in userdata:
                self._error = True
                Logger.logwarn("MotorMoveState requires userdata.%s!" % key)
                return
            if not isinstance(getattr(userdata, key), expected_type):
                self._error = True
                Logger.logwarn(
                    "MotorMoveState: userdata.%s is %s, expected %s."
                    % (key, type(getattr(userdata, key)).__name__, expected_type))
                return

        # -- build action topic from namespace ------------------------------
        self._topic = "/%s/motor_move_action" % userdata.namespace

        # -- setup proxy client for this topic ------------------------------
        ProxyActionClient.initialize(MotorMoveState._node)
        self._client = ProxyActionClient(
            {self._topic: MotorMove}, wait_duration=0.0)

        if not self._client.is_available(self._topic):
            self._error = True
            Logger.logwarn(
                "MotorMoveState: action server '%s' is not available!" % self._topic)
            return

        # -- build goal -----------------------------------------------------
        goal = MotorMove.Goal()
        goal.motor_goal.header.frame_id = "%s/%s" % (userdata.namespace, userdata.frame)
        goal.motor_goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.motor_goal.pose.position.x = float(userdata.target_x)
        goal.motor_goal.pose.position.y = float(userdata.target_y)
        goal.motor_goal.pose.position.z = 0.0

        quat = euler2quat(0, 0, float(userdata.target_yaw))
        goal.motor_goal.pose.orientation.w = quat[0]
        goal.motor_goal.pose.orientation.x = quat[1]
        goal.motor_goal.pose.orientation.y = quat[2]
        goal.motor_goal.pose.orientation.z = quat[3]

        self._start_time = self._node.get_clock().now()

        Logger.loginfo(
            "MotorMoveState: sending goal to '%s' "
            "(frame=%s, x=%.3f, y=%.3f, yaw=%.3f)"
            % (self._topic, goal.motor_goal.header.frame_id,
               userdata.target_x, userdata.target_y, userdata.target_yaw))

        # -- send goal ------------------------------------------------------
        try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
        except Exception as exc:
            Logger.logwarn(
                "MotorMoveState: failed to send goal to '%s':\n  %s - %s"
                % (self._topic, type(exc).__name__, exc))
            self._error = True

    def execute(self, userdata):
        if self._error:
            return 'failed'

        if self._return is not None:
            return self._return

        # -- check for result -----------------------------------------------
        if self._client.has_result(self._topic):
            status = self._client.get_status(self._topic)
            result = self._client.get_result(self._topic)

            if status == GoalStatus.STATUS_SUCCEEDED and result.success:
                Logger.loginfo("MotorMoveState: target pose reached.")
                self._return = 'reached'
            elif status == GoalStatus.STATUS_CANCELED:
                Logger.logwarn("MotorMoveState: goal was canceled.")
                self._return = 'canceled'
            else:
                Logger.logwarn(
                    "MotorMoveState: action finished with status %s (success=%s)."
                    % (self._client.get_status_string(self._topic), result.success))
                self._return = 'failed'
            return self._return

        # -- log feedback ---------------------------------------------------
        if self._client.has_feedback(self._topic):
            feedback = self._client.get_feedback(self._topic)
            userdata.distance = feedback.distance_to_target
            Logger.loginfo(
                "MotorMoveState: distance to target: %.3f" % feedback.distance_to_target)

        # -- check timeout --------------------------------------------------
        elapsed = self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds
        if elapsed > self._timeout.nanoseconds:
            Logger.logwarn(
                "MotorMoveState: timed out after %.1f s." % self._timeout_sec)
            self._return = 'timeout'
            return self._return

        return None

    def on_exit(self, userdata):
        if self._topic and self._client and not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo("MotorMoveState: cancelled active goal on '%s'." % self._topic)
