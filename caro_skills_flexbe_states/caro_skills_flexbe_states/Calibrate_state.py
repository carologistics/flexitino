#!/usr/bin/env python
# Copyright 2023-2026 Philipp Schillinger, Christopher Newport University
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
"""use to calibrate gripper."""
from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyActionClient
from gigatino_msgs.action import Calibrate
from rclpy.duration import Duration


class CalibratetoOrigin(EventState):
    """
    This state calibrate the gripper to the origin position

    Parameters
    -- timeout             Maximum time allowed (seconds)
    -- action_topic        Name of action to invoke

    Outputs
    <= home reached        Robot reached pose successful.
    <= failed              Failed for some reason.
    <= canceled            User canceled before completion.
    <= timeout             The action has timed out.

    >#

    """

    def __init__(self, timeout, action_topic="robotinobase2/gigatino/calibrate"):

        super().__init__(outcomes=["home_reached", "failed", "canceled", "timeout"], output_keys=[])
        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._topic = action_topic

        # Create the action client when building the behavior.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior.
        ProxyActionClient.initialize(CalibratetoOrigin._node)

        self._client = ProxyActionClient(
            {self._topic: Calibrate}, wait_duration=0.0
        )  # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._error = False
        self._return = None  # Retain return value in case the outcome is blocked by operator
        self._start_time = None

    def execute(self, userdata):
        """
        Call this method periodically while the state is active.

        If no outcome is returned, the state will stay active.
        """
        # Check if the client failed to send the goal.
        if self._error:
            return "failed"

        if self._return is not None:
            # Return prior outcome in case transition is blocked by autonomy level
            return self._return

        if self._client.has_result(self._topic):
            _ = self._client.get_result(self._topic)  # The delta result value is not useful here
            # userdata.duration = self._node.get_clock().now() - self._start_time
            Logger.loginfo("Home reached")
            self._return = "home_reached"
            return self._return

        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._target_time.nanoseconds:
            self._return = "timeout"
            return "timeout"

        return None

    def on_enter(self, userdata):
        """
        Call this method when the state becomes active.

        i.e. a transition from another state to this one is taken.
        """
        self._error = False
        self._return = None
        # Initialize start time
        self._start_time = self._node.get_clock().now()

        # Define timeout duration (if not already initialized)
        self._target_time = Duration(seconds=self._timeout_sec)

        # create goal msg
        goal = Calibrate.Goal()
        self._client.send_goal(self._topic, goal)

    def on_exit(self, userdata):

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo("Cancelled active action goal.")
