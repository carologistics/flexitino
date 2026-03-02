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
    This state calibrates the gripper to the origin position.

    Parameters
    -- timeout             Maximum time allowed (seconds)

    Outputs
    <= home_reached        Calibration successful.
    <= failed              Failed for some reason.
    <= canceled            User canceled before completion.
    <= timeout             The action has timed out.

    User data
    ># ns                   Robot namespace (e.g. 'robotinobase3')
    """

    def __init__(self, timeout):

        super().__init__(
            outcomes=["home_reached", "failed", "canceled", "timeout"],
            input_keys=["ns"],
            output_keys=[],
        )
        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout

        ProxyActionClient.initialize(CalibratetoOrigin._node)

        self._client = None
        self._topic = None
        self._error = False
        self._return = None
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

        if "ns" not in userdata or not isinstance(userdata.ns, str):
            self._error = True
            Logger.logwarn("CalibratetoOrigin requires userdata.ns (string)!")
            return

        self._topic = f"{userdata.ns}/gigatino/calibrate"
        self._client = ProxyActionClient(
            {self._topic: Calibrate}, wait_duration=0.0
        )

        self._start_time = self._node.get_clock().now()
        self._target_time = Duration(seconds=self._timeout_sec)

        goal = Calibrate.Goal()

        Logger.loginfo(f"Calibrating gripper (topic: {self._topic})")

        try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
        except Exception as exc:
            Logger.logwarn(f"Failed to send Calibrate command:\n  {type(exc)} - {exc}")
            self._error = True

    def on_exit(self, userdata):

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo("Cancelled active action goal.")
