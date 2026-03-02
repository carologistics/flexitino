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
"""Use to open gripper."""
from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyActionClient
from gigatino_msgs.action import Gripper
from rclpy.duration import Duration


class GripperState(EventState):
    """
    Use to open or close the gripper.

    Parameters
    -- timeout             Maximum time allowed (seconds)

    Outputs
    <= success             Gripper action completed.
    <= failed              Failed for some reason.
    <= canceled            User canceled before completion.
    <= timeout             The action has timed out.

    User data
    ># open                 True to open the gripper, False to close
    ># ns                   Robot namespace (e.g. 'robotinobase3')
    """

    def __init__(self, timeout):

        super().__init__(
            outcomes=["success", "failed", "canceled", "timeout"],
            input_keys=["open", "ns"],
            output_keys=[],
        )
        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout

        ProxyActionClient.initialize(GripperState._node)

        self._client = None
        self._topic = None
        self._error = False
        self._return = None
        self._start_time = None

    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._error:
            return "failed"

        if self._return is not None:
            # Return prior outcome in case transition is blocked by autonomy level
            return self._return

        if self._client.has_result(self._topic):
            _ = self._client.get_result(self._topic)  # The delta result value is not useful here
            Logger.loginfo("gripped work piece")
            self._return = "success"
            return self._return

        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
            self._return = "timeout"
            return "timeout"

        # If the action has not yet finished, no outcome will be returned and the state stays active.
        return None

    def on_enter(self, userdata):

        self._error = False
        self._return = None

        if "ns" not in userdata or not isinstance(userdata.ns, str):
            self._error = True
            Logger.logwarn("GripperState requires userdata.ns (string)!")
            return

        if "open" not in userdata or not isinstance(userdata.open, bool):
            self._error = True
            Logger.logwarn("GripperState requires userdata.open (bool)!")
            return

        self._topic = f"{userdata.ns}/gigatino/gripper"
        self._client = ProxyActionClient(
            {self._topic: Gripper}, wait_duration=0.0
        )

        self._start_time = self._node.get_clock().now()
        self._target_time = Duration(seconds=self._timeout_sec)

        goal = Gripper.Goal()
        goal.open = userdata.open

        Logger.loginfo(f"Gripper {'open' if userdata.open else 'close'} (topic: {self._topic})")

        try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
        except Exception as exc:
            Logger.logwarn(f"Failed to send Gripper command:\n  {type(exc)} - {exc}")
            self._error = True

    def on_exit(self, userdata):
        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo("Cancelled active action goal.")
