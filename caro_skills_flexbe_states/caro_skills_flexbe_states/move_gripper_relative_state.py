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
from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyActionClient
from gigatino_msgs.action import Move
from rclpy.duration import Duration


class GripperMoveRelative(EventState):
    """
    This state moves the gripper to a position relative to a given frame.

    Parameters
    -- timeout             Maximum time allowed (seconds)

    Outputs
    <= reached              Robot reached pose successfully
    <= failed               Failed for some reason.
    <= canceled             User canceled before completion.
    <= timeout              The action has timed out.

    User data
    ># target_frame         Frame to move relative to
    ># x                    X offset in target_frame (float)
    ># y                    Y offset in target_frame (float)
    ># z                    Z offset in target_frame (float)
    ># ns                   Robot namespace (e.g. 'robotinobase2')
    """

    def __init__(self, timeout):

        super().__init__(
            outcomes=["reached", "failed", "canceled", "timeout"],
            input_keys=["target_frame", "x", "y", "z", "ns"],
            output_keys=[],
        )
        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout

        ProxyActionClient.initialize(GripperMoveRelative._node)

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
        if self._error:
            return "failed"

        if self._return is not None:
            return self._return

        if self._client.has_result(self._topic):
            _ = self._client.get_result(self._topic)
            Logger.loginfo("Pose reached")
            self._return = "reached"
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
            Logger.logwarn("GripperMoveRelative requires userdata.ns (string)!")
            return

        if "target_frame" not in userdata or not isinstance(userdata.target_frame, str):
            self._error = True
            Logger.logwarn("GripperMoveRelative requires userdata.target_frame (string)!")
            return

        if "x" not in userdata or not isinstance(userdata.x, float):
            self._error = True
            Logger.logwarn("GripperMoveRelative requires userdata.x (float)!")
            return

        if "y" not in userdata or not isinstance(userdata.y, float):
            self._error = True
            Logger.logwarn("GripperMoveRelative requires userdata.y (float)!")
            return

        if "z" not in userdata or not isinstance(userdata.z, float):
            self._error = True
            Logger.logwarn("GripperMoveRelative requires userdata.z (float)!")
            return

        self._topic = f"{userdata.ns}/gigatino/move"
        self._client = ProxyActionClient(
            {self._topic: Move}, wait_duration=0.0
        )

        goal = Move.Goal()
        self._start_time = self._node.get_clock().now()
        self._target_time = Duration(seconds=self._timeout_sec)

        goal.target_frame = userdata.target_frame
        goal.x = userdata.x
        goal.y = userdata.y
        goal.z = userdata.z
        goal.relative = False
        goal.use_gripper = False
        goal.gripper_state = False

        Logger.loginfo(f"Moving to ({userdata.x}, {userdata.y}, {userdata.z}) "
                       f"in frame: {userdata.target_frame} (topic: {self._topic})")

        try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
        except Exception as exc:
            Logger.logwarn(f"Failed to send Move command:\n  {type(exc)} - {exc}")
            self._error = True

    def on_exit(self, userdata):

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo("Cancelled active action goal.")
