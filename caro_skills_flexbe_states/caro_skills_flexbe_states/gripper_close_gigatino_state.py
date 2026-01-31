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

"""Use to move gripper to target pose."""

from rclpy.duration import Duration
from gigatino_msgs.action import Gripper
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

class GripperCloseState(EventState):
    """
    Use to grip the workpiece
    
    Parameters
    -- timeout             Maximum time allowed (seconds)
    -- action_topic        Name of action to invoke example ='robotinobase2/gigatino/gripper'

    Outputs
    <= success             congrats we got the workpiece.
    <= failed              Failed for some reason.
    <= canceled            User canceled before completion.
    <= timeout             The action has timed out.

    User data
    ># close   bool         false to close the gripper

    """

    def __init__(self, timeout, action_topic):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['success', 'failed', 'canceled', 'timeout'],
                         input_keys=['close'], output_keys=[])

        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._topic = action_topic

        # Create the action client when building the behavior.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior
        ProxyActionClient.initialize(GripperCloseState._node)

        self._client = ProxyActionClient({self._topic: Gripper},
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
        
        if self._client.has_result(self._topic):
            _ = self._client.get_result(self._topic)  # The delta result value is not useful here
            Logger.loginfo('gripped work piece')
            self._return = 'success'
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
        # Initialize start time
        self._start_time = self._node.get_clock().now()

        # Define timeout duration (if not already initialized)
        self._target_time = Duration(seconds=self._timeout_sec)  


        if 'close' not in userdata:
            self._error = True
            Logger.logwarn("GripperCloseState requires data")
            return
        goal = Gripper.Goal()

        if isinstance(userdata.close, bool):
            goal.open = userdata.close  
        else:
            self._error = True
            Logger.logwarn("Input is %s. Expects an bool", type(userdata.close).__name__)

        # Send the goal.
        try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
        except Exception as exc:  # pylint: disable=W0703
            # Since a state failure not necessarily causes a behavior failure,
            # it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn(f"Failed to send the RotateAbsolute command:\n  {type(exc)} - {exc}")
            self._error = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
