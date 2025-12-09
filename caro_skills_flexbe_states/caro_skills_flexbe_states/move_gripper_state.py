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

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from gigatino_msgs.action import Move
from rclpy.duration import Duration
class GripperMove(EventState):
    """
    This state allows the gripper to move to target position

    Parameters
    -- timeout             Maximum time allowed (seconds)
    -- action_topic        Name of action to invoke

    Outputs
    <= reached              Robot reached pose successful 
    <= failed                Failed for some reason.
    <= canceled              User canceled before completion.
    <= timeout               The action has timed out.

    User data
    ># relative             set default as false
    ># target_frame         Frame of the goal pose
    ># x                    X value of goal pose
    ># y                    Y value of goal pose
    ># z                    z value of goal pose
    ># gripper_state        the gripper state
    ># use_gripper          are we using the gripper
    """
    def __init__(self, timeout,action_topic='robotinobase2/gigatino/move'):

        super().__init__(outcomes=['reached', 'failed', 'canceled', 'timeout'],
                         input_keys=['relative','target_frame', 'x','y', 'z','gripper_state','use_gripper'],
                         output_keys=[])
        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._topic = action_topic

        # Create the action client when building the behavior.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior.
        ProxyActionClient.initialize(GripperMove._node)

        self._client = ProxyActionClient({self._topic: Move},
                                         wait_duration=0.0)  # pass required clients as dict (topic: type)

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
            return 'failed'

        if self._return is not None:
            # Return prior outcome in case transition is blocked by autonomy level
            return self._return

        if self._client.has_result(self._topic):
            _ = self._client.get_result(self._topic)  # The delta result value is not useful here
            #userdata.duration = self._node.get_clock().now() - self._start_time
            Logger.loginfo('Pose reached')
            self._return = 'reached'
            return self._return
        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._target_time.nanoseconds:
            # Normal completion, do not bother repeating the publish
            # We won't bother publishing a 0 command unless blocked (above)
            # so that we can chain multiple motions together
            self._return = 'timeout'
            return 'timeout'

        return None

    def on_enter(self, userdata):
        """
        Call this method when the state becomes active.

        i.e. a transition from another state to this one is taken.
        """
        self._error = False
        if 'x' not in userdata:
            self._error = True
            Logger.logwarn("MoveToState requires userdata.target_x key!")
            return
        
        if 'y' not in userdata:
            self._error = True
            Logger.logwarn("MoveToState requires userdata.target_y key!")
            return
        
        if 'z' not in userdata:
            self._error = True
            Logger.logwarn("MoveToState requires userdata.target_z key!")
            return
        if 'target_frame' not in userdata:
            self._error = True
            Logger.logwarn("MoveToState requires userdata.target_z key!")
            return
        # create goal msg
        goal = Move.Goal()
        # Recording the start time to set rotation duration output
        self._start_time = self._node.get_clock().now()
        # goal.pose.header.stamp = self._start_time
        # Define timeout duration (if not already initialized)
        self._target_time = Duration(seconds=self._timeout_sec)  
        
        if isinstance(userdata.target_frame,str):
            goal.target_frame = userdata.target_frame
            Logger.loginfo(f"{goal.target_frame}")
        else:
            Logger.logwarn(f"Invalid frame_id type: {type(userdata.target_frame).__name__}. Expected a string.")
            self._error = True
            return

        if isinstance(userdata.relative,bool):
            goal.relative = userdata.relative  # Assign only if provided
            Logger.loginfo(f"{goal.relative}")
        else:
            Logger.logwarn(f"Invalid relative type: {type(userdata.relative).__name__}. Expected a bool.")
            self._error = True
            return

        if isinstance(userdata.gripper_state,bool):
            goal.gripper_state = userdata.gripper_state  # Assign only if provided
            Logger.loginfo(f"{goal.gripper_state}")
        else:
            Logger.logwarn(f"Invalid gripper_state type: {type(userdata.gripper_state).__name__}. Expected a bool.")
            self._error = True
            return

        if isinstance(userdata.use_gripper,bool):
            goal.use_gripper = userdata.use_gripper  
            Logger.loginfo(f"{goal.use_gripper}")
        else:
            Logger.logwarn(f"Invalid use_gripper type: {type(userdata.use_gripper).__name__}. Expected a bool.")
            self._error = True
            return

        if isinstance(userdata.x, float):
            goal.x = userdata.x
            Logger.loginfo(f"{goal.x}")
        else:
            Logger.logwarn(f"Invalid target_x type: {type(userdata.x).__name__}. Expected float.")
            self._error = True
            return

        if isinstance(userdata.y, float):
            goal.y = userdata.y
            Logger.loginfo(f"{goal.y}")
        else:
            Logger.logwarn(f"Invalid target_y type: {type(userdata.y).__name__}. Expected float.")
            self._error = True
            return

        if isinstance(userdata.z, float):
            goal.z = userdata.z
            Logger.loginfo(f"{goal.z}")
        else:
            Logger.logwarn(f"Invalid target_z type: {type(userdata.z).__name__}. Expected float.")
            self._error = True
            return

        # Send the goal.
        try:
            self._client.send_goal(self._topic, goal, wait_duration=self._timeout_sec)
            Logger.localinfo(f"{goal}") 
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