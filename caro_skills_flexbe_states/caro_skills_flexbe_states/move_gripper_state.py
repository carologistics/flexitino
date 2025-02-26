from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from gripper_msgs.action import Gripper
from rclpy.duration import Duration
class gripperMoveForward(EventState):
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
    ># frame           Frame of the goal pose
    ># x_target              X value of goal pose
    ># y_target              Y value of goal pose
    ># z_target            z value of goal pose
    """
    def __init__(self, timeout,action_topic):

        super().__init__(outcomes=['pose_reached', 'failed', 'canceled', 'timeout'],
                         input_keys=['frame', 'x_target','y_target', 'z_target'],
                         output_keys=[])
        self._timeout = Duration(seconds=timeout)
        self._timeout_sec = timeout
        self._topic = action_topic

        # Create the action client when building the behavior.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior.
        ProxyActionClient.initialize(gripperMoveForward._node)

        self._client = ProxyActionClient({self._topic: Gripper},
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

        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._target_time.nanoseconds:
            # Normal completion, do not bother repeating the publish
            # We won't bother publishing a 0 command unless blocked (above)
            # so that we can chain multiple motions together
            self._return = 'timeout'
            return 'timeout'

        # Normal operation
        if self._cmd_topic:
            Logger.localinfo(f"{self._name} : {self._twist}")  # For initial debugging
            self._pub.publish(self._cmd_topic, self._twist)

        return None

    def on_enter(self, userdata):
        """
        Call this method when the state becomes active.

        i.e. a transition from another state to this one is taken.
        """
        self._error = False
        self._return = None  # reset the completion flag
        if 'x_target' not in userdata:
            self._error = True
            Logger.logwarn("MoveTotState requires userdata.target_x key!")
            return
        
        if 'y_target' not in userdata:
            self._error = True
            Logger.logwarn("MoveTotState requires userdata.target_y key!")
            return
        
        if 'z_target' not in userdata:
            self._error = True
            Logger.logwarn("MoveTotState requires userdata.target_yaw key!")
            return
        # create goal msg
        goal = Gripper.Goal()
        # Recording the start time to set rotation duration output
        self._start_time = self._node.get_clock().now()
        # goal.pose.header.stamp = self._start_time

        Logger.logwarn("frame_id type %s. Expects a string.", type(userdata.frame).__name__)
        Logger.logwarn("frame_id = %s.", userdata.frame)
        Logger.logwarn("target_x is %s. Expects a float.", type(userdata.x_target).__name__)
        Logger.logwarn("target_x is %f.", userdata.x_target)
        Logger.logwarn("target_y is %s. Expects  a float.", type(userdata.y_target).__name__)
        Logger.logwarn("target_y is %f.", userdata.y_target)
        Logger.logwarn("target_yaw is %s. Expects  a float.", type(userdata.z_target).__name__)
        Logger.logwarn("target_yaw is %f.", userdata.z_target)

        if isinstance(userdata.frame, str):
            goal.frame = userdata.frame
        else:
            Logger.logwarn(f"Invalid frame_id type: {type(userdata.frame).__name__}. Expected a string.")
            self._error = True
            return
        
        
        if isinstance(userdata.x_target, float):
            goal.x_target = userdata.x_target
        else:
            Logger.logwarn(f"Invalid target_x type: {type(userdata.x_target).__name__}. Expected float.")
            self._error = True
            return

        if isinstance(userdata.y_target, float):
            goal.y_target = userdata.y_target
        else:
            Logger.logwarn(f"Invalid target_y type: {type(userdata.y_target).__name__}. Expected float.")
            self._error = True
            return

        if isinstance(userdata.z_target, float):
            goal.z_target = userdata.z_target
        else:
            Logger.logwarn(f"Invalid target_z type: {type(userdata.z_target).__name__}. Expected float.")
            self._error = True
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