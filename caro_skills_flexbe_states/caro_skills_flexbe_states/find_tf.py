#!/usr/bin/env python
# # Copyright 2026 Carologistics
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.
"""FlexBE State to find a TF transform between two frames."""
from flexbe_core import EventState
from flexbe_core import Logger
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FindTF(EventState):
    """
    This state attempts to find the TF (transform) from a specified child frame
    (source_frame_id) to a specified parent frame (target_frame_id).

    Elements defined here for UI
    Parameters
    -- timeout             float     Maximum time (in seconds) to wait for the transform lookup.

    Outputs
    <= frame_found         The transform was successfully found (from source_frame_id to target_frame_id).
    <= failed              The transform could not be found (e.g., timeout, frames don't exist, or other error).

    User data
    ># target_frame_id    string    The name of the target frame (e.g., 'map', 'odom').
    >#                              This is the frame into which the source_frame will be transformed.
    ># source_frame_id    string    The name of the source frame (e.g., 'base_link', 'camera_link').
    >#                              This is the frame that will be transformed.
    #> transform          TransformStamped  The found transform.
    """

    def __init__(self, timeout):
        """Initialize the state with outcomes, input/output keys, and timeout."""
        super().__init__(
            outcomes=["frame_found", "failed"],
            input_keys=["source_frame_id", "target_frame_id"],
            output_keys=["transform"],
        )

        # TF resources initialized in on_enter
        self._tf_buffer = None
        self._tf_listener = None
        self._lookup_target_frame = None
        self._lookup_source_frame = None

        # Parse timeout with fallback to default
        try:
            timeout_float = float(timeout)
            self._lookup_timeout_duration = Duration(seconds=timeout_float)
        except ValueError:
            Logger.logwarn(
                f"Invalid timeout value '{timeout}' for FindTF state. " f"It must be a number. Using default 1.0s."
            )
            self._lookup_timeout_duration = Duration(seconds=1.0)

    def on_enter(self, userdata):
        """Called when the state becomes active. Sets up TF listener and validates input."""
        super().on_enter(userdata)

        # Initialize TF listener
        try:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self._node, spin_thread=False)
            Logger.loginfo(f"TF Listener initialized for state '{self.name}'.")
        except Exception as e:
            Logger.logerr(f"Failed to initialize TF Buffer/Listener for state '{self.name}': {str(e)}")
            # No outcome is returned here, execute will handle the error by returning 'failed'
            # because _tf_buffer will be None.
            return  # Return here to prevent further processing in on_enter if listener fails

        if self._tf_buffer is None:
            Logger.logerr(f"TF Buffer not initialized in state '{self.name}'.")
            return  # execute will return 'failed'

        # Validate and get frame_ids from userdata
        if (
            "target_frame_id" not in userdata
            or not isinstance(userdata.target_frame_id, str)
            or not userdata.target_frame_id
        ):
            Logger.logwarn(f"'{self.name}': Invalid or missing 'target_frame_id' in userdata.")
            self._lookup_target_frame = None  # Mark as invalid
            return
        if (
            "source_frame_id" not in userdata
            or not isinstance(userdata.source_frame_id, str)
            or not userdata.source_frame_id
        ):
            Logger.logwarn(f"'{self.name}': Invalid or missing 'source_frame_id' in userdata.")
            self._lookup_source_frame = None  # Mark as invalid
            return

        # lookup_transform(target, source, ...) gives the transform from source to target.
        # The userdata.target_frame_id is the frame we want to transform *into*.
        # The userdata.source_frame_id is the frame we want to transform *from*.
        self._lookup_target_frame = userdata.target_frame_id
        self._lookup_source_frame = userdata.source_frame_id
        Logger.loginfo(
            f"'{self.name}': Attempting to find transform from '{self._lookup_source_frame}' (source) "
            f"to '{self._lookup_target_frame}' (target)."
        )

    def execute(self, userdata):
        """Perform the TF lookup and return outcome based on success or failure."""
        if self._tf_buffer is None:
            Logger.logerr(f"'{self.name}': TF Buffer not available in execute.")
            return "failed"

        if self._lookup_target_frame is None or self._lookup_source_frame is None:
            Logger.logwarn(f"'{self.name}': Frame IDs not properly set in on_enter.")
            return "failed"

        try:
            # Get the transform from source_frame_id to target_frame_id
            transform_stamped = self._tf_buffer.lookup_transform(
                self._lookup_target_frame,  # Target frame
                self._lookup_source_frame,  # Source frame
                Time(),
                timeout=self._lookup_timeout_duration,
            )
            userdata.transform = transform_stamped
            Logger.loginfo(
                f"'{self.name}': Successfully found transform "
                f"from '{self._lookup_source_frame}' to '{self._lookup_target_frame}'."
            )
            return "frame_found"

        except TransformException as ex:
            Logger.logwarn(
                f"'{self.name}': Could not transform '{self._lookup_source_frame}' to '{self._lookup_target_frame}' "
                f"within {self._lookup_timeout_duration.nanoseconds / 1e9:.2f}s: {ex}"
            )
            return "failed"
        except Exception as e:
            Logger.logerr(f"'{self.name}': An unexpected error occurred during TF lookup: {str(e)}")
            return "failed"

    def on_exit(self, userdata):
        """Clean up TF resources when leaving the state."""
        if self._tf_listener is not None:
            # TransformListener doesn't have an explicit shutdown, relying on garbage collection.
            # Setting to None helps the garbage collector and signals it's no longer in use.
            self._tf_listener = None
        if self._tf_buffer is not None:
            self._tf_buffer.clear()  # Clear the buffer if desired, though not strictly necessary for just one lookup.
            self._tf_buffer = None
        Logger.loginfo(f"'{self.name}': Cleaned up TF resources.")
