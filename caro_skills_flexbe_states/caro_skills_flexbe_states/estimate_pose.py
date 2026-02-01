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
"""
This state applies an X and Y offset to a given TransformStamped
and outputs the resulting X, Y, Yaw, and Frame ID.
"""
import transforms3d.euler
import transforms3d.quaternions
from flexbe_core import EventState
from flexbe_core import Logger
from geometry_msgs.msg import TransformStamped


class EstimatedPoseState(EventState):
    """
    Applies an X and Y offset to an input_transform and outputs the
    resulting pose (x, y, yaw) and the frame_id.

    Input Keys:
    ># transform_in     TransformStamped  The input transform.
    ># offset_x         float             The offset to apply in the X-direction of the child frame.
    ># offset_y         float             The offset to apply in the Y-direction of the child frame.
    ># ns               string            Namespace (currently not used, but accepted as an input key).

    Output Keys:
    #> target_x         float             The resulting X-coordinate.
    #> target_y         float             The resulting Y-coordinate.
    #> target_yaw       float             The resulting Yaw angle (in radians).
    #> frame_id         string            The frame ID of the transform.

    Outcomes:
    <= done             The calculation was successful.
    <= failed           An error occurred during calculation.
    """

    def __init__(self):
        super().__init__(
            outcomes=["done", "failed"],
            input_keys=["transform_in", "offset_x", "offset_y", "ns"],
            output_keys=["target_x", "target_y", "target_yaw", "frame_id"],
        )
        self._return_code = None  # Store the outcome for execute()

    def on_enter(self, userdata):
        self._return_code = None  # Reset return code before processing

        try:
            # 1. Validate inputs
            if not all(key in userdata for key in ["transform_in", "offset_x", "offset_y"]):
                missing_keys = [key for key in ["transform_in", "offset_x", "offset_y"] if key not in userdata]
                Logger.logwarn(f"'{self.name}': Missing input keys: {', '.join(missing_keys)}.")
                self._return_code = "failed"
                return

            transform_in = userdata.transform_in
            offset_x = userdata.offset_x
            offset_y = userdata.offset_y
            _ = userdata.get("ns", None)  # Acknowledge 'ns' even if not actively used

            if not isinstance(transform_in, TransformStamped):
                Logger.logwarn(
                    f"'{self.name}': Input 'transform_in' is not a TransformStamped. Received: {type(transform_in)}."
                )
                self._return_code = "failed"
                return
            if not isinstance(offset_x, (float, int)):
                Logger.logwarn(
                    f"'{self.name}': Input 'offset_x' is not a number (float/int). Received: {type(offset_x)}."
                )
                self._return_code = "failed"
                return
            if not isinstance(offset_y, (float, int)):
                Logger.logwarn(
                    f"'{self.name}': Input 'offset_y' is not a number (float/int). Received: {type(offset_y)}."
                )
                self._return_code = "failed"
                return

            # 2. Perform calculations
            original_translation = transform_in.transform.translation
            q_rotation = transform_in.transform.rotation

            # Prepare quaternion for transforms3d (w, x, y, z)
            q_t3d_format = [q_rotation.w, q_rotation.x, q_rotation.y, q_rotation.z]

            # Create offset vector in the child frame's coordinate system
            offset_vector_child = [float(offset_x), float(offset_y), 0.0]

            # Rotate the offset vector from the child frame to the parent frame
            rotated_offset_parent = transforms3d.quaternions.rotate_vector(offset_vector_child, q_t3d_format)

            # Add the rotated offset to the original translation in the parent frame
            output_x = original_translation.x + rotated_offset_parent[0]
            output_y = original_translation.y + rotated_offset_parent[1]
            # The Z component of the translation would be original_translation.z + rotated_offset_parent[2],
            # but it's not used as an output key here, and the Z-offset is defined as 0.0 in offset_vector_child.

            # 3. Extract Yaw
            _, _, output_yaw = transforms3d.euler.quat2euler(q_t3d_format, axes="sxyz")

            # 4. Extract Frame ID
            output_frame_id = transform_in.header.frame_id

            # 5. Set output userdata
            userdata.target_x = output_x
            userdata.target_y = output_y
            userdata.target_yaw = output_yaw
            userdata.frame_id = output_frame_id

            Logger.loginfo(
                f"'{self.name}': Offset applied successfully. New pose in frame '{output_frame_id}': "
                f"x={output_x:.3f}, y={output_y:.3f}, yaw={output_yaw:.3f} rad."
            )
            self._return_code = "done"
        except KeyError as e:
            Logger.logwarn(f"'{self.name}': Missing input key in userdata: {str(e)}.")
            self._return_code = "failed"
        except Exception as e:
            Logger.logerr(f"'{self.name}': Error during processing: {str(e)}")
            self._return_code = "failed"
        # on_enter does not return an outcome, so execute() will be called next.

    def execute(self, userdata):
        # All logic is in on_enter, as this state's operations are immediate.
        # execute() simply returns the pre-calculated result.
        return self._return_code

    def on_exit(self, userdata):
        # Nothing to clean up for this state.
        pass
