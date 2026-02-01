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
"""Example Action FlexBE State."""
import math

import transforms3d.euler
import transforms3d.quaternions
from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyServiceCaller
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from laser_scan_integrator_msg.msg import LineSegments
from laser_scan_integrator_msg.srv import ToggleSegmentation
from laser_scan_integrator_msg.srv import ToggleSegmentation_Request
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class MapLaserToTF(EventState):
    """FlexBE state that detects line segments from laser scans and outputs a TF when
    a segment matches the robot's pose within configured tolerances."""

    def __init__(self, timeout=5):
        super().__init__(
            outcomes=["frame_found", "failed"],
            input_keys=["source_frame_id", "target_frame_id", "ns"],
            output_keys=["transform"],
        )

        self._timeout_sec = timeout
        self._service_client = ProxyServiceCaller()
        self._subscriber = None
        self._error = False
        self._pose_condition_met = False
        self._start_time = None
        self._estimated_pose_output = None
        try:
            timeout_float = float(timeout)
            self._lookup_timeout_duration = Duration(seconds=timeout_float)
        except ValueError:
            Logger.logerr(
                f"Invalid timeout value '{timeout}' for EstimatePose state. "
                f"It must be a number. Using default 1.0s."
            )
            self._lookup_timeout_duration = Duration(seconds=5.0)

        self._execute_timeout_duration = Duration(seconds=2.0)  #
        self._tf_buffer = None
        self._tf_listener = None

        self.position_tolerance = 0.3
        self.angle_tolerance = 0.5
        self.target_distance_to_segment = 0.15

    def on_enter(self, userdata):
        self._pose_condition_met = False
        self._error = False
        self._mapped_segemnt = None

        service_topic = "/" + userdata.ns + "/toggle_segmentation"
        self._service_client = ProxyServiceCaller({service_topic: ToggleSegmentation})

        if not self._service_client.is_available(service_topic):
            Logger.logwarn(f"Service {service_topic} not available!")
            self._error = True
            return

        request = ToggleSegmentation_Request()
        request.enable_segmentation = True

        try:
            result = self._service_client.call(service_topic, request)
            if result.success:
                Logger.loginfo(f"Service responded: {result.message}")
            else:
                Logger.logwarn(f"Service reported failure: {result.message}")
                self._error = True
        except Exception as e:
            Logger.logerr(f"Service call failed: {str(e)}")
            self._error = True
            return

        try:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self._node, spin_thread=False)
            Logger.loginfo(f"TF Listener initialized for state '{self.name}'.")
        except Exception as e:
            Logger.logerr(f"Failed to initialize TF Buffer/Listener for state '{self.name}': {str(e)}")
            self._error = True
            return

        if self._tf_buffer is None:
            Logger.logerr(f"TF Buffer not initialized in state '{self.name}'.")
            return "failed"

        try:
            target_frame = userdata.target_frame_id
            self.target_frame = target_frame
            source_frame = userdata.source_frame_id
            self.source_frame = source_frame

            transform_stamped = self._tf_buffer.lookup_transform(
                target_frame, source_frame, Time(), timeout=self._lookup_timeout_duration
            )
            self.transform = transform_stamped
            Logger.loginfo(
                f"Initial pose (transform from '{source_frame}' to '{target_frame}') acquired:\n"
                f"Translation: [x: {transform_stamped.transform.translation.x:.3f}, "
                f"y: {transform_stamped.transform.translation.y:.3f}, "
                f"z: {transform_stamped.transform.translation.z:.3f}]\n"
                f"Rotation: [x: {transform_stamped.transform.rotation.x:.3f}, "
                f"y: {transform_stamped.transform.rotation.y:.3f}, "
                f"z: {transform_stamped.transform.rotation.z:.3f}, "
                f"w: {transform_stamped.transform.rotation.w:.3f}]"
            )

        except TransformException as ex:
            Logger.logwarn(
                f"Could not transform '{source_frame}' to '{target_frame}' "
                f"within {self._lookup_timeout_duration.nanoseconds / 1e9:.2f}s: {ex}"
            )
            self._error = True
            return "failed"
        except Exception as e:
            Logger.logerr(f"An unexpected error occurred during TF lookup in state '{self.name}': {str(e)}")
            return "failed"

        self.topic_name = "/" + userdata.ns + "/line_segments"
        self._subscriber = ProxySubscriberCached({self.topic_name: LineSegments})
        self._subscriber.subscribe(
            self.topic_name, LineSegments, callback=self._line_segments_callback, buffered=True, inst_id=id(self)
        )
        Logger.loginfo(f"Subscribed to topic: {self.topic_name} with callback _line_segments_callback.")

        # Start timeout timer
        self._start_time = self._node.get_clock().now()

    def execute(self, userdata):
        if self._error:
            return "failed"

        # Check if the condition was met by the callback
        if self._pose_condition_met:
            Logger.loginfo(f"Pose estimation condition met (flag set by callback) in state '{self.name}'.")
            if self._mapped_segemnt is not None:
                userdata.transform = self._mapped_segemnt
                return "frame_found"
            else:
                Logger.logwarn(f"Pose condition met but no output transform was generated in state '{self.name}'.")
                return "failed"

        # Check for timeout
        if self._start_time is not None:
            elapsed_time = self._node.get_clock().now() - self._start_time
            if elapsed_time >= self._execute_timeout_duration:
                Logger.logwarn(
                    f"Timeout in state '{self.name}': Condition not met within "
                    f"{self._execute_timeout_duration.nanoseconds / 1e9:.1f} seconds."
                )
                return "failed"

        return None

    def on_exit(self, userdata):
        if self._tf_listener is not None:
            Logger.loginfo(f"Cleaning up TF Listener for state '{self.name}'.")
            self._tf_listener = None
            self._tf_buffer = None

        # Unsubscribe and clear buffer
        if self._subscriber and hasattr(self, "topic_name") and self.topic_name:
            self._subscriber.unsubscribe_topic(self.topic_name, inst_id=id(self))
            Logger.loginfo(f"Unsubscribed from topic: {self.topic_name} for instance {id(self)}")

    def _line_segments_callback(self, msg: LineSegments):
        """Process incoming line segments and check if any matches robot pose."""
        if self._error or self._pose_condition_met:
            return

        if self.transform is None:  # This is the robot's initial pose from on_enter
            Logger.logwarn("Initial transform not available in _line_segments_callback.")
            return

        # Helper: compute smallest angle difference (handles wraparound)
        def angle_diff(a, b):
            diff = a - b
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            return diff

        for segment in msg.segments:
            if self._pose_condition_met:
                break
            # 1) Transform segment endpoints to map frame
            pt1 = PointStamped()
            pt1.header.stamp = Time().to_msg()
            pt1.header.frame_id = segment.frame_id
            pt1.point = segment.end_point1

            pt2 = PointStamped()
            pt2.header = pt1.header
            pt2.point = segment.end_point2

            try:
                pt1_map = self._tf_buffer.transform(pt1, self.target_frame, timeout=self._lookup_timeout_duration)
                pt2_map = self._tf_buffer.transform(pt2, self.target_frame, timeout=self._lookup_timeout_duration)
            except TransformException as ex:
                Logger.logwarn(f"Transform failed: {ex}")
                continue

            # 2) Calculate segment midpoint and angle
            mx = 0.5 * (pt1_map.point.x + pt2_map.point.x)
            my = 0.5 * (pt1_map.point.y + pt2_map.point.y)
            dx = pt2_map.point.x - pt1_map.point.x
            dy = pt2_map.point.y - pt1_map.point.y
            segment_angle = math.atan2(dy, dx)

            # 3) Extract robot pose from stored transform and convert to yaw
            if self.transform is None:
                Logger.logwarn("_ros_message_handler: self.transform is None, cannot compare.")
                return

            t = self.transform.transform.translation
            r = self.transform.transform.rotation
            q_t3d = [r.w, r.x, r.y, r.z]  # transforms3d expects [w, x, y, z]
            rotation_matrix = transforms3d.quaternions.quat2mat(q_t3d)
            _, _, ref_yaw = transforms3d.euler.mat2euler(rotation_matrix, axes="sxyz")

            # 4) Calculate distance and angle deviations
            dist = math.hypot(mx - t.x, my - t.y)
            actual_dist_offset = abs(dist - self.target_distance_to_segment)

            yaw_diff = abs(angle_diff(segment_angle, ref_yaw))
            diff_plus_90 = abs(angle_diff(segment_angle + math.pi / 2.0, ref_yaw))
            diff_minus_90 = abs(angle_diff(segment_angle - math.pi / 2.0, ref_yaw))
            yaw_diff_to_normal = min(diff_plus_90, diff_minus_90)  # Robot should face perpendicular to segment

            # 5) Check if segment matches within tolerances
            if actual_dist_offset <= self.position_tolerance and yaw_diff_to_normal <= self.angle_tolerance:
                Logger.loginfo(
                    f"Pose condition met for segment. "
                    f"DistOffset: {actual_dist_offset:.3f} "
                    f"(Cur: {dist:.3f}, Tgt: {self.target_distance_to_segment:.2f}), "
                    f"YawDiffToNormal: {yaw_diff_to_normal:.3f}"
                )

                # Create output transform with segment midpoint and orientation
                segment_pose_ts = TransformStamped()
                segment_pose_ts.header.stamp = self._node.get_clock().now().to_msg()
                segment_pose_ts.header.frame_id = self.target_frame  # e.g., "map"
                # Use the state's instance name to make the child_frame_id somewhat unique
                segment_pose_ts.child_frame_id = f"{self.name}_detected_segment"

                segment_pose_ts.transform.translation.x = mx
                segment_pose_ts.transform.translation.y = my
                segment_pose_ts.transform.translation.z = (pt1_map.point.z + pt2_map.point.z) / 2.0

                # Convert yaw to quaternion (euler2quat returns w, x, y, z)
                q_w, q_x, q_y, q_z = transforms3d.euler.euler2quat(0.0, 0.0, segment_angle, axes="sxyz")
                segment_pose_ts.transform.rotation.w = q_w
                segment_pose_ts.transform.rotation.x = q_x
                segment_pose_ts.transform.rotation.y = q_y
                segment_pose_ts.transform.rotation.z = q_z

                self._mapped_segemnt = segment_pose_ts
                self._pose_condition_met = True
                return
            Logger.logwarn(
                f"Segment not met. DistOffset: {actual_dist_offset:.4f} "
                f"(Cur: {dist:.4f}, Tgt: {self.target_distance_to_segment:.2f}), "
                f"YawDiffToNormal: {yaw_diff_to_normal:.4f}, OriginalYawDiff: {yaw_diff:.4f}"
            )
