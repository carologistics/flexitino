#!/usr/bin/env python3
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
Generic FlexBE Service State for boolean toggle services.

This state allows calling any ROS2 service that has a boolean field in its request,
making it reusable across different robots and service types.

Author: Sam
License: BSD-3-Clause
"""
import importlib
import json

from flexbe_core import EventState
from flexbe_core import Logger
from flexbe_core.proxy import ProxyServiceCaller


class GenericToggleServiceState(EventState):
    """
    A generic FlexBE state for calling services with multiple parameters.

    This state dynamically loads the service type at runtime, allowing it to be
    used with any service. Supports multiple fields via a JSON string or dict.

    -- namespace        str         The ROS2 namespace (e.g., 'robotinobase1')
    -- service_name     str         Name of the service (e.g., 'toggle_segmentation')
    -- service_type     str         Full service type path (e.g., 'laser_scan_integrator_msg.srv.ToggleSegmentation')
    -- fields           str/dict    JSON string or dict with field name-value pairs
                                    e.g., '{"enable": true, "threshold": 0.5}' or {"enable": True}
    -- timeout          float       Timeout for service availability in seconds (default: 5.0)

    ># none             No input keys required

    #> success          bool    True if service call was successful (optional output)

    <= done             Service was called successfully
    <= failed           Service call failed due to an error
    <= unavailable      Service is not available within timeout

    Example usage:
        ros2 service call /robotinobase1/toggle_segmentation
            laser_scan_integrator_msg/srv/ToggleSegmentation "{enable_segmentation: true}"

        Would be configured as:
            namespace='robotinobase1'
            service_name='toggle_segmentation'
            service_type='laser_scan_integrator_msg.srv.ToggleSegmentation'
            fields='{"enable_segmentation": true}'
    """

    def __init__(self, namespace, service_name, service_type, fields, timeout=5.0):
        """
        Initialize the state.

        Args:
            namespace: ROS2 namespace for the service
            service_name: Name of the service to call
            service_type: Full Python import path for the service type
            fields: JSON string or dict with field name-value pairs
            timeout: Maximum time to wait for service availability
        """
        super().__init__(outcomes=["done", "failed", "unavailable"], output_keys=["success"])

        # Store parameters
        self._namespace = namespace
        self._service_name = service_name
        self._timeout = timeout

        # Parse fields - accept both JSON string and dict
        if isinstance(fields, str):
            try:
                self._fields = json.loads(fields)
            except json.JSONDecodeError as e:
                Logger.logerr(f"Failed to parse fields JSON: {str(e)}")
                self._fields = {}
        else:
            self._fields = fields

        # Construct the full service topic
        # Handle empty namespace case
        if namespace:
            self._service_topic = f"/{namespace}/{service_name}"
        else:
            self._service_topic = f"/{service_name}"

        # Dynamically load the service class
        try:
            module_name, class_name = service_type.rsplit(".", 1)
            module = importlib.import_module(module_name)
            self._service_class = getattr(module, class_name)
        except (ValueError, ImportError, AttributeError) as e:
            Logger.logerr(f'Failed to load service type "{service_type}": {str(e)}')
            self._service_class = None

        # Runtime state variables
        self._srv_result = None
        self._error = False
        self._unavailable = False

    def on_enter(self, userdata):
        """
        Called when the state becomes active.

        Initializes the service proxy and sends the request.
        """
        # Reset state variables
        self._error = False
        self._unavailable = False
        self._srv_result = None
        userdata.success = False

        # Check if service class was loaded successfully
        if self._service_class is None:
            Logger.logerr(f"[{self.name}] Service class not loaded - check service_type parameter")
            self._error = True
            return

        try:
            # Create service proxy
            srv_proxy = ProxyServiceCaller({self._service_topic: self._service_class}, wait_duration=self._timeout)

            # Check service availability
            if not srv_proxy.is_available(self._service_topic):
                Logger.logwarn(f'[{self.name}] Service "{self._service_topic}" is not available!')
                self._unavailable = True
                return

            # Create and populate the request
            request = self._service_class.Request()

            # Set all fields from the fields dict
            for field_name, field_value in self._fields.items():
                if not hasattr(request, field_name):
                    Logger.logerr(f'[{self.name}] Request has no field named "{field_name}"')
                    self._error = True
                    return
                setattr(request, field_name, field_value)

            # Log the service call
            Logger.loginfo(f"[{self.name}] Calling {self._service_topic} with {self._fields}")

            # Execute the service call (blocking)
            self._srv_result = srv_proxy.call(self._service_topic, request)
            userdata.success = True

        except Exception as e:
            Logger.logerr(f"[{self.name}] Service call failed: {str(e)}")
            self._error = True

    def execute(self, userdata):
        """
        Called periodically while the state is active.

        Returns the appropriate outcome based on the service call result.
        """
        if self._unavailable:
            return "unavailable"

        if self._error:
            return "failed"

        if self._srv_result is not None:
            Logger.loginfo(f"[{self.name}] Service call completed successfully")
            return "done"

        # No result yet - continue waiting
        # Note: This should not happen with blocking calls, but included for safety
        return None

    def on_exit(self, userdata):
        """
        Called when the state is exited.

        Cleanup can be performed here if needed.
        """

    def on_start(self):
        """
        Called when the behavior starts.

        Can be used for one-time initialization.
        """

    def on_stop(self):
        """
        Called when the behavior stops.

        Can be used for cleanup when behavior is stopped prematurely.
        """
