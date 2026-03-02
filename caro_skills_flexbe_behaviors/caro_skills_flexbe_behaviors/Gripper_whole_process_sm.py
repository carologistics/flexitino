#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2026 Sam Köhler
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

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Whole_process.

This behavior is a showcase for all the commen states we use to grip an object.
You can use this beahvior as an refance.

Created on  02.03.2026
@author: Sam Köhler
"""


from caro_skills_flexbe_states.Calibrate_state import CalibratetoOrigin
from caro_skills_flexbe_states.gripper_gigatino_state import GripperState
from caro_skills_flexbe_states.home_state import BackToOrigin
from caro_skills_flexbe_states.motor_move_state import MotorMoveState
from caro_skills_flexbe_states.move_gripper_state import GripperMove
from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class Whole_processSM(Behavior):
    """
    Define Whole_process.

    This behavior is a showcase for all the commen states we use to grip an object.
    You can use this beahvior as an refance.
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Whole_process'

        # parameters of this behavior
        self.add_parameter('timeout', 0)
        self.add_parameter('action_topic', '')

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:410 y:375, x:1313 y:137
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.target_frame_gripper = 'end_effector_home'
        _state_machine.userdata.open = True
        _state_machine.userdata.close = False
        _state_machine.userdata.ns = 'robotinobase3'
        _state_machine.userdata.target_frame_motor_move = 'base_link'
        _state_machine.userdata.target_x = 0.1
        _state_machine.userdata.target_y = 0.1
        _state_machine.userdata.target_yaw_euler = 0

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:118 y:27
            OperatableStateMachine.add('move robot',
                                       MotorMoveState(timeout=self.timeout),
                                       transitions={'reached': 'calibrate gripper'  # 291 53 -1 -1 -1 -1
                                                    , 'failed': 'finished'  # 340 240 -1 -1 -1 -1
                                                    , 'canceled': 'finished'  # 340 240 -1 -1 -1 -1
                                                    , 'timeout': 'finished'  # 340 240 -1 -1 -1 -1
                                                    },
                                       autonomy={'reached': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'namespace': 'ns',
                                                  'frame': 'target_frame_motor_move',
                                                  'target_x': 'target_x',
                                                  'target_y': 'target_y',
                                                  'target_yaw': 'target_yaw_euler',
                                                  'distance': 'distance'})

            # x:330 y:24
            OperatableStateMachine.add('calibrate gripper',
                                       CalibratetoOrigin(timeout=self.timeout),
                                       transitions={'home_reached': 'open gripper'  # 516 47 -1 -1 -1 -1
                                                    , 'failed': 'finished'  # 421 205 -1 -1 -1 -1
                                                    , 'canceled': 'finished'  # 421 205 -1 -1 -1 -1
                                                    , 'timeout': 'finished'  # 421 205 -1 -1 -1 -1
                                                    },
                                       autonomy={'home_reached': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'ns': 'ns'})

            # x:1009 y:18
            OperatableStateMachine.add('close gripper',
                                       GripperState(timeout=self.timeout),
                                       transitions={'success': 'move gripper back'  # 1080 93 -1 -1 -1 -1
                                                    , 'failed': 'finished'  # 711 241 -1 -1 -1 -1
                                                    , 'canceled': 'finished'  # 711 241 -1 -1 -1 -1
                                                    , 'timeout': 'finished'  # 711 241 -1 -1 -1 -1
                                                    },
                                       autonomy={'success': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'open': 'close', 'ns': 'ns'})

            # x:784 y:23
            OperatableStateMachine.add('move gripper',
                                       GripperMove(timeout=self.timeout),
                                       transitions={'reached': 'close gripper'  # 964 44 -1 -1 -1 -1
                                                    , 'failed': 'finished'  # 604 234 -1 -1 -1 -1
                                                    , 'canceled': 'finished'  # 604 234 -1 -1 -1 -1
                                                    , 'timeout': 'finished'  # 604 234 -1 -1 -1 -1
                                                    },
                                       autonomy={'reached': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'target_frame': 'target_frame_gripper',
                                                  'ns': 'ns'})

            # x:1016 y:120
            OperatableStateMachine.add('move gripper back',
                                       BackToOrigin(timeout=self.timeout),
                                       transitions={'home_reached': 'failed'  # 1234 147 -1 -1 -1 -1
                                                    , 'failed': 'finished'  # 734 331 -1 -1 -1 -1
                                                    , 'canceled': 'finished'  # 734 331 -1 -1 -1 -1
                                                    , 'timeout': 'finished'  # 734 331 -1 -1 -1 -1
                                                    },
                                       autonomy={'home_reached': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'ns': 'ns'})

            # x:566 y:22
            OperatableStateMachine.add('open gripper',
                                       GripperState(timeout=self.timeout),
                                       transitions={'success': 'move gripper'  # 745 46 -1 -1 -1 -1
                                                    , 'failed': 'finished'  # 494 226 -1 -1 -1 -1
                                                    , 'canceled': 'finished'  # 494 226 -1 -1 -1 -1
                                                    , 'timeout': 'finished'  # 494 226 -1 -1 -1 -1
                                                    },
                                       autonomy={'success': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'open': 'open', 'ns': 'ns'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
