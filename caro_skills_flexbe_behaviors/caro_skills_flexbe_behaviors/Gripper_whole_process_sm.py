#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 Zhen Yan Khaw
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.

#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Gripper_whole_process.

to grab the work piece and return to origin

Created on Thu Mar 13 2025
@author: Zhen Yan Khaw
"""


from caro_skills_flexbe_states.Calibrate_state import CalibratetoOrigin
from caro_skills_flexbe_states.gripper_close_gigatino_state import GripperCloseState
from caro_skills_flexbe_states.gripper_gigatino_state import GripperState
from caro_skills_flexbe_states.home_state import BackToOrigin
from caro_skills_flexbe_states.move_gripper_state import GripperMove
from caro_skills_flexbe_states.move_gripper_up_state import GripperMoveUp
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


class Gripper_whole_processSM(Behavior):
    """
    Define Gripper_whole_process.

    to grab the work piece and return to origin
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Gripper_whole_process'

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
        # x:909 y:591, x:130 y:400
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.x = 0.15
        _state_machine.userdata.y = 0.08
        _state_machine.userdata.z = 0.03
        _state_machine.userdata.z_offset = 0.05
        _state_machine.userdata.gripper_state = False
        _state_machine.userdata.use_gripper = False
        _state_machine.userdata.target_frame = 'robotinobase1/end_effector_home'
        _state_machine.userdata.relative = False
        _state_machine.userdata.open = True
        _state_machine.userdata.close = False

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:105 y:42
            OperatableStateMachine.add('calibrate',
                                       CalibratetoOrigin(timeout=10,
                                                         action_topic='robotinobase1/gigatino/calibrate'),
                                       transitions={'pose_reached': 'move_above'  # 314 10 -1 -1 -1 -1
                                                    , 'failed': 'move_above'  # 314 10 -1 -1 -1 -1
                                                    , 'canceled': 'move_above'  # 314 10 -1 -1 -1 -1
                                                    , 'timeout': 'move_above'  # 314 10 -1 -1 -1 -1
                                                    },
                                       autonomy={'pose_reached': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off})

            # x:891 y:155
            OperatableStateMachine.add('close_gripper',
                                       GripperCloseState(timeout=10,
                                                         action_topic='robotinobase1/gigatino/gripper'),
                                       transitions={'success': 'up'  # 943 266 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 514 301 -1 -1 -1 -1
                                                    , 'canceled': 'failed'  # 514 301 -1 -1 -1 -1
                                                    , 'timeout': 'failed'  # 514 301 -1 -1 -1 -1
                                                    },
                                       autonomy={'success': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'close': 'close'})

            # x:790 y:38
            OperatableStateMachine.add('down',
                                       GripperMove(timeout=10,
                                                   action_topic='robotinobase1/gigatino/move'),
                                       transitions={'reached': 'close_gripper'  # 873 95 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 470 250 -1 -1 -1 -1
                                                    , 'canceled': 'failed'  # 470 250 -1 -1 -1 -1
                                                    , 'timeout': 'failed'  # 470 250 -1 -1 -1 -1
                                                    },
                                       autonomy={'reached': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'relative': 'relative',
                                                  'target_frame': 'target_frame',
                                                  'x': 'x',
                                                  'y': 'y',
                                                  'z': 'z',
                                                  'gripper_state': 'gripper_state',
                                                  'use_gripper': 'use_gripper'})

            # x:529 y:37
            OperatableStateMachine.add('grip',
                                       GripperState(timeout=10,
                                                    action_topic='robotinobase1/gigatino/gripper'),
                                       transitions={'success': 'down'  # 706 38 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 332 252 -1 -1 -1 -1
                                                    , 'canceled': 'failed'  # 332 252 -1 -1 -1 -1
                                                    , 'timeout': 'failed'  # 332 252 -1 -1 -1 -1
                                                    },
                                       autonomy={'success': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'open': 'open'})

            # x:860 y:430
            OperatableStateMachine.add('home',
                                       BackToOrigin(timeout=10,
                                                    action_topic='robotinobase1/gigatino/home'),
                                       transitions={'pose_reached': 'finished'  # 931 568 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 496 445 -1 -1 -1 -1
                                                    , 'canceled': 'failed'  # 496 445 -1 -1 -1 -1
                                                    , 'timeout': 'failed'  # 496 445 -1 -1 -1 -1
                                                    },
                                       autonomy={'pose_reached': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off})

            # x:342 y:25
            OperatableStateMachine.add('move_above',
                                       GripperMoveUp(timeout=10,
                                                     action_topic='robotinobase1/gigatino/move'),
                                       transitions={'reached': 'grip'  # 516 28 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 240 242 -1 -1 -1 -1
                                                    , 'canceled': 'failed'  # 240 242 -1 -1 -1 -1
                                                    , 'timeout': 'failed'  # 240 242 -1 -1 -1 -1
                                                    },
                                       autonomy={'reached': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'relative': 'relative',
                                                  'target_frame': 'target_frame',
                                                  'x': 'x',
                                                  'y': 'y',
                                                  'z': 'z',
                                                  'gripper_state': 'gripper_state',
                                                  'use_gripper': 'use_gripper',
                                                  'z_offset': 'z_offset'})

            # x:883 y:288
            OperatableStateMachine.add('up',
                                       GripperMoveUp(timeout=10,
                                                     action_topic='robotinobase1/gigatino/move'),
                                       transitions={'reached': 'home'  # 940 428 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 509 374 -1 -1 -1 -1
                                                    , 'canceled': 'failed'  # 509 374 -1 -1 -1 -1
                                                    , 'timeout': 'failed'  # 509 374 -1 -1 -1 -1
                                                    },
                                       autonomy={'reached': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'relative': 'relative',
                                                  'target_frame': 'target_frame',
                                                  'x': 'x',
                                                  'y': 'y',
                                                  'z': 'z',
                                                  'gripper_state': 'gripper_state',
                                                  'use_gripper': 'use_gripper',
                                                  'z_offset': 'z_offset'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
