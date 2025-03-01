#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 Carologistics
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
Define MotorMove.

This state moves the robot to the given pose using MotorMove messages. It does
not avoid collisions!

Created on Sat Feb 01 2025
@author: Carologistics
"""


from caro_skills_flexbe_states.motor_move import MotorMoveState
from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from motor_move_msgs.action import MotorMove

# [/MANUAL_IMPORT]


class MoveToSM(Behavior):
    """
    Define MotorMove.

    This state moves the robot to the given pose using MotorMove messages. It does
    not avoid collisions!
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'MoveTo'

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
        # x:679 y:199, x:58 y:338
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.target_x = 3.0
        _state_machine.userdata.target_y = 4.5
        _state_machine.userdata.target_yaw = 1.0
        _state_machine.userdata.frame_id = "robotinobase1/base_link"

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:185 y:175
            OperatableStateMachine.add('MotorMove',
                                       MotorMoveState(timeout=60,
                                                      action_topic="/robotinobase1/motor_move_action"),
                                       transitions={'pose_reached': 'finished'  # 521 151 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 127 293 -1 -1 -1 -1
                                                    , 'canceled': 'failed'  # 127 293 -1 -1 -1 -1
                                                    , 'timeout': 'failed'  # 127 293 -1 -1 -1 -1
                                                    },
                                       autonomy={'pose_reached': Autonomy.Off,
                                                 'failed': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'timeout': Autonomy.Off},
                                       remapping={'frame_id': 'frame_id',
                                                  'target_x': 'target_x',
                                                  'target_y': 'target_y',
                                                  'target_yaw': 'target_yaw'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
