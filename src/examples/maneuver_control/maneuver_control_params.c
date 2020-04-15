/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file maneuver_control_params.c
 *
 * Definition of parameters to define and control the performance
 * of automatic preprogrammed maneuvers.
 *
 *
 * @author Alberto Ruiz Garcia <aruizgarcia-1@tudelft.nl>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Safety switch PC
 *
 * [DESCRIPTION]
 *
 * @unit -
 * @group maneuver control
 */
PARAM_DEFINE_INT32(MAN_PC_SWITCH, 0);

/**
 * Safety switch TX
 *
 * [DESCRIPTION]
 *
 * @unit -
 * @group maneuver control
 */
PARAM_DEFINE_INT32(MAN_TX_SWITCH, 0);


/**
 * Maneuver id
 *
 * [DESCRIPTION]
 *
 * @unit -
 * @group maneuver control
 */
PARAM_DEFINE_INT32(MAN_ID, 0);

/**
 * Maneuver start time
 *
 * [DESCRIPTION]
 *
 * @unit ms
 * @group maneuver control
 */
PARAM_DEFINE_INT32(MAN_START, 0);


/**
 * Flag to enable/disable fw_att_control 
 *
 * [DESCRIPTION]
 *
 * @unit 
 * @group maneuver control
 */
PARAM_DEFINE_INT32(MAN_CTRL_FLAG, 0);

/**
 * Maneuver duration
 *
 * [DESCRIPTION]
 *
 * @unit ms
 * @min 0
 * @max 10000000
 * @increment 1000
 * @group maneuver control
 */
PARAM_DEFINE_INT32(MAN_DURATION, 0);


/**
 * Maneuver amplitude
 *
 * [DESCRIPTION]
 *
 * @unit -
 * @min 0.0
 * @max 0.8
 * @increment 0.05
 * @group maneuver control
 */
PARAM_DEFINE_FLOAT(MAN_AMPLITUDE, 0.0f);


/**
 * Maneuver center position
 *
 * [DESCRIPTION]
 *
 * @unit -
 * @min -0.4
 * @max 0.4
 * @increment 0.05
 * @group maneuver control
 */
PARAM_DEFINE_FLOAT(MAN_CENTER_POS, 0.0f);

/**
 * Desired control surface for the maneuver
 *
 * [DESCRIPTION]
 *
 * @unit -
 * @min 0
 * @max 3
 * @increment 1
 * @group maneuver control
 */
PARAM_DEFINE_INT32(MAN_CTRL_SURF, 0);


/**
 * Joystick position to abort maneuver
 *
 * [DESCRIPTION]
 *
 * @unit -
 * @min -0.5
 * @max 0.5
 * @increment 0.01
 * @group maneuver control
 */
PARAM_DEFINE_FLOAT(MAN_TX_ABORT, 0.1f);
