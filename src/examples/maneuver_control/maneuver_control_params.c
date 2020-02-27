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
 * @file gnd_att_control_params.c
 *
 * Parameters defined by the attitude control task for ground rovers
 *
 * This is a modification of the fixed wing params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

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
PARAM_DEFINE_INT32(MAN_START_TIME, 0);

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
 * Maneuver maximum deflection 
 *
 * [DESCRIPTION]
 *
 * @unit -
 * @min 0.0
 * @max 0.8
 * @increment 0.05 
 * @group maneuver control
 */
PARAM_DEFINE_FLOAT(MAN_MAX_DEF, 0.0f);

/**
 * Maneuver minimum deflection 
 *
 * [DESCRIPTION]
 *
 * @unit -
 * @min -0.8
 * @max 0.0
 * @increment 0.05 
 * @group maneuver control
 */
PARAM_DEFINE_FLOAT(MAN_MIN_DEF, 0.0f);

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





