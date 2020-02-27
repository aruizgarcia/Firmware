/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

/* Auto-generated by genmsg_cpp from file actuator_controls.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define ACTUATOR_CONTROLS_INDEX_ROLL 0
#define ACTUATOR_CONTROLS_INDEX_PITCH 1
#define ACTUATOR_CONTROLS_INDEX_YAW 2
#endif


#ifdef __cplusplus
struct __EXPORT maneuver_control_setpoint_s {
#else
struct maneuver_control_setpoint_s {
#endif
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float pitch;
	float roll; 
	float yaw; 


#ifdef __cplusplus
	static constexpr uint8_t INDEX_ROLL = 0;
	static constexpr uint8_t INDEX_PITCH = 1;
	static constexpr uint8_t INDEX_YAW = 2;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(maneuver_control_setpoint);

#ifdef __cplusplus
void print_message(const maneuver_control_setpoint_s& message);
#endif
