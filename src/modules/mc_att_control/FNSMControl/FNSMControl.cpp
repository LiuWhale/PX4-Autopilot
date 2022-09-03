/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file FNSMControl.cpp
 */

#include <drivers/drv_hrt.h>
#include <FNSMControl.hpp>
#include <mathlib/math/Functions.hpp>
#include <lib/controllib/blocks.hpp>

FNSMControl::FNSMControl():
	SuperBlock(nullptr, "MPC"),
	_derivate_pitch_e(this, "VELD"),
	_derivate_pitch(this, "VELD"),
	_dderivate_pitch(this, "VELD"),
	_derivate_roll_e(this, "VELD"),
	_derivate_roll(this, "VELD"),
	_dderivate_roll(this, "VELD"),
	_derivate_yaw_e(this, "VELD"),
	_derivate_yaw(this, "VELD"),
	_dderivate_yaw(this, "VELD")
{
	j_mat(0, 0) = J1;
	j_mat(1, 1) = J2;
	j_mat(2, 2) = J3;
}

void FNSMControl::updateIntegral(const float dt)
{
	_control_int(0) = (_params(0).lambda1 *sig(_error(0), _params(0).r1) + _params(0).lambda2 *sig(_derivate_roll_e.update(_error(0)), _params(0).r2))*dt;
	_control_int(1) = (_params(1).lambda1 *sig(_error(1), _params(1).r1) + _params(1).lambda2 *sig(_derivate_pitch_e.update(_error(1)), _params(1).r2))*dt;
	_control_int(2) = (_params(2).lambda1 *sig(_error(2), _params(2).r1) + _params(2).lambda2 *sig(_derivate_yaw_e.update(_error(2)), _params(2).r2))*dt;

	for(int i = 0; i < 3; i++){
		if (PX4_ISFINITE(_control_int(i))) {
			_control_int(i) = math::constrain(_control_int(i), -_lim_int(i), _lim_int(i));
		}
	}
}

void FNSMControl::updateSlidingSurface(const float dt){
	_sliding_surface(0) = _derivate_roll_e.update(_error(0)) + _control_int(0);
	_sliding_surface(1) = _derivate_pitch_e.update(_error(1)) + _control_int(1);
	_sliding_surface(2) = _derivate_yaw_e.update(_error(2)) + _control_int(2);
}

matrix::Vector3f FNSMControl::update(const matrix::Vector3f &angle, const matrix::Vector3f &angle_sp, const matrix::Vector3f &angular_velocity, const float dt, const bool land)
{
	setDt(dt);
	_error = angle - angle_sp;
	matrix::Vector3f _temp, _derivate_mat;

	_temp(0) = _params(0).lambda1 *sig(_error(0), _params(0).r1) + _params(0).lambda2 *sig(_derivate_roll_e.update(_error(0)), _params(0).r2) + _params(0).beta1*_sliding_surface(0) + _params(0).beta2*sig(_sliding_surface(0), _params(0).b1);
	_temp(1) = _params(1).lambda1 *sig(_error(1), _params(1).r1) + _params(1).lambda2 *sig(_derivate_pitch_e.update(_error(1)), _params(1).r2) + _params(1).beta1*_sliding_surface(1) + _params(1).beta2*sig(_sliding_surface(1), _params(1).b1);
	_temp(2) = _params(2).lambda1 *sig(_error(2), _params(2).r1) + _params(2).lambda2 *sig(_derivate_yaw_e.update(_error(2)), _params(2).r2) + _params(2).beta1*_sliding_surface(2) + _params(2).beta2*sig(_sliding_surface(2), _params(2).b1);

	_derivate_mat(0) = _dderivate_roll.update(_derivate_roll.update(angle_sp(0)));
	_derivate_mat(1) = _dderivate_pitch.update(_derivate_pitch.update(angle_sp(1)));
	_derivate_mat(2) = _dderivate_yaw.update(_derivate_yaw.update(angle_sp(2)));


	const matrix::Vector3f res = K4*angular_velocity-j_mat*_temp/L + _derivate_mat;

	if (!land) {
		updateIntegral(dt);
		updateSlidingSurface(dt);
	}

	return res;
}
