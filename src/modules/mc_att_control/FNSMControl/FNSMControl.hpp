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
 * @file AttitudeControl.hpp
 *
 * A quaternion based attitude controller.
 *
 * @author Matthias Grob	<maetugr@gmail.com>
 *
 * Publication documenting the implemented Quaternion Attitude Control:
 * Nonlinear Quadrocopter Attitude Control (2013)
 * by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
 * Institute for Dynamic Systems and Control (IDSC), ETH Zurich
 *
 * https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <controllib/blocks.hpp>
#include <cmath>

#define sig(a, b) pow(abs(a), b)*(a/abs(a))

struct FNSMParam{
	float lambda1;
	float lambda2;
	float beta1;
	float beta2;
	float r1;
	float r2 = 2*r1/(r1+1);
	float b1;
};

class FNSMControl: public control::SuperBlock
{
public:
	FNSMControl();
	~FNSMControl() = default;

	void setParam(FNSMParam &roll, FNSMParam &pitch, FNSMParam &yaw){ _params(0) = roll; _params(1) = pitch; _params(2) = yaw; }
	void setIntegratorLimit(const matrix::Vector3f &integrator_limit) { _lim_int = integrator_limit; };
	void resetIntegral() { _control_int.zero(); }
	matrix::Vector3f update(const matrix::Vector3f &angle, const matrix::Vector3f &angle_sp, const matrix::Vector3f &angular_velocity, const float dt, const bool land);

	matrix::Vector3f _control_int;
	matrix::Vector3<FNSMParam> _params;

private:
	void updateIntegral(const float dt);
	void updateSlidingSurface(const float dt);

	matrix::Vector3f _error;
	matrix::Vector3f _lim_int;
	matrix::Vector3f _sliding_surface;
	matrix::Matrix<float, 3, 3> j_mat;
	control::BlockDerivative _derivate_pitch_e;
	control::BlockDerivative _derivate_pitch;
	control::BlockDerivative _dderivate_pitch;
	control::BlockDerivative _derivate_roll_e;
	control::BlockDerivative _derivate_roll;
	control::BlockDerivative _dderivate_roll;
	control::BlockDerivative _derivate_yaw_e;
	control::BlockDerivative _derivate_yaw;
	control::BlockDerivative _dderivate_yaw;

	float L = 0.11669;
	float K4 = 0.001;
	float J1 = 0.0053;
	float J2 = 0.0057;
	float J3 = 0.008;
};
