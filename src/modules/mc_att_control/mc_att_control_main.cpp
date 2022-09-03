/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"
#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

MulticopterAttitudeControl::MulticopterAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_fnsm_controller(),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_vtol(vtol)

{
	if (_vtol) {
		int32_t vt_type = -1;

		if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
			_vtol_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
		}
	}

	parameters_updated();
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	return true;
}

void
MulticopterAttitudeControl::parameters_updated()
{
	FNSMParam roll, pitch, yaw;
	roll.lambda1 = _param_mc_roll_a1.get();
	roll.lambda2 = _param_mc_roll_a2.get();
	roll.beta1 = _param_mc_roll_b1.get();
	roll.beta2 = _param_mc_roll_b2.get();
	roll.r1 = _param_mc_roll_y1.get();
	roll.b1 = _param_mc_roll_h.get();

	pitch.lambda1 = _param_mc_pitch_a1.get();
	pitch.lambda2 = _param_mc_pitch_a2.get();
	pitch.beta1 = _param_mc_pitch_b1.get();
	pitch.beta2 = _param_mc_pitch_b2.get();
	pitch.r1 = _param_mc_pitch_y1.get();
	pitch.b1 = _param_mc_pitch_h.get();

	yaw.lambda1 = _param_mc_yaw_a1.get();
	yaw.lambda2 = _param_mc_yaw_a2.get();
	yaw.beta1 = _param_mc_yaw_b1.get();
	yaw.beta2 = _param_mc_yaw_b2.get();
	yaw.r1 = _param_mc_yaw_y1.get();
	yaw.b1 = _param_mc_yaw_h.get();

	_fnsm_controller.setParam(roll, pitch, yaw);
	_fnsm_controller.setIntegratorLimit(Vector3f(10,10,10));

	// Store some of the parameters in a more convenient way & precompute often-used values
	_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()),
					      _param_mc_yaw_weight.get());

	// angular rate limits
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());
}

float
MulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
	const float throttle_min = _landed ? 0.0f : _param_mpc_manthr_min.get();

	// throttle_stick_input is in range [0, 1]
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling to hover throttle
		return throttle_min + throttle_stick_input * (_param_mpc_thr_max.get() - throttle_min);

	default: // 0 or other: rescale to hover throttle at 0.5 stick
		return math::gradual3(throttle_stick_input,
				      0.f, .5f, 1.f,
				      throttle_min, _param_mpc_thr_hover.get(), _param_mpc_thr_max.get());
	}
}

void
MulticopterAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt, bool reset_yaw_sp)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};
	const float yaw = Eulerf(q).psi();

	/* reset yaw setpoint to current position if needed */
	if (reset_yaw_sp) {
		_man_yaw_sp = yaw;

	} else if (math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f) > 0.05f
		   || _param_mc_airmode.get() == (int32_t)Mixer::Airmode::roll_pitch_yaw) {

		const float yaw_rate = math::radians(_param_mpc_man_y_max.get());
		attitude_setpoint.yaw_sp_move_rate = _manual_control_setpoint.r * yaw_rate;
		_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
	}

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(x*x + y*y)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	_man_x_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_y_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_x_input_filter.update(_manual_control_setpoint.x * _man_tilt_max);
	_man_y_input_filter.update(_manual_control_setpoint.y * _man_tilt_max);
	const float x = _man_x_input_filter.getState();
	const float y = _man_y_input_filter.getState();

	// we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
	Vector2f v = Vector2f(y, -x);
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rpy = AxisAnglef(v(0), v(1), 0.f);
	Eulerf euler_sp = q_sp_rpy;
	attitude_setpoint.roll_body = euler_sp(0);
	attitude_setpoint.pitch_body = euler_sp(1);
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);

	/* modify roll/pitch only if we're a VTOL */
	if (_vtol) {
		// Construct attitude setpoint rotation matrix. Modify the setpoints for roll
		// and pitch such that they reflect the user's intention even if a large yaw error
		// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
		// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
		// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
		// heading of the vehicle.
		// However there's also a coupling effect that causes oscillations for fast roll/pitch changes
		// at higher tilt angles, so we want to avoid using this on multicopters.
		// The effect of that can be seen with:
		// - roll/pitch into one direction, keep it fixed (at high angle)
		// - apply a fast yaw rotation
		// - look at the roll and pitch angles: they should stay pretty much the same as when not yawing

		// calculate our current yaw error
		float yaw_error = wrap_pi(attitude_setpoint.yaw_body - yaw);

		// compute the vector obtained by rotating a z unit vector by the rotation
		// given by the roll and pitch commands of the user
		Vector3f zB = {0.0f, 0.0f, 1.0f};
		Dcmf R_sp_roll_pitch = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, 0.0f);
		Vector3f z_roll_pitch_sp = R_sp_roll_pitch * zB;

		// transform the vector into a new frame which is rotated around the z axis
		// by the current yaw error. this vector defines the desired tilt when we look
		// into the direction of the desired heading
		Dcmf R_yaw_correction = Eulerf(0.0f, 0.0f, -yaw_error);
		z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

		// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
		// R_tilt is computed from_euler; only true if cos(roll) not equal zero
		// -> valid if roll is not +-pi/2;
		attitude_setpoint.roll_body = -asinf(z_roll_pitch_sp(1));
		attitude_setpoint.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
	}

	/* copy quaternion setpoint to attitude setpoint topic */
	Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
	q_sp.copyTo(attitude_setpoint.q_d);

	attitude_setpoint.thrust_body[2] = -throttle_curve(math::constrain(_manual_control_setpoint.z, 0.0f,
					   1.0f));
	attitude_setpoint.timestamp = hrt_absolute_time();

	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

void
MulticopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	// run controller on attitude updates
	vehicle_attitude_s v_att;

	if (_vehicle_attitude_sub.update(&v_att)) {

		const matrix::Quatf q(v_att.q);//YJJ: 取出更新的姿态四元数
		const matrix::Eulerf euler = q;//YJJ: 四元数转欧拉角，得到euler

		vehicle_status_s vehicle_status;
		control_data_s _control_data;
                vehicle_angular_velocity_s angular_velocity;
		vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

		_vehicle_angular_velocity_sub.copy(&angular_velocity);
		_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint);

		Vector3f des_angle;
                des_angle(1) = vehicle_attitude_setpoint.pitch_body;
		des_angle(0) = vehicle_attitude_setpoint.roll_body;
		des_angle(2) = vehicle_attitude_setpoint.yaw_body;

		// Check for new attitude setpoint
		if (_vehicle_attitude_setpoint_sub.updated()) {
			_vehicle_attitude_setpoint_sub.update(&vehicle_attitude_setpoint);
			_attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
			_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
		}

		// Check for a heading reset
		if (_quat_reset_counter != v_att.quat_reset_counter) {
			const Quatf delta_q_reset(v_att.delta_q_reset);
			// for stabilized attitude generation only extract the heading change from the delta quaternion
			_man_yaw_sp += Eulerf(delta_q_reset).psi();
			_attitude_control.adaptAttitudeSetpoint(delta_q_reset);

			_quat_reset_counter = v_att.quat_reset_counter;
		}

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((v_att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = v_att.timestamp_sample;

		/* check for updates in other topics */
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_v_control_mode_sub.update(&_v_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		if (_landing_gear_sub.updated()) {
			landing_gear_s landing_gear;

			if (_landing_gear_sub.copy(&landing_gear)) {
				if (landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {
					if (landing_gear.landing_gear == landing_gear_s::GEAR_UP && (_landed || _maybe_landed)) {
						mavlink_log_critical(&_mavlink_log_pub, "Landed, unable to retract landing gear\t");
					} else {
						_landing_gear = landing_gear.landing_gear;
					}
				}
			}
		}

		if (_vehicle_status_sub.updated()) {

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				_vehicle_type_rotary_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				_vtol = vehicle_status.is_vtol;
				_vtol_in_transition_mode = vehicle_status.in_transition_mode;
			}
		}

		bool attitude_setpoint_generated = false;

		const bool is_hovering = (_vehicle_type_rotary_wing && !_vtol_in_transition_mode);

		// vehicle is a tailsitter in transition mode
		const bool is_tailsitter_transition = (_vtol_tailsitter && _vtol_in_transition_mode);

		bool run_att_ctrl = _v_control_mode.flag_control_attitude_enabled && (is_hovering || is_tailsitter_transition);

		_control_data.run_att_ctrl = run_att_ctrl;

		if (run_att_ctrl) {

			//const Quatf q{v_att.q};YJJ

			// Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
			if (_v_control_mode.flag_control_manual_enabled &&
			    !_v_control_mode.flag_control_altitude_enabled &&
			    !_v_control_mode.flag_control_velocity_enabled &&
			    !_v_control_mode.flag_control_position_enabled) {

				generate_attitude_setpoint(q, dt, _reset_yaw_sp);
				attitude_setpoint_generated = true;

			} else {
				_man_x_input_filter.reset(0.f);
				_man_y_input_filter.reset(0.f);
			}

			Vector3f _att_res = _fnsm_controller.update(euler, des_angle, Vector3f(angular_velocity.xyz), dt, _maybe_landed || _landed);
			_att_res(0) = math::constrain(_att_res(0), -0.3f, 0.3f);
			_att_res(1) = math::constrain(_att_res(1), -0.3f, 0.3f);
			_att_res(2) = math::constrain(_att_res(2), -0.3f, 0.3f);

			_control_data.timestamp_sample = v_att.timestamp_sample;
			_control_data.timestamp = hrt_absolute_time();
			_control_data.int_yaw = _fnsm_controller._control_int(2);
			_control_data.int_pitch = _fnsm_controller._control_int(1);
			_control_data.int_roll = _fnsm_controller._control_int(0);

			_control_data.tau_yaw = _att_res(2);
			_control_data.tau_roll = _att_res(0);
			_control_data.tau_pitch = _att_res(1);
			_control_data.pitch = euler(1);
			_control_data.roll = euler(0);
			_control_data.yaw = euler(2);
			_control_data.des_pitch = des_angle(1);
			_control_data.des_roll = des_angle(0);
			_control_data.des_yaw = des_angle(2);

			_thrust_sp = math::constrain(-vehicle_attitude_setpoint.thrust_body[2], 0.0f, 1.0f);//YJJ： 取了负，因为Z轴朝下

			actuator_controls_s actuators{};//YJJ
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(_att_res(0)) ? _att_res(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(_att_res(1)) ? _att_res(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(_att_res(2)) ? _att_res(2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
			actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = _landing_gear;
			actuators.timestamp_sample = v_att.timestamp_sample;

			//const hrt_abstime now = hrt_absolute_time();

			if (!vehicle_status.is_vtol) {
				publishTorqueSetpoint(_att_res, v_att.timestamp_sample);
				publishThrustSetpoint(v_att.timestamp_sample);
			}

			// FILE *fichier = fopen("info.txt","a");
   			// fprintf(fichier,"             \tpitch\troll\tyaw\n");
    			// fprintf(fichier,"control_outputs: \t%f\t%f\t%f\n",(double)tau_Pitch, tau_Roll, tau_Yaw);
			// fclose(fichier);

			actuators.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(actuators);
			_control_data_pub.publish(_control_data);

			updateActuatorControlsStatus(actuators, dt);
		}

		// reset yaw setpoint during transitions, tailsitter.cpp generates
		// attitude setpoint for the transition
		_reset_yaw_sp = !attitude_setpoint_generated || _landed || (_vtol && _vtol_in_transition_mode);
	}

	perf_end(_loop_perf);
}

void MulticopterAttitudeControl::publishTorqueSetpoint(const Vector3f &torque_sp, const hrt_abstime &timestamp_sample)
{
	vehicle_torque_setpoint_s v_torque_sp = {};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.timestamp_sample = timestamp_sample;
	v_torque_sp.xyz[0] = (PX4_ISFINITE(torque_sp(0))) ? torque_sp(0) : 0.0f;
	v_torque_sp.xyz[1] = (PX4_ISFINITE(torque_sp(1))) ? torque_sp(1) : 0.0f;
	v_torque_sp.xyz[2] = (PX4_ISFINITE(torque_sp(2))) ? torque_sp(2) : 0.0f;

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
}

void MulticopterAttitudeControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
{
	vehicle_thrust_setpoint_s v_thrust_sp = {};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.timestamp_sample = timestamp_sample;
	v_thrust_sp.xyz[0] = 0.0f;
	v_thrust_sp.xyz[1] = 0.0f;
	v_thrust_sp.xyz[2] = PX4_ISFINITE(_thrust_sp) ? -_thrust_sp : 0.0f; // Z is Down

	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
}

void MulticopterAttitudeControl::updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt)
{
	for (int i = 0; i < 4; i++) {
		_control_energy[i] += actuators.control[i] * actuators.control[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = actuators.timestamp;

		for (int i = 0; i < 4; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_0_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterAttitudeControl *instance = new MulticopterAttitudeControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
