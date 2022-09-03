/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <lib/mixer/MixerBase/Mixer.hpp> // Airmode
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>//YJJ
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_angular_acceleration.h>//YJJ
#include <uORB/topics/vehicle_angular_velocity.h>//YJJ
#include <uORB/topics/actuator_controls.h>//YJJ
#include <uORB/topics/vehicle_thrust_setpoint.h>//YJJ
#include <uORB/topics/vehicle_torque_setpoint.h>//YJJ
#include <vtol_att_control/vtol_type.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <uORB/topics/control_data.h>

#include <AttitudeControl.hpp>

#include <FNSMControl.hpp>

using namespace time_literals;

/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

class MulticopterAttitudeControl : public ModuleBase<MulticopterAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterAttitudeControl(bool vtol = false);
	~MulticopterAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	float		throttle_curve(float throttle_stick_input);

	/**
	 * Generate & publish an attitude setpoint from stick inputs
	 */
	void		generate_attitude_setpoint(const matrix::Quatf &q, float dt, bool reset_yaw_sp);

	void publishTorqueSetpoint(const matrix::Vector3f &torque_sp, const hrt_abstime &timestamp_sample);//YJJ
	void publishThrustSetpoint(const hrt_abstime &timestamp_sample);//YJJ
	void updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt);

	AttitudeControl _attitude_control; ///< class for attitude control calculations

	FNSMControl _fnsm_controller;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription */
	uORB::Subscription _autotune_attitude_control_status_sub{ORB_ID(autotune_attitude_control_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< manual control setpoint subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::Subscription _landing_gear_sub{ORB_ID(landing_gear)};

	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};//YJJ: 订阅角速度，欧拉角的一次导数

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	uORB::Publication<vehicle_rates_setpoint_s>	_v_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};			/**< rate setpoint publication */
	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_0_pub{ORB_ID(actuator_controls_status_0)};
	uORB::Publication<vehicle_attitude_setpoint_s>	_vehicle_attitude_setpoint_pub;
	uORB::Publication<actuator_controls_s>		_actuators_0_pub;//YJJ: 发布控制量
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};//YJJ: 发布油门值
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};//YJJ： 发布扭矩值
	uORB::Publication<control_data_s>               _control_data_pub{ORB_ID(control_data)};

	orb_advert_t _mavlink_log_pub{nullptr};

	struct manual_control_setpoint_s	_manual_control_setpoint {};	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode {};	/**< vehicle control mode */

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	matrix::Vector3f _thrust_setpoint_body; ///< body frame 3D thrust vector

	float _thrust_sp{0.0f};//YJJ

	float _man_yaw_sp{0.f};				/**< current yaw setpoint in manual mode */
	float _man_tilt_max;			/**< maximum tilt allowed for manual flight [rad] */
	float _energy_integration_time{0.0f};
	float _control_energy[4] {};
	AlphaFilter<float> _man_x_input_filter;
	AlphaFilter<float> _man_y_input_filter;

	hrt_abstime _last_run{0};

	bool _landed{true};
	bool _maybe_landed{true};
	bool _reset_yaw_sp{true};
	bool _vehicle_type_rotary_wing{true};
	bool _vtol{false};
	bool _vtol_tailsitter{false};
	bool _vtol_in_transition_mode{false};

	uint8_t _quat_reset_counter{0};
	int8_t _landing_gear{landing_gear_s::GEAR_DOWN};


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLL_A1>) _param_mc_roll_a1,
		(ParamFloat<px4::params::MC_ROLL_A2>) _param_mc_roll_a2,
		(ParamFloat<px4::params::MC_ROLL_B1>) _param_mc_roll_b1,
		(ParamFloat<px4::params::MC_ROLL_B2>) _param_mc_roll_b2,
		(ParamFloat<px4::params::MC_ROLL_H>) _param_mc_roll_h,
		(ParamFloat<px4::params::MC_ROLL_Y1>) _param_mc_roll_y1,

		(ParamFloat<px4::params::MC_PITCH_A1>) _param_mc_pitch_a1,
		(ParamFloat<px4::params::MC_PITCH_A2>) _param_mc_pitch_a2,
		(ParamFloat<px4::params::MC_PITCH_B1>) _param_mc_pitch_b1,
		(ParamFloat<px4::params::MC_PITCH_B2>) _param_mc_pitch_b2,
		(ParamFloat<px4::params::MC_PITCH_H>) _param_mc_pitch_h,
		(ParamFloat<px4::params::MC_PITCH_Y1>) _param_mc_pitch_y1,

		(ParamFloat<px4::params::MC_YAW_A1>) _param_mc_yaw_a1,
		(ParamFloat<px4::params::MC_YAW_A2>) _param_mc_yaw_a2,
		(ParamFloat<px4::params::MC_YAW_B1>) _param_mc_yaw_b1,
		(ParamFloat<px4::params::MC_YAW_B2>) _param_mc_yaw_b2,
		(ParamFloat<px4::params::MC_YAW_H>) _param_mc_yaw_h,
		(ParamFloat<px4::params::MC_YAW_Y1>) _param_mc_yaw_y1,

		(ParamInt<px4::params::MC_ROLL_SWITCH>) _param_mc_roll_switch,
		(ParamInt<px4::params::MC_PITCH_SWITCH>) _param_mc_pitch_switch,
		(ParamInt<px4::params::MC_YAW_SWITCH>) _param_mc_yaw_switch,


		(ParamFloat<px4::params::MC_ROLL_P>) _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>) _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>) _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAW_WEIGHT>) _param_mc_yaw_weight,

		(ParamFloat<px4::params::MC_ROLLRATE_MAX>) _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>) _param_mc_yawrate_max,

		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,			/**< scaling factor from stick to yaw rate */

		/* Stabilized mode params */
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,			/**< maximum tilt allowed for manual flight */
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) _param_mpc_manthr_min,			/**< minimum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,				/**< maximum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_HOVER>)
		_param_mpc_thr_hover,			/**< throttle at which vehicle is at hover equilibrium */
		(ParamInt<px4::params::MPC_THR_CURVE>) _param_mpc_thr_curve,				/**< throttle curve behavior */

		(ParamInt<px4::params::MC_AIRMODE>) _param_mc_airmode,
		(ParamFloat<px4::params::MC_MAN_TILT_TAU>) _param_mc_man_tilt_tau
	)
};
