/**
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <px4_tasks.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/err.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/offboard_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>

static bool thread_should_exit = false;     /**< daemon exit flag */
static bool thread_running = false;     /**< daemon status flag */
static int offboard_pub_task;             /**< Handle of daemon task / thread */
//static int gps_sequence = 0;

static uint8_t SETPOINT_TYPE_POSITION = 0;
static uint8_t SETPOINT_TYPE_LOITER = 2;
static uint8_t SETPOINT_TYPE_TAKEOFF = 3;
static uint8_t SETPOINT_TYPE_LAND = 4;
static uint8_t SETPOINT_TYPE_IDLE = 5;
static uint8_t VELOCITY_FRAME_LOCAL_NED = 1;
static uint8_t VELOCITY_FRAME_BODY_NED = 8;


__EXPORT int offboard_pub_main(int argc, char *argv[]);

int offboard_pub_thread_main(int argc, char *argv[]);

static void usage(const char *reason);


static void usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("usage: offboard_pub {start|stop|numb} [-p <additional params>]\n\n");
}


int offboard_pub_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            warnx("daemon already running\n");
            /* this is not an error */
            return 0;
        }

        thread_should_exit = false;
        offboard_pub_task = px4_task_spawn_cmd("offboard_pub",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2000,
                         offboard_pub_thread_main,
                         (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("\trunning\n");

        } else {
            warnx("\tnot started\n");
        }

        return 0;
    }


    usage("unrecognized command");
    return 1;

}

int offboard_pub_thread_main(int argc, char *argv[])
{

    warnx("[daemon] starting\n");

    thread_running = true;
    int rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));//订阅topic
    int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    int offboard_setpoint_sub = orb_subscribe(ORB_ID(offboard_setpoint));

    /*定义offboard话题结构体*/
    struct rc_channels_s _rc_channels = {};//定义topic的结构体
    struct offboard_control_mode_s _offboard_control_mode = {};
//    struct vehicle_control_mode_s _control_mode;
    struct position_setpoint_triplet_s _pos_sp_triplet = {};
    struct vehicle_control_mode_s _control_mode;
    struct offboard_setpoint_s _offboard_sp = {};

    /*公告主题*/
    /*offboard_pub_pub为handle指针*/
    orb_advert_t offboard_pub_pub = orb_advertise(ORB_ID(offboard_control_mode), &_offboard_control_mode);
    orb_advert_t pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);
    while (!thread_should_exit) {

        /*结构体变量赋值*/
        _offboard_control_mode.ignore_acceleration_force = true;
        _offboard_control_mode.ignore_alt_hold = _offboard_sp.ignore_alt_hold;
        _offboard_control_mode.ignore_attitude = _offboard_sp.ignore_attitude;
        _offboard_control_mode.ignore_bodyrate = true;
        _offboard_control_mode.ignore_position = _offboard_sp.ignore_position;
        _offboard_control_mode.ignore_thrust = true;
        _offboard_control_mode.ignore_velocity = _offboard_sp.ignore_velocity;

        _offboard_control_mode.timestamp = hrt_absolute_time();


        /*发布数据*/
        orb_publish(ORB_ID(offboard_control_mode), offboard_pub_pub, &_offboard_control_mode);
	orb_copy(ORB_ID(rc_channels), rc_channels_sub, &_rc_channels);//获取数据
	printf("channel 4 = %8.4f\n", (double)_rc_channels.channels[4]);
        bool updated_mode;
        bool updated_data;
	bool updated_rcch;
        orb_check(control_mode_sub, &updated_mode);//检查topic是否获取成功
        orb_check(offboard_setpoint_sub, &updated_data);
	orb_check(rc_channels_sub, &updated_rcch);//检测订阅是否成功
        if (updated_mode) {
            orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &_control_mode);
        }
        if (updated_data) {
            orb_copy(ORB_ID(offboard_setpoint), offboard_setpoint_sub, &_offboard_sp);//获取数据
        }

        if (_control_mode.flag_control_offboard_enabled) {

            _pos_sp_triplet.timestamp = hrt_absolute_time();
            _pos_sp_triplet.previous.valid = false;
            _pos_sp_triplet.next.valid = false;
            _pos_sp_triplet.current.valid = true;


            bool is_takeoff_sp = _offboard_sp.is_takeoff_sp;
            bool is_land_sp = _offboard_sp.is_land_sp;
            bool is_loiter_sp = _offboard_sp.is_loiter_sp;
            bool is_idle_sp = _offboard_sp.is_idle_sp;

            if (is_loiter_sp) {
                _pos_sp_triplet.current.type = SETPOINT_TYPE_LOITER;

            } else if (is_takeoff_sp) {
                _pos_sp_triplet.current.type = SETPOINT_TYPE_TAKEOFF;

            } else if (is_land_sp) {
                _pos_sp_triplet.current.type = SETPOINT_TYPE_LAND;

            } else if (is_idle_sp) {
                _pos_sp_triplet.current.type = SETPOINT_TYPE_IDLE;

            } else {
                _pos_sp_triplet.current.type = SETPOINT_TYPE_POSITION;
            }

            if (!_offboard_control_mode.ignore_position) {
                _pos_sp_triplet.current.position_valid = true;
                _pos_sp_triplet.current.x = _offboard_sp.x;//设的点赋给系统参数
                _pos_sp_triplet.current.y = _offboard_sp.y;
                _pos_sp_triplet.current.z = _offboard_sp.z;

                warnx("ignore position");

            } else {
                _pos_sp_triplet.current.position_valid = false;
            }

            /* set the local vel values */
            if (!_offboard_control_mode.ignore_velocity) {
                _pos_sp_triplet.current.velocity_valid = true;
                _pos_sp_triplet.current.vx = _offboard_sp.vx;
                _pos_sp_triplet.current.vy = _offboard_sp.vy;
                _pos_sp_triplet.current.vz = _offboard_sp.vz;

                if (_offboard_sp.is_local_frame) {
                    _pos_sp_triplet.current.velocity_frame = VELOCITY_FRAME_LOCAL_NED;
                } else {
                    _pos_sp_triplet.current.velocity_frame = VELOCITY_FRAME_BODY_NED;
                }

//                    set_position_target_local_ned.coordinate_frame;
                warnx("ignore velocity");

            } else {
                _pos_sp_triplet.current.velocity_valid = false;
            }

            if (!_offboard_control_mode.ignore_alt_hold) {
                _pos_sp_triplet.current.alt_valid = true;
                _pos_sp_triplet.current.z = _offboard_sp.z;

            } else {
                _pos_sp_triplet.current.alt_valid = false;
            }

            /* set the local acceleration values if the setpoint type is 'local pos' and none
             * of the accelerations fields is set to 'ignore' */
            if (!_offboard_control_mode.ignore_acceleration_force) {
                _pos_sp_triplet.current.acceleration_valid = true;
                _pos_sp_triplet.current.a_x = 0.0;
                _pos_sp_triplet.current.a_y = 0.0;
                _pos_sp_triplet.current.a_z = 0.0;
//                _pos_sp_triplet.current.acceleration_is_force =
//                    is_force_sp;

            } else {
                _pos_sp_triplet.current.acceleration_valid = false;
            }

            /* set the yaw sp value */
            if (!_offboard_control_mode.ignore_attitude) {
                _pos_sp_triplet.current.yaw_valid = true;
                _pos_sp_triplet.current.yaw = _offboard_sp.yaw;

            } else {
                _pos_sp_triplet.current.yaw_valid = false;
            }

            /* set the yawrate sp value */
            if (!_offboard_control_mode.ignore_bodyrate) {
                _pos_sp_triplet.current.yawspeed_valid = true;
//                _pos_sp_triplet.current.yawspeed = set_position_target_local_ned.yaw_rate;

            } else {
                _pos_sp_triplet.current.yawspeed_valid = false;
            }

            orb_publish(ORB_ID(position_setpoint_triplet), pos_sp_triplet_pub, &_pos_sp_triplet);
        }


        usleep(200000);
    }

    warnx("[daemon] exiting.\n");

    thread_running = false;

    return 0;
}
