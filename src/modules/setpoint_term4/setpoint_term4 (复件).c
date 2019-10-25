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
#include <math.h>
#include <px4_tasks.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/read_uart_term4.h>
#include <uORB/topics/offboard_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/rc_channels.h>

static bool thread_should_exit = false;     /**< daemon exit flag */
static bool thread_running = false;     /**< daemon status flag */
static int setpoint_term4_task;             /**< Handle of daemon task / thread */

static int set_count_mean = 5;


__EXPORT int setpoint_term4_main(int argc, char *argv[]);

int setpoint_term4_thread_main(int argc, char *argv[]);

static void usage(const char *reason);


static void usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("usage: setpoint_term4 {start|stop|status|numb} [-p <additional params>]\n\n");
}


int setpoint_term4_main(int argc, char *argv[])
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
        setpoint_term4_task = px4_task_spawn_cmd("setpoint_term4",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2000,
                         setpoint_term4_thread_main,
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

int setpoint_term4_thread_main(int argc, char *argv[])
{

    warnx("[daemon] starting\n");

    thread_running = true;

    //int token = 0;

    int count_1s = 0;
    int count_data = 0;
    int x_arr[10];
    int y_arr[10];
    int dis_x[4];
    int dis_y[4];
    bool updated_rcch;
    int rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));

	struct rc_channels_s _rc_channels = {};
    memset(&x_arr, 0, sizeof(x_arr));
    memset(&y_arr, 0, sizeof(y_arr));

    memset(&dis_x, 0, sizeof(dis_x));
    memset(&dis_y, 0, sizeof(dis_y));

	
    //计算速度方向
//    float32 square_vxy = 0.0;
    //一些设定值高度/vxy
    float32 set_hight = -0.8;
//    float32 set_vxy = 0.11;
//    float32 set_vxy_max = 0.12;
//    float32 set_vxy_min = 0.08;
    //找到图像档位的阈值
    bool is_data_useful = true;
    bool get_uart_data = false;

    float32 set_N = 0;
    float32 set_E = 0;
//    float32 set_min = 7;

    // 0.42 pid throttle
    float32 throttle_loiter = 0.5;
    float32 throttle_max = throttle_loiter + (float)0.15;//throttle_loiter + (float)0.15
    float32 throttle_min = throttle_loiter - (float)0.15;
    float32 throttle_out = 0.0;
    float32 error_last = 0.0;
    float32 error_now = 0.0;
    float32 error_sum = 0.0;
    float32 kp = 0.17;//0.17   0.17
    float32 ki = 0.08;//0.1
    float32 kd = 0.12;//0.12    0.12
    float32 error_sum_max = (float)1.5 / ki;

    //pid x(channnel 2) 0.0386  //ge -0.058 前后方向的
    float32 x_mid = 0.0;//-0.085
    float32 x_max = x_mid + (float)0.2;//0.2
    float32 x_min = x_mid - (float)0.2;
    float32 error_last_x = 0.0;
    float32 error_now_x = 0.0;
    float32 error_sum_x = 0.0;
    float32 kp_x = 0.0035;//0.0035
    float32 ki_x = 0.001;//0.001
    float32 kd_x = 0.0028;//0.0025
    float32 error_sum_max_x = (float)1.0/ki_x;  //饱和积分最多输出0.8
    float32 x_out = 0.0;
    float32 x_out_last = 0.0;

    //pid y(channnel 1) 0.108   //ge -0.076 左右方向的
    float32 y_mid = 0.0;//0.11
    float32 y_max = y_mid + (float)0.2;//0.2
    float32 y_min = y_mid - (float)0.2;
    float32 error_last_y = 0.0;
    float32 error_now_y = 0.0;
    float32 error_sum_y = 0.0;
    float32 kp_y = 0.0035;
    float32 ki_y = 0.001;
    float32 kd_y = 0.0028;
    float32 error_sum_max_y = (float)1.0/ki_y;
    float32 y_out = 0.0;
    float32 y_out_last = 0.0;
   // float32 sum_square = 0.0;
 



    bool updated_vlp;
    bool updated_ru;
    bool get_vehicle_high = true;


    /*订阅*/
    int read_uart_term4_sub = orb_subscribe(ORB_ID(read_uart_term4));
    int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    /*定义offboard话题结构体*/
    struct read_uart_term4_s _read_uart_term4 = {};
    struct vehicle_local_position_s _vehicle_local_position = {};
//    struct vehicle_local_position_s stay_position = {};
    struct offboard_setpoint_s _offboard_sp = {};

    /*公告主题*/
    /*offboard_pub_pub为handle指针*/
    orb_advert_t offboard_setpoint_pub = orb_advertise(ORB_ID(offboard_setpoint), &_offboard_sp);

    _offboard_sp.ignore_alt_hold = true;

    orb_publish(ORB_ID(offboard_setpoint), offboard_setpoint_pub, &_offboard_sp);

	 
	

    while (!thread_should_exit) {

        _offboard_sp.is_local_frame = true;
	orb_check(rc_channels_sub, &updated_rcch);
        orb_check(vehicle_local_position_sub, &updated_vlp);
        orb_check(read_uart_term4_sub, &updated_ru);
	if (updated_rcch) {
            orb_copy(ORB_ID(rc_channels), rc_channels_sub, &_rc_channels);
        }


        if (updated_vlp) {
            orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &_vehicle_local_position);
            if (get_vehicle_high) {
                set_hight = set_hight + _vehicle_local_position.z;
                get_vehicle_high = false;
            }
            printf("z=%8.4f\n", (double)_vehicle_local_position.z);
	    //printf("sethigh=%8.4f\n", (double)set_hight);
            error_last = error_now;
            error_now = -(set_hight - _vehicle_local_position.z);
            error_sum = error_sum + error_now;
        }
        if (updated_ru) {
            orb_copy(ORB_ID(read_uart_term4), read_uart_term4_sub, &_read_uart_term4);
            if (_read_uart_term4.is_valid) {

                if (get_uart_data) {

              //  sum_square=set_E*set_E+set_N*set_N;

        /*        switch (token){           */

       /*         case 0:                  */
                        set_E=_read_uart_term4.datax;
                        set_N=-_read_uart_term4.datay;

		

			if(_read_uart_term4.flag){

				_read_uart_term4.flag=false;

				error_last_x = 0.0;
				error_now_x =0.0;
				error_sum_x = 0.0;
				x_out_last = 0.0;
				//x_out = 0.0;

				error_last_y = 0.0;
				error_now_y = 0.0;
				error_sum_y = 0.0;
				y_out_last = 0.0;
				y_out = 0.0;	
		
						}







                      //  if(sum_square < (float)100.0){
                               // token =1;
                                //is_data_useful=false;
                                //set_E=0.0;
                                //set_N=0.0;
                               // error_now_x=0.0;
                              //  error_now_y=0.0;

                                             //   }  
     /*                   break;

                case 1:
                        is_data_useful=true;
                        set_E=_read_uart_term4.datax;
                        set_N=_read_uart_term4.datay;
                        if(sum_square < (float)500.0){
                                token =2;
                                is_data_useful=false;
                                set_E=0.0;
                                set_N=0.0;
                                error_now_x=0.0;
                                error_now_y=0.0;
                                                }
                        break;

                case 2:
                        is_data_useful=true;
                        set_E=_read_uart_term4.datax;
                        set_N=_read_uart_term4.datay;
                        if(sum_square < (float)500.0){
                                token =3;
                                is_data_useful=false;
                                set_E=0.0;
                                set_N=0.0;
                                error_now_x=0.0;
                                error_now_y=0.0;
                                                }
                        break;

                case 3:
                        is_data_useful=true;
                        set_E=_read_uart_term4.datax;
                        set_N=_read_uart_term4.datay;
                        if(sum_square < (float)500.0){
                                token =0;
                                is_data_useful=false;
                                set_E=0.0;
                                set_N=0.0;
                                error_now_x=0.0;
                                error_now_y=0.0;
                                                }
                        break;

                default:
                        break;

                                                }              */


                  //  set_E = _read_uart_term4.datax;//- (float)160
                  //  set_N = _read_uart_term4.datay;//(float)120 - _
                    printf("set_N=%8.4f\t",(double)set_N);
                    printf("set_E=%8.4f\n",(double)set_E);
                    get_uart_data = false;
						}



                is_data_useful = true;
                count_data = 0;
            } else {
                count_data ++;
            }

            if (count_data > set_count_mean) {
                is_data_useful = false;
                printf("date is useless\n");
            }
        }
        printf("read    x=%6d\t",_read_uart_term4.datax);
        printf("y=%6d\n",_read_uart_term4.datay);
        if (_read_uart_term4.is_valid)
            printf("data valid\n");




       /*                                                                      **    油门大小进行PID    **                                                                */
        if (error_sum > error_sum_max)
            error_sum = error_sum_max;
        if (error_sum < (float)-1.0 * error_sum_max)
            error_sum = (float)-1.0 * error_sum_max;

        throttle_out = throttle_loiter
                + kp * error_now
                + ki * (float)0.1 * error_sum
                + kd * (float)10 * (error_now - error_last);
        if (throttle_out > throttle_max)
            throttle_out = throttle_max;
        if (throttle_out < throttle_min)
            throttle_out = throttle_min;
        _offboard_sp.z = throttle_out;                    //油门输出量

        
        count_1s++;
    if (count_1s == 1)
        get_uart_data = true;
    if (count_1s == 1) {
        count_1s = 0;
            if (is_data_useful) {
        /*        //x pid                                                      **     前后距离控制PID   **                                                                   */
                error_last_x = error_now_x;
                error_now_x = set_N;

//                if (error_last_x * error_now_x < 0)
//                    error_now_x = (error_now_x + error_last_x) / (float)2.0;

                error_sum_x = error_sum_x + error_now_x;
                //x 饱和积分
                if (error_sum_x > error_sum_max_x)
                    error_sum_x = error_sum_max_x;
                if (error_sum_x < (float)-1.0 * error_sum_max_x)
                    error_sum_x = (float)-1.0 * error_sum_max_x;
                //x pid value
                x_out_last = x_out;
                x_out = kp_x * error_now_x
                        + ki_x * (float)0.1 * error_sum_x
                        + kd_x * (float)10 * (error_now_x - error_last_x);
//                if (x_out * x_out_last < 0)
                    x_out = (x_out + x_out_last) / (float)2.0 + x_mid;
                //x pid out饱和输出
                if (x_out > x_max)
                    x_out = x_max;
                if (x_out < x_min)
                    x_out = x_min;
                _offboard_sp.x = x_out;   //前后距离输出量





            /*    //y pid                                                     **       左右距离控制PID    **                                                                   */
                error_last_y = error_now_y;
                error_now_y = set_E;

//                if (error_last_y * error_now_y < 0)
//                    error_now_y = (error_now_y + error_last_y) / (float)2.0;

                error_sum_y = error_sum_y + error_now_y;
                //y 饱和积分
                if (error_sum_y > error_sum_max_y)
                    error_sum_y = error_sum_max_y;
                if (error_sum_y < (float)-1.0 * error_sum_max_y)
                    error_sum_y = (float)-1.0 * error_sum_max_y;
                //y pid value
                y_out_last = y_out;
                y_out = kp_y * error_now_y
                        + ki_y * (float)0.1 * error_sum_y
                        + kd_y * (float)10 * (error_now_y - error_last_y);
//                if (y_out * y_out_last < 0)
                    y_out = (y_out + y_out_last) / (float)2.0 + y_mid;
                //y pid out饱和输出
                if (y_out > y_max)
                    y_out = y_max;
                if (y_out < y_min)
                    y_out = y_min;
                _offboard_sp.y = y_out; //左右方向输出量
//                square_vxy = (float)sqrt(set_E * set_E + set_N * set_N);
//                if (square_vxy > (float)120.0) {
//                    set_vxy = set_vxy_max;
//                } else if (square_vxy < (float)20.0) {
//                    set_vxy = set_vxy_min;
//                } else {
//                    set_vxy = set_vxy_min + (square_vxy - (float)20) / (float)100 * (set_vxy_max - set_vxy_min);
//                }
//                printf("set_vxy = %8.4f\n", (double)set_vxy);
//                _offboard_sp.x = set_N / square_vxy * set_vxy + x_mid;
//                _offboard_sp.y = set_E / square_vxy * set_vxy + y_mid;
//                printf("suqare = %8.4f\n", (double)square_vxy);
//                if (set_N < set_min && set_N > (float)-1.0 * set_min){
//                    _offboard_sp.x = x_mid;
//                }
//                if (set_E < set_min && set_E > (float)-1.0 * set_min)
//                    _offboard_sp.y = y_mid;

            } else {
                _offboard_sp.x = x_mid;
                _offboard_sp.y = y_mid;
                printf("2");
            }
            printf("set_x = %8.4f\n", (double)_offboard_sp.x);
            printf("set_y = %8.4f\n", (double)_offboard_sp.y);
	   // printf("set_z = %8.4f\n", (double)_offboard_sp.z);
	    printf("thro = %8.4f\n", (double)_rc_channels.channels[2]);
	    
	  // printf("roll=%8.4f\n",(double)_rc_channels.channels[0]);
		//printf("pitch=%8.4f\n",(double)_rc_channels.channels[1]);
		//printf("error_sum=%8.4f\n",(double)error_sum);
        }
        orb_publish(ORB_ID(offboard_setpoint), offboard_setpoint_pub, &_offboard_sp);
        usleep(100000);
    }

    warnx("[daemon] exiting.\n");

    thread_running = false;

    return 0;
}
