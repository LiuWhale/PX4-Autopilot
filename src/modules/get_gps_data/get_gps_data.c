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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <mathlib/mathlib.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/delivery_signal.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/offboard_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/read_uart.h>
static bool thread_should_exit = false;     /**< daemon exit flag */
static bool thread_running = false;     /**< daemon status flag */
static int get_gps_data_task;             /**< Handle of daemon task / thread */
static int gps_sequence = 0;
int n = 4;				//M的下初始点数量为：3,4等分
int mw = 0;				//0:为M，1：为W
int flyToRed = 0;
int set = 1;
//static double CONSTANTS_RADIUS_OF_EARTH = 6371000.0;
//static double pi = 3.14159265358979323846;

struct ref_point {
    double lat_rad;
    double lon_rad;
    double sin_lat;
    double cos_lat;
};
struct point{//点的集合
    float32 x;
    float32 y;
};

struct point* r_point;
struct point* set_point;//根据范围结算目标点
struct point check_r_p;
__EXPORT int get_gps_data_main(int argc, char *argv[]);

int get_data_thread_main(int argc, char *argv[]);

//static int wgs_ned(const struct ref_point *ref, double lat, double lon, float *x, float *y);

static void usage(const char *reason);

void check_red_point(float32 x, float32 y);

//int wgs_ned(const struct ref_point *ref, double lat, double lon, float *x, float *y)
//{
//    const double lat_rad = lat * (pi / 180);
//    const double lon_rad = lon * (pi / 180);

//    const double sin_lat = sin(lat_rad);
//    const double cos_lat = cos(lat_rad);

//    const double cos_d_lon = cos(lon_rad - ref->lon_rad);

//    int val = ref->sin_lat * sin_lat + ref->cos_lat * cos_lat * cos_d_lon;
//    const double arg = (val < -1.0) ? -1.0 : ((val > 1.0) ? 1.0 : val);
//    const double c = acos(arg);

//    double k = 1.0;

//    if (fabs(c) > 0) {
//        k = (c / sin(c));
//    }

//    *x = (float32)(k * (ref->cos_lat * sin_lat - ref->sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);
//    *y = (float32)(k * cos_lat * sin(lon_rad - ref->lon_rad) * CONSTANTS_RADIUS_OF_EARTH);

//    return 0;
//}

static void usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("usage: get_gps_data {start|stop|status|numb} [-p <additional params>]\n\n");
}


int get_gps_data_main(int argc, char *argv[])
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
        get_gps_data_task = px4_task_spawn_cmd("get_gps_data",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2000,
                         get_data_thread_main,
                         (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {//模块运行状态
        if (thread_running) {
            warnx("\trunning\n");

        } else {
            warnx("\tnot started\n");
        }

        return 0;
    }

    if (!strcmp(argv[1], "1")) {//定点命令的置位
        gps_sequence = 1;
        return 0;
    }

    if (!strcmp(argv[1], "2")) {
        gps_sequence = 2;
        return 0;
    }

    if (!strcmp(argv[1], "3")) {
        gps_sequence = 3;
        return 0;
    }
    if (!strcmp(argv[1], "4")) {
        gps_sequence = 4;
        return 0;
    }
    if (!strcmp(argv[1], "5")) {
        gps_sequence = 5;
        return 0;
    }

    usage("unrecognized command");
    return 1;

}

int get_data_thread_main(int argc, char *argv[])
{

    warnx("[daemon] starting\n");

    thread_running = true;

    bool get_a = false;
    bool get_b = false;
    bool get_c = false;
    bool get_d = false;
    bool get_e = false;

    bool close_a = false;
    bool close_point =false;
    bool close_red_point = false;
    bool is_vxy_zero = false;

    int count = 0;
    int count_time = 0;
    int count_drop = 0;
    int token = 1;
    int inside_token = 0;
    int point_num_now = 0;
    int three_point = 0;
    int a,b,c,d;

    float32 sum_x = 0.0;
    float32 sum_y = 0.0;
    float32 sum_z = 0.0;

    double sum_square = 0.0;


    int rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));//订阅topic
    int vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    int home_position_sub = orb_subscribe(ORB_ID(home_position));
    int read_uart_sub = orb_subscribe(ORB_ID(read_uart));
    struct rc_channels_s _rc_channels = {};//定义topic的结构体
    struct vehicle_global_position_s _vehicle_global_position = {};
    struct delivery_signal_s _delivery_signal = {};
    struct vehicle_local_position_s local_now = {};
    struct home_position_s _home_position = {};
    struct offboard_setpoint_s _offboard_sp = {};
    struct read_uart_s _read_uart = {};
//    struct vehicle_global_position_s global_a = {};
//    struct vehicle_global_position_s global_b = {};
//    struct vehicle_global_position_s global_c = {};

    struct vehicle_local_position_s local_a = {};
    struct vehicle_local_position_s local_b = {};
    struct vehicle_local_position_s local_c = {};
    struct vehicle_local_position_s local_d = {};
    struct vehicle_local_position_s local_e = {};

//    float32 local_b.x = 0.0;
//    float32 local_b.y = 0.0;
//    float32 local_c.x = 0.0;
//    float32 local_c.y = 0.0;
//    float32 local_now.x = 0.0;
//    float32 local_now.y = 0.0;
//    float32 z_home = 0.0;
    float32 vx = 0.0;
    float32 vy = 0.0;
    float32 v_x = 0.0;
    float32 v_y = 0.0;
    float32 vxy_max = 0.0;
    float32 set_high = -7.0;
//    float32 set_threshold_up = -6.5;
//    float32 set_threshold_down = -2.0;

//    struct ref_point ref_a = {};
    _delivery_signal.is_point_a = false;
    _delivery_signal.is_point_b = false;
    _delivery_signal.is_point_c = false;
    _offboard_sp.ignore_alt_hold = true;
    _offboard_sp.ignore_attitude = true;
    _offboard_sp.ignore_position = true;
    _offboard_sp.ignore_velocity = true;
    _offboard_sp.is_idle_sp = false;
    _offboard_sp.is_land_sp = false;
    _offboard_sp.is_local_frame = true;
    _offboard_sp.is_loiter_sp = false;
    _offboard_sp.is_takeoff_sp = false;
    _offboard_sp.timestamp = hrt_absolute_time();

    orb_advert_t delivery_signal_pub = orb_advertise(ORB_ID(delivery_signal), &_delivery_signal);//主题公告:发布主题之前是必须的,否则订阅者虽然能订阅，但是得不到数据
    orb_advert_t offboard_setpoint_pub = orb_advertise(ORB_ID(offboard_setpoint), &_offboard_sp);

    orb_publish(ORB_ID(offboard_setpoint), offboard_setpoint_pub, &_offboard_sp);//发布主题
    orb_publish(ORB_ID(delivery_signal), delivery_signal_pub, &_delivery_signal);

    while (!thread_should_exit) {

        bool updated_rcch;
        bool updated_vp_global;
        bool updated_vp_local;
        bool updated_home;
	bool updated_ru;

        orb_check(rc_channels_sub, &updated_rcch);//检测订阅是否成功
        orb_check(vehicle_global_position_sub, &updated_vp_global);
        orb_check(vehicle_local_position_sub, &updated_vp_local);
        orb_check(home_position_sub, &updated_home);
	orb_check(read_uart_sub, &updated_ru);
        if (updated_rcch) {
            orb_copy(ORB_ID(rc_channels), rc_channels_sub, &_rc_channels);
            //printf("channel 1 = %8.4f\n", (double)_rc_channels.channels[0]);
            //printf("channel 2 = %8.4f\n", (double)_rc_channels.channels[1]);
            //printf("channel 3 = %8.4f\n", (double)_rc_channels.channels[2]);
            //printf("channel 4 = %8.4f\n", (double)_rc_channels.channels[3]);
            //printf("channel 5 = %8.4f\n", (double)_rc_channels.channels[4]);
            //printf("channel 6 = %8.4f\n", (double)_rc_channels.channels[7]);
        }

        if (updated_vp_local) {
            orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &local_now);//从主题中获得更新数据
            vx = local_now.vx;
            vy = local_now.vy;
            vxy_max = local_now.vxy_max;

	    /*定点示意图*/
	    /****Point-C--------Point-D****/
	    /******|--------------|******/
	    /******|--------------|******/
	    /******|--------------|******/
	    /******|--------------|******/
	    /****Point-B--------Point-E****/
	    /***///***/
	    /*Point-A*/

            if (gps_sequence == 1 && !get_a) {
                sum_x = sum_x + local_now.x;
                sum_y = sum_y + local_now.y;
                sum_z = sum_z + local_now.z;
                count++;
                if (count == 10) {
                    local_a.x = sum_x/count;
                    local_a.y = sum_y/count;
                    local_a.z = sum_z/count;
                    get_a = true;
                    count = 0;
                    sum_x = 0.0;
                    sum_y = 0.0;
                    sum_z = 0.0;
                }
                warnx("[gaemon] sequence == 1.\n");
                printf("  a:  x=%6.3f\t", (double)local_a.x);
                printf("y=%6.3f\t", (double)local_a.y);
                printf("z=%6.3f\n", (double)local_a.z);

            }

            if (gps_sequence == 2 && !get_b) {
                sum_x = sum_x + local_now.x;
                sum_y = sum_y + local_now.y;
                sum_z = sum_z + local_now.z;
                count++;
                if (count == 10) {
                    local_b.x = sum_x/count;
                    local_b.y = sum_y/count;
                    local_b.z = sum_z/count;
                    get_b = true;
                    count = 0;
                    sum_x = 0.0;
                    sum_y = 0.0;
                    sum_z = 0.0;
                }
                warnx("[gaemon] sequence == 2.\n");
                printf("vxy_max = %6.3f\n",(double)vxy_max);
            }

            if (gps_sequence == 3 && !get_c) {
                sum_x = sum_x + local_now.x;
                sum_y = sum_y + local_now.y;
                sum_z = sum_z + local_now.z;
                count++;
                if (count == 10) {
                    local_c.x = sum_x/count;
                    local_c.y = sum_y/count;
                    local_c.z = sum_z/count;
                    get_c = true;
                    count = 0;
                    sum_x = 0.0;
                    sum_y = 0.0;
                    sum_z = 0.0;
                }
                warnx("[gaemon] sequence == 3.\n");
            }
            if (gps_sequence == 4 && !get_d) {
                sum_x = sum_x + local_now.x;
                sum_y = sum_y + local_now.y;
                sum_z = sum_z + local_now.z;
                count++;
                if (count == 10) {
                    local_d.x = sum_x/count;
                    local_d.y = sum_y/count;
                    local_d.z = sum_z/count;
                    get_d = true;
                    count = 0;
                    sum_x = 0.0;
                    sum_y = 0.0;
                    sum_z = 0.0;
                }
                warnx("[gaemon] sequence == 4.\n");
            }
            if (gps_sequence == 5 && !get_e) {
                sum_x = sum_x + local_now.x;
                sum_y = sum_y + local_now.y;
                sum_z = sum_z + local_now.z;
                count++;
                if (count == 10) {
                    local_e.x = sum_x/count;
                    local_e.y = sum_y/count;
                    local_e.z = sum_z/count;
                    get_e = true;
                    count = 0;
                    sum_x = 0.0;
                    sum_y = 0.0;
                    sum_z = 0.0;
                }
                warnx("[gaemon] sequence == 5.\n");
            }
        }

        if (updated_home) {
            orb_copy(ORB_ID(home_position), home_position_sub, &_home_position);
        }

        if (updated_vp_global) {
            orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub, &_vehicle_global_position);
        }

        if (get_a && get_b && get_c && get_d && get_e) {//5个点都定好后
	    orb_copy(ORB_ID(read_uart), read_uart_sub, &_read_uart);
            gps_sequence = 0;
            count_time++;

	    if(_read_uart.is_valid){
    	      v_x = _read_uart.datax - (float)160;//原点纠正
              v_y = (float)120 - _read_uart.datay;
	      sum_square = (double)( v_x * v_x + v_y * v_y);
              if (sum_square < 1.0) {//红点的范围判断
                  close_red_point = true;
                  printf("close_red_point[%d]\t",point_num_now);
              }
	      v_x += local_now.x;//向量算目标点
	      v_y += local_now.y;
	      if(three_point == 0){//第一个红点不需要检查
		      r_point[three_point].x = v_x;
		      r_point[three_point].y = v_y;
	      }else{
		      check_red_point(v_x,v_y);
		      r_point[three_point].x = check_r_p.x;
		      r_point[three_point].y = check_r_p.y;
	      }
	    }
	    if(!mw && set){
	      /*M型目标点*/
	      for(int i = 0;i < n;i++){
		 set_point[(int)2i].x = local_b.x * (n - i - 1)/n +  local_e.x * ( i + 1 )/n;
		 set_point[(int)2i].y = local_b.y * (n - i - 1)/n +  local_e.y * ( i + 1 )/n;
	      }
    	      for(int i = 0;i < n-1;i++){
		 set_point[(int)2i+1].x = local_c.x * ( n - i )/(n + 1) +  local_d.x * ( i + 1 )/(n + 1);
		 set_point[(int)2i+1].y = local_c.y * ( n - i )/(n + 1) +  local_d.y * ( i + 1 )/(n + 1);
	      }
	      set = 0;
	    }else if(mw && set){
	      /*W型目标点*/
	      for(int i = 0;i < n;i++){
		 set_point[2*i].x = local_d.x * (n - i - 1)/n +  local_c.x * ( i + 1 )/n;
		 set_point[2*i].y = local_d.y * (n - i - 1)/n +  local_c.y * ( i + 1 )/n;
	      }
    	      for(int i = 0;i < n-1;i++){
		 set_point[2*i+1].x = local_e.x * ( n - i )/(n + 1) +  local_b.x * ( i + 1 )/(n + 1);
		 set_point[2*i+1].y = local_e.y * ( n - i )/(n + 1) +  local_b.y * ( i + 1 )/(n + 1);
	      }
	      set = 0;
	    }
 
            /* 显示abc点的NED坐标*/

            if (count_time == 5) {
                printf("  a:  x=%6.3f\t", (double)local_a.x);
                printf("y=%6.3f\t", (double)local_a.y);
                printf("z=%6.3f\n", (double)local_a.z);

                printf("  b:  x=%6.3f\t", (double)local_b.x);
                printf("y=%6.3f\t", (double)local_b.y);
                printf("z=%6.3f\n", (double)local_b.z);

                printf("  c:  x=%6.3f\t", (double)local_c.x);
                printf("y=%6.3f\t", (double)local_c.y);
                printf("z=%6.3f\n", (double)local_c.z);

                printf("  d:  x=%6.3f\t", (double)local_d.x);
                printf("y=%6.3f\t", (double)local_d.y);
                printf("z=%6.3f\n", (double)local_d.z);

                printf("  e:  x=%6.3f\t", (double)local_e.x);
                printf("y=%6.3f\t", (double)local_e.y);
                printf("z=%6.3f\n", (double)local_e.z);

                printf("now:  x=%6.3f\t", (double)local_now.x);
                printf("y=%6.3f\t", (double)local_now.y);
                printf("z=%6.3f\n", (double)local_now.z);

                count_time = 0;
            }
            printf("count_time= %6d\n",count_time);

            sum_square = (double)( (local_a.x - local_now.x) * (local_a.x - local_now.x) +
                                   (local_a.y - local_now.y) * (local_a.y - local_now.y) );
            if (sum_square < 1.0) {//A点的范围判断
                close_a = true;
                printf("close_a\t");
            }

	    sum_square = (double)( (set_point[point_num_now].x - local_now.x) * (set_point[point_num_now].x - local_now.x) +
                                   (set_point[point_num_now].y - local_now.y) * (set_point[point_num_now].y - local_now.y) );
            if (sum_square < 1.0) {//目标点的范围判断
                close_point = true;
                printf("close_point[%d]\t",point_num_now);
            }



            sum_square = (double)(100 * vx * vx + 100 * vy * vy);
            if (sum_square < 0.5) {//xy方向速度范围内为0
                is_vxy_zero = true;
                printf("vxy_zero\n");
            }
	    if(point_num_now>0){
	      a = (local_b.x - local_a.x)*(set_point[point_num_now].y - local_a.y) - (local_b.y - local_a.y)*(set_point[point_num_now].x - local_a.x);
              b = (local_c.x - local_b.x)*(set_point[point_num_now].y - local_b.y) - (local_c.y - local_b.y)*(set_point[point_num_now].x - local_b.x);
              c = (local_d.x - local_c.x)*(set_point[point_num_now].y - local_c.y) - (local_d.y - local_c.y)*(set_point[point_num_now].x - local_c.x);
              d = (local_a.x - local_d.x)*(set_point[point_num_now].y - local_d.y) - (local_a.y - local_d.y)*(set_point[point_num_now].x - local_d.x);
	      if((a >0 && b>0 && c>0 && d>0)||(a<0 && b<0 && c<0 && d<0)){
		  inside_token = 1;
                  printf("Inside now\n");
	      }else{
		token = 3;
	      }
	    }
            switch (token) {
            case 1:     //A点起飞
                _offboard_sp.ignore_alt_hold = true;//忽略高度保持
                _offboard_sp.ignore_attitude = true;//忽略高度
                _offboard_sp.ignore_position = false;//不忽略位置
                _offboard_sp.ignore_velocity = true;//忽略速度
                _offboard_sp.is_idle_sp = false;
                _offboard_sp.is_land_sp = false;//不降落
                _offboard_sp.is_local_frame = true;
                _offboard_sp.is_loiter_sp = false;//不悬停
                _offboard_sp.is_takeoff_sp = true;//起飞
                _offboard_sp.x = local_a.x;//目标点
                _offboard_sp.y = local_a.y;
                _offboard_sp.z = local_a.z + set_high;
                _offboard_sp.timestamp = hrt_absolute_time();
                if (local_now.z < (local_a.z + set_high + (float)0.5)) {//判断是否到达目标位置一定范围
                    token = 2;
                }
                printf("A take off\n");
                break;

            case 2:     //飞向B点
                _offboard_sp.ignore_alt_hold = false;
                _offboard_sp.ignore_position = false;
                _offboard_sp.is_land_sp = false;
                _offboard_sp.is_takeoff_sp = false;
		_offboard_sp.is_loiter_sp = false;
                _offboard_sp.x = set_point[0].x;
                _offboard_sp.y = set_point[0].y;
                _offboard_sp.z = local_a.z + set_high;
                if ( inside_token && is_vxy_zero) {
                    token = 4;
                }
                printf("TO B inside\n");
                break;

            case 3:     //返回原点
                _offboard_sp.ignore_alt_hold = false;
                _offboard_sp.ignore_position = false;
                _offboard_sp.is_land_sp = false;
                _offboard_sp.is_takeoff_sp = false;
		_offboard_sp.is_loiter_sp = false;
                _offboard_sp.x = local_a.x;
                _offboard_sp.y = local_a.y;
                _offboard_sp.z = local_a.z + set_high;
                if ( close_a && is_vxy_zero) {
                    token =5;
                }
		break;
            case 4:     //飞向目标点
		if(inside_token && (three_point < 3)){
   		  _offboard_sp.ignore_alt_hold = false;
                  _offboard_sp.ignore_position = false;
                  _offboard_sp.is_land_sp = false;
                  _offboard_sp.is_takeoff_sp = false;
		  _offboard_sp.is_loiter_sp = false;
		  _offboard_sp.x = set_point[point_num_now].x;
		  _offboard_sp.y = set_point[point_num_now].y;
                  _offboard_sp.z = local_a.z + set_high;
		  if(close_point && is_vxy_zero){
			if(point_num_now + 1 != 2*n-1){
				point_num_now++;
			}else{
				n += 3;
				mw = (mw + 1) % 2;//  M/W切换
				set = 1;
				point_num_now = 0;
			}
			token = 4;
		  }
		  if(_read_uart.is_valid && flyToRed){
			token = 7;
		  }
		  
		 }else if(three_point == 3){
		  	token = 3;
		 }

                break;

            case 5:     //A点降落
                _offboard_sp.ignore_alt_hold = true;
                _offboard_sp.ignore_position = false;
                _offboard_sp.is_land_sp = true;
                _offboard_sp.is_takeoff_sp = false;
                _offboard_sp.x = local_a.x;
                _offboard_sp.y = local_a.y;
                _offboard_sp.z = local_a.z + (float32)0.2;
                printf("Land A\n");
                break;
	    case 6:	//red_point悬停
		_offboard_sp.ignore_alt_hold = true;
                _offboard_sp.ignore_position = false;
                _offboard_sp.is_land_sp = false;
                _offboard_sp.is_takeoff_sp = false;
		_offboard_sp.is_loiter_sp = true;
                _offboard_sp.x = r_point[three_point].x;
                _offboard_sp.y = r_point[three_point].y;
                _offboard_sp.z = local_a.z + set_high;
		count_drop++;
		if(count_drop > 10){
		   if(three_point==0){
		     _delivery_signal.is_point_a = true;
		    }else if(three_point==1){
		     _delivery_signal.is_point_b = true;
		    }else if(three_point==2){
		     _delivery_signal.is_point_c = true;
		    }
		}
                if (count_drop > 50) {//50s
                    count_drop = 0;
                    token = 4;
		    flyToRed = 0;
		    three_point++;
                }
	    case 7://向红点飞去
   		_offboard_sp.ignore_alt_hold = false;
                _offboard_sp.ignore_position = false;
                _offboard_sp.is_land_sp = false;
                _offboard_sp.is_takeoff_sp = false;
		_offboard_sp.is_loiter_sp = false;
                _offboard_sp.x = r_point[three_point].x;
                _offboard_sp.y = r_point[three_point].y;
                _offboard_sp.z = local_a.z + set_high;
		if(close_red_point && is_vxy_zero){
		   token = 6;
		}
            default:
                break;
            }
            close_a = false;
	    close_point = false;
	    close_red_point = false;
            inside_token = 0;
            is_vxy_zero = false;
	    flyToRed = 0;

        }

        _offboard_sp.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(delivery_signal), delivery_signal_pub, &_delivery_signal);
        orb_publish(ORB_ID(offboard_setpoint), offboard_setpoint_pub, &_offboard_sp);//发布到offboard_setpoint主题里,在offboard_pub中执行
        usleep(100000);
    }

    warnx("[daemon] exiting.\n");

    thread_running = false;

    return 0;
}


void check_red_point(float32 x,float32 y){
	int length = sizeof( r_point ) / sizeof( *r_point );
	for(int i = 0;i<length;i++){
	  if(abs(r_point[i].x - x) > 0.5 && abs(r_point[i].y - y) > 0.5){//
		check_r_p.x = x;
		check_r_p.y = y;
		flyToRed = 1;
	  }
	}
	
}
