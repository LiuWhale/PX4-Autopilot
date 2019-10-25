/*
 * read_uart_term4_sensor.c
 * 
 * read sensor through uart
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <px4_tasks.h>
#include <px4_config.h>
#include <nuttx/sched.h>

#include <uORB/uORB.h>
#include <uORB/topics/read_uart_term4.h>



__EXPORT int read_uart_term4_main(int argc, char *argv[]);


static bool thread_should_exit = false; /*Ddemon exit flag*/
static bool thread_running = false;  /*Daemon status flag*/
static int read_uart_term4_task;

/**
 * Main loop
 */
int read_uart_term4_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);
static void usage(const char *reason);

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: read_uart_term4_sensor {start|stop|status} [param]\n\n");
    exit(1);
}

int read_uart_term4_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("[Read_uart]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("[Read_uart]already running\n");
            return 0;
        }

        thread_should_exit = false;
        read_uart_term4_task = px4_task_spawn_cmd("read_uart",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 5,
                         2000,
                         read_uart_term4_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[Read_uart]running");
            return 0;

        } else {
            warnx("[Read_uart]stopped");
            return 1;
        }

        return 0;
    }

    usage("unrecognized command");
    return 1;
}

int read_uart_term4_thread_main(int argc, char *argv[])
{

    if (argc < 2) {
        errx(1, "[Read_uart]need a serial port name as argument");
        usage("eg:");
    }

    const char *uart_name = argv[1];

    warnx("[Read_uart]opening port %s", uart_name);
    char data = '0';
    char buffer[4] = "";
    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    int uart_read = uart_init(uart_name);
    if(false == uart_read)return -1;
    if(false == set_uart_baudrate(uart_read,9600)){
        printf("[Read_uart]set_uart_baudrate is failed\n");
        return -1;
    }
    printf("[Read_uart]uart init is successful\n");

    thread_running = true;

    /*初始化数据结构体 */
    struct read_uart_term4_s _uart_data;
    memset(&_uart_data, 0, sizeof(_uart_data));
    _uart_data.is_valid = false;


    _uart_data.flag = false;


    /* 公告主题 */
    orb_advert_t read_uart_term4_pub = orb_advertise(ORB_ID(read_uart_term4), &_uart_data);
    orb_publish(ORB_ID(read_uart_term4), read_uart_term4_pub, &_uart_data);

    while(!thread_should_exit) {
        read(uart_read,&data,1);


        if (data == 'X'){
            for(int i = 0;i <4;++i){
                read(uart_read,&data,1);
                buffer[i] = data;
                data = '0';
            }
            strncpy(_uart_data.datastr,buffer,4);// 复制字符串Buffer中前４个数字到Datastr中
	   // _uart_data.datax = atoi(_uart_data.datastr);//将字符串转换成整形数据     ---室内跟踪
            _uart_data.datax =(float)0.1*atoi(_uart_data.datastr);//将字符串转换成整形数据    ---室内避障雷达单位为mm
            //printf("[Read_uart]sensor.data=%d\n",_uart_data.datax);
            //orb_publish(ORB_ID(read_uart_term4), read_uart_pub, &_uart_data);
        }
	
	if (data == 'x'){
            for(int i = 0;i <4;++i){
                read(uart_read,&data,1);
                buffer[i] = data;
                data = '0';
            }
            strncpy(_uart_data.datastr,buffer,4);// 复制字符串Buffer中前４个数字到Datastr中
            _uart_data.datax =-(float)0.1*atoi(_uart_data.datastr);//将字符串转换成整形数据
            //printf("[Read_uart]sensor.data=%d\n",_uart_data.datax);
            //orb_publish(ORB_ID(read_uart_term4), read_uart_pub, &_uart_data);
        
	}                          

        if (data == 'Y'){
            for(int i = 0;i <4;++i){
                read(uart_read,&data,1);
                buffer[i] = data;
                data = '0';
            }
            strncpy(_uart_data.datastr,buffer,4);// 复制字符串Buffer中前４个数字到Datastr中
	   // _uart_data.datax = atoi(_uart_data.datastr);//将字符串转换成整形数据            ---室内跟踪
            _uart_data.datay =(float)0.1*atoi(_uart_data.datastr);//将字符串转换成整形数据            ---室内避障雷达单位为mm
			}

	if (data == 'y'){
            for(int i = 0;i <4;++i){
                read(uart_read,&data,1);
                buffer[i] = data;
                data = '0';
            }
            strncpy(_uart_data.datastr,buffer,4);// 复制字符串Buffer中前４个数字到Datastr
            _uart_data.datay = -(float)0.1*atoi(_uart_data.datastr);//将字符串转换成整形数据
	
			}              

	if(data == 'H'){
		_uart_data.flag = true;
			}


            _uart_data.is_valid = true;
            if (_uart_data.datay > 999)
                _uart_data.is_valid = false;
            _uart_data.timestamp = hrt_absolute_time();
            //printf("[Read_uart]sensor.data=%d\n",_uart_data.datay);
            orb_publish(ORB_ID(read_uart_term4), read_uart_term4_pub, &_uart_data);
        
    }

    warnx("[Read_uart]exiting");
    thread_running = false;
    close(uart_read);

    fflush(stdout);
    return 0;
}
