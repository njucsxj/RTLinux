#include "rt_com.h"
#include /* printk level */
#include /* kernel version etc. */

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Wind-Son");
MODULE_DESCRIPTION("Pulse-Control system");

typedef unsigned short __u16;

/* 实时应用－IO输出控制部分 */

void io_bit_on(__u16 port, unsigned int pos, __u16 *status)
{
        __asm__ __volatile__(
                "movl %1,%%edx\n\t"
                "movl %0,%%ecx\n\t"
                "btsl %2,(%%ecx)\n\t"
                "mov (%%ecx),%%al\n\t"
        "out %%al,(%%dx)\n\t"
        "out %%al,$0x80\n\t"
                :
                :"m"(status), "rm"(port), "Ir"(pos)
        );
}

void io_bit_off(__u16 port, unsigned int pos, __u16 *status)
{
        __asm__ __volatile__(        
                "movl %1,%%edx\n\t"
                "movl %0,%%ecx\n\t"
                "btrl %2,(%%ecx)\n\t"
                "mov (%%ecx),%%al\n\t"
        "out %%al,(%%dx)\n\t"
        "out %%al,$0x80\n\t"
                :
                :"m"(status), "rm"(port), "Ir"(pos)
        );
}

/* 
* 实时应用－以实时线程模拟定时器产生脉冲输出部分 
*/

#define dbg_print rtl_printf

#define MIN_TIME              5000

static void get_time_interval(void)
{
}

void* pulse_generate_thread(void *arg) 
{
        static __u16 io_status = 0;
        struct sched_param p;
        hrtime_t current_time;
        REAL_TIME_GET_ENABLE;
        int intrrupt_sched_period = 180000;
        p.sched_priority = 1; /* 设置实时线程的优先级 */

        struct timespec resolution;
        /* RT时钟设置 */
        rtl_setclockmode(CLOCK_REALTIME, RTL_CLOCK_MODE_PERIODIC, 
                        intrrupt_sched_period);
        clock_getres(rtl_getschedclock(), &resolution);
        intrrupt_sched_period = timespec_to_ns(&resolution);
        
        /* 设置RT-调度参数 */
        pthread_make_periodic_np(pthread_self(), clock_gethrtime(rtl_getschedclock()), 
                intrrupt_sched_period);
        pthread_setschedparam (pthread_self(), SCHED_FIFO, &p);

        for (;;) {
                dbg_print("debug entry\n");
                while (!ready) /* 空闲等待 */
                    pthread_wait_np(); /* 空闲状态一定调用该函数让出对cpu的控制，否则死机！*/
                dbg_print("debug exit\n");

                if (!init_rt_clock) {
                        /* 初始化或重新设置RT时钟 */
                        init_rt_clock = 1;
                        pthread_wait_np();
                        current_time = clock_gethrtime(CLOCK_REALTIME);
                } else {
                    if (intrrupt_sched_period < MIN_TIME)
                                intrrupt_sched_period = MIN_TIME;
                    current_time += intrrupt_sched_period;
                        /*
                         * 这一步很关键！clock_nanosleep()使本线程直接睡眠在定时器上，
                         * 睡眠时间current_time(ns)，然后被唤醒。实验结果，这种方式
                         * 不但保证实时性且基本上接近于硬件定时器精度了。
                         * 而通过pthread_wait_np()实时调度虽然在实时性上也是有保证的，
                         * 但在精度上是有限的。
                         */
                    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, hrt2ts(current_time), NULL);
                }
        
                /* 脉冲输出控制…… */
                io_bit_on(IO_PORT_OUT, XPULSE, &io_status);

                /* 
                 * 获取下一个脉冲间隔,从而产生精确的脉冲控制序列
                 * Note! 该实时线程用于模拟定时器输出,因此不能有太复杂的计算。
                 * 一般情况下将脉冲间隔时间的计算以及其他控制部分放到另外一个独立的线程。
                 * 在本系统中time_interval_calc_thread即用于该目的。
                 */
            intrrupt_sched_period = get_time_interval();
        }

        return 0;
}

/*
* 实时应用－动态内存申请部分。
* 直接在RT-thread中申请内存实际结果证明很不稳定，时不时的crash，
* 大概是因为alloc过程被高优先级RT-thread强制中断产生异常。
* 因此需要动态申请内存时开辟独立的内核线程专门负责内存申请。
*/

static void init_for_rt_mm(void)
{
}

static void rt_alloc_mm(void)
{
        thread_wait_np();
        buf = kmalloc(size, GFP_ATOMIC);
}

static int kmalloc_thread(void * kthread_arg)
{
        unsigned long timeout = HZ;
        init_for_rt_mm();
        
        for (;;) {
                while (!get_flag(MM_ALLOC_FLAG)) {
                        /* 没有内存申请任务则睡眠 */
                        if( signal_pending(current)) 
                                return 0;
                        timeout = interruptible_sleep_on_timeout(&wq, timeout);
                }
                rt_alloc_mm();
                clear_flag(MM_ALLOC_FLAG);
        }
        return -1;
}

/* 实时应用－主程序-脉冲控制部分 */
wait_queue_head_t wq;
static pid_t kmalloc_kthread_id;
static int kmalloc_kthread_state = 1;
static int pulse_generate_thread_created = 0;
static int main_ctrl_thread_created = 0;

static pthread_t pulse_generate_pthread;        
static pthread_t main_ctrl_pthread;
static pthread_mutex_t cache_mutex;

void rt_mm_request(void)
{
        set_flag(MM_ALLOC_FLAG);
        /*
         *        通过设置标志位通知kmalloc_thread内核线程申请内存
         */
        while(get_flag(MM_ALLOC_FLAG))        
                pthread_wait_np();
}

void* main_ctrl_thread(void *arg)
{
        int work_sched_period = 160000;
        struct timespec resolution;
                
        int ret1 = rtl_setclockmode(rtl_getschedclock(), RTL_CLOCK_MODE_PERIODIC, 
                work_sched_period);                
        if (ret1) {
                dbg_print("seting periodic mode failed\n");
                clear_flag(WORK_SCHED_MODE);
        }
        clock_getres(rtl_getschedclock(), &resolution);
        work_sched_period = timespec_to_ns(&resolution);
        
        pthread_make_periodic_np(pthread_self(), clock_gethrtime(rtl_getschedclock()),
            work_sched_period);

        init_task();        
        for (;;) {
                if (work) {
                        dbg_print("work\n");
                        rt_mm_request();
                        calc_time_interval();
                        if (exit)
                            break;
                } else
                        pthread_wait_np();
        }
        exit_task();
        
    return 0;
}

int init_module(void)
{
        pthread_attr_t attr;
        struct sched_param p;
        int ret;
        
        rtf_destroy(0); 
        rtf_destroy(1);
        rt_com_clr_in(0);
        rt_com_clr_out(0);
        
        /* 创建实时管道，用于RT模块和普通应用程序之间的通信 */
        int fifo_status = rtf_create(0,100);
        if(fifo_status)
                dbg_print("FIFO Create failed!");
        
        fifo_status = rtf_create(1, 4000);
        if(fifo_status)
                dbg_print("FIFO Create failed!");
        
        /* 设置实时串口，用于RT模块控制串口输出 */
        rt_com_setup(0, 9600, RT_COM_PARITY_NONE, 1, 8);
        
        hrtime_t now = gethrtime();

        pthread_attr_init(&attr);
        pthread_mutex_init(&cache_mutex, NULL);
        pthread_attr_setfp_np(&attr, 1);

        /* pulse_generate_thread */
        ret = pthread_create(&pulse_generate_pthread, &attr, 
                pulse_generate_thread, (void *)0);
        if (!ret)
                pulse_generate_thread_created = 1;
        pthread_make_periodic_np (pulse_generate_pthread, now + 2 * 240000, 80000);
        p . sched_priority = 1;
        pthread_setschedparam (pulse_generate_pthread, SCHED_FIFO, &p);
        
        /* main_ctrl_thread */
        ret = pthread_create(&main_ctrl_pthread, &attr, main_ctrl_thread, (void *)1);
        if (!ret) 
                main_ctrl_thread_created=1;
        pthread_make_periodic_np (main_ctrl_pthread, now + 2 * 160000, 30000);
        p . sched_priority = 2;
        pthread_setschedparam (main_ctrl_pthread, SCHED_FIFO, &p);
        
        init_waitqueue_head(&wq);
        kmalloc_kthread_id = kernel_thread(kmalloc_thread, NULL, 0);
        if (kmalloc_kthread_id < 0) {
                printk(KERN_ERR "fork failed, errno %d\n", -kmalloc_kthread_id);
                return kmalloc_kthread_id;
        }
        
        return ret;
}

void cleanup_module(void)
{
        /* send a term signal to the kthread */
        int ret = kill_proc(kmalloc_kthread_id, SIGKILL, 1);
        if (!ret) {
                int count = 10 * HZ;
                /* wait for the kthread to exit befor terminating */
                while (kmalloc_kthread_state && --count) {
                        current->state = TASK_INTERRUPTIBLE;
                        schedule_timeout(1);
                }
        }

        if (main_ctrl_thread_created) {
                pthread_cancel(main_ctrl_pthread);
                pthread_join(main_ctrl_pthread, NULL);
                pthread_delete_np(main_ctrl_pthread);
        }

        if (pulse_generate_thread_created) {
                pthread_cancel(pulse_generate_pthread);
                pthread_join(pulse_generate_pthread, NULL);
                pthread_delete_np(pulse_generate_pthread);
        }

        rt_com_setup(0, -1, 0, 0, 0);
        rtf_destroy(0);
        rtf_destroy(1);
        pthread_mutex_destroy (&cache_mutex);
}