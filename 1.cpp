#include "rt_com.h"
#include /* printk level */
#include /* kernel version etc. */

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Wind-Son");
MODULE_DESCRIPTION("Pulse-Control system");

typedef unsigned short __u16;

/* ʵʱӦ�ã�IO������Ʋ��� */

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
* ʵʱӦ�ã���ʵʱ�߳�ģ�ⶨʱ����������������� 
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
        p.sched_priority = 1; /* ����ʵʱ�̵߳����ȼ� */

        struct timespec resolution;
        /* RTʱ������ */
        rtl_setclockmode(CLOCK_REALTIME, RTL_CLOCK_MODE_PERIODIC, 
                        intrrupt_sched_period);
        clock_getres(rtl_getschedclock(), &resolution);
        intrrupt_sched_period = timespec_to_ns(&resolution);
        
        /* ����RT-���Ȳ��� */
        pthread_make_periodic_np(pthread_self(), clock_gethrtime(rtl_getschedclock()), 
                intrrupt_sched_period);
        pthread_setschedparam (pthread_self(), SCHED_FIFO, &p);

        for (;;) {
                dbg_print("debug entry\n");
                while (!ready) /* ���еȴ� */
                    pthread_wait_np(); /* ����״̬һ�����øú����ó���cpu�Ŀ��ƣ�����������*/
                dbg_print("debug exit\n");

                if (!init_rt_clock) {
                        /* ��ʼ������������RTʱ�� */
                        init_rt_clock = 1;
                        pthread_wait_np();
                        current_time = clock_gethrtime(CLOCK_REALTIME);
                } else {
                    if (intrrupt_sched_period < MIN_TIME)
                                intrrupt_sched_period = MIN_TIME;
                    current_time += intrrupt_sched_period;
                        /*
                         * ��һ���ܹؼ���clock_nanosleep()ʹ���߳�ֱ��˯���ڶ�ʱ���ϣ�
                         * ˯��ʱ��current_time(ns)��Ȼ�󱻻��ѡ�ʵ���������ַ�ʽ
                         * ������֤ʵʱ���һ����Ͻӽ���Ӳ����ʱ�������ˡ�
                         * ��ͨ��pthread_wait_np()ʵʱ������Ȼ��ʵʱ����Ҳ���б�֤�ģ�
                         * ���ھ����������޵ġ�
                         */
                    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, hrt2ts(current_time), NULL);
                }
        
                /* ����������ơ��� */
                io_bit_on(IO_PORT_OUT, XPULSE, &io_status);

                /* 
                 * ��ȡ��һ��������,�Ӷ�������ȷ�������������
                 * Note! ��ʵʱ�߳�����ģ�ⶨʱ�����,��˲�����̫���ӵļ��㡣
                 * һ������½�������ʱ��ļ����Լ��������Ʋ��ַŵ�����һ���������̡߳�
                 * �ڱ�ϵͳ��time_interval_calc_thread�����ڸ�Ŀ�ġ�
                 */
            intrrupt_sched_period = get_time_interval();
        }

        return 0;
}

/*
* ʵʱӦ�ã���̬�ڴ����벿�֡�
* ֱ����RT-thread�������ڴ�ʵ�ʽ��֤���ܲ��ȶ���ʱ��ʱ��crash��
* �������Ϊalloc���̱������ȼ�RT-threadǿ���жϲ����쳣��
* �����Ҫ��̬�����ڴ�ʱ���ٶ������ں��߳�ר�Ÿ����ڴ����롣
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
                        /* û���ڴ�����������˯�� */
                        if( signal_pending(current)) 
                                return 0;
                        timeout = interruptible_sleep_on_timeout(&wq, timeout);
                }
                rt_alloc_mm();
                clear_flag(MM_ALLOC_FLAG);
        }
        return -1;
}

/* ʵʱӦ�ã�������-������Ʋ��� */
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
         *        ͨ�����ñ�־λ֪ͨkmalloc_thread�ں��߳������ڴ�
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
        
        /* ����ʵʱ�ܵ�������RTģ�����ͨӦ�ó���֮���ͨ�� */
        int fifo_status = rtf_create(0,100);
        if(fifo_status)
                dbg_print("FIFO Create failed!");
        
        fifo_status = rtf_create(1, 4000);
        if(fifo_status)
                dbg_print("FIFO Create failed!");
        
        /* ����ʵʱ���ڣ�����RTģ����ƴ������ */
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