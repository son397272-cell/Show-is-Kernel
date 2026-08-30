/*
 * CPU Governor - ShowTime
 *
 * ShowTime By Skinger49
 * 
 * Device: G988B ( Samsung Galaxy S20 Ultra )
 * 
 * Show forward!
 * 
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "sched.h"

#include <linux/sched/cpufreq.h>
#include <trace/events/power.h>

#ifdef CONFIG_SCHED_FFSI_GLUE
#include <linux/ffsi.h>
/**
 * 2nd argument of ffsi_obj_creator() experimentally decided by client itself,
 * which represents how much variant the random variable registered to FFSI
 * instance can behave at most, in terms of referencing d2u_decl_cmtpdf table
 * (maximum index of d2u_decl_cmtpdf table).
 */
#define UTILAVG_FFSI_VARIANCE        16
DECLARE_ELASTICITY(cpufreq, 32, 25, 24, 25);
#define FFSI_CLUSTER_TRAVERSING
#endif

struct sugov_tunables {
        struct gov_attr_set        attr_set;
        unsigned int                up_rate_limit_us;
        unsigned int                down_rate_limit_us;
#ifdef CONFIG_SCHED_FFSI_GLUE
        bool                         fb_legacy;
#endif
};

struct sugov_policy {
        struct cpufreq_policy        *policy;

        struct sugov_tunables        *tunables;
        struct list_head        tunables_hook;

        raw_spinlock_t                update_lock;        /* For shared policies */
        u64                        last_freq_update_time;
        s64                        min_rate_limit_ns;
        s64                        up_rate_delay_ns;
        s64                        down_rate_delay_ns;
        unsigned int                next_freq;
        unsigned int                cached_raw_freq;
        unsigned int                prev_cached_raw_freq;

        /* The next fields are only needed if fast switch cannot be used: */
        struct                        irq_work irq_work;
        struct                        kthread_work work;
        struct                        mutex work_lock;
        struct                        kthread_worker worker;
        struct task_struct        *thread;
        bool                        work_in_progress;

        bool                        limits_changed;
        bool                        need_freq_update;
#ifdef CONFIG_SCHED_FFSI_GLUE
        bool                         be_stochastic;
#endif
};

static DEFINE_PER_CPU(struct sugov_policy *, sugov_policy);

struct sugov_cpu {
        struct update_util_data        update_util;
        struct sugov_policy        *sg_policy;
        unsigned int                cpu;

        bool                        iowait_boost_pending;
        unsigned int                iowait_boost;
        u64                        last_update;

#ifdef CONFIG_SCHED_FFSI_GLUE
        /**
         * FFSI instance which should be referenced in percpu manner,
         * and data accordingly to handle the target job intensity.
         */
        struct ffsi_class         *util_vessel;
        unsigned long                 cached_util;
        unsigned long                 util;
#endif
        unsigned long                bw_dl;
        unsigned long                min;
        unsigned long                max;

        /* The field below is for single-CPU policies only: */
#ifdef CONFIG_NO_HZ_COMMON
        unsigned long                saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);

/************************ Governor internals ***********************/

static bool sugov_should_update_freq(struct sugov_policy *sg_policy, u64 time)
{
        s64 delta_ns;

        /*
         * Since cpufreq_update_util() is called with rq->lock held for
         * the @target_cpu, our per-CPU data is fully serialized.
         *
         * However, drivers cannot in general deal with cross-CPU
         * requests, so while get_next_freq() will work, our
         * sugov_update_commit() call may not for the fast switching platforms.
         *
         * Hence stop here for remote requests if they aren't supported
         * by the hardware, as calculating the frequency is pointless if
         * we cannot in fact act on it.
         *
         * This is needed on the slow switching platforms too to prevent CPUs
         * going offline from leaving stale IRQ work items behind.
         */
        if (!cpufreq_this_cpu_can_update(sg_policy->policy))
                return false;

        if (unlikely(sg_policy->limits_changed)) {
                sg_policy->limits_changed = false;
                sg_policy->need_freq_update = true;
                return true;
        }

        /* No need to recalculate next freq for min_rate_limit_us
         * at least. However we might still decide to further rate
         * limit once frequency change direction is decided, according
         * to the separate rate limits.
         */

        delta_ns = time - sg_policy->last_freq_update_time;
        return delta_ns >= sg_policy->min_rate_limit_ns;
}

static bool sugov_up_down_rate_limit(struct sugov_policy *sg_policy, u64 time,
                                     unsigned int next_freq)
{
        s64 delta_ns;

        delta_ns = time - sg_policy->last_freq_update_time;

        if (next_freq > sg_policy->next_freq &&
            delta_ns < sg_policy->up_rate_delay_ns)
                        return true;

        if (next_freq < sg_policy->next_freq &&
            delta_ns < sg_policy->down_rate_delay_ns)
                        return true;

        return false;
}

static bool sugov_update_next_freq(struct sugov_policy *sg_policy, u64 time,
                                   unsigned int next_freq)
{
        if (sg_policy->next_freq == next_freq)
                return false;

        if (sugov_up_down_rate_limit(sg_policy, time, next_freq)) {
                /* Restore cached freq as next_freq is not changed */
                sg_policy->cached_raw_freq = sg_policy->prev_cached_raw_freq;
                return false;
        }

        sg_policy->next_freq = next_freq;
        sg_policy->last_freq_update_time = time;

        return true;
}

static void sugov_fast_switch(struct sugov_policy *sg_policy, u64 time,
                              unsigned int next_freq)
{
        struct cpufreq_policy *policy = sg_policy->policy;
        int cpu;

        if (!sugov_update_next_freq(sg_policy, time, next_freq))
                return;

        next_freq = cpufreq_driver_fast_switch(policy, next_freq);
        if (!next_freq)
                return;

        policy->cur = next_freq;

        if (trace_cpu_frequency_enabled()) {
                for_each_cpu(cpu, policy->cpus)
                        trace_cpu_frequency(next_freq, cpu);
        }
}

static void sugov_deferred_update(struct sugov_policy *sg_policy, u64 time,
                                  unsigned int next_freq)
{
        if (!sugov_update_next_freq(sg_policy, time, next_freq))
                return;

        if (!sg_policy->work_in_progress) {
                sg_policy->work_in_progress = true;
                irq_work_queue(&sg_policy->irq_work);
        }
}

/**
 * get_next_freq - Compute a new frequency for a given cpufreq policy.
 * @sg_policy: schedutil policy object to 
{