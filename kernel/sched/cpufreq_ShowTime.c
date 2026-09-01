/*
 * ShowTime By Skinger49
 *
 * Developer - Skinger49
 *
 * Device - Samsung Galaxy S20 Ultra (Exynos)
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "sched.h"

#include <linux/sched/cpufreq.h>
#include <trace/events/power.h>

#ifdef CONFIG_SCHED_FFSI_GLUE
#include <linux/ffsi.h>
#define UTILAVG_FFSI_VARIANCE        16
DECLARE_ELASTICITY(cpufreq, 32, 25, 24, 25);
#define FFSI_CLUSTER_TRAVERSING
#endif

struct showtime_tunables {
        struct gov_attr_set        attr_set;
        unsigned int                up_rate_limit_us;
        unsigned int                down_rate_limit_us;
#ifdef CONFIG_SCHED_FFSI_GLUE
        bool                         fb_legacy;
#endif
};

struct showtime_policy {
        struct cpufreq_policy        *policy;

        struct showtime_tunables        *tunables;
        struct list_head        tunables_hook;

        raw_spinlock_t                update_lock;
        u64                        last_freq_update_time;
        s64                        min_rate_limit_ns;
        s64                        up_rate_delay_ns;
        s64                        down_rate_delay_ns;
        unsigned int                next_freq;
        unsigned int                cached_raw_freq;
        unsigned int                prev_cached_raw_freq;

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

static DEFINE_PER_CPU(struct showtime_policy *, showtime_policy);

struct showtime_cpu {
        struct update_util_data        update_util;
        struct showtime_policy        *sg_policy;
        unsigned int                cpu;

        bool                        iowait_boost_pending;
        unsigned int                iowait_boost;
        u64                        last_update;

#ifdef CONFIG_SCHED_FFSI_GLUE
        struct ffsi_class         *util_vessel;
        unsigned long                 cached_util;
        unsigned long                 util;
#endif
        unsigned long                bw_dl;
        unsigned long                min;
        unsigned long                max;

#ifdef CONFIG_NO_HZ_COMMON
        unsigned long                saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct showtime_cpu, showtime_cpu);

static bool showtime_should_update_freq(struct showtime_policy *sg_policy, u64 time)
{
        s64 delta_ns;

        if (!cpufreq_this_cpu_can_update(sg_policy->policy))
                return false;

        if (unlikely(sg_policy->limits_changed)) {
                sg_policy->limits_changed = false;
                sg_policy->need_freq_update = true;
                return true;
        }

        delta_ns = time - sg_policy->last_freq_update_time;
        return delta_ns >= sg_policy->min_rate_limit_ns;
}

static bool showtime_up_down_rate_limit(struct showtime_policy *sg_policy, u64 time,
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

static bool showtime_update_next_freq(struct showtime_policy *sg_policy, u64 time,
                                   unsigned int next_freq)
{
        if (sg_policy->next_freq == next_freq)
                return false;

        if (showtime_up_down_rate_limit(sg_policy, time, next_freq)) {
                sg_policy->cached_raw_freq = sg_policy->prev_cached_raw_freq;
                return false;
        }

        sg_policy->next_freq = next_freq;
        sg_policy->last_freq_update_time = time;

        return true;
}

static void showtime_fast_switch(struct showtime_policy *sg_policy, u64 time,
                              unsigned int next_freq)
{
        struct cpufreq_policy *policy = sg_policy->policy;
        int cpu;

        if (!showtime_update_next_freq(sg_policy, time, next_freq))
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

static void showtime_deferred_update(struct showtime_policy *sg_policy, u64 time,
                                  unsigned int next_freq)
{
        if (!showtime_update_next_freq(sg_policy, time, next_freq))
                return;

        if (!sg_policy->work_in_progress) {
                sg_policy->work_in_progress = true;
                irq_work_queue(&sg_policy->irq_work);
        }
}

static unsigned int get_next_freq(struct showtime_policy *sg_policy,
                                  unsigned long util, unsigned long max)
{
        struct cpufreq_policy *policy = sg_policy->policy;
        unsigned int freq = arch_scale_freq_invariant() ?
                                policy->max : policy->cur;

#ifdef CONFIG_SCHED_FFSI_GLUE
        struct showtime_cpu *sg_cpu;
        struct ffsi_class *vessel;
        unsigned int delta_max, delta_min;
        int util_delta;
        unsigned int legacy_freq;

#ifdef FFSI_CLUSTER_TRAVERSING
        unsigned int each;
        unsigned int sigma_cpu = policy->cpu;
        randomness most_rand = 0;
#endif
        int cur_rand = FFSI_DIVERGING;
        RV_DECLARE(rv);
#endif

        freq = map_util_freq(util, freq, max);

#ifdef CONFIG_SCHED_FFSI_GLUE
        legacy_freq = freq;

        if (sg_policy->tunables->fb_legacy)
                goto skip_betting;

#ifndef FFSI_CLUSTER_TRAVERSING
        sg_cpu = &per_cpu(showtime_cpu, policy->cpu);
        vessel = sg_cpu->util_vessel;

        if (!vessel)
                goto skip_betting;

        cur_rand = vessel->job_inferer(vessel);
        if (cur_rand == FFSI_DIVERGING)
                goto skip_betting;
#else
        for_each_cpu(each, policy->cpus) {
                sg_cpu = &per_cpu(showtime_cpu, each);

                vessel = sg_cpu->util_vessel;
                if (vessel) {
                        cur_rand = vessel->job_inferer(vessel);
                        if (cur_rand == FFSI_DIVERGING)
                                goto skip_betting;
                        else {
                                if (cur_rand > (int)most_rand) {
                                        most_rand = (randomness)cur_rand;
                                        sigma_cpu = each;
                                }
                        }
                } else
                        goto skip_betting;
        }

        sg_cpu        = &per_cpu(showtime_cpu, sigma_cpu);
        vessel        = sg_cpu->util_vessel;
#endif
        util_delta = sg_cpu->util - sg_cpu->cached_util;
        delta_max  = sg_cpu->max - sg_cpu->cached_util;
        delta_min  = sg_cpu->cached_util;

        RV_SET(rv, util_delta, delta_max, delta_min);
        freq = vessel->cap_bettor(vessel, &rv, freq);

skip_betting:
        trace_sugov_ffsi_freq(policy->cpu, util, max, cur_rand, legacy_freq, freq);
#else
#endif
        if (freq == sg_policy->cached_raw_freq && !sg_policy->need_freq_update)
                return sg_policy->next_freq;

        sg_policy->need_freq_update = false;
        sg_policy->prev_cached_raw_freq = sg_policy->cached_raw_freq;
        sg_policy->cached_raw_freq = freq;
        return cpufreq_driver_resolve_freq(policy, freq);
}

static unsigned long showtime_cpu_util(int cpu, unsigned long util_cfs,
                                 unsigned long max, enum schedutil_type type,
                                 struct task_struct *p)
{
        unsigned long dl_util, util, irq;
        struct rq *rq = cpu_rq(cpu);

        if (!uclamp_is_used() &&
            type == FREQUENCY_UTIL && rt_rq_is_runnable(&rq->rt)) {
                return max;
        }

        irq = cpu_util_irq(rq);
        if (unlikely(irq >= max))
                return max;

        util = util_cfs + cpu_util_rt(rq);
        if (type == FREQUENCY_UTIL)
                util = uclamp_rq_util_with(rq, util, p);

        dl_util = cpu_util_dl(rq);

        if (util + dl_util >= max)
                return max;

        if (type == ENERGY_UTIL)
                util += dl_util;

        util = scale_irq_capacity(util, irq, max);
        util += irq;

        if (type == FREQUENCY_UTIL)
                util += cpu_bw_dl(rq);

        return min(max, util);
}

static unsigned long showtime_get_util(struct showtime_cpu *sg_cpu)
{
        struct rq *rq = cpu_rq(sg_cpu->cpu);

        unsigned long util = cpu_util_cfs(rq);
        unsigned long max = arch_scale_cpu_capacity(NULL, sg_cpu->cpu);

        sg_cpu->max = max;
        sg_cpu->bw_dl = cpu_bw_dl(rq);

        util = emstune_freq_boost(sg_cpu->cpu, util);

        part_cpu_active_ratio(&util, &max, sg_cpu->cpu);

        return showtime_cpu_util(sg_cpu->cpu, util, max, FREQUENCY_UTIL, NULL);
}

#ifdef CONFIG_SCHED_FFSI_GLUE
static inline void showtime_util_collapse(struct showtime_cpu *sg_cpu)
{
        struct ffsi_class *vessel = sg_cpu->util_vessel;
        int util_delta = min(sg_cpu->max, sg_cpu->util) - sg_cpu->cached_util;
        unsigned int delta_max = sg_cpu->max - sg_cpu->cached_util;
        unsigned int delta_min = sg_cpu->cached_util;

        RV_DECLARE(job);

        if (vessel) {
                RV_SET(job, util_delta, delta_max, delta_min);
                vessel->job_learner(vessel, &job);
        }
}
#endif

static bool showtime_iowait_reset(struct showtime_cpu *sg_cpu, u64 time,
                               bool set_iowait_boost)
{
        s64 delta_ns = time - sg_cpu->last_update;

        if (delta_ns <= TICK_NSEC)
                return false;

        sg_cpu->iowait_boost = set_iowait_boost ? sg_cpu->min : 0;
        sg_cpu->iowait_boost_pending = set_iowait_boost;

        return true;
}

static void showtime_iowait_boost(struct showtime_cpu *sg_cpu, u64 time,
                               unsigned int flags)
{
        bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

        if (sg_cpu->iowait_boost &&
            showtime_iowait_reset(sg_cpu, time, set_iowait_boost))
                return;

        if (!set_iowait_boost)
                return;

        if (sg_cpu->iowait_boost_pending)
                return;
        sg_cpu->iowait_boost_pending = true;

        if (sg_cpu->iowait_boost) {
                sg_cpu->iowait_boost =
                        min_t(unsigned int, sg_cpu->iowait_boost << 1, SCHED_CAPACITY_SCALE);
                return;
        }

        sg_cpu->iowait_boost = sg_cpu->min;
}

static unsigned long showtime_iowait_apply(struct showtime_cpu *sg_cpu, u64 time,
                                        unsigned long util, unsigned long max)
{
        unsigned long boost;

        if (!sg_cpu->iowait_boost)
                return util;

        if (showtime_iowait_reset(sg_cpu, time, false))
                return util;

        if (!sg_cpu->iowait_boost_pending) {
                sg_cpu->iowait_boost >>= 1;
                if (sg_cpu->iowait_boost < sg_cpu->min) {
                        sg_cpu->iowait_boost = 0;
                        return util;
                }
        }

        sg_cpu->iowait_boost_pending = false;

        boost = (sg_cpu->iowait_boost * max) >> SCHED_CAPACITY_SHIFT;
        return max(boost, util);
}

#ifdef CONFIG_NO_HZ_COMMON
static bool showtime_cpu_is_busy(struct showtime_cpu *sg_cpu)
{
        unsigned long idle_calls = tick_nohz_get_idle_calls_cpu(sg_cpu->cpu);
        bool ret = idle_calls == sg_cpu->saved_idle_calls;

        sg_cpu->saved_idle_calls = idle_calls;
        return ret;
}
#else
static inline bool showtime_cpu_is_busy(struct showtime_cpu *sg_cpu) { return false; }
#endif

static inline void ignore_dl_rate_limit(struct showtime_cpu *sg_cpu, struct showtime_policy *sg_policy)
{
        if (cpu_bw_dl(cpu_rq(sg_cpu->cpu)) > sg_cpu->bw_dl)
                sg_policy->limits_changed = true;
}

static void showtime_update_single(struct update_util_data *hook, u64 time,
                                unsigned int flags)
{
        struct showtime_cpu *sg_cpu = container_of(hook, struct showtime_cpu, update_util);
        struct showtime_policy *sg_policy = sg_cpu->sg_policy;
        unsigned long util, max;
        unsigned int next_f;
        bool busy;

        showtime_iowait_boost(sg_cpu, time, flags);
        sg_cpu->last_update = time;

        ignore_dl_rate_limit(sg_cpu, sg_policy);

        if (!showtime_should_update_freq(sg_policy, time))
                return;

        busy = !sg_policy->need_freq_update && showtime_cpu_is_busy(sg_cpu);

#ifndef CONFIG_SCHED_FFSI_GLUE
        util = showtime_get_util(sg_cpu);
#else
        sg_cpu->cached_util = sg_cpu->util;
        sg_cpu->util = util = showtime_get_util(sg_cpu);
        showtime_util_collapse(sg_cpu);
#endif
        max = sg_cpu->max;
        util = showtime_iowait_apply(sg_cpu, time, util, max);
        next_f = get_next_freq(sg_policy, util, max);
        if (busy && next_f < sg_policy->next_freq) {
                next_f = sg_policy->next_freq;

                sg_policy->cached_raw_freq = sg_policy->prev_cached_raw_freq;
        }

        if (sg_policy->policy->fast_switch_enabled) {
                showtime_fast_switch(sg_policy, time, next_f);
        } else {
                raw_spin_lock(&sg_policy->update_lock);
                showtime_deferred_update(sg_policy, time, next_f);
                raw_spin_unlock(&sg_policy->update_lock);
        }
}

static unsigned int showtime_next_freq_shared(struct showtime_cpu *sg_cpu, u64 time)
{
        struct showtime_policy *sg_policy = sg_cpu->sg_policy;
        struct cpufreq_policy *policy = sg_policy->policy;
        unsigned long util = 0, max = 1;
        unsigned int j;

        for_each_cpu_and(j, policy->related_cpus, cpu_online_mask) {
                struct showtime_cpu *j_sg_cpu = &per_cpu(showtime_cpu, j);
                unsigned long j_util, j_max;

#ifndef CONFIG_SCHED_FFSI_GLUE
                j_util = showtime_get_util(j_sg_cpu);
#else
                j_sg_cpu->cached_util = j_sg_cpu->util;
                j_sg_cpu->util = j_util = showtime_get_util(j_sg_cpu);
                showtime_util_collapse(j_sg_cpu);
#endif
                j_max = j_sg_cpu->max;
                j_util = showtime_iowait_apply(j_sg_cpu, time, j_util, j_max);

                if (j_util * max > j_max * util) {
                        util = j_util;
                        max = j_max;
                }
        }

        return get_next_freq(sg_policy, util, max);
}

static void
showtime_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
        struct showtime_cpu *sg_cpu = container_of(hook, struct showtime_cpu, update_util);
        struct showtime_policy *sg_policy = sg_cpu->sg_policy;
        unsigned int next_f;

        raw_spin_lock(&sg_policy->update_lock);

        showtime_iowait_boost(sg_cpu, time, flags);
        sg_cpu->last_update = time;

        ignore_dl_rate_limit(sg_cpu, sg_policy);

        if (showtime_should_update_freq(sg_policy, time)) {
                next_f = showtime_next_freq_shared(sg_cpu, time);

                if (sg_policy->policy->fast_switch_enabled)
                        showtime_fast_switch(sg_policy, time, next_f);
                else
                        showtime_deferred_update(sg_policy, time, next_f);
        }

        raw_spin_unlock(&sg_policy->update_lock);
}

static void showtime_work(struct kthread_work *work)
{
        struct showtime_policy *sg_policy = container_of(work, struct showtime_policy, work);
        unsigned int freq;
        unsigned long flags;

        raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
        freq = sg_policy->next_freq;
        sg_policy->work_in_progress = false;
        raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);

        down_write(&sg_policy->policy->rwsem);
        mutex_lock(&sg_policy->work_lock);
        __cpufreq_driver_target(sg_policy->policy, freq, CPUFREQ_RELATION_L);
        mutex_unlock(&sg_policy->work_lock);
        up_write(&sg_policy->policy->rwsem);
}

static void showtime_irq_work(struct irq_work *irq_work)
{
        struct showtime_policy *sg_policy;

        sg_policy = container_of(irq_work, struct showtime_policy, irq_work);

        kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}

static struct showtime_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct showtime_tunables *to_showtime_tunables(struct gov_attr_set *attr_set)
{
        return container_of(attr_set, struct showtime_tunables, attr_set);
}

static DEFINE_MUTEX(min_rate_lock);

static void update_min_rate_limit_ns(struct showtime_policy *sg_policy)
{
        mutex_lock(&min_rate_lock);
        sg_policy->min_rate_limit_ns = min(sg_policy->up_rate_delay_ns,
                                           sg_policy->down_rate_delay_ns);
        mutex_unlock(&min_rate_lock);
}

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
        struct showtime_tunables *tunables = to_showtime_tunables(attr_set);

        return sprintf(buf, "%u\n", tunables->up_rate_limit_us);
}

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
        struct showtime_tunables *tunables = to_showtime_tunables(attr_set);

        return sprintf(buf, "%u\n", tunables->down_rate_limit_us);
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
                                      const char *buf, size_t count)
{
        struct showtime_tunables *tunables = to_showtime_tunables(attr_set);
        struct showtime_policy *sg_policy;
        unsigned int rate_limit_us;

        if (kstrtouint(buf, 10, &rate_limit_us))
                return -EINVAL;

        tunables->up_rate_limit_us = rate_limit_us;

        list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
                sg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
                update_min_rate_limit_ns(sg_policy);
        }

        return count;
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
                                        const char *buf, size_t count)
{
        struct showtime_tunables *tunables = to_showtime_tunables(attr_set);
        struct showtime_policy *sg_policy;
        unsigned int rate_limit_us;

        if (kstrtouint(buf, 10, &rate_limit_us))
                return -EINVAL;

        tunables->down_rate_limit_us = rate_limit_us;

        list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
                sg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
                update_min_rate_limit_ns(sg_policy);
        }

        return count;
}

#ifdef CONFIG_SCHED_FFSI_GLUE
static ssize_t fb_legacy_show(struct gov_attr_set *attr_set, char *buf)
{
        struct showtime_tunables *tunables = to_showtime_tunables(attr_set);

        return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->fb_legacy);
}

static ssize_t fb_legacy_store(struct gov_attr_set *attr_set, const char *buf,
                               size_t count)
{
        struct showtime_tunables *tunables = to_showtime_tunables(attr_set);

        if (kstrtobool(buf, &tunables->fb_legacy))
                return -EINVAL;

        return count;
}
#endif

static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);
#ifdef CONFIG_SCHED_FFSI_GLUE
static struct governor_attr fb_legacy = __ATTR_RW(fb_legacy);
#endif

static struct attribute *showtime_attributes[] = {
        &up_rate_limit_us.attr,
        &down_rate_limit_us.attr,
#ifdef CONFIG_SCHED_FFSI_GLUE
        &fb_legacy.attr,
#endif
        NULL
};

static void showtime_tunables_free(struct kobject *kobj)
{
        struct gov_attr_set *attr_set = container_of(kobj, struct gov_attr_set, kobj);

        kfree(to_showtime_tunables(attr_set));
}

static struct kobj_type showtime_tunables_ktype = {
        .default_attrs = showtime_attributes,
        .sysfs_ops = &governor_sysfs_ops,
        .release = &showtime_tunables_free,
};

struct cpufreq_governor showtime_gov;

static struct showtime_policy *showtime_policy_alloc(struct cpufreq_policy *policy)
{
        struct showtime_policy *sg_policy;

        sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
        if (!sg_policy)
                return NULL;

        sg_policy->policy = policy;
        raw_spin_lock_init(&sg_policy->update_lock);
        return sg_policy;
}

static void showtime_policy_free(struct showtime_policy *sg_policy)
{
        kfree(sg_policy);
}

static int showtime_kthread_create(struct showtime_policy *sg_policy)
{
        struct task_struct *thread;
        struct sched_attr attr = {
                .size                = sizeof(struct sched_attr),
                .sched_policy        = SCHED_FIFO,
                .sched_flags        = SCHED_FLAG_SUGOV,
                .sched_nice        = 0,
                .sched_priority        = MAX_RT_PRIO / 4,
                .sched_runtime        =  1000000,
                .sched_deadline = 10000000,
                .sched_period        = 10000000,
        };
        struct cpufreq_policy *policy = sg_policy->policy;
        int ret;

        if (policy->fast_switch_enabled)
                return 0;

        kthread_init_work(&sg_policy->work, showtime_work);
        kthread_init_worker(&sg_policy->worker);
        thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
                                "showtime:%d",
                                cpumask_first(policy->related_cpus));
        if (IS_ERR(thread)) {
                pr_err("failed to create showtime thread: %ld\n", PTR_ERR(thread));
                return PTR_ERR(thread);
        }

        ret = sched_setattr_nocheck(thread, &attr);
        if (ret) {
                kthread_stop(thread);
                pr_warn("%s: failed to set SCHED_DEADLINE\n", __func__);
                return ret;
        }

        sg_policy->thread = thread;
        init_irq_work(&sg_policy->irq_work, showtime_irq_work);
        mutex_init(&sg_policy->work_lock);

        wake_up_process(thread);

        return 0;
}

static void showtime_kthread_stop(struct showtime_policy *sg_policy)
{
        if (sg_policy->policy->fast_switch_enabled)
                return;

        kthread_flush_worker(&sg_policy->worker);
        kthread_stop(sg_policy->thread);
        mutex_destroy(&sg_policy->work_lock);
}

static struct showtime_tunables *showtime_tunables_alloc(struct showtime_policy *sg_policy)
{
        struct showtime_tunables *tunables;

        tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
        if (tunables) {
                gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
                if (!have_governor_per_policy())
                        global_tunables = tunables;
        }
        return tunables;
}

static void showtime_clear_global_tunables(void)
{
        if (!have_governor_per_policy())
                global_tunables = NULL;
}

static int showtime_init(struct cpufreq_policy *policy)
{
        struct showtime_policy *sg_policy;
        struct showtime_tunables *tunables;
        int ret = 0;
        int cpu;

        if (policy->governor_data)
                return -EBUSY;

        sg_policy = per_cpu(showtime_policy, policy->cpu);
        if (sg_policy) {
                policy->governor_data = sg_policy;
                return 0;
        }

        cpufreq_enable_fast_switch(policy);

        sg_policy = showtime_policy_alloc(policy);
        if (!sg_policy) {
                ret = -ENOMEM;
                goto disable_fast_switch;
        }

        ret = showtime_kthread_create(sg_policy);
        if (ret)
                goto free_sg_policy;

        mutex_lock(&global_tunables_lock);

        if (global_tunables) {
                if (WARN_ON(have_governor_per_policy())) {
                        ret = -EINVAL;
                        goto stop_kthread;
                }
                policy->governor_data = sg_policy;
                sg_policy->tunables = global_tunables;

                gov_attr_set_get(&global_tunables->attr_set, &sg_policy->tunables_hook);
                goto out;
        }

        tunables = showtime_tunables_alloc(sg_policy);
        if (!tunables) {
                ret = -ENOMEM;
                goto stop_kthread;
        }

        tunables->up_rate_limit_us = cpufreq_policy_transition_delay_us(policy);
        tunables->down_rate_limit_us = cpufreq_policy_transition_delay_us(policy);
#ifdef CONFIG_SCHED_FFSI_GLUE
        tunables->fb_legacy = false;
        sg_policy->be_stochastic = false;
#endif

        policy->governor_data = sg_policy;
        sg_policy->tunables = tunables;

        for_each_cpu(cpu, policy->related_cpus)
                per_cpu(showtime_policy, cpu) = sg_policy;

        ret = kobject_init_and_add(&tunables->attr_set.kobj, &showtime_tunables_ktype,
                                   get_governor_parent_kobj(policy), "%s",
                                   showtime_gov.name);
        if (ret)
                goto fail;

out:
        mutex_unlock(&global_tunables_lock);
        return 0;

fail:
        kobject_put(&tunables->attr_set.kobj);
        policy->governor_data = NULL;
        showtime_clear_global_tunables();

stop_kthread:
        showtime_kthread_stop(sg_policy);
        mutex_unlock(&global_tunables_lock);

free_sg_policy:
        showtime_policy_free(sg_policy);

disable_fast_switch:
        cpufreq_disable_fast_switch(policy);

        pr_err("initialization failed (error %d)\n", ret);
        return ret;
}

static void showtime_exit(struct cpufreq_policy *policy)
{
        struct showtime_policy *sg_policy = policy->governor_data;
        struct showtime_tunables *tunables = sg_policy->tunables;
        unsigned int count;
#ifdef CONFIG_SCHED_FFSI_GLUE
        struct showtime_cpu *sg_cpu = &per_cpu(showtime_cpu, policy->cpu);
#endif

        if (per_cpu(showtime_policy, policy->cpu)) {
                policy->governor_data = NULL;
                return;
        }

        mutex_lock(&global_tunables_lock);

        count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
        policy->governor_data = NULL;
        if (!count)
                showtime_clear_global_tunables();

        mutex_unlock(&global_tunables_lock);

#ifdef CONFIG_SCHED_FFSI_GLUE
        if (sg_cpu->util_vessel) {
                sg_cpu->util_vessel->finalizer(sg_cpu->util_vessel);
                ffsi_obj_destructor(sg_cpu->util_vessel);
                sg_cpu->util_vessel = NULL;
        }
        sg_policy->be_stochastic = false;
#endif

        showtime_kthread_stop(sg_policy);
        showtime_policy_free(sg_policy);
        cpufreq_disable_fast_switch(policy);
}

static int showtime_start(struct cpufreq_policy *policy)
{
        struct showtime_policy *sg_policy = policy->governor_data;
        unsigned int cpu;
#ifdef CONFIG_SCHED_FFSI_GLUE
        char alias[FFSI_ALIAS_LEN] = {0,};
#endif

        sg_policy->up_rate_delay_ns =
                sg_policy->tunables->up_rate_limit_us * NSEC_PER_USEC;
        sg_policy->down_rate_delay_ns =
                sg_policy->tunables->down_rate_limit_us * NSEC_PER_USEC;
        update_min_rate_limit_ns(sg_policy);
        sg_policy->last_freq_update_time        = 0;
        sg_policy->next_freq                        = 0;
        sg_policy->work_in_progress                = false;
        sg_policy->limits_changed                = false;
        sg_policy->need_freq_update                = false;
        sg_policy->cached_raw_freq                = 0;
        sg_policy->prev_cached_raw_freq                = 0;

        for_each_cpu(cpu, policy->cpus) {
                struct showtime_cpu *sg_cpu = &per_cpu(showtime_cpu, cpu);

#ifdef CONFIG_SCHED_FFSI_GLUE
#ifndef FFSI_CLUSTER_TRAVERSING
                if (cpu != policy->cpu) {
                        memset(sg_cpu, 0, sizeof(*sg_cpu));
                        goto skip_subcpus;
                }
#endif
                if (!sg_policy->be_stochastic) {
                        sprintf(alias, "govern%d", cpu);
                        memset(sg_cpu, 0, sizeof(*sg_cpu));
                        sg_cpu->util_vessel =
                                ffsi_obj_creator(alias,
                                                 UTILAVG_FFSI_VARIANCE,
                                                 policy->cpuinfo.max_freq,
                                                 policy->cpuinfo.min_freq,
                                                 &elasticity_cpufreq);
                        if (sg_cpu->util_vessel->initializer(sg_cpu->util_vessel) < 0) {
                                sg_cpu->util_vessel->finalizer(sg_cpu->util_vessel);
                                ffsi_obj_destructor(sg_cpu->util_vessel);
                                sg_cpu->util_vessel = NULL;
                        }
                } else {
                        struct ffsi_class *vptr = sg_cpu->util_vessel;
                        memset(sg_cpu, 0, sizeof(*sg_cpu));
                        sg_cpu->util_vessel = vptr;
                }
#ifndef FFSI_CLUSTER_TRAVERSING                
skip_subcpus:
#endif
#else
                memset(sg_cpu, 0, sizeof(*sg_cpu));
#endif
                sg_cpu->cpu                        = cpu;
                sg_cpu->sg_policy                = sg_policy;
                sg_cpu->min                        =
                        (SCHED_CAPACITY_SCALE * policy->cpuinfo.min_freq) /
                        policy->cpuinfo.max_freq;
        }

#ifdef CONFIG_SCHED_FFSI_GLUE
        sg_policy->be_stochastic = true;
#endif
        for_each_cpu(cpu, policy->cpus) {
                struct showtime_cpu *sg_cpu = &per_cpu(showtime_cpu, cpu);

                cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util,
                                             policy_is_shared(policy) ?
                                                        showtime_update_shared :
                                                        showtime_update_single);
        }
        return 0;
}

static void showtime_stop(struct cpufreq_policy *policy)
{
        struct showtime_policy *sg_policy = policy->governor_data;
        unsigned int cpu;

        for_each_cpu(cpu, policy->cpus)
                cpufreq_remove_update_util_hook(cpu);

        synchronize_sched();

#ifdef CONFIG_SCHED_FFSI_GLUE
        for_each_cpu(cpu, policy->cpus) {
                struct showtime_cpu *sg_cpu = &per_cpu(showtime_cpu, cpu);
                if (sg_cpu->util_vessel) {
                        sg_cpu->util_vessel->stopper(sg_cpu->util_vessel);
                }
        }
#endif

        if (!policy->fast_switch_enabled)
                irq_work_sync(&sg_policy->irq_work);
}

static void showtime_limits(struct cpufreq_policy *policy)
{
        struct showtime_policy *sg_policy = policy->governor_data;

        if (!policy->fast_switch_enabled) {
                mutex_lock(&sg_policy->work_lock);
                cpufreq_policy_apply_limits(policy);
                mutex_unlock(&sg_policy->work_lock);
        }

        sg_policy->limits_changed = true;
}

struct cpufreq_governor showtime_gov = {
        .name                        = "ShowTime",
        .owner                        = THIS_MODULE,
        .dynamic_switching        = true,
        .init                        = showtime_init,
        .exit                        = showtime_exit,
        .start                        = showtime_start,
        .stop                        = showtime_stop,
        .limits                        = showtime_limits,
};

static int __init showtime_register(void)
{
        return cpufreq_register_governor(&showtime_gov);
}
fs_initcall(showtime_register);