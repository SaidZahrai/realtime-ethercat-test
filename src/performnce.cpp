/** \file
 * \brief Example code for Simple Open EtherCAT master (SOEM) - simple_ng
 *
 * Usage: simple_ng IFNAME
 * IFNAME is the NIC interface name, e.g. 'eth0'
 *
 * This variant is adapted for long-running cyclic latency characterization:
 * - Runs until interrupted (Ctrl+C).
 * - Uses a 1 ms absolute-time scheduler (CLOCK_MONOTONIC + TIMER_ABSTIME).
 * - Prints a compact report every 10 seconds.
 * - Avoids per-cycle stdout spam (cycle details are suppressed).
 *
 * Notes on metrics:
 * - roundtrip_us: measured inside fieldbus_roundtrip() using SOEM/osal timing.
 * - wakeup_late_us: scheduler wakeup lateness relative to the ideal 1 ms grid.
 */

#include "soem/soem.h"

#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct
{
   ecx_contextt context;
   char *iface;
   uint8 group;
   int roundtrip_time_us;
   uint8 map[4096];

   /* Last cycle values for reporting */
   int last_wkc;
   int expected_wkc;
} Fieldbus;

/* ------------------------- time helpers ------------------------- */

static inline int64_t ts_to_ns(const struct timespec *ts)
{
   return (int64_t)ts->tv_sec * 1000000000LL + (int64_t)ts->tv_nsec;
}

static inline struct timespec ns_to_ts(int64_t ns)
{
   struct timespec ts;
   ts.tv_sec  = (time_t)(ns / 1000000000LL);
   ts.tv_nsec = (long)(ns % 1000000000LL);
   if (ts.tv_nsec < 0)
   {
      ts.tv_nsec += 1000000000L;
      ts.tv_sec -= 1;
   }
   return ts;
}

static inline int64_t now_ns_monotonic(void)
{
   struct timespec ts;
   clock_gettime(CLOCK_MONOTONIC, &ts);
   return ts_to_ns(&ts);
}

/* ------------------------- stop handling ------------------------- */

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int sig)
{
   (void)sig;
   g_stop = 1;
}

/* ------------------------- fieldbus API ------------------------- */

static void fieldbus_initialize(Fieldbus *fieldbus, char *iface)
{
   /* Avoid surprises */
   memset(fieldbus, 0, sizeof(*fieldbus));

   fieldbus->iface = iface;
   fieldbus->group = 0;
   fieldbus->roundtrip_time_us = 0;
   fieldbus->last_wkc = 0;
   fieldbus->expected_wkc = 0;
}

static int fieldbus_roundtrip(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_timet start, end, diff;
   int wkc;

   start = osal_current_time();
   ecx_send_processdata(context);
   wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
   end = osal_current_time();

   osal_time_diff(&start, &end, &diff);
   fieldbus->roundtrip_time_us = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);

   return wkc;
}

static boolean fieldbus_start(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_groupt *grp = context->grouplist + fieldbus->group;
   ec_slavet *slave;
   int i;

   printf("Initializing SOEM on '%s'... ", fieldbus->iface);
   if (!ecx_init(context, fieldbus->iface))
   {
      printf("no socket connection\n");
      return FALSE;
   }
   printf("done\n");

   printf("Finding autoconfig slaves... ");
   if (ecx_config_init(context) <= 0)
   {
      printf("no slaves found\n");
      return FALSE;
   }
   printf("%d slaves found\n", context->slavecount);

   printf("Sequential mapping of I/O... ");
   ecx_config_map_group(context, fieldbus->map, fieldbus->group);
   printf("mapped %dO+%dI bytes from %d segments\n", grp->Obytes, grp->Ibytes, grp->nsegments);

   printf("Configuring distributed clock... ");
   ecx_configdc(context);
   printf("done\n");

   printf("Waiting for all slaves in SAFE_OP... ");
   ecx_statecheck(context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
   printf("done\n");

   printf("Priming with a roundtrip... ");
   (void)fieldbus_roundtrip(fieldbus);
   printf("done\n");

   printf("Setting OPERATIONAL state... ");
   slave = context->slavelist; /* slave 0 is broadcast */
   slave->state = EC_STATE_OPERATIONAL;
   ecx_writestate(context, 0);

   for (i = 0; i < 10; ++i)
   {
      (void)fieldbus_roundtrip(fieldbus);
      ecx_statecheck(context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE / 10);
      if (slave->state == EC_STATE_OPERATIONAL)
      {
         printf("done (all slaves OP)\n");
         return TRUE;
      }
   }

   printf("failed\n");
   ecx_readstate(context);
   for (i = 1; i <= context->slavecount; ++i)
   {
      slave = context->slavelist + i;
      if (slave->state != EC_STATE_OPERATIONAL)
      {
         printf("  slave %d state=0x%04X AL=0x%04X (%s)\n",
                i, slave->state, slave->ALstatuscode,
                ec_ALstatuscode2string(slave->ALstatuscode));
      }
   }

   return FALSE;
}

static void fieldbus_stop(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_slavet *slave = context->slavelist; /* slave 0 is broadcast */

   printf("Requesting INIT state on all slaves... ");
   slave->state = EC_STATE_INIT;
   ecx_writestate(context, 0);
   printf("done\n");

   printf("Closing socket... ");
   ecx_close(context);
   printf("done\n");
}

/* Single cycle: compute roundtrip and WKC validity. No per-cycle printing. */
static boolean fieldbus_cycle(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_groupt *grp = context->grouplist + fieldbus->group;

   int wkc = fieldbus_roundtrip(fieldbus);
   int expected_wkc = grp->outputsWKC * 2 + grp->inputsWKC;

   fieldbus->last_wkc = wkc;
   fieldbus->expected_wkc = expected_wkc;

   return (wkc >= expected_wkc) ? TRUE : FALSE;
}

static void fieldbus_check_state(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_groupt *grp = context->grouplist + fieldbus->group;
   ec_slavet *slave;
   int i;

   grp->docheckstate = FALSE;
   ecx_readstate(context);

   for (i = 1; i <= context->slavecount; ++i)
   {
      slave = context->slavelist + i;
      if (slave->group != fieldbus->group)
      {
         continue;
      }

      if (slave->state != EC_STATE_OPERATIONAL)
      {
         grp->docheckstate = TRUE;

         if (slave->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
         {
            printf("* Slave %d SAFE_OP+ERROR, attempting ACK\n", i);
            slave->state = EC_STATE_SAFE_OP + EC_STATE_ACK;
            ecx_writestate(context, i);
         }
         else if (slave->state == EC_STATE_SAFE_OP)
         {
            printf("* Slave %d SAFE_OP, requesting OP\n", i);
            slave->state = EC_STATE_OPERATIONAL;
            ecx_writestate(context, i);
         }
         else if (slave->state > EC_STATE_NONE)
         {
            if (ecx_reconfig_slave(context, i, EC_TIMEOUTRET))
            {
               slave->islost = FALSE;
               printf("* Slave %d reconfigured\n", i);
            }
         }
         else if (!slave->islost)
         {
            ecx_statecheck(context, i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (slave->state == EC_STATE_NONE)
            {
               slave->islost = TRUE;
               printf("* Slave %d lost\n", i);
            }
         }
      }
      else if (slave->islost)
      {
         if (slave->state != EC_STATE_NONE)
         {
            slave->islost = FALSE;
            printf("* Slave %d found\n", i);
         }
         else if (ecx_recover_slave(context, i, EC_TIMEOUTRET))
         {
            slave->islost = FALSE;
            printf("* Slave %d recovered\n", i);
         }
      }
   }

   if (!grp->docheckstate)
   {
      printf("All slaves resumed OPERATIONAL\n");
   }
}

/* ------------------------- main ------------------------- */

int main(int argc, char *argv[])
{
   Fieldbus fieldbus;

   signal(SIGINT, on_sigint);
   signal(SIGTERM, on_sigint);

   if (argc != 2)
   {
      ec_adaptert *adapter = NULL;
      ec_adaptert *head = NULL;
      printf("Usage: simple_ng IFNAME\n"
             "IFNAME is the NIC interface name, e.g. 'eth0'\n");

      printf("\nAvailable adapters:\n");
      head = adapter = ec_find_adapters();
      while (adapter != NULL)
      {
         printf("  - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(head);
      return 1;
   }

   fieldbus_initialize(&fieldbus, argv[1]);
   if (!fieldbus_start(&fieldbus))
   {
      return 1;
   }

   const int64_t period_ns = 1000000LL;                  /* 1 ms */
   const int64_t report_period_ns = 10LL * 1000000000LL; /* 10 s */
   const int64_t deadline_miss_ns = 1000000LL;           /* > 1 ms late */

   /* Stats */
   int rt_min_us = 0;
   int rt_max_us = 0;
   int64_t rt_sum_us = 0;

   int64_t late_min_ns = 0;
   int64_t late_max_ns = 0;
   int64_t late_sum_ns = 0;

   uint64_t iter_total = 0;
   uint64_t deadline_misses = 0;
   uint64_t bad_wkc_count = 0;

   int64_t t_start_ns = now_ns_monotonic();
   int64_t next_release_ns = (t_start_ns / period_ns + 1) * period_ns;
   int64_t next_report_ns = t_start_ns + report_period_ns;
   uint64_t last_report_iter = 0;

   printf("Running 1 ms cyclic loop. Press Ctrl+C to stop.\n");
   fflush(stdout);

   while (!g_stop)
   {
      /* Sleep until next release */
      struct timespec ts_release = ns_to_ts(next_release_ns);
      for (;;)
      {
         int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts_release, NULL);
         if (rc == 0)
         {
            break;
         }
         if (rc == EINTR)
         {
            if (g_stop) break;
            continue;
         }
         perror("clock_nanosleep");
         g_stop = 1;
         break;
      }
      if (g_stop) break;

      int64_t t_wakeup_ns = now_ns_monotonic();
      int64_t late_ns = t_wakeup_ns - next_release_ns;
      if (late_ns < 0) late_ns = 0;

      if (late_ns > deadline_miss_ns) deadline_misses++;

      if (iter_total == 0)
      {
         late_min_ns = late_max_ns = late_ns;
      }
      else
      {
         if (late_ns < late_min_ns) late_min_ns = late_ns;
         if (late_ns > late_max_ns) late_max_ns = late_ns;
      }
      late_sum_ns += late_ns;

      /* EtherCAT cycle */
      if (!fieldbus_cycle(&fieldbus))
      {
         bad_wkc_count++;
         fieldbus_check_state(&fieldbus);
      }
      else
      {
         int rt_us = fieldbus.roundtrip_time_us;
         if (iter_total == 0)
         {
            rt_min_us = rt_max_us = rt_us;
         }
         else
         {
            if (rt_us < rt_min_us) rt_min_us = rt_us;
            if (rt_us > rt_max_us) rt_max_us = rt_us;
         }
         rt_sum_us += (int64_t)rt_us;
      }

      iter_total++;

      /* Periodic reporting: exactly every ~10 seconds, no spam */
      if (t_wakeup_ns >= next_report_ns)
      {
         double elapsed_s = (double)(t_wakeup_ns - t_start_ns) / 1e9;
         uint64_t iters_since = iter_total - last_report_iter;
         last_report_iter = iter_total;

         double rate_hz = (double)iters_since / 10.0; /* 10 s window by design */

         double rt_avg_us = (iter_total > 0) ? ((double)rt_sum_us / (double)iter_total) : 0.0;

         double late_avg_us = (iter_total > 0) ? ((double)late_sum_ns / (double)iter_total / 1000.0) : 0.0;
         double late_min_us = (double)late_min_ns / 1000.0;
         double late_max_us = (double)late_max_ns / 1000.0;

         printf("t=%.1fs  iter=%llu  rate=%.1f Hz  "
                "roundtrip_us(min/avg/max)=%d/%.2f/%d  "
                "wakeup_late_us(min/avg/max)=%.2f/%.2f/%.2f  "
                "miss=%llu  bad_wkc=%llu  last_wkc=%d exp=%d\n",
                elapsed_s,
                (unsigned long long)iter_total,
                rate_hz,
                rt_min_us, rt_avg_us, rt_max_us,
                late_min_us, late_avg_us, late_max_us,
                (unsigned long long)deadline_misses,
                (unsigned long long)bad_wkc_count,
                fieldbus.last_wkc,
                fieldbus.expected_wkc);
         fflush(stdout);

         /* Advance report boundary; catch up if we were delayed */
         do {
            next_report_ns += report_period_ns;
         } while (t_wakeup_ns >= next_report_ns);
      }

      /* Advance release time; catch up if we slipped by multiple periods */
      next_release_ns += period_ns;
      int64_t t_after_ns = now_ns_monotonic();
      if (t_after_ns > next_release_ns + period_ns)
      {
         int64_t behind_ns = t_after_ns - next_release_ns;
         int64_t skip = behind_ns / period_ns;
         next_release_ns += skip * period_ns;
      }
   }

   /* Final summary */
   {
      double rt_avg_us = (iter_total > 0) ? ((double)rt_sum_us / (double)iter_total) : 0.0;

      double late_avg_us = (iter_total > 0) ? ((double)late_sum_ns / (double)iter_total / 1000.0) : 0.0;
      double late_min_us = (double)late_min_ns / 1000.0;
      double late_max_us = (double)late_max_ns / 1000.0;

      printf("Final: iter=%llu  roundtrip_us(min/avg/max)=%d/%.2f/%d  "
             "wakeup_late_us(min/avg/max)=%.2f/%.2f/%.2f  "
             "miss=%llu  bad_wkc=%llu  last_wkc=%d exp=%d\n",
             (unsigned long long)iter_total,
             rt_min_us, rt_avg_us, rt_max_us,
             late_min_us, late_avg_us, late_max_us,
             (unsigned long long)deadline_misses,
             (unsigned long long)bad_wkc_count,
             fieldbus.last_wkc,
             fieldbus.expected_wkc);
      fflush(stdout);
   }

   fieldbus_stop(&fieldbus);
   return 0;
}
