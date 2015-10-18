/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is system adaptation of the lwIP TCP/IP stack
 * by Adam Dunkels <adam@sics.se> for RTEMS system.
 *
 * Author: Premysl Houdek <houdepre@fel.cvut.cz>
 * Author: Pavel Pisa <pisa@cmp.felk.cvut.cz>
 * Industrial Informatics Group, FEE, Czech Technical University in Prague
 *
 */
/*
 * mapping of lwIP system dependencies to generic POSIX system services and types.
 * DETAILS: ./lwip/doc/sys_arch.txt
 */

#include <stdint.h>
#include <errno.h>
#include <arch/cc.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include "sys_arch.h"
#include "lwip/err.h"
#include "lwip/tcpip.h"
#include "lwipopts.h"

#define SYS_LWIP_MBOX_SIZE (sizeof(void *))

clockid_t sys_sem_clockid = CLOCK_REALTIME;

static void
sys_timespec_msec_add_to(struct timespec *y, uint32_t amsec)
{
  y->tv_sec += amsec / 1000;
  amsec %= 1000;
  y->tv_nsec += amsec * 1000000;
  if (y->tv_nsec > 1000000000) {
    y->tv_nsec -= 1000000000;
    y->tv_sec++;
  }
}

static uint32_t
sys_timespec_diff_msec(const struct timespec *a, const struct timespec *b)
{
  uint32_t diffmsec;
  uint32_t diffnsec;
  struct timespec dummy;
  typeof(dummy.tv_sec) sec;

  sec = a->tv_sec - b->tv_sec;
  if (sec >= 4294966)
    sec = 4294966;

  diffmsec = sec * 1000;

  diffnsec = a->tv_nsec - b->tv_nsec;
  if (diffnsec >= 1000000000) {
    diffnsec += 1000000000;
    diffmsec -= 1000;
  }
  diffmsec += diffnsec / 1000000;
  return diffmsec;
}

uint32_t
sys_now(void)
{
  uint32_t actmsec;
  struct timespec act_time;

  clock_gettime(sys_sem_clockid, &act_time);
  actmsec = act_time.tv_sec;
  actmsec *= 1000;
  actmsec += act_time.tv_nsec / 1000000000;

  return actmsec;
}

void
sys_init(void)
{
  //  Is called to initialize the sys_arch layer.
  return;
}

err_t
sys_sem_new(sys_sem_t *sem, u8_t count)
{
  int res;
  res = sem_init(sem, 0, count);
  if (res != 0)
    return ERR_MEM;

  return ERR_OK;
}


void
sys_sem_free(sys_sem_t *sem)
{
  sem_destroy(sem);
}

void
sys_sem_signal(sys_sem_t *sem)
{
  sem_post(sem);
}

void
sys_sem_signal_from_ISR(sys_sem_t *sem)
{
  sem_post(sem);
}


u32_t
sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
  int res;
  struct timespec start_time;
  struct timespec wait_time;

  clock_gettime(sys_sem_clockid, &start_time);

  if (timeout == 0) {
    res = sem_wait(sem);
  } else {
    wait_time = start_time;
    sys_timespec_msec_add_to(&wait_time, timeout);
    res = sem_timedwait(sem, &wait_time);
  }
  if (res) {
    return SYS_ARCH_TIMEOUT;
  }
  clock_gettime(sys_sem_clockid, &wait_time);

  return sys_timespec_diff_msec(&wait_time, &start_time);
}

int
sys_sem_valid(sys_sem_t *sem)
{
  return 1;
}

void
sys_sem_set_invalid(sys_sem_t *sem)
{
  memset(sem, 0, sizeof(*sem));
}

err_t
sys_mbox_new(sys_mbox_t *mbox, int size)
{
  int res;

  res = sem_init(&mbox->free_slots, 0, size);
  if (res != 0)
    goto error_free_slots_sem;

  res = sem_init(&mbox->full_slots, 0, 0);
  if (res != 0)
    goto error_full_slots_sem;

  if (sys_mutex_new(&mbox->mutex) != ERR_OK)
    goto error_mutex;

  mbox->slots = (typeof(mbox->slots))malloc(sizeof(*mbox->slots) * size);
  if (mbox->slots == NULL)
    goto error_alloc_slots;

  mbox->inidx = 0;
  mbox->outidx = 0;
  mbox->size = size;

  return ERR_OK;

error_alloc_slots:
  sys_mutex_free(&mbox->mutex);
error_mutex:
  sem_destroy(&mbox->full_slots);
error_full_slots_sem:
  sem_destroy(&mbox->free_slots);
error_free_slots_sem:
  return ERR_MEM;
}

void
sys_mbox_free(sys_mbox_t *mbox)
{
  sem_destroy(&mbox->full_slots);
  sem_destroy(&mbox->free_slots);
  free(mbox->slots);
  sys_mutex_free(&mbox->mutex);
  sys_mbox_set_invalid(mbox);
}

void
sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
  int res;

  res = sem_wait(&mbox->free_slots);
  if (res)
    return;

  pthread_mutex_lock(&mbox->mutex);
  mbox->slots[mbox->inidx++] = msg;
  if (mbox->inidx >= mbox->size)
    mbox->inidx = 0;
  pthread_mutex_unlock(&mbox->mutex);

  sem_post(&mbox->full_slots);
}

err_t
sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
  int res;

  res = sem_trywait(&mbox->free_slots);
  if (res)
    return ERR_MEM;

  pthread_mutex_lock(&mbox->mutex);
  mbox->slots[mbox->inidx++] = msg;
  if (mbox->inidx >= mbox->size)
    mbox->inidx = 0;
  pthread_mutex_unlock(&mbox->mutex);

  sem_post(&mbox->full_slots);

  return ERR_OK;
}

u32_t
sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
  int res;
  struct timespec start_time;
  struct timespec wait_time;

  clock_gettime(sys_sem_clockid, &start_time);

  if (timeout == 0) {
    res = sem_wait(&mbox->full_slots);
  } else {
    wait_time = start_time;
    sys_timespec_msec_add_to(&wait_time, timeout);
    res = sem_timedwait(&mbox->full_slots, &wait_time);
  }
  if (res)
    return SYS_ARCH_TIMEOUT;

  clock_gettime(sys_sem_clockid, &wait_time);

  pthread_mutex_lock(&mbox->mutex);
  *msg = mbox->slots[mbox->outidx++];
  if (mbox->outidx >= mbox->size)
    mbox->outidx = 0;
  pthread_mutex_unlock(&mbox->mutex);

  sem_post(&mbox->free_slots);

  return sys_timespec_diff_msec(&wait_time, &start_time);
}

u32_t
sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
  int res;
  struct timespec start_time;
  struct timespec wait_time;

  clock_gettime(sys_sem_clockid, &start_time);

  res = sem_trywait(&mbox->full_slots);
  if (res)
    return SYS_MBOX_EMPTY;

  clock_gettime(sys_sem_clockid, &wait_time);

  pthread_mutex_lock(&mbox->mutex);
  *msg = mbox->slots[mbox->outidx++];
  if (mbox->outidx >= mbox->size)
    mbox->outidx = 0;
  pthread_mutex_unlock(&mbox->mutex);

  sem_post(&mbox->free_slots);

  return 0;
}

int
sys_mbox_valid(sys_mbox_t *mbox)
{
  return mbox->size ? 1 : 0;
}

void
sys_mbox_set_invalid(sys_mbox_t *mbox)
{
  memset(mbox, 0, sizeof(*mbox));
}

sys_thread_t
sys_thread_new(const char *name, lwip_thread_fn function, void *arg, int stack_size, int prio)
{
  int res;
  pthread_attr_t taskattr;
  sys_thread_t thrid;
  struct sched_param schedparam;

  do {
    res = pthread_attr_init(&taskattr);
    if(res != 0)
      break;
    res = pthread_attr_setinheritsched(&taskattr, PTHREAD_EXPLICIT_SCHED);
    if(res != 0)
      break;
    res = pthread_attr_setschedpolicy(&taskattr, SCHED_FIFO);
    if(res != 0)
      break;

    schedparam.sched_priority = sched_get_priority_min(SCHED_FIFO) + prio;
    res = pthread_attr_setschedparam(&taskattr, &schedparam);
    if(res != 0)
      break;

    if (stack_size) {
      res = pthread_attr_setstacksize(&taskattr, stack_size);
      if(res != 0)
        break;
    }

    res = pthread_create(&thrid, &taskattr,
                         (void *(*)(void *))function, arg);
    if(res != 0)
      break;

    pthread_detach(thrid);
  } while(0);

  pthread_attr_destroy(&taskattr);

  if(res != 0)
    return 0;

  return thrid;
}

err_t
sys_mutex_new(sys_mutex_t *mutex)
{
  pthread_mutexattr_t mutexattr;

  if (pthread_mutexattr_init(&mutexattr)) {
    return -1;
  }

 #ifdef _POSIX_THREAD_PRIO_INHERIT
  #if _POSIX_THREAD_PRIO_INHERIT > 0
  if (pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT)) {
    return -1;
  }
  #endif
 #endif

  if (pthread_mutex_init(mutex, &mutexattr)) {
    return -1;
  }

  pthread_mutexattr_destroy(&mutexattr);

  return ERR_OK;
}

/** Lock a mutex
 * @param mutex the mutex to lock */
void
sys_mutex_lock(sys_mutex_t *mutex)
{
  pthread_mutex_lock(mutex);
}
/** Unlock a mutex
 * @param mutex the mutex to unlock */
void
sys_mutex_unlock(sys_mutex_t *mutex)
{
  pthread_mutex_unlock(mutex);
}
/** Delete a semaphore
 * @param mutex the mutex to delete */
void
sys_mutex_free(sys_mutex_t *mutex)
{
  pthread_mutex_destroy(mutex);
}

void
sys_arch_delay(unsigned int timeout)
{
  struct timespec wait_time;

  wait_time.tv_sec = timeout / 1000;
  wait_time.tv_nsec = timeout * 1000000;

  nanosleep(&wait_time, NULL);
}

/** Ticks/jiffies since power up. */
uint32_t
sys_jiffies(void)
{
  return sys_now();
}

/*
int
sys_request_irq(unsigned int irqnum, sys_irq_handler_t handler,
		unsigned long flags, const char *name, void *context)
{
  rtems_status_code res;

  res = rtems_interrupt_handler_install(irqnum,  name, flags,
					handler, context);
  return (res != RTEMS_SUCCESSFUL) ? -1 : 0;
}
*/
