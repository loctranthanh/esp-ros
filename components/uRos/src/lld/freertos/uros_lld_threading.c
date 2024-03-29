/*
Copyright (c) 2012-2013, Politecnico di Milano. All rights reserved.

Andrea Zoppi <texzk@email.it>
Martino Migliavacca <martino.migliavacca@gmail.com>

http://airlab.elet.polimi.it/
http://www.openrobots.com/

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file    uros_lld_threading.c
 * @author  Andrea Zoppi <texzk@email.it>
 *
 * @brief   Low-level threading features implementation.
 */

/*===========================================================================*/
/* HEADER FILES                                                              */
/*===========================================================================*/

#include "uros_lld_threading.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/queue.h"

// #include <ch.h>

/*===========================================================================*/
/* LOCAL TYPES & MACROS                                                      */
/*===========================================================================*/

#if UROS_THREADING_C_USE_ASSERT == UROS_FALSE && !defined(__DOXYGEN__)
#undef urosAssert
#define urosAssert(expr)
#endif

typedef struct thread_item_ {
  TaskHandle_t                task_handle;
  SemaphoreHandle_t           sem;
  UrosThreadId                id;
  STAILQ_ENTRY(thread_item_)  next;
} thread_item_t;

typedef STAILQ_HEAD(thread_list_, thread_item_) thread_list_t;

thread_list_t*  g_thread_list = NULL;

/*===========================================================================*/
/* GLOBAL FUNCTIONS                                                          */
/*===========================================================================*/

/** @addtogroup threading_lld_funcs */
/** @{ */

/*~~~ SEMAPHORE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Semaphore */
/** @{ */

/**
 * @brief   Initializes a semaphore.
 * @details The semaphore object is initialized, and an initial count is
 *          assigned.
 *
 * @pre     The semaphore is not initialized.
 *
 * @param[in,out] semp
 *          Pointer to an allocated @p UrosSem object.
 * @param n
 *          Initial semaphore count.
 */
void uros_lld_sem_objectinit(UrosSem *semp, uros_cnt_t n) {

  urosAssert(semp != NULL);

  // chSemInit(semp, n);
  *semp = xSemaphoreCreateCounting(n, 0);
}

/**
 * @brief   Cleans a semaphore.
 * @details Deallocates any memory chunks allocated by the object itself.
 *
 * @pre     The semaphore is initialized.
 * @post    The semaphore is not initialized, call @p urosSemObjectInit() to
 *          use it again.
 *
 * @param[in,out] semp
 *          Pointer to an initialized @p UrosSem object.
 */
void uros_lld_sem_clean(UrosSem *semp) {

  urosAssert(semp != NULL);

  // (void)semp;
  vSemaphoreDelete(*semp);
}

/**
 * @brief   Waits for a semaphore signal.
 * @details Waits until the semaphore counter is positive. When positive, it
 *          is decremented and the procedure returns the control to the caller.
 * @note    Depending on the low-level implementation, it may support a
 *          <i>priority inversion</i> mechanism.
 *
 * @param[in,out] semp
 *          Pointer to an initialized @p UrosSem object.
 */
void uros_lld_sem_wait(UrosSem *semp) {

  urosAssert(semp != NULL);

  // chSemWait(semp);
  xSemaphoreTake(*semp, portMAX_DELAY);
}

/**
 * @brief   Semaphore signal.
 * @details Increments a semaphore counter.
 *
 * @param[in,out] semp
 *          Pointer to an initialized @p UrosSem object.
 */
void uros_lld_sem_signal(UrosSem *semp) {

  urosAssert(semp != NULL);

  // chSemSignal(semp);
  xSemaphoreGive(*semp);
}

/**
 * @brief   Gets the semaphore value.
 * @details Reads the current semaphore value.
 * @warning Thread-safe, but the value may not be consistent after a call to
 *          this function.
 *
 * @param[in] semp
 *          Pointer to an initialized @p UrosSem object.
 * @return
 *          Current semaphore count.
 */
uros_cnt_t uros_lld_sem_value(UrosSem *semp) {

  uros_cnt_t value;

  urosAssert(semp != NULL);

  // chSysLock();
  // value = chSemGetCounterI(semp);
  // chSysUnlock();
  // xSemaphoreTake(semp->mutex, portTICK_PERIOD_MS);
  value = (int)uxSemaphoreGetCount(*semp);
  return value;
}

/** @} */

/*~~~ MUTEX ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Mutex */
/** @{ */

/**
 * @brief   Initializes a mutex.
 * @details The mutex object is initialized, and set as unlocked.
 *
 * @pre     The mutex is not initialized.
 * @post    The mutex is unlocked.
 *
 * @param[in,out] mtxp
 *          Pointer to an allocated @p UrosMutex object.
 */
void uros_lld_mutex_objectinit(UrosMutex *mtxp) {

  urosAssert(mtxp != NULL);

  // chMtxInit(mtxp);
  *mtxp = xSemaphoreCreateMutex();
}

/**
 * @brief   Cleans a mutex.
 * @details Deallocates any memory chunks allocated by the object itself.
 *
 * @pre     The mutex is initialized.
 * @post    The mutex is not initialized, call @p urosMutexObjectInit() to
 *          use it again.
 *
 * @param[in,out] mtxp
 *          Pointer to an initialized @p UrosMutex object.
 */
void uros_lld_mutex_clean(UrosMutex *mtxp) {

  urosAssert(mtxp != NULL);

  // (void)mtxp;
  vSemaphoreDelete(*mtxp);
}

/**
 * @brief   Locks a mutex.
 * @details Tries to lock a mutex. If the mutex is already locked, it waits
 *          until it is unlocked.
 * @note    Depending on the low-level implementation, it may support a
 *          <i>priority inversion</i> mechanism.
 *
 * @post    The mutex is locked.
 *
 * @param[in,out] mtxp
 *          Pointer to an initialized @p UrosMutex object.
 */
void uros_lld_mutex_lock(UrosMutex *mtxp) {
  urosAssert(mtxp != NULL);

  // chMtxLock(mtxp);
  xSemaphoreTake(*mtxp, portMAX_DELAY);
}

/**
 * @brief   Unlocks a mutex.
 *
 * @pre     The mutex is locked.
 * @post    The mutex is unlocked.
 *
 * @param[in,out] mtxp
 *          Pointer to a locked @p UrosMutex object.
 */
void uros_lld_mutex_unlock(UrosMutex *mtxp) {

  // UrosMutex *unlocked;
  // (void)unlocked;
  // (void)mtxp;

  // urosAssert(mtxp != NULL);

  // unlocked = chMtxUnlock();
  // urosAssert(unlocked == mtxp);
  xSemaphoreGive(*mtxp);
}

/** @} */

/*~~~ CONDITION VARIABLE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Condition variable */
/** @{ */

/**
 * @brief   Initializes a condvar.
 *
 * @pre     The condvar is not initialized.
 * @post    The condvar is initialized.
 *
 * @param[in,out] cvp
 *          Pointer to an allocated @p UrosCondVar object.
 */
void uros_lld_condvar_objectinit(UrosCondVar *cvp) {

  urosAssert(cvp != NULL);

  // chCondInit(cvp);
  // cvp = xEventGroupCreate();
  *cvp = xSemaphoreCreateMutex();
}

/**
 * @brief   Cleans a condvar.
 * @details Deallocates any memory chunks allocated by the object itself.
 *
 * @pre     The condvar is initialized.
 * @post    The condvar is not initialized, call @p urosConvVarObjectInit() to
 *          use it again.
 *
 * @param[in,out] cvp
 *          Pointer to an initialized @p UrosCondVar object.
 */
void uros_lld_condvar_clean(UrosCondVar *cvp) {

  urosAssert(cvp != NULL);

  // (void)cvp;
  vSemaphoreDelete(*cvp);
}

/**
 * @brief   Waits for a condvar signal.
 * @details Waits until the condvar is signalled.
 * @note    This procedure must be called within a lock zone guarded by a
 *          mutex, shared by the waiting and the signalling thread.
 *
 * @param[in,out] cvp
 *          Pointer to an initialized @p UrosCondVar object.
 * @param[in,out] mtxp
 *          Pointer to the mutex guarding this condvar.
 */
void uros_lld_condvar_wait(UrosCondVar *cvp, UrosMutex *mtxp) {

  urosAssert(cvp != NULL);
// #if UROS_THREADING_C_USE_ASSERT != UROS_FALSE
//   chSysLock();
//   urosAssert(mtxp == currp->p_mtxlist);
//   chSysUnlock();
// #endif

//   (void)mtxp;
//   chCondWait(cvp);
  // xSemaphoreGive(*mtxp);
  xSemaphoreTake(*cvp, portMAX_DELAY);
  // xSemaphoreTake(*mtxp, portMAX_DELAY);
}

/**
 * @brief   Single condvar signal.
 * @details Signals a condvar to a single waiting thread.
 * @note    This procedure may be called within a lock zone guarded by a mutex,
 *          shared by the waiting and the signalling thread.
 *
 * @param[in,out] cvp
 *          Pointer to an initialized @p UrosCondVar object.
 */
void uros_lld_condvar_signal(UrosCondVar *cvp) {

  urosAssert(cvp != NULL);

  // chCondSignal(cvp);
  xSemaphoreGive(*cvp);
}

/**
 * @brief   Broadcast condvar signal.
 * @details Signals a condvar to all of the waiting threads.
 * @note    This procedure may be called within a lock zone guarded by a mutex,
 *          shared by the waiting and the signalling thread.
 *
 * @param[in,out] cvp
 *          Pointer to an initialized @p UrosCondVar object.
 */
void uros_lld_condvar_broadcast(UrosCondVar *cvp) {

  urosAssert(cvp != NULL);
  xSemaphoreGive(*cvp);
  // chCondBroadcast(cvp);
}

/** @} */

/*~~~ THREAD ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Thread */
/** @{ */

/**
 * @brief   Gets the current thread identifier
 *
 * @return
 *          The current thread identifier.
 */
UrosThreadId uros_lld_thread_self(void) {

  // return chThdSelf();
  TaskHandle_t task_handle = xTaskGetCurrentTaskHandle();
  thread_item_t* item = NULL;
  STAILQ_FOREACH(item, g_thread_list, next) {
    if (item->task_handle == task_handle) {
      return item->id;
    }
  }
  return 0;
}

/**
 * @brief   Gets the name of the thread.
 *
 * @param[in] id
 *          Thread identifier.
 * @return
 *          Pointer to the name of the thread string, null-terminated.
 * @retval NULL
 *          No name assigned to the thread.
 */
const char *uros_lld_thread_getname(UrosThreadId id) {
// #if CH_USE_REGISTRY
//   return id->p_name;
// #else
//   return NULL;
// #endif
  thread_item_t* item = NULL;
  STAILQ_FOREACH(item, g_thread_list, next) {
    if (item->id == id) {
      return pcTaskGetTaskName(item->task_handle);
    }
  }
  return "";
}

/**
 * @brief   Creates a thread with a static stack.
 * @details Creates a new thread. The memory chunk for the stack is externally
 *          declared, and is simply referenced. It is usually allocated as a
 *          static buffer.
 *
 * @pre     The stack is big enough to avoid stack overflow.
 *
 * @param[out] idp
 *          Pointer to the @p UrosThreadId where the created thread id is
 *          stored.
 * @param[in] namep
 *          Pointer to the default thread name, valid for the whole the thread
 *          life. Null-terminated string.
 * @param[in] priority
 *          Thread priority.
 * @param[in] routine
 *          Thread routine, entry point of the thread program.
 * @param[in] argp
 *          Argument passed to the thread routine. Can be @p NULL.
 * @param[in] stackp
 *          Pointer to the externally allocated stack buffer.
 * @param[in] stacksize
 *          Stack size, in bytes.
 * @return
 *          Error code.
 */

typedef struct {
  uros_proc_f routine;
  void *argp;
  SemaphoreHandle_t lock;
} adapter_task_param_t;

uros_err_t adapter_task(void* pv)
{
  adapter_task_param_t* param = pv;
  uros_err_t ret = param->routine(param->argp);
  thread_item_t* item = NULL;
  thread_item_t* target_item = NULL;
  STAILQ_FOREACH(item, g_thread_list, next) {
    if (item->sem == param->lock) {
      target_item = item;
      break;
    }
  }
  if (target_item) {
    STAILQ_REMOVE(g_thread_list, target_item, thread_item_, next);
  }
  xSemaphoreGive(param->lock);
  vTaskDelete(NULL);
  return ret;
}

uros_err_t uros_lld_thread_createstatic(UrosThreadId *idp, const char *namep,
                                        uros_prio_t priority,
                                        uros_proc_f routine, void *argp,
                                        void *stackp, size_t stacksize) {

  urosAssert(idp != NULL);
  urosAssert(routine != NULL);
  urosAssert(stackp != NULL);
  urosAssert(stacksize > 0);
  adapter_task_param_t param = {
    .routine = routine,
    .argp = argp,
    .lock = xSemaphoreCreateMutex(),
  };
  if (g_thread_list == NULL) {
    g_thread_list = malloc(sizeof(thread_list_t));
    STAILQ_INIT(g_thread_list);
  }
  thread_item_t* new_thread = malloc(sizeof(thread_item_t));
  new_thread->id = idp;
  new_thread->sem = param.lock;
  STAILQ_INSERT_TAIL(g_thread_list, new_thread, next);
  if (xTaskCreate(adapter_task, namep, stacksize, &param, priority, &new_thread->task_handle) == pdTRUE) {
    printf("created static task name: %s\n", namep);
    return UROS_OK;
  }
  vSemaphoreDelete(param.lock);
  STAILQ_REMOVE(g_thread_list, new_thread, thread_item_, next);
  free(new_thread);
  return UROS_ERR_NOMEM;

//   *idp = chThdCreateStatic(stackp, stacksize, priority, (void*)routine, argp);
//   if (*idp != NULL) {
// #if CH_USE_REGISTRY
//     /* Set the thread name.*/
//     chSysLock();
//     (*idp)->p_name = namep;
//     chSysUnlock();
// #else
//     (void)namep;
// #endif
  // }
  // return UROS_ERR_NOMEM;
}

/**
 * @brief   Creates a thread allocating the stack from a memory pool.
 * @details Creates a new thread. The memory chunk for the stack is allocated
 *          from a memory pool.
 * @warning The thread creation is suspended until there is a free slot in the
 *          memory pool.
 *
 * @pre     The stack is big enough to avoid stack overflow.
 *
 * @param[out] idp
 *          Pointer to the @p UrosThreadId where the created thread id is
 *          stored.
 * @param[in] namep
 *          Pointer to the default thread name, valid for the whole the thread
 *          life. Null-terminated string.
 * @param[in] priority
 *          Thread priority.
 * @param[in] routine
 *          Thread routine, entry point of the thread program.
 * @param[in] argp
 *          Argument passed to the thread routine. Can be @p NULL.
 * @param[in] mempoolp
            Pointer to the memory pool where to allocate a stack.
 * @return
 *          Error code.
 */
uros_err_t uros_lld_thread_createfrommempool(UrosThreadId *idp, const char *namep,
                                             uros_prio_t priority,
                                             uros_proc_f routine, void *argp,
                                             UrosMemPool *mempoolp) {

  urosAssert(idp != NULL);
  urosAssert(routine != NULL);
  urosAssert(mempoolp != NULL);
  // if (xTaskCreate(routine, namep, 1024, argp, priority, idp) == pdTRUE) {
  //   return UROS_OK;
  // }
  // return UROS_ERR_NOMEM;

  adapter_task_param_t param = {
    .routine = routine,
    .argp = argp,
    .lock = xSemaphoreCreateMutex(),
  };
  if (g_thread_list == NULL) {
    g_thread_list = malloc(sizeof(thread_list_t));
    STAILQ_INIT(g_thread_list);
  }
  thread_item_t* new_thread = malloc(sizeof(thread_item_t));
  new_thread->id = idp;
  new_thread->sem = param.lock;
  STAILQ_INSERT_TAIL(g_thread_list, new_thread, next);
  if (xTaskCreate(adapter_task, namep, 1024 * 2, &param, priority, &new_thread->task_handle) == pdTRUE) {
    return UROS_OK;
  }
  STAILQ_REMOVE(g_thread_list, new_thread, thread_item_, next);
  free(new_thread);
  vSemaphoreDelete(param.lock);
  return UROS_ERR_NOMEM;
//   *idp = chThdCreateFromMemoryPool(mempoolp, priority, routine, argp);
//   if (*idp != NULL) {
// #if CH_USE_REGISTRY
//     /* Set the thread name.*/
//     chSysLock();
//     (*idp)->p_name = namep;
//     chSysUnlock();
// #else
//     (void)namep;
// #endif
//     return UROS_OK;
//   }
}

/**
 * @brief   Creates a thread allocating the stack on the deafult heap.
 *
 * @pre     The stack is big enough to avoid stack overflow.
 *
 * @param[out] idp
 *          Pointer to the @p UrosThreadId where the created thread id is
 *          stored.
 * @param[in] namep
 *          Pointer to the default thread name, valid for the whole the thread
 *          life. Null-terminated string.
 * @param[in] priority
 *          Thread priority.
 * @param[in] routine
 *          Thread routine, entry point of the thread program.
 * @param[in] argp
 *          Argument passed to the thread routine. Can be @p NULL.
 * @param[in] stacksize
 *          Stack size, in bytes.
 * @return
 *          Error code.
 */
uros_err_t uros_lld_thread_createfromheap(UrosThreadId *idp, const char *namep,
                                          uros_prio_t priority,
                                          uros_proc_f routine, void *argp,
                                          size_t stacksize) {

  urosAssert(idp != NULL);
  urosAssert(routine != NULL);
  urosAssert(stacksize > 0);

//   *idp = chThdCreateFromHeap(NULL, stacksize, priority, (void*)routine, argp);
//   if (*idp != NULL) {
// #if CH_USE_REGISTRY
//     /* Set the thread name.*/
//     chSysLock();
//     (*idp)->p_name = namep;
//     chSysUnlock();
// #else
//     (void)namep;
// #endif
  adapter_task_param_t param = {
    .routine = routine,
    .argp = argp,
    .lock = xSemaphoreCreateMutex(),
  };
  if (g_thread_list == NULL) {
    g_thread_list = malloc(sizeof(thread_list_t));
    STAILQ_INIT(g_thread_list);
  }
  thread_item_t* new_thread = malloc(sizeof(thread_item_t));
  new_thread->id = idp;
  new_thread->sem = param.lock;
  STAILQ_INSERT_TAIL(g_thread_list, new_thread, next);
  if (xTaskCreate(adapter_task, namep, stacksize, &param, priority, &new_thread->task_handle) == pdTRUE) {
    return UROS_OK;
  }
  STAILQ_REMOVE(g_thread_list, new_thread, thread_item_, next);
  free(new_thread);
  vSemaphoreDelete(param.lock);
  return UROS_ERR_NOMEM;
}

/**
 * @brief   Joins a thread.
 * @details Waits until the required thread has exited, and releases its
 *          allocated memory.
 *
 * @post    The required thread does not exist anymore.
 * @post    Any resources allocated by the thread creation are released (e.g.
 *          stack).
 *
 * @param[in] id
 *          Identifier of the thread being joined.
 * @return
 *          Error code.
 */
uros_err_t uros_lld_thread_join(UrosThreadId id) {

  urosAssert(id != NULL);
  thread_item_t* item;
  SemaphoreHandle_t sem = NULL;
  STAILQ_FOREACH(item, g_thread_list, next) {
    if (item->id == id) {
      sem = item->sem;
      break;
    }
  }
  if (sem == NULL) {
    return UROS_ERR_BADCONN;
  }
  xSemaphoreTake(sem, portMAX_DELAY);
  // return chThdWait(id);
  return UROS_OK;
}

/**
 * @brief   Sleeps for some seconds.
 * @details Puts the thread in sleep state for the provided amount of time.
 *
 * @param[in] sec
 *          Number of seconds to sleep.
 */
void uros_lld_thread_sleepsec(uint32_t sec) {
  vTaskDelay(sec * 1000 / portTICK_PERIOD_MS);
  // chThdSleepSeconds((systime_t)sec);
}

/**
 * @brief   Sleeps for some milliseconds.
 * @details Puts the thread in sleep state for the provided amount of time.
 *
 * @param[in] msec
 *          Number of milliseconds to sleep.
 */
void uros_lld_thread_sleepmsec(uint32_t msec) {
  vTaskDelay(msec / portTICK_PERIOD_MS);
  // chThdSleepMilliseconds((systime_t)msec);
}

/**
 * @brief   Sleeps for some microseconds.
 * @details Puts the thread in sleep state for the provided amount of time.
 *
 * @param[in] usec
 *          Number of microseconds to sleep.
 */
void uros_lld_thread_sleepusec(uint32_t usec) {
  vTaskDelay(usec / 1000 / portTICK_PERIOD_MS);
  // chThdSleepMicroseconds((systime_t)usec);
}

/**
 * @brief   Current timestamp in milliseconds.
 * @note    The resolution is in milliseconds, but the precision may not be.
 *
 * @return
 *          The current timestamp, with a resolution of one millisecond.
 */
uint32_t uros_lld_threading_gettimestampmsec(void) {
  return xTaskGetTickCount();
// #if CH_FREQUENCY == 1000
//   return (uint32_t)chTimeNow();
// #else
//   return (((uint32_t)chTimeNow() - 1) * 1000) / CH_FREQUENCY + 1;
// #endif
}

/** @} */
/** @} */
