/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef NVGPU_POSIX_COND_H
#define NVGPU_POSIX_COND_H

#include <nvgpu/static_analysis.h>
#include <nvgpu/bug.h>
#include <nvgpu/lock.h>

/**
 * Define value used to indicate a wait without timeout.
 */
#define NVGPU_COND_WAIT_TIMEOUT_MAX_MS	~0U

/**
 * This struct wraps all the variables used internally to support a condition
 * variable implementation in Posix. The structure holds pthread_cond_t,
 * pthread_condattr_t and pthread_mutex_t objects which are used to provide the
 * desired functionality. A boolean flag to indicate the initialization status
 * of the struct is also maintained inside this structure. An user has to make
 * sure that the init function is invoked first for this struct before invoking
 * any other functions provided by this unit.
 */
struct nvgpu_cond {
	/**
	 * Indicates the initialization status of the condition variable.
	 */
	bool initialized;
	/**
	 * Mutex associated with the condition variable.
	 */
	struct nvgpu_mutex mutex;
	/**
	 * Underlying pthread condition variable.
	 */
	pthread_cond_t cond;
	/**
	 * Attributes associated with the condition variable.
	 */
	pthread_condattr_t attr;
};

#ifdef NVGPU_UNITTEST_FAULT_INJECTION_ENABLEMENT
struct nvgpu_posix_fault_inj *nvgpu_cond_get_fault_injection(void);
struct nvgpu_posix_fault_inj *nvgpu_cond_broadcast_get_fault_injection(void);
#endif

/**
 * @brief Timed wait for a condition variable.
 *
 * Waits for a condition variable for the time duration passed as param \a ms.
 * - If the value of \a ms is equal to #NVGPU_COND_WAIT_TIMEOUT_MAX_MS, invoke
 *   the function #pthread_cond_wait with \a cond in #nvgpu_cond and the mutex
 *   encapsulated inside #nvgpu_cond as parameters.
 * - For other values of \a ms, invoke the function #clock_gettime with
 *   \a CLOCK_MONOTONIC and a local variable of type \a timespec as parameters
 *   to fetch the clock value. Return \a -EFAULT if the function returns error.
 * - From the received \a timespec value, calculate the nanosecond equivalent
 *   value. Invoke #nvgpu_safe_mult_s64() with the received seconds value in
 *   \a timespec variable and a constant to convert into nanosecond and then
 *   invoke the function #nvgpu_safe_add_s64() with the calculated nanosecond
 *   value and the received nanosecond value in \a timespec variable to get the
 *   current nanosecond value.
 * - Convert the input parameter \a ms into nanosecond equivalent as above.
 * - Populate the \a timespec variable with the time value needed to wait for
 *   the condition variable.
 * - Invoke the function #pthread_cond_timedwait with \a cond in #nvgpu_cond,
 *   mutex associated with #nvpgu_cond and the \a timespec value calculated to
 *   wait for the condition variable.
 * - If #pthread_cond_timedwait returns 0, use the function #clock_gettime with
 *   \a CLOCK_MONOTONIC and a local variable of type \a timespec as parameters
 *   to fetch the current clock value. Use the current clock value to populate
 *   \a ms with the return value.
 * Return the value returned by #pthread_cond_timedwait.
 * Function does not perform any validation of the parameter.
 *
 * @param cond [in]	Condition variable to wait.
 * @param ms [in, out]	Timeout to wait as input and the time remaining as
 *			output.
 *
 * @return If successful, this function returns 0. Otherwise, an error number
 * is returned to indicate the error. The error number returned are either
 * generated by this function or generated by one of the OS APIs used
 * internally.
 *
 * @retval 0 for success.
 * @retval -EFAULT generated within this function if clock API used
 * internally by this function fails.
 * @retval EAGAIN insufficient memory resources available to wait on the
 * condition variable.
 * @retval EFAULT a fault occurred trying to access the buffers.
 * @retval EINVAL if one or more of the following is true:
 * 	- One or more of condition variable, mutex, time spec is invalid.
 * 	- Concurrent waits or timed waits on condition variable using
 * 	different mutexes.
 * 	- The mutex has died.
 * @retval EPERM the current thread doesn't own the mutex.
 * @retval ETIMEDOUT the time specified for wait has passed.
 */
int nvgpu_cond_timedwait(struct nvgpu_cond *c, unsigned int *ms);

/**
 * @brief Signal a condition variable.
 *
 * Wakes up a waiter for a condition variable to check if its condition has
 * been satisfied. This API has to be used after explicitly locking the mutex
 * associated with the condition variable. Internally invokes the function
 * #pthread_cond_signal with \a cond in #nvgpu_cond as parameter to signal.
 * If #pthread_cond_signal returns an error, #nvgpu_assert() function is
 * invoked.
 *
 * @param cond [in]	Condition variable to signal.
 * 			  - Should not be equal to NULL.
 * 			  - Structure pointed by \a cond should be initialized
 * 			    before invoking this function.
 */
void nvgpu_cond_signal_locked(struct nvgpu_cond *cond);

/**
 * @brief Signal all waiters of a condition variable.
 *
 * Wake up all waiters for a condition variable to check if their conditions
 * have been satisfied. This API has to be used after explicitly locking the
 * mutex associated with the condition variable. Internally invokes the function
 * #pthread_cond_broadcast with \a cond in #nvgpu_cond as parameter and returns
 * the return value as it is.
 *
 * @param cond [in]	Condition variable to broadcast.
 * 			  - Structure pointed by \a cond should be initialized
 * 			    before invoking this function.
 *
 * @return If successful a value of 0 shall be returned; otherwise, an error
 * number to indicate the error is returned.
 *
 * @retval 0 for success.
 * @retval -EINVAL if the condition variable is not initialized. This error is
 * generated within this function.
 * @retval EFAULT a fault occurred trying to access the buffers provided.
 * @retval EINVAL invalid condition variable. This error is generated by the OS
 * API used internally and returned as it is by this function.
 */
int nvgpu_cond_broadcast_locked(struct nvgpu_cond *cond);

/**
 * @brief Acquire the mutex associated with condition variable.
 *
 * Acquires the mutex associated with the condition variable referenced
 * by the param \a cond. Invokes the function #nvgpu_mutex_acquire() with
 * \a mutex in #nvgpu_cond as parameter to acquire the mutex. Function does not
 * perform any validation of the parameter.
 *
 * @param cond [in]	Condition variable for which the mutex has to be
 *			acquired. Structure pointed by \a cond has to be
 *			initialized before invoking this function.
 */
void nvgpu_cond_lock(struct nvgpu_cond *cond);

/**
 * @brief Release the mutex associated with condition variable.
 *
 * Releases the mutex associated with the condition variable referenced
 * by the param \a cond. Invokes the function #nvgpu_mutex_release() with
 * \a mutex in #nvgpu_cond as parameter to release the mutex. Function does
 * not perform any validation of the parameter.
 *
 * @param cond [in]	Condition variable for which the mutex has to be
 *			released.
 */
void nvgpu_cond_unlock(struct nvgpu_cond *cond);

/**
 * @brief Wait for a condition to be true.
 *
 * Wait for a condition to become true. Differentiates between timed wait
 * and infinite wait from the parameter \a timeout_ms. Need to acquire the
 * mutex associated with the condition variable before using this macro.
 * Invokes the macro #NVGPU_COND_WAIT_TIMEOUT_LOCKED internally with \a cond,
 * \a condition, a local variable to get the return value and a calculated
 * timeout value  based on \a timeout_ms as parameters. Does not perform any
 * validation of the parameters.
 *
 * @param cond [in]		The condition variable to sleep on.
 * @param condition [in]	The condition that needs to be checked.
 * @param timeout_ms [in]	Timeout in milliseconds or 0 for infinite wait.
 *
 * @return If successful, this macro returns 0. Otherwise, an error number
 * is returned to indicate the error.
 *
 * @retval 0 for success.
 * @retval -EFAULT if clock API used internally fails.
 * @retval EAGAIN insufficient memory resources available to wait on the
 * condition variable.
 * @retval EFAULT a fault occurred trying to access the buffers.
 * @retval EINVAL if one or more of the following is true:
 * 	- One or more of condition variable, mutex, time spec is invalid.
 * 	- Concurrent waits or timed waits on condition variable using
 * 	different mutexes.
 * 	- The mutex has died.
 * @retval EPERM the current thread doesn't own the mutex.
 * @retval ETIMEDOUT the time specified for wait has passed.
 */
#define NVGPU_COND_WAIT_LOCKED(cond, condition, timeout_ms)	\
({								\
	int ret = 0;						\
	u32 cond_timeout_ms = (timeout_ms);			\
	NVGPU_COND_WAIT_TIMEOUT_LOCKED((cond), (condition),	\
		(ret),						\
		((cond_timeout_ms) != 0U) ? (cond_timeout_ms) :	\
		NVGPU_COND_WAIT_TIMEOUT_MAX_MS);		\
	ret;							\
})

/**
 * @brief Initiate a wait for a condition variable.
 *
 * Wait for a condition to become true. Acquires the mutex associated with the
 * condition variable using the function #nvgpu_mutex_acquire() with \a mutex
 * in #nvgpu_cond as parameter before attempting to wait.
 * Invokes the macro #NVGPU_COND_WAIT_TIMEOUT_LOCKED internally with \a cond,
 * \a condition, a local variable to get the return value and a calculated
 * timeout value  based on \a timeout_ms as parameters. Releases the mutex
 * using the function #nvgpu_mutex_release() with \a mutex in #nvgpu_cond as
 * parameter after the wait. Does not perform any validation of the parameters.
 *
 * @param cond [in]		The condition variable to sleep on.
 * @param condition [in]	The condition that needs to be true.
 * @param timeout_ms [in]	Timeout in milliseconds or 0 for infinite wait.
 *
 * @return If successful, this macro returns 0. Otherwise, an error number
 * is returned to indicate the error.
 *
 * @retval 0 for success.
 * @retval -EFAULT if clock API used internally fails.
 * @retval EAGAIN insufficient memory resources available to wait on the
 * condition variable.
 * @retval EFAULT a fault occurred trying to access the buffers.
 * @retval EINVAL if one or more of the following is true:
 * 	- One or more of condition variable, mutex, time spec is invalid.
 * 	- Concurrent waits or timed waits on condition variable using
 * 	different mutexes.
 * 	- The mutex has died.
 * @retval EPERM the current thread doesn't own the mutex.
 * @retval ETIMEDOUT the time specified for wait has passed.
 */
#define NVGPU_COND_WAIT(cond, condition, timeout_ms)			\
({									\
	int cond_wait_ret = 0;						\
	u32 cond_wait_timeout = (timeout_ms);				\
	nvgpu_mutex_acquire(&(cond)->mutex);				\
	NVGPU_COND_WAIT_TIMEOUT_LOCKED((cond), (condition),		\
		(cond_wait_ret),					\
		(cond_wait_timeout != 0U) ?				\
			(cond_wait_timeout) :				\
			NVGPU_COND_WAIT_TIMEOUT_MAX_MS);		\
	nvgpu_mutex_release(&(cond)->mutex);				\
	cond_wait_ret;							\
})

/**
 * @brief Interruptible wait for a condition to be true.
 *
 * In Posix implementation the functionality of interruptible wait is same as
 * uninterruptible wait. Macro is defined to be congruent with implementations
 * which has interruptible and uninterruptible waits. Invokes the macro
 * #NVGPU_COND_WAIT internally.
 *
 * @param cond [in]		The condition variable to sleep on.
 * @param condition [in]	The condition that needs to be true.
 * @param timeout_ms [in]	Timeout in milliseconds or 0 for infinite wait.
 *
 * @return If successful, this macro returns 0. Otherwise, an error number
 * is returned to indicate the error.
 *
 * @retval 0 for success.
 * @retval -EFAULT if clock API used internally fails.
 * @retval EAGAIN insufficient memory resources available to wait on the
 * condition variable.
 * @retval EFAULT a fault occurred trying to access the buffers.
 * @retval EINVAL if one or more of the following is true:
 * 	- One or more of condition variable, mutex, time spec is invalid.
 * 	- Concurrent waits or timed waits on condition variable using
 * 	different mutexes.
 * 	- The mutex has died.
 * @retval EPERM the current thread doesn't own the mutex.
 * @retval ETIMEDOUT the time specified for wait has passed.
 */
#define NVGPU_COND_WAIT_INTERRUPTIBLE(cond, condition, timeout_ms) \
			NVGPU_COND_WAIT((cond), (condition), (timeout_ms))

/**
 * @brief Wait for a condition to be true.
 *
 * Wait for a condition to become true. Invokes the function
 * #nvgpu_cond_timedwait() internally with \a cond \a condition and a timeout
 * variable calculate from \a timeout_ms as parameters. Does not perform any
 * validation of the parameters.
 *
 * @param cond [in]		The condition variable to sleep on.
 * @param condition [in]	The condition that needs to be true.
 * @param ret [out]		Return value.
 * @param timeout_ms [in]	Timeout in milliseconds or 0 for infinite wait.
 *
 * @return If successful, this macro returns 0. Otherwise, an error number
 * is returned to indicate the error.
 *
 * @retval 0 for success.
 * @retval -EFAULT if clock API used internally fails.
 * @retval EAGAIN insufficient memory resources available to wait on the
 * condition variable.
 * @retval EFAULT a fault occurred trying to access the buffers.
 * @retval EINVAL if one or more of the following is true:
 * 	- One or more of condition variable, mutex, time spec is invalid.
 * 	- Concurrent waits or timed waits on condition variable using
 * 	different mutexes.
 * 	- The mutex has died.
 * @retval EPERM the current thread doesn't own the mutex.
 * @retval ETIMEDOUT the time specified for wait has passed.
 */
#define NVGPU_COND_WAIT_TIMEOUT_LOCKED(cond, condition, ret, timeout_ms)\
do {									\
	unsigned int cond_wait_timeout_timeout = (timeout_ms);		\
	ret = 0;							\
	while (!(condition) && ((ret) == 0)) {				\
		ret = nvgpu_cond_timedwait(cond,			\
				&cond_wait_timeout_timeout);		\
	}								\
NVGPU_COV_WHITELIST(false_positive, NVGPU_MISRA(Rule, 14_4), "Bug 2623654") \
} while (false)

#endif /* NVGPU_POSIX_COND_H */
