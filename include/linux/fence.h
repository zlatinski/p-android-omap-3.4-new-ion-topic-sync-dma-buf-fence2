/*
 * Fence mechanism for dma-buf to allow for asynchronous dma access
 *
 * Copyright (C) 2012 Canonical Ltd
 * Copyright (C) 2012 Texas Instruments
 *
 * Authors:
 * Rob Clark <rob.clark@linaro.org>
 * Maarten Lankhorst <maarten.lankhorst@canonical.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __FENCE_H__
#define __FENCE_H__

#include <linux/err.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/kref.h>
#include <linux/sched.h>

struct fence;
struct fence_ops;
struct fence_cb;

/**
 * struct fence - software synchronization primitive
 * @refcount: refcount for this fence
 * @ops: fence_ops associated with this fence
 * @cb_list: list of all callbacks to call
 * @priv: fence specific private data
 * @flags: A mask of FENCE_FLAG_* defined below
 *
 * FENCE_FLAG_NEED_SW_SIGNAL - enable_signaling has been called
 * FENCE_FLAG_SIGNALED - fence is already signaled
 */
struct fence {
	struct kref refcount;
	const struct fence_ops *ops;
	struct list_head cb_list;
	spinlock_t *lock;
	void *priv;
	unsigned long flags;
};
#define FENCE_FLAG_SIGNALED BIT(0)
#define FENCE_FLAG_NEED_SW_SIGNAL BIT(1)

typedef void (*fence_func_t)(struct fence *fence, struct fence_cb *cb, void *priv);

/**
 * struct fence_cb - callback for fence_add_callback
 * @func: fence_func_t to call
 * @priv: value of priv to pass to function
 *
 * This struct will be initialized by fence_add_callback, additional
 * data can be passed along by embedding fence_cb in another struct.
 */
struct fence_cb {
	struct list_head node;
	fence_func_t func;
	void *priv;
};

/**
 * struct fence_ops - operations implemented for fence
 * @enable_signaling: enable software signaling of fence
 * @signaled: [optional] peek whether the fence is signaled
 * @release: [optional] called on destruction of fence
 *
 * Notes on enable_signaling:
 * For fence implementations that have the capability for hw->hw
 * signaling, they can implement this op to enable the necessary
 * irqs, or insert commands into cmdstream, etc.  This is called
 * in the first wait() or add_callback() path to let the fence
 * implementation know that there is another driver waiting on
 * the signal (ie. hw->sw case).
 *
 * This function can be called called from atomic context, but not
 * from irq context, so normal spinlocks can be used.
 *
 * A return value of false indicates the fence already passed,
 * or some failure occured that made it impossible to enable
 * signaling. True indicates succesful enabling.
 *
 * Calling fence_signal before enable_signaling is called allows
 * for a tiny race window in which enable_signaling is called during,
 * before, or after fence_signal. To fight this, it is recommended
 * that before enable_signaling returns true an extra reference is
 * taken on the fence, to be released when the fence is signaled.
 * This will mean fence_signal will still be called twice, but
 * the second time will be a noop since it was already signaled.
 *
 * Notes on release:
 * Can be NULL, this function allows additional commands to run on
 * destruction of the fence. Can be called from irq context.
 */

struct fence_ops {
	bool (*enable_signaling)(struct fence *fence);
	bool (*signaled)(struct fence *fence);
	long (*wait)(struct fence *fence, bool intr, signed long);
	void (*release)(struct fence *fence);
};

/**
 * __fence_init - Initialize a custom fence.
 * @fence:	[in]	the fence to initialize
 * @ops:	[in]	the fence_ops for operations on this fence
 * @lock:	[in]	the irqsafe spinlock to use for locking this fence
 * @priv:	[in]	the value to use for the priv member
 *
 * Initializes an allocated fence, the caller doesn't have to keep its
 * refcount after committing with this fence, but it will need to hold a
 * refcount again if fence_ops.enable_signaling gets called. This can
 * be used for other implementing other types of fence.
 */
static inline void
__fence_init(struct fence *fence, const struct fence_ops *ops,
	     spinlock_t *lock, void *priv)
{
	BUG_ON(!ops || !lock || !ops->enable_signaling || !ops->wait);

	kref_init(&fence->refcount);
	fence->ops = ops;
	fence->priv = priv;
	fence->flags = 0UL;
	fence->lock = lock;
	INIT_LIST_HEAD(&fence->cb_list);
}

void fence_get(struct fence *fence);
void fence_put(struct fence *fence);

int fence_signal(struct fence *fence);
long fence_default_wait(struct fence *fence, bool intr, signed long);
int fence_add_callback(struct fence *fence, struct fence_cb *cb,
		       fence_func_t func, void *priv);
bool fence_remove_callback(struct fence *fence, struct fence_cb *cb);
void fence_enable_sw_signaling(struct fence *fence);

/**
 * fence_is_signaled - Return an indication if the fence is signaled yet.
 * @fence:	[in]	the fence to check
 *
 * Returns true if the fence was already signaled, false if not. Since this
 * function doesn't enable signaling, it is not guaranteed to ever return true
 * If fence_add_callback, fence_wait or fence_enable_sw_signaling
 * haven't been called before.
 *
 * It's recommended for seqno fences to call fence_signal when the
 * operation is complete, it makes it possible to prevent issues from
 * wraparound between time of issue and time of use by checking the return
 * value of this function before calling hardware-specific wait instructions.
 */
static inline bool
fence_is_signaled(struct fence *fence)
{
	rmb();

	if (fence->flags & FENCE_FLAG_SIGNALED)
		return true;

	if (fence->ops->signaled && fence->ops->signaled(fence)) {
		fence_signal(fence);
		return true;
	}

	return false;
}

/**
 * fence_wait_timeout - sleep until the fence gets signaled
 * or until timeout elapses
 * @fence:	[in]	the fence to wait on
 * @intr:	[in]	if true, do an interruptible wait
 * @timeout:	[in]	timeout value in jiffies, or MAX_SCHEDULE_TIMEOUT
 *
 * Returns -ERESTARTSYS if interrupted, 0 if the wait timed out, or the
 * remaining timeout in jiffies on success. Other error values may be
 * returned on custom implementations.
 *
 * Performs a synchronous wait on this fence. It is assumed the caller
 * directly or indirectly (buf-mgr between reservation and committing)
 * holds a reference to the fence, otherwise the fence might be
 * freed before return, resulting in undefined behavior.
 */
static inline long
fence_wait_timeout(struct fence *fence, bool intr, signed long timeout)
{
	if (WARN_ON(timeout < 0))
		return -EINVAL;

	return fence->ops->wait(fence, intr, timeout);
}

/**
 * fence_wait - sleep until the fence gets signaled
 * @fence:	[in]	the fence to wait on
 * @intr:	[in]	if true, do an interruptible wait
 *
 * This function will return -ERESTARTSYS if interrupted by a signal,
 * or 0 if the fence was signaled. Other error values may be
 * returned on custom implementations.
 *
 * Performs a synchronous wait on this fence. It is assumed the caller
 * directly or indirectly (buf-mgr between reservation and committing)
 * holds a reference to the fence, otherwise the fence might be
 * freed before return, resulting in undefined behavior.
 */
static inline long fence_wait(struct fence *fence, bool intr)
{
	long ret;

	/* Since fence_wait_timeout cannot timeout with
	 * MAX_SCHEDULE_TIMEOUT, only valid return values are
	 * -ERESTARTSYS and MAX_SCHEDULE_TIMEOUT.
	 */
	ret = fence_wait_timeout(fence, intr, MAX_SCHEDULE_TIMEOUT);

	return ret < 0 ? ret : 0;
}

#endif /* __FENCE_H__ */
