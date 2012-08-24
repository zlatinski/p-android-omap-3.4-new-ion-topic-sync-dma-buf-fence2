/*
 * Copyright (C) 2012 Canonical Ltd
 *
 * Based on bo.c which bears the following copyright notice,
 * but is dual licensed:
 *
 * Copyright (c) 2006-2009 VMware, Inc., Palo Alto, CA., USA
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 **************************************************************************/
/*
 * Authors: Thomas Hellstrom <thellstrom-at-vmware-dot-com>
 */

#include <linux/fence.h>
#include <linux/reservation.h>
#include <linux/export.h>
#include <linux/sched.h>
#include <linux/slab.h>

atomic64_t reservation_counter = ATOMIC64_INIT(1);
EXPORT_SYMBOL(reservation_counter);

int
object_reserve(struct reservation_object *obj, bool intr, bool no_wait,
	       reservation_ticket_t *ticket)
{
	int ret;
	u64 sequence = ticket ? ticket->seqno : 1;
	u64 oldseq;

	while (unlikely(oldseq = atomic64_cmpxchg(&obj->reserved, 0, sequence))) {

		/**
		 * Deadlock avoidance for multi-obj reserving.
		 */
		if (sequence > 1 && oldseq > 1) {
			/**
			 * We've already reserved this one.
			 */
			if (unlikely(sequence == oldseq))
				return -EDEADLK;
			/**
			 * Already reserved by a thread that will not back
			 * off for us. We need to back off.
			 */
			if (unlikely(sequence - oldseq < (1ULL << 63)))
				return -EAGAIN;
		}

		if (no_wait)
			return -EBUSY;

		ret = object_wait_unreserved(obj, intr);

		if (unlikely(ret))
			return ret;
	}

	/**
	 * Wake up waiters that may need to recheck for deadlock,
	 * if we decreased the sequence number.
	 */
	wake_up_all(&obj->event_queue);

	return 0;
}
EXPORT_SYMBOL(object_reserve);

int
object_wait_unreserved(struct reservation_object *obj, bool intr)
{
	if (intr) {
		return wait_event_interruptible(obj->event_queue,
				!object_is_reserved(obj));
	} else {
		wait_event(obj->event_queue,
			   !object_is_reserved(obj));
		return 0;
	}
}
EXPORT_SYMBOL(object_wait_unreserved);

void
object_unreserve(struct reservation_object *obj,
		 reservation_ticket_t *ticket)
{
	smp_mb();
	atomic64_set(&obj->reserved, 0);
	wake_up_all(&obj->event_queue);
}
EXPORT_SYMBOL(object_unreserve);

/**
 * ticket_backoff - cancel a reservation
 * @ticket:	[in] a reservation_ticket
 * @entries:	[in] the list list of reservation_entry entries to unreserve
 *
 * This function cancels a previous reservation done by
 * ticket_reserve. This is useful in case something
 * goes wrong between reservation and committing.
 *
 * This should only be called after ticket_reserve returns success.
 */
void
ticket_backoff(struct reservation_ticket *ticket, struct list_head *entries)
{
	struct list_head *cur;

	if (list_empty(entries))
		return;

	list_for_each(cur, entries) {
		struct reservation_object *obj;

		reservation_entry_get(cur, &obj, NULL);

		object_unreserve(obj, ticket);
	}
	reservation_ticket_fini(ticket);
}
EXPORT_SYMBOL(ticket_backoff);

static void
ticket_backoff_early(struct reservation_ticket *ticket,
			 struct list_head *list,
			 struct reservation_entry *entry)
{
	list_for_each_entry_continue_reverse(entry, list, head) {
		struct reservation_object *obj;

		reservation_entry_get(&entry->head, &obj, NULL);
		object_unreserve(obj, ticket);
	}
	reservation_ticket_fini(ticket);
}

/**
 * ticket_reserve - reserve a list of reservation_entry
 * @ticket:	[out]	a reservation_ticket
 * @entries:	[in]	a list of entries to reserve.
 *
 * Do not initialize ticket, it will be initialized by this function.
 *
 * XXX: Nuke rest
 * The caller will have to queue waits on those fences before calling
 * ufmgr_fence_buffer_objects, with either hardware specific methods,
 * fence_add_callback will, or fence_wait.
 *
 * As such, by incrementing refcount on reservation_entry before calling
 * fence_add_callback, and making the callback decrement refcount on
 * reservation_entry, or releasing refcount if fence_add_callback
 * failed, the reservation_entry will be freed when all the fences
 * have been signaled, and only after the last ref is released, which should
 * be after ufmgr_fence_buffer_objects. With proper locking, when the
 * list_head holding the list of reservation_entry's becomes empty it
 * indicates all fences for all bufs have been signaled.
 */
int
ticket_reserve(struct reservation_ticket *ticket,
		   struct list_head *entries)
{
	struct list_head *cur;
	int ret;

	if (list_empty(entries))
		return 0;

retry:
	reservation_ticket_init(ticket);

	list_for_each(cur, entries) {
		struct reservation_entry *entry;
		struct reservation_object *bo;
		bool shared;

		entry = reservation_entry_get(cur, &bo, &shared);

		ret = object_reserve(bo, true, false, ticket);
		switch (ret) {
		case 0:
			break;
		case -EAGAIN:
			ticket_backoff_early(ticket, entries, entry);
			ret = object_wait_unreserved(bo, true);
			if (unlikely(ret != 0))
				return ret;
			goto retry;
		default:
			ticket_backoff_early(ticket, entries, entry);
			return ret;
		}

		if (shared &&
		    bo->fence_shared_count == BUF_MAX_SHARED_FENCE) {
			WARN_ON_ONCE(1);
			ticket_backoff_early(ticket, entries, entry);
			return -EINVAL;
		}
	}

	return 0;
}
EXPORT_SYMBOL(ticket_reserve);

/**
 * ticket_commit - commit a reservation with a new fence
 * @ticket:	[in]	the reservation_ticket returned by
 * ticket_reserve
 * @entries:	[in]	a linked list of struct reservation_entry
 * @fence:	[in]	the fence that indicates completion
 *
 * This function will call reservation_ticket_fini, no need
 * to do it manually.
 *
 * This function should be called after a hardware command submission is
 * completed succesfully. The fence is used to indicate completion of
 * those commands.
 */
void
ticket_commit(struct reservation_ticket *ticket,
		  struct list_head *entries, struct fence *fence)
{
	struct list_head *cur;

	if (list_empty(entries))
		return;

	if (WARN_ON(!fence)) {
		ticket_backoff(ticket, entries);
		return;
	}

	list_for_each(cur, entries) {
		struct reservation_object *bo;
		bool shared;

		reservation_entry_get(cur, &bo, &shared);

		if (!shared) {
			int i;
			for (i = 0; i < bo->fence_shared_count; ++i) {
				fence_put(bo->fence_shared[i]);
				bo->fence_shared[i] = NULL;
			}
			bo->fence_shared_count = 0;
			if (bo->fence_excl)
				fence_put(bo->fence_excl);

			bo->fence_excl = fence;
		} else {
			if (WARN_ON(bo->fence_shared_count >=
				    ARRAY_SIZE(bo->fence_shared))) {
				continue;
			}

			bo->fence_shared[bo->fence_shared_count++] = fence;
		}
		fence_get(fence);

		object_unreserve(bo, ticket);
	}
	reservation_ticket_fini(ticket);
}
EXPORT_SYMBOL(ticket_commit);
