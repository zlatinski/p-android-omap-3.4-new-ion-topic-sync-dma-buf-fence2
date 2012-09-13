/*
 * Header file for reservations for dma-buf and ttm
 *
 * Copyright(C) 2011 Linaro Limited. All rights reserved.
 * Copyright (C) 2012 Canonical Ltd
 * Copyright (C) 2012 Texas Instruments
 *
 * Authors:
 * Rob Clark <rob.clark@linaro.org>
 * Maarten Lankhorst <maarten.lankhorst@canonical.com>
 * Thomas Hellstrom <thellstrom-at-vmware-dot-com>
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
 */
#ifndef __RESERVATION_H__
#define __RESERVATION_H__

#define BUF_MAX_SHARED_FENCE 8

#include <linux/fence.h>

extern atomic64_t reservation_counter;
extern const char reservation_object_name[];
extern struct lock_class_key reservation_object_class;
extern const char reservation_ticket_name[];
extern struct lock_class_key reservation_ticket_class;

struct reservation_object {
	wait_queue_head_t event_queue;

	atomic64_t reserved;

	u32 fence_shared_count;
	struct fence *fence_excl;
	struct fence *fence_shared[BUF_MAX_SHARED_FENCE];

#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map dep_map;
#endif
};

typedef struct reservation_ticket {
	u64 seqno;
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map dep_map;
#endif
} reservation_ticket_t;

/**
 * struct reservation_entry - reservation structure for a
 * reservation_object
 * @head:	list entry
 * @obj_shared:	pointer to a reservation_object to reserve
 *
 * Bit 0 of obj_shared is set to bool shared, as such pointer has to be
 * converted back, which can be done with reservation_entry_get.
 */
struct reservation_entry {
	struct list_head head;
	unsigned long obj_shared;
};

static inline void
__reservation_object_init(struct reservation_object *obj)
{
	init_waitqueue_head(&obj->event_queue);

	lockdep_init_map(&obj->dep_map, reservation_object_name,
			 &reservation_object_class, 0);
}

static inline void
reservation_object_init(struct reservation_object *obj)
{
	memset(obj, 0, sizeof(*obj));
	__reservation_object_init(obj);
}

static inline bool
object_is_reserved(struct reservation_object *obj)
{
	return !!atomic64_read(&obj->reserved);
}

static inline void
reservation_object_fini(struct reservation_object *obj)
{
	int i;

	BUG_ON(waitqueue_active(&obj->event_queue));
	BUG_ON(object_is_reserved(obj));

	if (obj->fence_excl)
		fence_put(obj->fence_excl);
	for (i = 0; i < obj->fence_shared_count; ++i)
		fence_put(obj->fence_shared[i]);
}

static inline void
reservation_ticket_init(struct reservation_ticket *t)
{
#ifdef CONFIG_LOCKDEP
	/*
	 * Make sure we are not reinitializing a held ticket:
	 */

	debug_check_no_locks_freed((void *)t, sizeof(*t));
#endif
	lockdep_init_map(&t->dep_map, reservation_ticket_name,
			 &reservation_ticket_class, 0);
	mutex_acquire(&t->dep_map, 0, 0, _RET_IP_);
	do {
		t->seqno = atomic64_inc_return(&reservation_counter);
	} while (unlikely(t->seqno < 2));
}

/**
 * reservation_ticket_fini - end a reservation ticket
 * @t:	[in]	reservation_ticket that completed all reservations
 *
 * This currently does nothing, but should be called after all reservations
 * made with this ticket have been unreserved. It is likely that in the future
 * it will be hooked up to perf events, or aid in debugging in other ways.
 */
static inline void
reservation_ticket_fini(struct reservation_ticket *t)
{
	mutex_release(&t->dep_map, 1, _RET_IP_);
}

/**
 * reservation_entry_init - initialize and append a reservation_entry
 * to the list
 * @entry:	entry to initialize
 * @list:	list to append to
 * @obj:	reservation_object to initialize the entry with
 * @shared:	whether shared or exclusive access is requested
 */
static inline void
reservation_entry_init(struct reservation_entry *entry,
			   struct list_head *list,
			   struct reservation_object *obj, bool shared)
{
	entry->obj_shared = (unsigned long)obj | !!shared;
}

static inline struct reservation_entry *
reservation_entry_get(struct list_head *list,
			  struct reservation_object **obj, bool *shared)
{
	struct reservation_entry *e = container_of(list, struct reservation_entry, head);
	unsigned long val = e->obj_shared;

	if (obj)
		*obj = (struct reservation_object*)(val & ~1);
	if (shared)
		*shared = val & 1;
	return e;
}

extern int
object_reserve(struct reservation_object *obj,
			       bool intr, bool no_wait,
			       reservation_ticket_t *ticket);

extern void
object_unreserve(struct reservation_object *,
				 reservation_ticket_t *ticket);

extern int
object_wait_unreserved(struct reservation_object *, bool intr);

extern int ticket_reserve(struct reservation_ticket *,
					  struct list_head *entries);
extern void ticket_backoff(struct reservation_ticket *,
			       struct list_head *entries);
extern void ticket_commit(struct reservation_ticket *,
			      struct list_head *entries, struct fence *);

#endif /* __BUF_MGR_H__ */
