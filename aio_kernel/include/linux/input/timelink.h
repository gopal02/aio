#ifndef __TIMELINK_H__
#define __TIMELINK_H__

#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38))
#   include <linux/input.h>
#else
#   include <linux/input/mt.h>
#endif

#define TIMELINK_NOT_SEEN_MEANS_UP

/* It is recommended to use MT protocol B than MT protocol A, because
 * MT protocol B is more effective. MT protocol B is available since kernel
 * version 2.6.36, but some APIs for it is added since 2.6.38 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38))
#   define TIMELINK_USE_MT_PROTOCOL_B
#else
#   define TIMELINK_USE_MT_PROTOCOL_A
#endif

#define timelink_error(fmt, ...) \
    printk(KERN_ERR "timelink: %s: "fmt"\n", __func__, ##__VA_ARGS__)

#define timelink_info(fmt, ...) \
    printk(KERN_INFO "timelink: %s: "fmt"\n", __func__, ##__VA_ARGS__)

#if defined(DEBUG)
#define timelink_debug(fmt, ...) \
    printk(KERN_DEBUG "timelink: %s: "fmt"\n", __func__, ##__VA_ARGS__)
#else
#define timelink_debug(fmt, ...)
#endif

struct timelink_contact {
    bool state;
    unsigned char id;
    unsigned short x;
    unsigned short y;
    unsigned short width;
    unsigned short height;
    unsigned char p;
};

struct timelink_slot {
    struct timelink_contact contact; 
    bool state;
};

struct timelink_range {
    ssize_t min;
    ssize_t max;
};
struct timelink_feature {
    struct timelink_range x_logical;
    struct timelink_range y_logical;
    struct timelink_range width_logical;
    struct timelink_range height_logical;
    size_t max_contact_count;
};

struct timelink_device {
    struct timelink_feature feature;
    struct timelink_slot* slots;
    struct timelink_slot incoming_slot;
    struct timelink_slot* received_slots;
    size_t received_contact_count;
    size_t expected_contact_count;
    int pointer_index;
    bool idle;
};


static bool timelink_alloc(struct timelink_device* tp)
{ 
    bool result = false;
    size_t slots_count = 0;
    size_t slots_size = 0;

    if (!tp || !tp->feature.max_contact_count) {
        timelink_debug("invalid arguments: tp=%p", tp);
        return result;
    }

    slots_count = tp->feature.max_contact_count;
    slots_size = sizeof(struct timelink_slot);
    tp->received_slots = kzalloc(slots_size * slots_count, GFP_KERNEL);
    if (!tp->received_slots) {
        timelink_error("cannot allocate received_slots[%zu]", slots_count);
        goto err_free_received_slots;
    }
    tp->slots = kzalloc(slots_size * slots_count, GFP_KERNEL);
    if (!tp->slots) {
        timelink_error("cannot allocate slots[%zu]", slots_count);
        goto err;
    }
    tp->idle = true;
    return true;

err_free_received_slots:
    kfree(tp->received_slots);
    tp->received_slots = NULL;
err:
    return result;
} 

static void timelink_free(struct timelink_device* tp)
{ 
    if (!tp)
        return;

    kfree(tp->received_slots);
    kfree(tp->slots);
}

static void timelink_complete_slot(struct timelink_device* tp)
{
    if (tp->received_slots == NULL)
        return;

    if (tp->incoming_slot.state) {
        int slotnum = tp->received_contact_count;
        if (slotnum >= 0 && slotnum < tp->feature.max_contact_count) {
            tp->received_slots[slotnum] = tp->incoming_slot;
            timelink_debug("complete: %d", slotnum);
        }
    }
}

static void timelink_complete_slots(struct timelink_device* tp)
{
    size_t i, count = tp->received_contact_count;

    if (tp->received_slots == NULL || tp->slots == NULL)
        return;

    /* Some legacy data might be received with the firmwares before v4.2,
     * this is going to fix that. */
    count = min(count, tp->expected_contact_count);

    for (i = 0; i < count; ++i) {
        size_t slotnum = tp->received_slots[i].contact.id % tp->feature.max_contact_count;
        tp->slots[slotnum] = tp->received_slots[i];
    }
    timelink_debug("complete all(%zu)", count);
}

static void timelink_clean_slots(struct timelink_device* tp)
{
    size_t i;

    if (tp->slots == NULL)
        return;

    for (i = 0; i < tp->feature.max_contact_count; ++i) {
        tp->slots[i].state = false;
#ifdef TIMELINK_NOT_SEEN_MEANS_UP
        tp->slots[i].contact.state = false;
#endif
    }
}

#define SCREEN_MAX_HEIGHT		1600
#define SCREEN_MAX_WIDTH		900
#define DEFAULT_X_LOGICAL_MAX	7680
#define DEFAULT_Y_LOGICAL_MAX	4320

static void timelink_emit_contact_detail(struct input_dev* input, struct timelink_contact* contact)
{
    int orient, major, minor;

    orient = (contact->width > contact->height);
    major = max(contact->width, contact->height);
    minor = min(contact->width, contact->height);

	//liuwei mark this
	//contact->x = contact->x * SCREEN_MAX_HEIGHT / DEFAULT_X_LOGICAL_MAX;
	//contact->y = contact->y * SCREEN_MAX_WIDTH / DEFAULT_Y_LOGICAL_MAX;

    input_event(input, EV_ABS, ABS_MT_POSITION_X, contact->x);
    input_event(input, EV_ABS, ABS_MT_POSITION_Y, contact->y);
    input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, major);
    input_event(input, EV_ABS, ABS_MT_TOUCH_MINOR, minor);
    input_event(input, EV_ABS, ABS_MT_ORIENTATION, orient);
    timelink_debug("x=%d y=%d m=%d n=%d o=%d", contact->x, contact->y, major, minor, orient);
}

#ifndef TIMELINK_USE_MT_PROTOCOL_B
static int timelink_find_first_active_slot(const struct timelink_slot* slots, size_t count)
{
    int i;

    for (i = 0; i < count; ++i) {
        if (slots[i].state && slots[i].contact.state)
            return i;
    }
    return -1;
};

static void timelink_pointer_emulation(struct timelink_device* tp, struct input_dev* input)
{
    int i = timelink_find_first_active_slot(tp->slots, tp->max_contact_count);
    if (i >= 0) {
        if (tp->idle) {
            input_event(input, EV_KEY, BTN_TOUCH, 1);
            tp->idle = false;
            tp->pointer_index = i;
        }
        if (tp->slots[tp->pointer_index].state
            && tp->slots[tp->pointer_index].contact.state) {
            input_event(input, EV_ABS, ABS_X, tp->slots[tp->pointer_index].contact.x);
            input_event(input, EV_ABS, ABS_Y, tp->slots[tp->pointer_index].contact.y);
        }
    }
    else {
        if (!tp->idle) {
            input_event(input, EV_KEY, BTN_TOUCH, 0);
            tp->idle = true;
            tp->pointer_index = -1;
        }
    }
}

static void timelink_emit_event_a(struct timelink_device* tp, struct input_dev* input)
{
    size_t i;

    timelink_debug("");
    for (i = 0; i < tp->max_contact_count; ++i) {
        struct timelink_slot* s = tp->slots + i;

        if (!s->state)
            continue;

        /* In order to avoid reporting wrong details of disappearing contact,
         * details will be ignored. */
        if (s->contact.state) {
            input_event(input, EV_ABS, ABS_MT_TRACKING_ID, s->contact.id);
            timelink_emit_contact_detail(input, &s->contact);
            input_mt_sync(input);
        }
    }
    timelink_pointer_emulation(tp, input);
}

#else /* #ifndef TIMELINK_USE_MT_PROTOCOL_B */

static void timelink_emit_event_b(struct timelink_device* tp, struct input_dev* input)
{
    size_t i;

    timelink_debug("");
    for (i = 0; i < tp->feature.max_contact_count; ++i) {
        struct timelink_slot* s = tp->slots + i;

        input_mt_slot(input, i);
        input_mt_report_slot_state(input, MT_TOOL_FINGER, s->contact.state);
        /* In order to avoid reporting wrong details of disappearing contact,
         * details will be ignored. */
        if (s->contact.state) {
            timelink_emit_contact_detail(input, &s->contact);
        }
    }
    input_mt_report_pointer_emulation(input, true);
}
#endif /* #ifndef TIMELINK_USE_MT_PROTOCOL_B */

static void timelink_emit_event(struct timelink_device* tp, struct input_dev* input)
{
    if (tp == NULL || input == NULL || tp->slots == NULL)
        return;

#ifdef TIMELINK_USE_MT_PROTOCOL_B
    timelink_emit_event_b(tp, input);
#else
    timelink_emit_event_a(tp, input);
#endif

    input_sync(input);
}

#endif

