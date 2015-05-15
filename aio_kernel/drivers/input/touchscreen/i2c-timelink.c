/*
 * I2c driver for the TimeLink TouchWin MultiTouch Panel, based on many sources.
 * Copyright (c) 2013 ShenZhen TimeLink Inc.
 *
 * This program is free software; You can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; Either version 2 of the License or (at your option) any
 * later version.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/input/timelink.h>
#include <linux/input/timelink-sysfs.h>
#include <linux/input/i2c-timelink.h>

#define I2C_TIMELINK_USE_DEFAULT_PALFORM_DATA
#define I2C_TIMELINK_POLLING_MODE

#define i2c_timelink_error(fmt, ...) \
    printk(KERN_ERR "i2c-timelink: %s: "fmt"\n", __func__, ##__VA_ARGS__)

#define i2c_timelink_info(fmt, ...) \
    printk(KERN_INFO "i2c-timelink: %s: "fmt"\n", __func__, ##__VA_ARGS__)

#if defined(DEBUG)
#define i2c_timelink_debug(fmt, ...) \
    printk(KERN_DEBUG "i2c-timelink: %s: "fmt"\n", __func__, ##__VA_ARGS__)
#else
#define i2c_timelink_debug(fmt, ...)
#endif

#define DEFAULT_INTERVAL        10
#define DEFAULT_MAX_CONTACTS    2
#define DEFAULT_X_LOGICAL_MIN   0
#define DEFAULT_X_LOGICAL_MAX   7680
#define DEFAULT_Y_LOGICAL_MIN   0
#define DEFAULT_Y_LOGICAL_MAX   4320
#define DEFAULT_W_LOGICAL_MIN   0
#define DEFAULT_W_LOGICAL_MAX   7680
#define DEFAULT_H_LOGICAL_MIN   0
#define DEFAULT_H_LOGICAL_MAX   4320

#define I2C_TIMELINK_REPORT_TYPE_TOUCH      0x01
#define I2C_TIMELINK_REPORT_TYPE_SPECIFICS  0x11
#define I2C_TIMELINK_REPORT_TYPE_CALIBRATE  0x12
#define I2C_TIMELINK_REPORT_TYPE_DIAGNOSE   0x13
#define I2C_TIMELINK_REPORT_TYPE_DIAGNOSE_CS   0x14
#define I2C_TIMELINK_REPORT_TYPE_BOOTLOADER_MODE    0xf0
#define I2C_TIMELINK_REPORT_TYPE_BOOTLOADER_ADDR    0xf1
#define I2C_TIMELINK_REPORT_TYPE_BOOTLOADER_CODE    0xf2
#define I2C_TIMELINK_REPORT_TYPE_BOOTLOADER_ERASE   0xf3
#define I2C_TIMELINK_REPORT_TYPE_EXTEND     0xff

#define I2C_TIMELINK_BOOTLOADER_MODE_BOOT               0xc1
#define I2C_TIMELINK_BOOTLOADER_MODE_UPGRADE            0xc2
#define I2C_TIMELINK_BOOTLOADER_MODE_UPGRADE_ONESHOT    0xc3
#define I2C_TIMELINK_BOOTLOADER_MODE_BOOT_ONESHOT       0xc4


MODULE_AUTHOR("Canmor Lam < canmor.lam@gmail.com >");
MODULE_DESCRIPTION ("TimeLink TouchWin I2C Multitouch Panel");
MODULE_LICENSE("GPL");


struct i2c_timelink_device {
    struct i2c_client* client;
    struct input_dev* input;
    struct timelink_device touch;
    bool stopped;
    wait_queue_head_t wait_for_stop;
    wait_queue_head_t wait_for_start;
#if defined(I2C_TIMELINK_POLLING_MODE)
    /* Use a timer and a wait queue to simulate irq. */
    size_t interval;
    struct timer_list timer;
    wait_queue_head_t wait_for_input;
    bool has_input;
    struct task_struct* working_thread;
    size_t continuously_error;
#else
    int irq;
#endif
    u16 version;
    char phys[32];
    struct timelink_spec spec;
    struct timelink_calibrate_sysfs calibrate_sysfs;
    struct timelink_spec_sysfs spec_sysfs;
    struct timelink_diagnose_sysfs diagnose_sysfs;
    struct timelink_bootloader_sysfs bootloader_sysfs;
    struct mutex mutex;
    uint32_t programming_base_addr;
    bool programming;
};

struct i2c_timelink_platform_data the_default_platform_data = {
    .version = 0,
    .interval = DEFAULT_INTERVAL,
    .max_contacts = DEFAULT_MAX_CONTACTS,
    .x_logical_min = DEFAULT_X_LOGICAL_MIN,
    .x_logical_max = DEFAULT_X_LOGICAL_MAX,
    .y_logical_min = DEFAULT_Y_LOGICAL_MIN,
    .y_logical_max = DEFAULT_Y_LOGICAL_MAX,
    .width_logical_min = DEFAULT_W_LOGICAL_MIN,
    .width_logical_max = DEFAULT_W_LOGICAL_MAX,
    .height_logical_min = DEFAULT_H_LOGICAL_MIN,
    .height_logical_max = DEFAULT_H_LOGICAL_MAX
};

static dev_t i2c_timelink_major;
static struct cdev* i2c_timelink_cdev;
static struct device* i2c_timelink_class_dev;
static const struct file_operations i2c_timelink_fops = {
    .owner = THIS_MODULE,
};


static int i2c_timelink_alloc_major(void)
{
    static const char name[] = "i2c-timelink";
    struct cdev *cdev = NULL;
    dev_t i2c_dev = 0;
    int result;
    i2c_timelink_debug("...");

    result = alloc_chrdev_region(&i2c_dev, 0, 1, name);
    if (result) {
        i2c_timelink_error("can not alloc chrdev region, ret=%d", result);
        goto out;
    }

    result = -ENOMEM;
    cdev = cdev_alloc();
    if (!cdev) {
        i2c_timelink_error("can not alloc cdev");
        goto out_unregister;
    }

    cdev->owner = THIS_MODULE;
    cdev->ops = &i2c_timelink_fops;
    kobject_set_name(&cdev->kobj, "%s", name);

    result = cdev_add(cdev, i2c_dev, 1);
    if (result) {
        i2c_timelink_error("can not add cdev, ret=%d", result);
        goto out_put;
    }

    i2c_timelink_major = MAJOR(i2c_dev);
    i2c_timelink_cdev = cdev;
    return 0;
out_put:
    kobject_put(&cdev->kobj);
out_unregister:
    unregister_chrdev_region(i2c_timelink_major, 1);
out:
    return result;
}

static void i2c_timelink_free_major(void)
{
    i2c_timelink_debug("...");
    unregister_chrdev_region(MKDEV(i2c_timelink_major, 0), 1);
    cdev_del(i2c_timelink_cdev);
}

static struct class i2c_timelink_class = {
    .name = "i2c-timelink",
};

static int i2c_timelink_register_class(void)
{
    int ret;
    i2c_timelink_debug("...");

    /* This is the first time in here, set everything up properly */
    ret = i2c_timelink_alloc_major();
    if (ret) {
        i2c_timelink_error("can not alloc major");
        goto exit;
    }

    ret = class_register(&i2c_timelink_class);
    if (ret) {
        i2c_timelink_error("class_register failed");
        goto err_class_register;
    }
    return 0;

err_class_register:
    i2c_timelink_free_major();
exit:
    return ret;
};

static void i2c_timelink_unregister_class(void)
{
    class_unregister(&i2c_timelink_class);
    i2c_timelink_free_major();
}

static struct device* i2c_timelink_create_class_device(void)
{
    i2c_timelink_debug("...");
    /* /sys/class/class_name/touchscreen */
    return device_create(&i2c_timelink_class, NULL, MKDEV(i2c_timelink_major, 0), NULL,
        "touchscreen%d", 0);
}

static void i2c_timelink_destroy_class_device(void)
{
    device_destroy(&i2c_timelink_class, MKDEV(i2c_timelink_major, 0));
}

static void i2c_timelink_set_feature(struct timelink_feature* feature,
    struct i2c_timelink_platform_data* pdata)
{
    feature->max_contact_count = pdata->max_contacts;
    feature->x_logical.min = pdata->x_logical_min;
    feature->x_logical.max = pdata->x_logical_max;
    feature->y_logical.min = pdata->y_logical_min;
    feature->y_logical.max = pdata->y_logical_max;
    feature->width_logical.min = pdata->width_logical_min;
    feature->width_logical.max = pdata->width_logical_max;
    feature->height_logical.min = pdata->height_logical_min;
    feature->height_logical.max = pdata->height_logical_max;
}

static void i2c_timelink_emit_event(struct i2c_timelink_device* device, struct input_dev* input)
{
    if (device == NULL || input == NULL)
        return;

    timelink_emit_event(&device->touch, input);
}

/*
 * Data format:
 * 0               8               16              24              32
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | status        | id            | x                             |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | y                             | width                         |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | height                        | ...
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
#define I2C_TIMELINK_REPORT_TOUCH_SIZE  0x0a
static bool timelink_parse_contact(struct timelink_contact* contact, const __u8* buf, size_t size)
{
    if (!buf || size < I2C_TIMELINK_REPORT_TOUCH_SIZE) {
        i2c_timelink_debug("invalid buffer, buf=%p size=%zu", buf, size);
        return false;
    }

    contact->state = true;
    contact->id = *(buf + 1);
    contact->x = le16_to_cpup((__le16*)(buf + 2));
    contact->y = le16_to_cpup((__le16*)(buf + 4));
    contact->width = le16_to_cpup((__le16*)(buf + 6));
    contact->height = le16_to_cpup((__le16*)(buf + 8));
    /* no pressure support from device yet */
    contact->p = 0;
    i2c_timelink_debug("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
        buf[0], buf[1], buf[2], buf[3], buf[4],
        buf[5], buf[6], buf[7], buf[8], buf[9]);
    return true;
};

/*
 * Data format:
 * 0               8               16              24              32
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * | id            | size          | count         | contact(s) ...
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
/* each report required 3 bytes overhead for type, size and count */
#define I2C_TIMELINK_REPORT_OVERHEAD    0x03
static int i2c_timelink_parse_contacts_report(struct i2c_timelink_device* device, __u8* buf, size_t size)
{
    size_t result = 0;
    size_t report_type, report_size, report_count;
    const __u8* reports = buf + I2C_TIMELINK_REPORT_OVERHEAD;
    struct timelink_device* tp;

    if (!buf || size < I2C_TIMELINK_REPORT_OVERHEAD) {
        return result;
        i2c_timelink_debug("invalid buffer: buf=%p size=%zu", buf, size);
    }

    i2c_timelink_debug("type=%u size=%u count=%u", buf[0], buf[1], buf[2]);

    report_type = buf[0];
    report_size = buf[1];
    report_count = buf[2];
    report_count = min(report_count, device->touch.feature.max_contact_count);

    if (report_type != I2C_TIMELINK_REPORT_TYPE_TOUCH)
        return result;

    tp = &device->touch;
    if (tp->received_contact_count == 0 && tp->expected_contact_count == 0) {
        tp->expected_contact_count = report_count;
    }

    for (result = 0; result < report_count; ++result) {
        const __u8* ptr = reports + report_size * result;
        timelink_parse_contact(&tp->incoming_slot.contact, ptr, report_size);
        tp->incoming_slot.state = true;
        timelink_complete_slot(tp);
        ++tp->received_contact_count;
    }

    if (tp->idle && tp->received_contact_count > 0) {
        tp->idle = false;
    }

    if (tp->received_contact_count == tp->expected_contact_count && !tp->idle) {
        if (device->input) {
            timelink_complete_slots(tp);
            i2c_timelink_emit_event(device, device->input);
            timelink_clean_slots(tp);
        }
        if (tp->received_contact_count == 0) {
            tp->idle = true;
        }
        tp->received_contact_count = 0;
        tp->expected_contact_count = 0;
    }
    // FIXME: returns the corresponding value
    return result;
}

#define COMPUTE_TOUCH_REPORT_SIZE(count) \
    (I2C_TIMELINK_REPORT_OVERHEAD + I2C_TIMELINK_REPORT_TOUCH_SIZE * count)

static void i2c_timelink_input_cfg(struct input_dev* input, struct timelink_feature* feature)
{
    set_bit(INPUT_PROP_DIRECT, input->propbit);
    set_bit(EV_SYN, input->evbit);
    set_bit(EV_KEY, input->evbit);
    set_bit(EV_ABS, input->evbit);

    input_set_capability(input, EV_KEY, BTN_TOUCH);
    input_set_abs_params(input, ABS_MT_POSITION_X,
                    feature->x_logical.min,
                    feature->x_logical.max, 0, 0);
    input_set_abs_params(input, ABS_X,
                    feature->x_logical.min,
                    feature->x_logical.max, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y,
                    feature->y_logical.min,
                    feature->y_logical.max, 0, 0);
    input_set_abs_params(input, ABS_Y,
                    feature->y_logical.min,
                    feature->y_logical.max, 0, 0);
    input_set_abs_params(input, ABS_MT_TOUCH_MAJOR,
                    feature->width_logical.min,
                    feature->width_logical.max, 0, 0);
    input_set_abs_params(input, ABS_MT_TOUCH_MINOR,
                    feature->height_logical.min,
                    feature->height_logical.max, 0, 0);
    input_set_abs_params(input, ABS_MT_ORIENTATION, 0, 1, 0, 0);
    if (feature->max_contact_count > 0) {
#ifdef TIMELINK_USE_MT_PROTOCOL_B
        input_mt_init_slots(input, feature->max_contact_count);
#else
        input_set_abs_params(input, ABS_MT_TRACKING_ID,
                        0, feature->max_contact_count, 0, 0);
#endif
    }
}

static int i2c_timelink_read_regs(struct i2c_timelink_device *ts,
        uint8_t cmd, void *data, uint8_t len)
{
    int ret;

    mutex_lock(&ts->mutex);

    ret = i2c_master_send(ts->client, &cmd, 1);
    i2c_timelink_debug("send ret=%d", ret);
    if (ret == 1) {
        ret = i2c_master_recv(ts->client, data, len);
        i2c_timelink_debug("recv ret=%d len=%d", ret, len);
        if (ret != len)
            i2c_timelink_error("recv failure: %d", ret);
    }
    else
        i2c_timelink_error("send failure: %d", ret);

    mutex_unlock(&ts->mutex);

    return ret;
}

static int i2c_timelink_write_regs(struct i2c_timelink_device *ts,
        uint8_t cmd, const void *data, uint8_t len)
{
    int ret;
    uint8_t buf[len + 1];

    mutex_lock(&ts->mutex);

    buf[0] = cmd;
    memcpy(buf + 1, data, len);

    ret = i2c_master_send(ts->client, buf, len + 1);
    if (ret != len + 1)
        i2c_timelink_error("i2c write data cmd failed");

    mutex_unlock(&ts->mutex);

    return ret - 1;
}

static int i2c_timelink_clean(struct i2c_timelink_device* ts)
{
    /* emit an empty report to clean all contacts */
    uint8_t data[I2C_TIMELINK_REPORT_OVERHEAD] = {
        I2C_TIMELINK_REPORT_TYPE_TOUCH
    };
    i2c_timelink_debug("...");
    return i2c_timelink_parse_contacts_report(ts, data, sizeof(data));
}

static int i2c_timelink_process(struct i2c_timelink_device* ts)
{
    uint8_t data[COMPUTE_TOUCH_REPORT_SIZE(ts->touch.feature.max_contact_count)];
    int ret;

    ret = i2c_timelink_read_regs(ts, I2C_TIMELINK_REPORT_TYPE_TOUCH, data, sizeof(data));
    if (ret != sizeof(data)) {
        i2c_timelink_clean(ts);
        return -EIO;
    }
    ret = i2c_timelink_parse_contacts_report(ts, data, sizeof(data));
    return ret;
}

#if defined(I2C_TIMELINK_POLLING_MODE)
static void i2c_timelink_timer(unsigned long handle)
{
    struct i2c_timelink_device *ts = (void *) handle;
    const size_t max_pow = 9;
    const size_t interval = ts->interval << min(ts->continuously_error, max_pow);
    if (ts->stopped)
        return;
    ts->has_input = true;
    wake_up(&ts->wait_for_input);
    mod_timer(&ts->timer, jiffies + interval);
}

static int i2c_timelink_thread(void* handle)
{
    int ret;
    struct i2c_timelink_device *ts = (void *) handle;

    i2c_timelink_debug("kthread run");
    allow_signal(SIGKILL);
    while (!kthread_should_stop()) {
        if (signal_pending(current))
            break;

        if (ts->stopped) {
            i2c_timelink_clean(ts);
            wait_event_interruptible(ts->wait_for_start, !ts->stopped);
        }
        else {
            ret = wait_event_timeout(ts->wait_for_input, ts->has_input, ts->interval << 1);
            if (ret > 0) {
                ret = i2c_timelink_process(ts);
                ts->continuously_error = (ret == -EIO) ? ts->continuously_error + 1 : 0;
                ts->has_input = false;
            }
        }
    }

    set_current_state(TASK_RUNNING);
    i2c_timelink_debug("kthread exit");
    return 0;
}
#else
static int i2c_timelink_loop(struct i2c_timelink_device* ts)
{
    while (!ts->stopped) {
        i2c_timelink_process(ts);
        wait_event_timeout(ts->wait_for_stop, ts->stopped, REFRESH_TIME);
    }
    i2c_timelink_clean(ts);

    return 0;
}

static irqreturn_t i2c_timelink_soft_irq(int irq, void *handle)
{
    struct i2c_timelink_device *ts = handle;
    i2c_timelink_loop(ts);
    return IRQ_HANDLED;
}
#endif

static void i2c_timelink_start(struct i2c_timelink_device *ts)
{
    ts->stopped = false;
    wake_up(&ts->wait_for_start);
#if defined(I2C_TIMELINK_POLLING_MODE)
    mod_timer(&ts->timer, jiffies + ts->interval);
#else
    mb();
    enable_irq(ts->irq);
#endif
}

static void i2c_timelink_stop(struct i2c_timelink_device *ts)
{
    ts->stopped = true;
    wake_up(&ts->wait_for_stop);
#if defined(I2C_TIMELINK_POLLING_MODE)
    i2c_timelink_clean(ts);
#else
    mb();
    disable_irq(ts->irq);
#endif
}

static int i2c_timelink_open(struct input_dev *input_dev)
{
    struct i2c_timelink_device *ts = input_get_drvdata(input_dev);
    i2c_timelink_debug("...");
    i2c_timelink_start(ts);
    return 0;
}

static void i2c_timelink_close(struct input_dev *input_dev)
{
    struct i2c_timelink_device *ts = input_get_drvdata(input_dev);
    i2c_timelink_debug("...");
    i2c_timelink_stop(ts);
}

static ssize_t i2c_timelink_calibrate_read(struct timelink_calibrate_sysfs* sysfs,
    struct timelink_calibrate* calibrate)
{
    size_t i;
    struct i2c_timelink_device* ts = container_of(sysfs, struct i2c_timelink_device, calibrate_sysfs);
    struct timelink_calibrate data = {0};
    ssize_t ret = i2c_timelink_read_regs(ts, I2C_TIMELINK_REPORT_TYPE_CALIBRATE, &data, sizeof(data));
    if (ret != sizeof(data))
        return -EIO;
    calibrate->calibrator = le32_to_cpup((__le32*)(&data.calibrator));
    /* Some legacy tools simply set this value to 1 indicates calibrate is enabled, now it should be fixed
     * up to 4 in default. */
    if (calibrate->calibrator == 1)
        calibrate->calibrator = 4;
    for (i = 0; i < calibrate->calibrator; ++i) {
        calibrate->raw.points[i].position.x = le16_to_cpup((__le16*)(&data.raw.points[i].position.x));
        calibrate->raw.points[i].position.y = le16_to_cpup((__le16*)(&data.raw.points[i].position.y));
        calibrate->raw.points[i].origin.x = le16_to_cpup((__le16*)(&data.raw.points[i].origin.x));
        calibrate->raw.points[i].origin.y = le16_to_cpup((__le16*)(&data.raw.points[i].origin.y));
    }
    return 0;
}

static int i2c_timelink_calibrate_match_clear(const struct timelink_calibrate* calibrate)
{
    size_t i;
    if (calibrate == NULL
        || calibrate->calibrator == 0)
        return 1;
    for (i = 0; i < calibrate->calibrator; ++i) {
        if (calibrate->raw.points[i].position.x != 0
            && calibrate->raw.points[i].position.y != 0)
            return 0;
    }
    return 1;
}

static ssize_t i2c_timelink_calibrate_write(struct timelink_calibrate_sysfs* sysfs,
    const struct timelink_calibrate* calibrate)
{
    size_t i;
    ssize_t ret;
    struct i2c_timelink_device* ts = container_of(sysfs, struct i2c_timelink_device, calibrate_sysfs);
    struct timelink_calibrate data = {0};
    if (!i2c_timelink_calibrate_match_clear(calibrate)) {
        data.calibrator = cpu_to_le32(calibrate->calibrator);
        for (i = 0; i < calibrate->calibrator; ++i) {
            data.raw.points[i].position.x = cpu_to_le16(calibrate->raw.points[i].position.x);
            data.raw.points[i].position.y = cpu_to_le16(calibrate->raw.points[i].position.y);
            data.raw.points[i].origin.x = cpu_to_le16(calibrate->raw.points[i].origin.x);
            data.raw.points[i].origin.y = cpu_to_le16(calibrate->raw.points[i].origin.y);
        }
    }
    ret = i2c_timelink_write_regs(ts, I2C_TIMELINK_REPORT_TYPE_CALIBRATE, &data, sizeof(data));
    if (ret != sizeof(data))
        return -EIO;
    return 0;
}

static int i2c_timelink_load_spec(struct i2c_timelink_device *ts)
{
    ssize_t ret = i2c_timelink_read_regs(ts, I2C_TIMELINK_REPORT_TYPE_SPECIFICS, &ts->spec, sizeof(ts->spec));
    if (ret != sizeof(ts->spec))
        return -EIO;
    ts->spec.version = le32_to_cpup((__le32*)&ts->spec.version);
    ts->spec.module_size = le16_to_cpup((__le16*)&ts->spec.module_size);
    ts->spec.ht = le16_to_cpup((__le16*)&ts->spec.ht);
    ts->spec.wt = le16_to_cpup((__le16*)&ts->spec.wt);
    ts->spec.hr = le16_to_cpup((__le16*)&ts->spec.hr);
    ts->spec.wr = le16_to_cpup((__le16*)&ts->spec.wr);
    i2c_timelink_info("version: %08x", ts->spec.version);
    return 0;
}

static ssize_t i2c_timelink_spec_read(struct timelink_spec_sysfs* sysfs, struct timelink_spec* spec)
{
    int ret = 0;
    struct i2c_timelink_device* ts = container_of(sysfs, struct i2c_timelink_device, spec_sysfs);
    if ((ret = i2c_timelink_load_spec(ts)) != 0) {
        i2c_timelink_error("failure to query specifics");
        return ret;
    }
    memcpy(spec, &ts->spec, sizeof(*spec));
    return ret;
}

static ssize_t i2c_timelink_diagnose_read(struct timelink_diagnose_sysfs* sysfs,
    struct timelink_diagnose_report* report, enum timelink_spec_field f)
{
    size_t i;
    struct i2c_timelink_device* ts = container_of(sysfs, struct i2c_timelink_device, diagnose_sysfs);
    uint8_t total = (f == hr) ? ts->spec.hr
            : ((f == ht) ? ts->spec.ht
            : ((f == wr) ? ts->spec.wr
            : ts->spec.wt));
    uint8_t data[total];
    uint8_t cs = (uint8_t) (f - ht);
    ssize_t ret = i2c_timelink_write_regs(ts, I2C_TIMELINK_REPORT_TYPE_DIAGNOSE_CS, &cs, sizeof(cs));
    if (ret != sizeof(cs))
        return -EIO;
    ret = i2c_timelink_read_regs(ts, I2C_TIMELINK_REPORT_TYPE_DIAGNOSE, data, sizeof(data));
    if (ret != sizeof(data))
        return -EIO;

    for (i = 0; i < total && data[i]; ++i)
        report->bad[i] = data[i];
    report->bad_count = i;
    return 0;
}

static ssize_t i2c_timelink_bootloader_read_mode(struct timelink_bootloader_sysfs* sysfs, uint8_t* mode)
{
    struct i2c_timelink_device* ts = container_of(sysfs, struct i2c_timelink_device, bootloader_sysfs);
    ssize_t ret = i2c_timelink_read_regs(ts, I2C_TIMELINK_REPORT_TYPE_BOOTLOADER_MODE, mode, 1);
    if (ret != 1)
        return -EIO;
    return ret;
}

static ssize_t i2c_timelink_bootloader_write_mode(struct timelink_bootloader_sysfs* sysfs, uint8_t mode)
{
    struct i2c_timelink_device* ts = container_of(sysfs, struct i2c_timelink_device, bootloader_sysfs);
    ssize_t ret = 0;

    switch (mode) {
    case I2C_TIMELINK_BOOTLOADER_MODE_BOOT:
    case I2C_TIMELINK_BOOTLOADER_MODE_BOOT_ONESHOT:
        i2c_timelink_start(ts);
        break;
    case I2C_TIMELINK_BOOTLOADER_MODE_UPGRADE:
    case I2C_TIMELINK_BOOTLOADER_MODE_UPGRADE_ONESHOT:
        i2c_timelink_stop(ts);
        break;
    default:
        return -EINVAL;
    }

    ret = i2c_timelink_write_regs(ts, I2C_TIMELINK_REPORT_TYPE_BOOTLOADER_MODE, &mode, sizeof(mode));
    if (ret != sizeof(mode))
        return -EIO;
    return ret;
}

static uint8_t intel_hex_checksum(const uint8_t* record, size_t size)
{
    uint8_t result = 0;
    while (size--)
        result += record[size];
    result = (~result) + 1;
    return result;
}

enum intel_hex_record_type {
    INTEL_HEX_RECORD_DATA = 0,
    INTEL_HEX_RECORD_EOF = 1,
    INTEL_HEX_RECORD_EXTEND_SEGMENT = 2,
    INTEL_HEX_RECORD_EXTEND_LINEAR = 4,
};

/* format:
 *  :llaaaatt[dd..]cc */
static ssize_t i2c_timelink_bootloader_write_program(struct timelink_bootloader_sysfs* sysfs,
        const char* hex, size_t size)
{
    ssize_t ret = 0;
    struct i2c_timelink_device* ts = container_of(sysfs, struct i2c_timelink_device, bootloader_sysfs);
    uint8_t data[(size - 1) / 2];
    uint8_t bootloader_mode = 0;
    uint32_t addr[2] = {0};
    uint8_t checksum = 0;
    uint32_t offset = 0;

    if (!hex || size < 11 || hex[0] != ':') {
        i2c_timelink_error("invalid hex record format");
        ret = -EINVAL;
        goto err;
    }
    /* chop off the fist byte for ':'
     * FIXME: make sure the convertion is okay */
    hex2bin(data, hex + 1, sizeof(data));
    checksum = (intel_hex_checksum(data, sizeof(data) - 1));
    if (checksum != data[sizeof(data) - 1]) {
        i2c_timelink_error("wrong hex record checksum, %hhu vs %hhu", checksum, data[sizeof(data) - 1]);
        ret = -EINVAL;
        goto err;
    }
    /* check the length of record except checksum */
    if (data[0] != sizeof(data) - 5) {
        i2c_timelink_error("wrong hex record length");
        ret = -EINVAL;
        goto err;
    }

    ret = i2c_timelink_bootloader_read_mode(sysfs, &bootloader_mode);
    if (ret <= 0) {
        i2c_timelink_error("failure to query bootloader mode");
        ret = -EIO;
        goto err;
    }
    if (bootloader_mode != I2C_TIMELINK_BOOTLOADER_MODE_UPGRADE && bootloader_mode != I2C_TIMELINK_BOOTLOADER_MODE_UPGRADE_ONESHOT) {
        i2c_timelink_error("bootloader not in upgrade mode now");
        ret = -EPERM;
        goto err;
    }

    switch (data[3]) {
    case INTEL_HEX_RECORD_DATA:
        ts->programming = true;
        offset = be16_to_cpup((__be16*)(data + 1));
        addr[0] = ts->programming_base_addr + offset;
        addr[1] = addr[0] + data[0];
        addr[0] = cpu_to_le32p(addr);
        addr[1] = cpu_to_le32p(addr + 1);
        i2c_timelink_debug("program addr=%08x base=%08x offset=%08x", addr[0], ts->programming_base_addr, offset);
        ret = i2c_timelink_write_regs(ts, I2C_TIMELINK_REPORT_TYPE_BOOTLOADER_ADDR,
            addr, sizeof(addr));
        if (ret != sizeof(addr)) {
            ret = -EIO;
            goto err;
        }
        ret = i2c_timelink_write_regs(ts, I2C_TIMELINK_REPORT_TYPE_BOOTLOADER_CODE,
            data + 4, data[0]);
        if (ret != data[0]) {
            ret = -EIO;
            goto err;
        }
        break;
    case INTEL_HEX_RECORD_EOF:
        ts->programming_base_addr = 0;
        ts->programming = false;
        /* set address to 0xffffffff to indicate EOF */
        addr[0] = 0xffffffff;
        addr[1] = 0xffffffff;
        ret = i2c_timelink_write_regs(ts, I2C_TIMELINK_REPORT_TYPE_BOOTLOADER_ADDR,
            addr, sizeof(addr));
        if (ret != sizeof(addr)) {
            ret = -EIO;
            goto err;
        }
        break;
    case INTEL_HEX_RECORD_EXTEND_SEGMENT:
        /* no support yet */
        break;
    case INTEL_HEX_RECORD_EXTEND_LINEAR:
        ts->programming_base_addr = be16_to_cpup((__be16*)(data + 4)) << 16;
        break;
    }
    return size;

err:
    i2c_timelink_error("failure ret=%d", ret);
    /* reset programming state while error occurred */
    ts->programming = false;
    return ret;
}

static ssize_t i2c_timelink_bootloader_write_erase(struct timelink_bootloader_sysfs* sysfs)
{
    struct i2c_timelink_device* ts = container_of(sysfs, struct i2c_timelink_device, bootloader_sysfs);
    uint8_t data[1] = {0};
    uint8_t bootloader_mode = 0;
    ssize_t ret = i2c_timelink_bootloader_read_mode(sysfs, &bootloader_mode);
    if (ret <= 0) {
        i2c_timelink_error("failure to query bootloader mode");
        ret = -EIO;
        goto err;
    }
    if (bootloader_mode != I2C_TIMELINK_BOOTLOADER_MODE_UPGRADE && bootloader_mode != I2C_TIMELINK_BOOTLOADER_MODE_UPGRADE_ONESHOT) {
        i2c_timelink_error("bootloader not in upgrade mode now");
        ret = -EPERM;
        goto err;
    }

    ret = i2c_timelink_write_regs(ts, I2C_TIMELINK_REPORT_TYPE_BOOTLOADER_ERASE, data, sizeof(data));
    if (ret != sizeof(data)) {
        ret = -EIO;
        goto err;
    }
    return 0;
err:
    return ret;
}

static int i2c_timelink_register_sysfs(struct i2c_timelink_device* ts)
{
    int err = 0;
    struct timelink_calibrate_sysfs* calibrate_sysfs = &ts->calibrate_sysfs;
    struct timelink_spec_sysfs* spec_sysfs = &ts->spec_sysfs;
    struct timelink_diagnose_sysfs* diagnose_sysfs = &ts->diagnose_sysfs;
    struct timelink_bootloader_sysfs* bootloader_sysfs = &ts->bootloader_sysfs;

    calibrate_sysfs->ops.read = i2c_timelink_calibrate_read;
    calibrate_sysfs->ops.write = i2c_timelink_calibrate_write;
    calibrate_sysfs->parent = &i2c_timelink_class_dev->kobj;
    if (timelink_register_calibrate(calibrate_sysfs)) {
        err = -EINVAL;
        goto out;
    }

    spec_sysfs->ops.read = i2c_timelink_spec_read;
    spec_sysfs->parent = &i2c_timelink_class_dev->kobj;
    if (timelink_register_spec(spec_sysfs)) {
        err = -EINVAL;
        goto err_free_calibrate;
    }

    diagnose_sysfs->ops.read = i2c_timelink_diagnose_read;
    diagnose_sysfs->parent = &i2c_timelink_class_dev->kobj;
    if (timelink_register_diagnose(diagnose_sysfs)) {
        err = -EINVAL;
        goto err_free_spec;
    }

    bootloader_sysfs->ops.read_mode = i2c_timelink_bootloader_read_mode;
    bootloader_sysfs->ops.write_mode = i2c_timelink_bootloader_write_mode;
    bootloader_sysfs->ops.write_program = i2c_timelink_bootloader_write_program;
    bootloader_sysfs->ops.write_erase = i2c_timelink_bootloader_write_erase;
    bootloader_sysfs->parent = &i2c_timelink_class_dev->kobj;
    if (timelink_register_bootloader(bootloader_sysfs)) {
        err = -EINVAL;
        goto err_free_diagnose;
    }
    goto out;

err_free_diagnose:
    timelink_unregister_diagnose(diagnose_sysfs);
err_free_spec:
    timelink_unregister_spec(spec_sysfs);
err_free_calibrate:
    timelink_unregister_calibrate(calibrate_sysfs);
out:
    return err;
}

void i2c_timelink_unregister_sysfs(struct i2c_timelink_device* ts)
{
    struct timelink_calibrate_sysfs* calibrate_sysfs = &ts->calibrate_sysfs;
    struct timelink_spec_sysfs* spec_sysfs = &ts->spec_sysfs;
    struct timelink_diagnose_sysfs* diagnose_sysfs = &ts->diagnose_sysfs;
    struct timelink_bootloader_sysfs* bootloader_sysfs = &ts->bootloader_sysfs;

    timelink_unregister_bootloader(bootloader_sysfs);
    timelink_unregister_diagnose(diagnose_sysfs);
    timelink_unregister_spec(spec_sysfs);
    timelink_unregister_calibrate(calibrate_sysfs);
}

static int __devinit i2c_timelink_probe(struct i2c_client *client,
       const struct i2c_device_id *id)
{
    struct i2c_timelink_device *ts;
    struct i2c_timelink_platform_data *pdata;
    struct input_dev *input_dev;
    int err;

    i2c_timelink_info("probe on %s bus: %d addr: %x", client->adapter->name, client->adapter->nr, client->addr);
#ifdef I2C_TIMELINK_USE_DEFAULT_PALFORM_DATA
    pdata = &the_default_platform_data;
#else
    pdata = client->dev.platform_data;
#endif

    if (!pdata) {
        i2c_timelink_error("platform data is required!");
        return -EINVAL;
    }

    if (!i2c_check_functionality(client->adapter,
            I2C_FUNC_SMBUS_READ_WORD_DATA))
        return -EIO;

    ts = kzalloc(sizeof(struct i2c_timelink_device), GFP_KERNEL);
    if (!ts) {
        i2c_timelink_error("cannot allocate data");
        err = -ENOMEM;
        goto err_free_mem;
    }

    ts->client = client;
    mutex_init(&ts->mutex);
    err = i2c_timelink_load_spec(ts);
    if (err)
        goto err_free_mem;

    input_dev = input_allocate_device();
    if (!input_dev) {
        err = -ENOMEM;
        goto err_free_mem;
    }

    ts->input = input_dev;

    ts->version = pdata->version;
#if defined(I2C_TIMELINK_POLLING_MODE)
    ts->interval = msecs_to_jiffies(pdata->interval);
#endif

    snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&client->dev));

    input_dev->name = "TimeLink I2C Touchscreen";
    input_dev->phys = ts->phys;
    input_dev->id.bustype = BUS_I2C;

    input_dev->open = i2c_timelink_open;
    input_dev->close = i2c_timelink_close;

    input_set_drvdata(input_dev, ts);

    i2c_timelink_set_feature(&ts->touch.feature, pdata);
    i2c_timelink_input_cfg(input_dev, &ts->touch.feature);

    if (!timelink_alloc(&ts->touch)) {
        i2c_timelink_error("cannot allocate for multitouch device");
        err = -ENOMEM;
        goto err_free_input;
    }

    init_waitqueue_head(&ts->wait_for_stop);
    init_waitqueue_head(&ts->wait_for_start);
    i2c_timelink_stop(ts);

#ifdef I2C_TIMELINK_POLLING_MODE
    ts->has_input = false;
    init_waitqueue_head(&ts->wait_for_input);

    init_timer(&ts->timer);
    ts->timer.data = (long) ts;
    ts->timer.function = i2c_timelink_timer;
    ts->working_thread = kthread_run(i2c_timelink_thread, ts, "%s", "i2c_timelinkd");
    if (IS_ERR(ts->working_thread)) {
        i2c_timelink_error("cannot create kernel thread");
        err = (ssize_t) ts->working_thread;
        goto err_free_touch;
    }
#else
    ts->irq = client->irq;
    err = request_threaded_irq(ts->irq, NULL, i2c_timelink_soft_irq,
            IRQF_ONESHOT, client->dev.driver->name, ts);
    if (err < 0) {
        i2c_timelink_error("irq %d busy?", ts->irq);
        goto err_free_touch;
    }
#endif

    err = input_register_device(input_dev);
    if (err)
        goto err_free_irq;

    if (i2c_timelink_register_sysfs(ts))
        goto err_free_irq;

    i2c_set_clientdata(client, ts);

    ts->touch.idle = true;

    return 0;

 err_free_irq:
#if defined(I2C_TIMELINK_POLLING_MODE)
    kthread_stop(ts->working_thread);
#else
    free_irq(ts->irq, ts);
#endif
 err_free_touch:
    timelink_free(&ts->touch);
 err_free_input:
    input_free_device(input_dev);
 err_free_mem:
    kfree(ts);
    return err;
}

static int __devexit i2c_timelink_remove(struct i2c_client *client)
{
    struct i2c_timelink_device *ts = i2c_get_clientdata(client);

#if defined(I2C_TIMELINK_POLLING_MODE)
    del_timer_sync(&ts->timer);
    /* kthread might sleeping infinitely for waiting start signal, then wake up it for stop it */
    if (ts->stopped) {
        ts->stopped = false;
        wake_up(&ts->wait_for_start);
    }
    kthread_stop(ts->working_thread);
#else
    free_irq(ts->irq, ts);
#endif

    i2c_timelink_unregister_sysfs(ts);
    input_unregister_device(ts->input);
    timelink_free(&ts->touch);
    kfree(ts);

    return 0;
}

static const struct i2c_device_id i2c_timelink_idtable[] = {
    { "i2c-timelink", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, i2c_timelink_idtable);

static struct i2c_driver i2c_timelink_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = "i2c-timelink"
    },
    .id_table   = i2c_timelink_idtable,
    .probe      = i2c_timelink_probe,
    .remove     = __devexit_p(i2c_timelink_remove),
};

static int __init i2c_timelink_init(void)
{
    int ret = 0;
    i2c_timelink_debug("...");

    if (i2c_timelink_register_class()) {
        i2c_timelink_error("can not register class");
        ret = -EINVAL;
        goto out;
    }

    i2c_timelink_class_dev = i2c_timelink_create_class_device();
    if (!i2c_timelink_class_dev) {
        i2c_timelink_error("can not create class device");
        ret = -EINVAL;
        goto unreg_class;
    }

    ret = i2c_add_driver(&i2c_timelink_driver);
    if (!ret)
        return ret;

unreg_class:
    i2c_timelink_unregister_class();
out:
    return ret;
}

static void __exit i2c_timelink_exit(void)
{
    i2c_timelink_debug("...");
    i2c_del_driver(&i2c_timelink_driver);
    i2c_timelink_destroy_class_device();
    i2c_timelink_unregister_class();
}

module_init(i2c_timelink_init);
module_exit(i2c_timelink_exit);

