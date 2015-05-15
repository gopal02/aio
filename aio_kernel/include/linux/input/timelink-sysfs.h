#ifndef __TIMELINK_SYSFS_H__
#define __TIMELINK_SYSFS_H__

#pragma pack(1)
/* For specifics */
struct timelink_spec
{
    uint32_t version;
    uint16_t module_size;
    uint16_t ht;
    uint16_t wt;
    uint16_t hr;
    uint16_t wr;
    uint8_t orientation;
};

struct timelink_point {
    uint16_t x;
    uint16_t y;
};

struct timelink_calibrate_point {
    struct timelink_point position;
    struct timelink_point origin;
};

#define TIMELINK_CALIBRATE_MAX_POINTS 9

struct timelink_calibrate {
    uint32_t calibrator;
    union {
        struct {
            struct timelink_calibrate_point points[TIMELINK_CALIBRATE_MAX_POINTS];
        } raw;
    };
};

#define TIMELINK_DIAGNOSE_MAX_SENSOR 128

struct timelink_diagnose_report {
    uint32_t bad_count;
    uint16_t bad[TIMELINK_DIAGNOSE_MAX_SENSOR];
};

#pragma pack()

enum timelink_spec_field {
    ht,
    wt,
    hr,
    wr,
    version,
    module_size,
    orientation
};

struct timelink_spec_sysfs;

struct timelink_spec_ops {
    ssize_t (*read) (struct timelink_spec_sysfs*, struct timelink_spec*);
};

struct timelink_spec_sysfs {
    struct kobject* parent;
    struct timelink_spec_ops ops;
    struct kobject kobj;
};

struct timelink_spec_attribute {
    struct attribute attr;
    ssize_t (*show) (struct timelink_spec_sysfs*, char*);
    ssize_t (*store) (struct timelink_spec_sysfs*, const char*, size_t);
};

static ssize_t timelink_spec_base_show(struct timelink_spec_sysfs* sysfs, char* out, enum timelink_spec_field f)
{
    ssize_t ret = 0;
    struct timelink_spec spec = {0};

    if (sysfs->ops.read)
        sysfs->ops.read(sysfs, &spec);

    switch (f) {
    case version:
        ret = sprintf(out, "%u.%u.%u.%u\n",
            spec.version >> 24 & 0xff, spec.version >> 16 & 0xff,
            spec.version >> 8 & 0xff, spec.version & 0xff);
        break;
    case module_size:
        ret = sprintf(out, "%u.%u\n", spec.module_size / 10, spec.module_size % 10);
        break;
    case ht:
        ret = sprintf(out, "%u\n", spec.ht);
        break;
    case wt:
        ret = sprintf(out, "%u\n", spec.wt);
        break;
    case hr:
        ret = sprintf(out, "%u\n", spec.hr);
        break;
    case wr:
        ret = sprintf(out, "%u\n", spec.wr);
        break;
    case orientation:
        ret = sprintf(out, "%u\n", spec.orientation);
        break;
    }
    return ret;
}

#define DEFINE_SPEC_ATTR(field) \
static ssize_t timelink_spec_show_##field(struct timelink_spec_sysfs* sysfs, char* out)\
{\
    return timelink_spec_base_show(sysfs, out, field);\
}\
\
static struct timelink_spec_attribute timelink_spec_attribute_##field =\
    __ATTR(field, 0444, timelink_spec_show_##field, NULL);\

DEFINE_SPEC_ATTR(version);
DEFINE_SPEC_ATTR(module_size);
DEFINE_SPEC_ATTR(ht);
DEFINE_SPEC_ATTR(wt);
DEFINE_SPEC_ATTR(hr);
DEFINE_SPEC_ATTR(wr);
DEFINE_SPEC_ATTR(orientation);

static ssize_t timelink_spec_show(struct kobject* kobj, struct attribute* a, char* buf)
{
    ssize_t ret;
    struct timelink_spec_sysfs* sysfs = container_of(kobj, struct timelink_spec_sysfs, kobj);
    struct timelink_spec_attribute* attr = container_of(a, struct timelink_spec_attribute, attr);
    ret = attr->show ? attr->show(sysfs, buf) : -EIO;
    return ret;
}

static struct sysfs_ops timelink_spec_sysfs_ops = {
    .show = timelink_spec_show,
};

static struct attribute* timelink_spec_attrs[] = {
    &timelink_spec_attribute_version.attr,
    &timelink_spec_attribute_module_size.attr,
    &timelink_spec_attribute_ht.attr,
    &timelink_spec_attribute_wt.attr,
    &timelink_spec_attribute_hr.attr,
    &timelink_spec_attribute_wr.attr,
    &timelink_spec_attribute_orientation.attr,
    NULL
};

static struct kobj_type timelink_spec_ktype = {
    .sysfs_ops = &timelink_spec_sysfs_ops,
    .default_attrs = timelink_spec_attrs,
};

static int timelink_register_spec(struct timelink_spec_sysfs* sysfs)
{
    int ret = 0;

    /* /sys/class/.../specifics/... */
    kobject_init(&sysfs->kobj, &timelink_spec_ktype);
    ret = kobject_add(&sysfs->kobj, sysfs->parent, "specifics");
    if (ret)
        timelink_error("can not add spec to sysfs");
    return ret;
}

static void timelink_unregister_spec(struct timelink_spec_sysfs* sysfs)
{
    kobject_put(&sysfs->kobj);
}


/* For diagnose */
struct timelink_diagnose_sysfs;

struct timelink_diagnose_ops {
    ssize_t (*read) (struct timelink_diagnose_sysfs*, struct timelink_diagnose_report*, enum timelink_spec_field);
};

struct timelink_diagnose_sysfs {
    struct kobject* parent;
    struct timelink_diagnose_ops ops;
    struct kobject kobj;
};

struct timelink_diagnose_attribute {
    struct attribute attr;
    ssize_t (*show) (struct timelink_diagnose_sysfs*, char*);
    ssize_t (*store) (struct timelink_diagnose_sysfs*, const char*, size_t count);
};

/* Formats:
 *  bad_n1 bad_n2 bad_n3 ...
 */
static ssize_t timelink_diagnose_print(char* out, const struct timelink_diagnose_report* diagnose)
{
    size_t i;
    ssize_t ret = 0;
    for (i = 0; i < diagnose->bad_count; ++i)
        ret += sprintf(out + ret, "%03u ", diagnose->bad[i]);
    out[ret++] = '\n';
    return ret;
}

static ssize_t timelink_diagnose_base_show(struct timelink_diagnose_sysfs* sysfs, char* out,
    enum timelink_spec_field f)
{
    ssize_t ret = 0;
    struct timelink_diagnose_report diagnose = {0};

    if (sysfs->ops.read)
        if (sysfs->ops.read(sysfs, &diagnose, f))
            timelink_error("failure to read diagnose report");

    ret = timelink_diagnose_print(out, &diagnose);
    return ret;
}

#define DEFINE_DIAGNOSE_ATTR(field) \
static ssize_t timelink_diagnose_show_##field(struct timelink_diagnose_sysfs* sysfs, char* out)\
{\
    return timelink_diagnose_base_show(sysfs, out, field);\
}\
\
static struct timelink_diagnose_attribute timelink_diagnose_attribute_##field =\
    __ATTR(field, 0444, timelink_diagnose_show_##field, NULL);\

DEFINE_DIAGNOSE_ATTR(ht);
DEFINE_DIAGNOSE_ATTR(wt);
DEFINE_DIAGNOSE_ATTR(hr);
DEFINE_DIAGNOSE_ATTR(wr);

static ssize_t timelink_diagnose_show(struct kobject* kobj, struct attribute* a, char* buf)
{
    ssize_t ret;
    struct timelink_diagnose_sysfs* sysfs = container_of(kobj, struct timelink_diagnose_sysfs, kobj);
    struct timelink_diagnose_attribute* attr = container_of(a, struct timelink_diagnose_attribute, attr);
    ret = attr->show ? attr->show(sysfs, buf) : -EIO;
    return ret;
}

static struct sysfs_ops timelink_diagnose_sysfs_ops = {
    .show = timelink_diagnose_show,
};

static struct attribute* timelink_diagnose_attrs[] = {
    &timelink_diagnose_attribute_ht.attr,
    &timelink_diagnose_attribute_wt.attr,
    &timelink_diagnose_attribute_hr.attr,
    &timelink_diagnose_attribute_wr.attr,
    NULL
};

static struct kobj_type timelink_diagnose_ktype = {
    .sysfs_ops = &timelink_diagnose_sysfs_ops,
    .default_attrs = timelink_diagnose_attrs,
};

static int timelink_register_diagnose(struct timelink_diagnose_sysfs* sysfs)
{
    int ret = 0;

    /* /sys/class/.../diagnose/... */
    kobject_init(&sysfs->kobj, &timelink_diagnose_ktype);
    ret = kobject_add(&sysfs->kobj, sysfs->parent, "diagnose");
    if (ret)
        timelink_error("can not add diagnose to sysfs");
    return ret;
}

static void timelink_unregister_diagnose(struct timelink_diagnose_sysfs* sysfs)
{
    kobject_put(&sysfs->kobj);
}


/* For calibrate */

struct timelink_calibrate_sysfs;

struct timelink_calibrate_ops {
    ssize_t (*read) (struct timelink_calibrate_sysfs*, struct timelink_calibrate*);
    ssize_t (*write) (struct timelink_calibrate_sysfs*, const struct timelink_calibrate*);
};

struct timelink_calibrate_sysfs {
    struct kobject* parent;
    struct timelink_calibrate_ops ops;
    struct kobject kobj;
};

struct timelink_calibrate_attribute {
    struct attribute attr;
    ssize_t (*show) (struct timelink_calibrate_sysfs*, char*);
    ssize_t (*store) (struct timelink_calibrate_sysfs*, const char*, size_t count);
};

/* Formats:
 *  calibrator
 *  p1_x,p1_y@o1_x,o1_y
 *  p2_x,p2_y@o2_x,o2_y
 *  p3_x,p3_y@o3_x,o3_y
 *  p4_x,p4_y@o4_x,o4_y
 */
static ssize_t timelink_calibrate_print(char* out, const struct timelink_calibrate* calibrate)
{
    size_t i;
    ssize_t ret = sprintf(out, "%u\n", calibrate->calibrator);
    for (i = 0; i < calibrate->calibrator; ++i) {
        ret += sprintf(out + ret, "%06d,%06d@%06d,%06d\n",
            calibrate->raw.points[i].position.x, calibrate->raw.points[i].position.y,
            calibrate->raw.points[i].origin.x, calibrate->raw.points[i].origin.y);
    }
    return ret;
}

static ssize_t timelink_calibrate_parse(struct timelink_calibrate* calibrate,
    const char* buffer, size_t count)
{
    ssize_t ret = -EINVAL;
    size_t i;
    char* token;
    /* I'm initially use 'kstrdup(buffer, GFP_KERNEL)' to duplicate buffer here,
     * but found that kernel might halt with the following operations. So I
     * finally take a forcibly cast here instead of, this was test on Ubuntu
     * with kernel release: 3.2.0-31-generic-pae. */
    char* data = (char*) buffer;
    if (!data)
        goto out;

    token = strsep(&data, " \t\n");
    if (!token || !strlen(token))
        goto out_free;

    if (sscanf(token, "%u", &calibrate->calibrator) != 1)
        goto out_free;

    if (calibrate->calibrator > TIMELINK_CALIBRATE_MAX_POINTS)
        goto out_free;

    for (i = 0; i < calibrate->calibrator; ++i) {
        token = strsep(&data, " \t\n");
        if (sscanf(token, "%hu,%hu@%hu,%hu",
            &calibrate->raw.points[i].position.x,
            &calibrate->raw.points[i].position.y,
            &calibrate->raw.points[i].origin.x,
            &calibrate->raw.points[i].origin.y) != 4)
            goto out_free;
        else
            timelink_debug("%zu: %u,%u@%u,%u", 
                i,
                calibrate->raw.points[i].position.x,
                calibrate->raw.points[i].position.y,
                calibrate->raw.points[i].origin.x,
                calibrate->raw.points[i].origin.y);
    }
    ret = count;
    timelink_debug("ret=%zd", ret);

out_free:
out:
    return ret;
}

static ssize_t timelink_calibrate_raw_show(struct timelink_calibrate_sysfs* sysfs, char* out)
{
    struct timelink_calibrate calibrate = {0};

    if (sysfs->ops.read)
        sysfs->ops.read(sysfs, &calibrate);

    return timelink_calibrate_print(out, &calibrate);
}

static ssize_t timelink_calibrate_raw_store(struct timelink_calibrate_sysfs* sysfs,
    const char* buffer, size_t count)
{
    struct timelink_calibrate calibrate = {0};
    ssize_t ret = timelink_calibrate_parse(&calibrate, buffer, count);
    if (ret <= 0)
        goto out;

    if (sysfs->ops.write)
        sysfs->ops.write(sysfs, &calibrate);

out:
    return ret;
}

static struct timelink_calibrate_attribute timelink_calibrate_raw_attribute =
    __ATTR(raw, 0666, timelink_calibrate_raw_show, timelink_calibrate_raw_store);

static ssize_t timelink_calibrate_show(struct kobject* kobj, struct attribute* a, char* buf)
{
    ssize_t ret;
    struct timelink_calibrate_sysfs* sysfs = container_of(kobj, struct timelink_calibrate_sysfs, kobj);
    struct timelink_calibrate_attribute* cattr = container_of(a, struct timelink_calibrate_attribute, attr);
    ret = cattr->show ? cattr->show(sysfs, buf) : -EIO;
    return ret;
}

static ssize_t timelink_calibrate_store(struct kobject* kobj, struct attribute* a, const char* buf, size_t count)
{
    ssize_t ret;
    struct timelink_calibrate_sysfs* sysfs = container_of(kobj, struct timelink_calibrate_sysfs, kobj);
    struct timelink_calibrate_attribute* cattr = container_of(a, struct timelink_calibrate_attribute, attr);
    ret = cattr->store ? cattr->store(sysfs, buf, count) : -EIO;
    return ret;
}

static struct sysfs_ops timelink_calibrate_sysfs_ops = {
    .show = timelink_calibrate_show,
    .store = timelink_calibrate_store,
};

static struct attribute* timelink_calibrate_attrs[] = {
    &timelink_calibrate_raw_attribute.attr,
    NULL
};

static struct kobj_type timelink_calibrate_ktype = {
    .sysfs_ops = &timelink_calibrate_sysfs_ops,
    .default_attrs = timelink_calibrate_attrs,
};


static int timelink_register_calibrate(struct timelink_calibrate_sysfs* sysfs)
{
    int ret = 0;

    /* /sys/class/.../calibrate/raw */
    kobject_init(&sysfs->kobj, &timelink_calibrate_ktype);
    ret = kobject_add(&sysfs->kobj, sysfs->parent, "calibrate");
    if (ret) {
        timelink_error("can not add calibrate to sysfs");
        return ret;
    }
    return ret;
}

static void timelink_unregister_calibrate(struct timelink_calibrate_sysfs* sysfs)
{
    kobject_put(&sysfs->kobj);
}


/* For bootloader */
struct timelink_bootloader_sysfs;

struct timelink_bootloader_ops {
    ssize_t (*read_mode) (struct timelink_bootloader_sysfs*, uint8_t* mode);
    ssize_t (*write_mode) (struct timelink_bootloader_sysfs*, uint8_t mode);
    ssize_t (*write_program) (struct timelink_bootloader_sysfs*, const char* buffer, size_t count);
    ssize_t (*write_erase) (struct timelink_bootloader_sysfs*);
};

struct timelink_bootloader_sysfs {
    struct kobject* parent;
    struct timelink_bootloader_ops ops;
    struct kobject kobj;
};

struct timelink_bootloader_attribute {
    struct attribute attr;
    ssize_t (*show) (struct timelink_bootloader_sysfs*, char*);
    ssize_t (*store) (struct timelink_bootloader_sysfs*, const char*, size_t);
};

static ssize_t timelink_bootloader_mode_show(struct timelink_bootloader_sysfs* sysfs, char* out)
{
    ssize_t ret = 0;
    uint8_t mode = 0;

    if (sysfs->ops.read_mode)
        sysfs->ops.read_mode(sysfs, &mode);

    ret = sprintf(out, "%u\n", mode);

    return ret;
}

static ssize_t timelink_bootloader_mode_store(struct timelink_bootloader_sysfs* sysfs,
    const char* buffer, size_t count)
{
    uint8_t mode = 0;

    if (sscanf(buffer, "%hhu", &mode) != 1)
        return -EINVAL;

    if (sysfs->ops.write_mode)
        if (sysfs->ops.write_mode(sysfs, mode))
            return count;

    return 0;
}

static ssize_t timelink_bootloader_program_store(struct timelink_bootloader_sysfs* sysfs,
    const char* buffer, size_t count)
{
    ssize_t ret = -EINVAL;
    size_t bytes = 0;
    size_t length = 0;
    char* token;
    /* I'm initially use 'kstrdup(buffer, GFP_KERNEL)' to duplicate buffer here,
     * but found that kernel might halt with the following operations. So I
     * finally take a forcibly cast here instead of, this was test on Ubuntu
     * with kernel release: 3.2.0-31-generic-pae. */
    char* data = (char*) buffer;
    if (!data)
        goto out;

    while (bytes < count) {
        token = strsep(&data, "\n");
        if (!token || !(length = strlen(token)))
            goto out_free;

        if (sysfs->ops.write_program)
            if ((ret = sysfs->ops.write_program(sysfs, token, length)) <= 0)
                goto out_free;
        /* count the separator '\n' as well */
        bytes += length + 1;
    }
    return bytes;

out_free:
out:
    return ret;
}

static ssize_t timelink_bootloader_erase_store(struct timelink_bootloader_sysfs* sysfs,
    const char* buffer, size_t count)
{
    ssize_t ret = 0;
    if (sysfs->ops.write_erase)
        if ((ret = sysfs->ops.write_erase(sysfs)) < 0)
            return ret;
    return count;
}

static struct timelink_bootloader_attribute timelink_bootloader_attribute_mode =
    __ATTR(mode, 0666, timelink_bootloader_mode_show, timelink_bootloader_mode_store);

static struct timelink_bootloader_attribute timelink_bootloader_attribute_program =
    __ATTR(program, 0222, NULL, timelink_bootloader_program_store);

static struct timelink_bootloader_attribute timelink_bootloader_attribute_erase =
    __ATTR(erase, 0222, NULL, timelink_bootloader_erase_store);


static ssize_t timelink_bootloader_show(struct kobject* kobj, struct attribute* a, char* buf)
{
    ssize_t ret;
    struct timelink_bootloader_sysfs* sysfs = container_of(kobj, struct timelink_bootloader_sysfs, kobj);
    struct timelink_bootloader_attribute* attr = container_of(a, struct timelink_bootloader_attribute, attr);
    ret = attr->show ? attr->show(sysfs, buf) : -EIO;
    return ret;
}

static ssize_t timelink_bootloader_store(struct kobject* kobj, struct attribute* a, const char* buf, size_t count)
{
    ssize_t ret;
    struct timelink_bootloader_sysfs* sysfs = container_of(kobj, struct timelink_bootloader_sysfs, kobj);
    struct timelink_bootloader_attribute* cattr = container_of(a, struct timelink_bootloader_attribute, attr);
    ret = cattr->store ? cattr->store(sysfs, buf, count) : -EIO;
    return ret;
}

static struct sysfs_ops timelink_bootloader_sysfs_ops = {
    .show = timelink_bootloader_show,
    .store = timelink_bootloader_store,
};

static struct attribute* timelink_bootloader_attrs[] = {
    &timelink_bootloader_attribute_mode.attr,
    &timelink_bootloader_attribute_program.attr,
    &timelink_bootloader_attribute_erase.attr,
    NULL
};

static struct kobj_type timelink_bootloader_ktype = {
    .sysfs_ops = &timelink_bootloader_sysfs_ops,
    .default_attrs = timelink_bootloader_attrs,
};

static int timelink_register_bootloader(struct timelink_bootloader_sysfs* sysfs)
{
    int ret = 0;

    /* /sys/class/.../bootloader/... */
    kobject_init(&sysfs->kobj, &timelink_bootloader_ktype);
    ret = kobject_add(&sysfs->kobj, sysfs->parent, "bootloader");
    if (ret)
        timelink_error("can not add bootloader to sysfs");
    return ret;
}

static void timelink_unregister_bootloader(struct timelink_bootloader_sysfs* sysfs)
{
    kobject_put(&sysfs->kobj);
}
#endif

