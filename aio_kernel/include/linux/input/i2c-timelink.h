#ifndef __I2C_TIMELINK_H__
#define __I2C_TIMELINK_H__

struct i2c_timelink_platform_data {
    size_t version;
    size_t interval;
    size_t max_contacts;
    ssize_t x_logical_min;
    ssize_t x_logical_max;
    ssize_t y_logical_min;
    ssize_t y_logical_max;
    ssize_t width_logical_min;
    ssize_t width_logical_max;
    ssize_t height_logical_min;
    ssize_t height_logical_max;
};
#endif

