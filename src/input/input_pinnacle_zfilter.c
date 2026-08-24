// #include <zephyr/init.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>

LOG_MODULE_REGISTER(pinnacle_zfilter, CONFIG_INPUT_LOG_LEVEL);

#define PINNACLE_READ (BIT(7) | BIT(5))
#define PINNACLE_WRITE BIT(7)
static inline uint8_t reg_access(const uint8_t flag, const uint8_t addr) {
  return flag | addr;
}
#define PINNACLE_FILLER_BYTE 0xFB
#define PINNACLE_AUTO_INCREMENT 0xFC

#define PINNACLE_REG_FIRMWARE_ID 0x00
#define PINNACLE_REG_FIRMWARE_VERSION 0x01
#define PINNACLE_REG_STATUS1 0x02
#define PINNACLE_REG_SYS_CONFIG1 0x03
#define PINNACLE_REG_FEED_CONFIG1 0x04
#define PINNACLE_REG_FEED_CONFIG2 0x05
// RESERVED 0x06
#define PINNACLE_REG_CAL_CONFIG1 0x07
#define PINNACLE_REG_Z_IDLE 0x0A
#define PINNACLE_REG_PACKET_BYTE0 0x12
#define PINNACLE_REG_PACKET_BYTE1 0x13
#define PINNACLE_REG_ERA_VALUE 0x1B
#define PINNACLE_REG_ERA_HIGH_BYTE 0x1C
#define PINNACLE_REG_ERA_LOW_BYTE 0x1D
#define PINNACLE_REG_ERA_CONTROL 0x1E

// I have no idea where are people finding this value other than the reference
// implementation. Where is the documentation for this?
#define PINNACLE_ERA_REG_CONFIG 0x0187

// SysConfig1 (register 0x03 data)
#define PINNACLE_SYS_CONFIG1_RESET BIT(0)
#define PINNACLE_SYS_CONFIG1_SHUTDOWN BIT(1)
#define PINNACLE_SYS_CONFIG1_SLEEP_ENABLE BIT(2)

// FeedConfig1 (register 0x04 data)
#define PINNACLE_FEED_CONFIG1_FEED_ENABLE BIT(0)
#define PINNACLE_FEED_CONFIG1_DATA_MODE_ABSOLUTE BIT(1)
#define PINNACLE_FEED_CONFIG1_FILTER_DISABLE BIT(2)
#define PINNACLE_FEED_CONFIG1_X_DISABLE BIT(3)
#define PINNACLE_FEED_CONFIG1_Y_DISABLE BIT(4)
// UNUSED BIT(5)
#define PINNACLE_FEED_CONFIG1_X_INVERT BIT(6)
#define PINNACLE_FEED_CONFIG1_Y_INVERT BIT(7)

// FeedConfig2 (register 0x05 data)
#define PINNACLE_FEED_CONFIG2_INTELLIMOUSE_ENABLE BIT(0)
#define PINNACLE_FEED_CONFIG2_ALL_TAPS_DISABLE BIT(1)
#define PINNACLE_FEED_CONFIG2_SECONDARY_TAP_DISABLE BIT(2)
#define PINNACLE_FEED_CONFIG2_SCROLL_DISABLE BIT(3)
#define PINNACLE_FEED_CONFIG2_GLIDE_EXTEND_DISABLE BIT(4)
// UNUSED BIT(5)
// UNUSED BIT(6)
#define PINNACLE_FEED_CONFIG2_SWAP_X_AND_Y BIT(7)

#define PINNACLE_ERA_CONTROL_READ BIT(0)
#define PINNACLE_ERA_CONTROL_WRITE BIT(1)
// #define PINNACLE_ERA_CONTROL_READ_AUTO_INCREMENT BIT(2)
// #define PINNACLE_ERA_CONTROL_WRITE_AUTO_INCREMENT BIT(3)
#define PINNACLE_ERA_CONTROL_COMPLETE 0x00

#define X_MAX 2048 // exclusive, actual max report 2047
#define Y_MAX 1536 // exclusive, actual max report 1535

#define XLEN 9
#define YLEN 7
#define MAP_SCALE_SHIFT 7
#define MAP_SCALE (1 << MAP_SCALE_SHIFT) // = 128
// clang-format off
const uint8_t ZMAP[YLEN][XLEN] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0},
};                                       // clang-format on

// fixed point with 10bit frac
#define SHIFT 10
#define ONE (1 << SHIFT)
#define HALF (1 << (SHIFT - 1))
#define MAP_SCALE_INV (1 << (SHIFT - MAP_SCALE_SHIFT))
static uint32_t lerp(uint32_t a, uint32_t b, uint32_t t) {
  return (a * (ONE - t) + b * t) >> SHIFT;
}
// NOTE: assumes x and y are in a valid range
static uint8_t zmap_lerp(uint16_t x, uint16_t y) {
  // mirror other 3 quadrants so that we only have to think about one quadrant
  if (x >= X_MAX / 2) {
    x = (X_MAX - 1) - x;
  }
  if (y >= Y_MAX / 2) {
    y = (Y_MAX - 1) - y;
  }

  uint32_t x_idx = x / MAP_SCALE;
  uint32_t y_idx = y / MAP_SCALE;
  uint32_t tx = ((x - MAP_SCALE * x_idx) << SHIFT) * MAP_SCALE_INV >> SHIFT;
  uint32_t ty = ((y - MAP_SCALE * y_idx) << SHIFT) * MAP_SCALE_INV >> SHIFT;

  uint32_t z00 = (uint32_t)ZMAP[y_idx][x_idx] << SHIFT;
  uint32_t z01 = (uint32_t)ZMAP[y_idx][x_idx + 1] << SHIFT;
  uint32_t z10 = (uint32_t)ZMAP[y_idx + 1][x_idx] << SHIFT;
  uint32_t z11 = (uint32_t)ZMAP[y_idx + 1][x_idx + 1] << SHIFT;

  uint32_t z0t = lerp(z00, z01, tx);
  uint32_t z1t = lerp(z10, z11, tx);

  return (lerp(z0t, z1t, ty) + HALF) >> SHIFT;
}

struct abs_touch {
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint8_t button;
};
static bool is_valid(struct abs_touch *touch) {
  return touch->z > zmap_lerp(touch->x, touch->y);
  return false;
}

struct pinnacle_bus {
  union {
    struct spi_dt_spec spi;
    struct i2c_dt_spec i2c;
  };
  int (*read)(const struct device *device, const uint8_t addr, uint8_t *buffer,
              const uint8_t len);
  int (*write)(const struct device *device, const uint8_t addr,
               const uint8_t value);
};

enum adc_sensitivity {
  X1,
  X2,
  X3,
  X4,
};

struct pinnacle_zfilter_config {
  const struct pinnacle_bus bus;
  const struct gpio_dt_spec data_ready_gpio;

  enum adc_sensitivity sensitivity;
  bool invert_x;
  bool invert_y;
  bool swap_xy;

  uint16_t clamp_x_min;
  uint16_t clamp_x_max;
  uint16_t clamp_y_min;
  uint16_t clamp_y_max;
};

struct pinnacle_zfilter_data {
  struct device *device;
  struct gpio_callback gpio_callback;
  struct k_work *callback_work;
};

static int set_gpio_interrrupt(const struct device *device, const bool enable) {
  const struct pinnacle_zfilter_config *config = device->config;
  int return_code = gpio_pin_interrupt_configure_dt(
      &config->data_ready_gpio,
      enable ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE);

  if (return_code < 0)
    LOG_ERR("failed to set gpio interrupt to %s", enable ? "true" : "false");

  return return_code;
}
static int enable_gpio_interrupt(const struct device *device) {
  return set_gpio_interrrupt(device, true);
}
static int disable_gpio_interrupt(const struct device *device) {
  return set_gpio_interrrupt(device, false);
}

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
static int pinnacle_spi_read(const struct device *device, const uint8_t addr,
                             uint8_t *buffer, const uint8_t len) {
  const struct pinnacle_zfilter_config *config = device->config;

  // send data:
  // READ addr, AUTO_INCREMENT...
  uint8_t send_bytes[len + 3];
  send_bytes[0] = reg_access(PINNACLE_READ, addr);
  memset(&send_bytes[1], PINNACLE_AUTO_INCREMENT, len + 2);
  const struct spi_buf send_buf[1] = {{
      .buf = send_bytes,
      .len = len + 3,
  }};
  const struct spi_buf_set send = {
      .buffers = send_buf,
      .count = 1,
  };

  const struct spi_buf recv_buf[2] = {
      {.buf = NULL, .len = 3},
      {.buf = buffer, .len = len},
  };
  const struct spi_buf_set recv = {
      .buffers = recv_buf,
      .count = 2,
  };
  return spi_transceive_dt(&config->bus.spi, &send, &recv);
}
#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
static int pinnacle_i2c_read(const struct device *device, const uint8_t addr,
                             uint8_t *buffer, const uint8_t len) {
  const struct pinnacle_zfilter_config *config = device->config;

  return i2c_burst_read_dt(&config->bus.i2c, reg_access(PINNACLE_READ, addr),
                           buffer, len);
}
#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
static int pinnacle_spi_write(const struct device *device, const uint8_t addr,
                              const uint8_t value) {
  const struct pinnacle_zfilter_config *config = device->config;

  uint8_t send_bytes[2] = {reg_access(PINNACLE_WRITE, addr), value};
  const struct spi_buf send_buf[1] = {{
      .buf = send_bytes,
      .len = 2,
  }};
  const struct spi_buf_set send = {.buffers = send_buf, .count = 1};
  return spi_write_dt(&config->bus.spi, &send);
}
#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
static int pinnacle_i2c_write(const struct device *device, const uint8_t addr,
                              const uint8_t value) {
  const struct pinnacle_zfilter_config *config = device->config;

  return i2c_reg_write_byte_dt(&config->bus.i2c,
                               reg_access(PINNACLE_WRITE, addr), value);
}
#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static inline int pinnacle_clear_status(const struct device *device) {
  const struct pinnacle_zfilter_config *config = device->config;
  return config->bus.write(device, PINNACLE_REG_STATUS1, 0);
}

static int pinnacle_era_read(const struct device *device, const uint16_t addr,
                             uint8_t *buffer) {
  const struct pinnacle_zfilter_config *config = device->config;
  int return_code;
  disable_gpio_interrupt(device);

  return_code = config->bus.write(device, PINNACLE_REG_ERA_HIGH_BYTE,
                                  (uint8_t)(addr >> 8));
  if (return_code < 0)
    return return_code;
  return_code = config->bus.write(device, PINNACLE_REG_ERA_LOW_BYTE,
                                  (uint8_t)(addr & 0xFF));
  if (return_code < 0)
    return return_code;
  return_code = config->bus.write(device, PINNACLE_REG_ERA_CONTROL,
                                  PINNACLE_ERA_CONTROL_READ);
  if (return_code < 0)
    return return_code;

  uint8_t control_code;
  while (true) {
    return_code =
        config->bus.read(device, PINNACLE_REG_ERA_CONTROL, &control_code, 1);
    if (return_code < 0)
      return return_code;

    if (control_code == PINNACLE_ERA_CONTROL_COMPLETE)
      break;
    k_msleep(10);
  }

  return_code = config->bus.read(device, PINNACLE_REG_ERA_VALUE, buffer, 1);
  if (return_code < 0)
    return return_code;

  return_code = pinnacle_clear_status(device);
  enable_gpio_interrupt(device);
  return return_code;
}

static int pinnacle_era_write(const struct device *device, const uint16_t addr,
                              uint8_t value) {
  const struct pinnacle_zfilter_config *config = device->config;
  int return_code;
  disable_gpio_interrupt(device);

  return_code = config->bus.write(device, PINNACLE_REG_ERA_VALUE, value);
  if (return_code < 0)
    return return_code;
  return_code = config->bus.write(device, PINNACLE_REG_ERA_HIGH_BYTE,
                                  (uint8_t)(addr >> 8));
  if (return_code < 0)
    return return_code;
  return_code = config->bus.write(device, PINNACLE_REG_ERA_LOW_BYTE,
                                  (uint8_t)(addr & 0xFF));
  if (return_code < 0)
    return return_code;
  return_code = config->bus.write(device, PINNACLE_REG_ERA_CONTROL,
                                  PINNACLE_ERA_CONTROL_WRITE);
  if (return_code < 0)
    return return_code;

  uint8_t control_code;
  while (true) {
    return_code =
        config->bus.read(device, PINNACLE_REG_ERA_CONTROL, &control_code, 1);
    if (return_code < 0)
      return return_code;

    if (control_code == PINNACLE_ERA_CONTROL_COMPLETE)
      break;
    k_msleep(10);
  }

  return_code = pinnacle_clear_status(device);
  enable_gpio_interrupt(device);
  return return_code;
}

static void clamp_touch(const struct device *device, struct abs_touch *touch) {
  const struct pinnacle_zfilter_config *config = device->config;

  touch->x = CLAMP(touch->x, config->clamp_x_min, config->clamp_x_max - 1);
  touch->y = CLAMP(touch->y, config->clamp_y_min, config->clamp_y_max - 1);
}

static void pinnacle_zfilter_fetch_and_report(const struct device *device) {
  const struct pinnacle_zfilter_config *config = device->config;
  uint8_t raw[4];
  int return_code;

  // fetch
  config->bus.read(device, PINNACLE_REG_PACKET_BYTE0, raw, 4);
  if (return_code < 0) {
    return;
  };
}

static void pinnacle_zfilter_gpio_callback(const struct device *device,
                                           struct gpio_callback *gpio_callback,
                                           uint32_t pins) {
  struct pinnacle_zfilter_data *data =
      CONTAINER_OF(gpio_callback, struct pinnacle_zfilter_data, gpio_callback);

  k_work_submit(data->callback_work);
}

static uint8_t pinnacle_construct_feed_config1(const bool invert_x,
                                               const bool invert_y) {
  uint8_t feed_config1 = PINNACLE_FEED_CONFIG1_FEED_ENABLE |
                         PINNACLE_FEED_CONFIG1_DATA_MODE_ABSOLUTE;
  feed_config1 |= invert_x ? PINNACLE_FEED_CONFIG1_X_INVERT : 0x00;
  feed_config1 |= invert_y ? PINNACLE_FEED_CONFIG1_Y_INVERT : 0x00;
  return feed_config1;
}

static uint8_t pinnacle_construct_feed_config2(const bool swap_xy) {
  uint8_t feed_config2 = PINNACLE_FEED_CONFIG2_ALL_TAPS_DISABLE |
                         PINNACLE_FEED_CONFIG2_SECONDARY_TAP_DISABLE |
                         PINNACLE_FEED_CONFIG2_SCROLL_DISABLE |
                         PINNACLE_FEED_CONFIG2_GLIDE_EXTEND_DISABLE;
  feed_config2 |= swap_xy ? PINNACLE_FEED_CONFIG2_SWAP_X_AND_Y : 0x00;
  return feed_config2;
}

static int pinnacle_set_sensitivity(const struct device *device) {
  const struct pinnacle_zfilter_config *config = device->config;
  int return_code;

  uint8_t value;
  return_code = pinnacle_era_read(device, PINNACLE_ERA_REG_CONFIG, &value);
  if (return_code < 0) {
    LOG_ERR("failed to read ADC sensitivity (%d)", return_code);
    return return_code;
  }

  value &= 0b00111111;
  switch (config->sensitivity) {
  case X2:
    value |= 1 << 6;
    break;
  case X3:
    value |= 2 << 6;
    break;
  case X4:
    value |= 3 << 6;
    break;
  case X1:
  default:
    break;
  }

  return_code = pinnacle_era_write(device, PINNACLE_ERA_REG_CONFIG, value);
  if (return_code < 0) {
    LOG_ERR("failed to write ADC sensitivity (%d)", return_code);
    return return_code;
  }

  return pinnacle_clear_status(device);
}

static int pinnacle_recalibrate(const struct device *device) {
  const struct pinnacle_zfilter_config *config = device->config;
  int return_code;
  uint8_t value;
  return_code = config->bus.read(device, PINNACLE_REG_CAL_CONFIG1, &value, 1);
  if (return_code < 0) {
    LOG_ERR("failed to read CalConfig1 (%d)", return_code);
    return return_code;
  }
  value |= 0x01;
  return_code = config->bus.write(device, PINNACLE_REG_CAL_CONFIG1, value);
  if (return_code < 0) {
    LOG_ERR("failed to write CalConfig1 (%d)", return_code);
    return return_code;
  }

  while (true) {
    return_code = config->bus.read(device, PINNACLE_REG_CAL_CONFIG1, &value, 1);
    if (return_code < 0) {
      LOG_ERR("failed to read CalConfig1 (%d)", return_code);
      return return_code;
    }

    if ((value & 0x01) == 0)
      break;
    k_msleep(10);
  }
  return 0;
}

static int pinnacle_init_gpio_callback(const struct device *device) {
  struct pinnacle_zfilter_data *data = device->data;
  const struct pinnacle_zfilter_config *config = device->config;
  const struct gpio_dt_spec *gpio = &config->data_ready_gpio;

  data->device = (struct device *)device;
  int return_code;

  if (!gpio_is_ready_dt(gpio)) {
    LOG_ERR("gpio %s(pin %d) is not ready", gpio->port->name, gpio->pin);
    return -ENODEV;
  };

  return_code = gpio_pin_configure_dt(gpio, GPIO_INPUT);
  if (return_code < 0) {
    LOG_ERR("failed to configure for %s(pin %d)", gpio->port->name, gpio->pin);
    return return_code;
  }

  return_code = gpio_pin_interrupt_configure_dt(gpio, GPIO_INT_EDGE_TO_ACTIVE);
  if (return_code < 0) {
    LOG_ERR("failed to configure interrupt for %s(pin %d)", gpio->port->name,
            gpio->pin);
    return return_code;
  }

  gpio_init_callback(&data->gpio_callback, pinnacle_zfilter_gpio_callback,
                     BIT(gpio->pin));

  return_code = gpio_add_callback(gpio->port, &data->gpio_callback);
  if (return_code < 0) {
    LOG_ERR("failed to add callback for %s(pin %d)", gpio->port->name,
            gpio->pin);
    return return_code;
  }

  return 0;
}

static int pinnacle_zfilter_init(const struct device *device) {
  const struct pinnacle_zfilter_config *config = device->config;
  int return_code;

  uint8_t firmware_info[2];
  return_code =
      config->bus.read(device, PINNACLE_REG_FIRMWARE_ID, firmware_info, 2);

  if (return_code < 0) {
    LOG_ERR("failed to read firmware info (%d)", return_code);
  } else {
    LOG_DBG("firmware id: 0x%02x, version: 0x%02x", firmware_info[0],
            firmware_info[1]);
  }

  pinnacle_clear_status(device);
  k_usleep(50);
  return_code = config->bus.write(device, PINNACLE_REG_SYS_CONFIG1,
                                  PINNACLE_SYS_CONFIG1_RESET);
  if (return_code < 0) {
    LOG_ERR("failed to write SysConfig1 (%d)", return_code);
    return return_code;
  }
  return_code =
      config->bus.write(device, PINNACLE_REG_FEED_CONFIG2,
                        pinnacle_construct_feed_config2(config->swap_xy));
  if (return_code < 0) {
    LOG_ERR("failed to write FeedConfig2 (%d)", return_code);
    return return_code;
  }
  return_code = config->bus.write(
      device, PINNACLE_REG_FEED_CONFIG1,
      pinnacle_construct_feed_config1(config->invert_x, config->invert_y));
  if (return_code < 0) {
    LOG_ERR("failed to write FeedConfig1 (%d)", return_code);
    return return_code;
  }
  return_code = config->bus.write(device, PINNACLE_REG_Z_IDLE, 1);
  if (return_code < 0) {
    LOG_ERR("failed to write ZIdle (%d)", return_code);
    return return_code;
  }

  pinnacle_recalibrate(device);

  return pinnacle_init_gpio_callback(device);
}

// clang-format off
#define PINNACLE_ZFILTER_INST(n)                                               \
  static struct pinnacle_zfilter_data pinnacle_zfilter_data_##n;               \
  static struct pinnacle_zfilter_config pinnacle_zfilter_config_##n = {        \
      COND_CODE_1(DT_INST_ON_BUS(n, i2c),                                      \
                  (.bus = {.i2c = I2C_DT_SPEC_INST_GET(n),                     \
                           .read = pinnacle_i2c_read,                          \
                           .write = pinnacle_i2c_write}, ),                    \
                  ())                                                          \
      COND_CODE_1(                                                             \
          DT_INST_ON_BUS(n, spi),                                              \
          (.bus = {.i2c = SPI_DT_SPEC_INST_GET(                                \
                       n, (SPI_OP_MODE_CONTROLLER | SPI_TRANSFER_MSB |         \
                           SPI_MODE_CPHA | SPI_WORD_SET(8))),                  \
                   .read = pinnacle_spi_read,                                  \
                   .write = pinnacle_spi_write}, ),                            \
          ())                                                                  \
      .data_ready_gpio = GPIO_DT_SPEC_INST_GET_OR(n, data_ready_gpio, {}),     \
      .sensitivity = DT_INST_ENUM_IDX(n, sensitivity),                         \
      .invert_x = DT_INST_PROP(n, invert_x),                                   \
      .invert_y = DT_INST_PROP(n, invert_y),                                   \
      .swap_xy = DT_INST_PROP(n, swap_xy),                                     \
      .clamp_x_min = DT_INST_PROP(n, clamp_x_min),                             \
      .clamp_x_max = DT_INST_PROP(n, clamp_x_max),                             \
      .clamp_y_min = DT_INST_PROP(n, clamp_y_min),                             \
      .clamp_y_max = DT_INST_PROP(n, clamp_y_max),                             \
  };                                                                           \
  DEVICE_DT_INST_DEFINE(                                                       \
      n, pinnacle_zfilter_init, NULL, &pinnacle_zfilter_data_##n,              \
      &pinnacle_zfilter_config_##n, POST_KERNEL, INPUT_INIT_PRIORITY,          \
      &input_processor_kinetic_xy_driver_api);                                 \
  BUILD_ASSERT(DT_INST_PROP(n, clamp_x_min) < DT_INST_PROP(n, clamp_x_max),    \
               "assert failed: clamp-x-min < clamp-x-max");                    \
  BUILD_ASSERT(DT_INST_PROP(n, clamp_y_min) < DT_INST_PROP(n, clamp_y_max),    \
               "assert failed: clamp-y-min < clamp-y-max");                    \
  BUILD_ASSERT(DT_INST_PROP(n, clamp_x_max) <= X_MAX,                          \
               "assert failed: clamp-x-max < X_MAX(2048)");                    \
  BUILD_ASSERT(DT_INST_PROP(n, clamp_y_max) <= Y_MAX,                          \
               "assert failed: clamp-y-max < Y_MAX(1536)");
// clang-format on

// #define FORMAT \
