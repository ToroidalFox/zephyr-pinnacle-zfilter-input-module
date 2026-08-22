// #include <zephyr/init.h>
#include "zephyr/kernel.h"
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <zephyr/drivers/i2c.h>
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <zephyr/drivers/spi.h>
#endif

#define PINNACLE_READ (BIT(7) | BIT(5))
#define PINNACLE_WRITE BIT(7)

#define PINNACLE_FEEDCONFIG1_FEED_ENABLE BIT(0)
#define PINNACLE_FEEDCONFIG1_DATA_MODE_ABSOLUTE BIT(1)
#define PINNACLE_FEEDCONFIG1_FILTER_DISABLE BIT(2)
#define PINNACLE_FEEDCONFIG1_X_DISABLE BIT(3)
#define PINNACLE_FEEDCONFIG1_Y_DISABLE BIT(4)
// UNUSED BIT(5)
#define PINNACLE_FEEDCONFIG1_X_INVERT BIT(6)
#define PINNACLE_FEEDCONFIG1_Y_INVERT BIT(7)

#define PINNACLE_FEED_CONFIG2_INTELLIMOUSE_ENABLE BIT(0)
#define PINNACLE_FEED_CONFIG2_ALL_TAPS_DISABLE BIT(1)
#define PINNACLE_FEED_CONFIG2_SECONDARY_TAP_DISABLE BIT(2)
#define PINNACLE_FEED_CONFIG2_SCROLL_DISABLE BIT(3)
#define PINNACLE_FEED_CONFIG2_GLIDE_EXTEND_DISABLE BIT(4)
// UNUSED BIT(5)
// UNUSED BIT(6)
#define PINNACLE_FEED_CONFIG2_SWAP_X_AND_Y BIT(7)

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
};
#define SHIFT 10
#define HALF (1 << (SHIFT - 1))
#define MAP_SCALE_INV (1 << (SHIFT - MAP_SCALE_SHIFT))
// fixed point lerp with 10bit frac
static uint32_t lerp(uint32_t a, uint32_t b, uint32_t t) {
  return a + (((b - a) * t) >> SHIFT);
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

// clang-format on
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

struct pinnacle_zfilter_config {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
  struct spi_dt_spec spi;
#elif DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
  struct i2c_dt_spec i2c;
#endif

  uint16_t clamp_x_min;
  uint16_t clamp_x_max;
  uint16_t clamp_y_min;
  uint16_t clamp_y_max;
};

struct pinnacle_zfilter_data {
  struct gpio_callback *gpio_callback;
  struct k_work *callback_work;
};

static int bus_read(const struct device *device, uint8_t addr) {
  const struct pinnacle_zfilter_config *config = device->config;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#elif DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#endif
}

static int bus_write(const struct device *device, uint8_t addr, uint8_t value) {
  const struct pinnacle_zfilter_config *config = device->config;
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#elif DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#endif
}

static void clamp_touch(const struct device *device, struct abs_touch *touch) {
  const struct pinnacle_zfilter_config *config = device->config;

  touch->x = CLAMP(touch->x, config->clamp_x_min, config->clamp_x_max - 1);
  touch->y = CLAMP(touch->y, config->clamp_y_min, config->clamp_y_max - 1);
}

static void pinnacle_zfilter_gpio_callback(const struct device *device,
                                           struct gpio_callback *gpio_callback,
                                           uint32_t pins) {
  struct pinnacle_zfilter_data *data =
      CONTAINER_OF(gpio_callback, struct pinnacle_zfilter_data, gpio_callback);

  k_work_submit(data->callback_work);
}

static int pinnacle_zfilter_init(const struct device *device) {
  const struct pinnacle_zfilter_config *config = device->config;

  return 0;
}
