/****************************************************************************
 * boards/arm/stm32/gd32f103-uwb-eval/src/stm32_dw1000.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/ioexpander/gpio.h>

#include <nuttx/wireless/uwb/dw1000.h>
#include <nuttx/wireless/uwb/uwb_phy.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "stm32f103_minimum.h"

#ifdef CONFIG_IEEE802154_DW1000

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_dw1000_lower_s {
  struct dw1000_lower_s base;
  bool enabled;
  xcpt_t handler;
  void *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gp_write(struct gpio_dev_s *dev, bool value);
static int gp_setpintype(FAR struct gpio_dev_s *dev,
                         enum gpio_pintype_e pintype);

static int attach_irq(FAR struct dw1000_lower_s *lower, xcpt_t handler,
                      FAR void *arg);

static void enable_irq(FAR struct dw1000_lower_s *lower, bool state);

static int stat_irq(FAR struct dw1000_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s g_pin_rst_ops = {
    .go_write = gp_write,
    .go_setpintype = gp_setpintype,
};

static struct gpio_dev_s g_pin_rst = {
    .gp_pintype = GPIO_INPUT_PIN,
    .gp_ops = &g_pin_rst_ops,
};

static struct dw1000_irq_ops g_irq_ops = {
    .attach = attach_irq,
    .enable = enable_irq,
    .stat = stat_irq,
};

static struct stm32_dw1000_lower_s g_dw1000_lower = {
    .base.irq = &g_irq_ops,
    .base.rst = &g_pin_rst,
    .enabled = false,
};

/****************************************************************************
 * Private Function
 ****************************************************************************/

static int gp_write(struct gpio_dev_s *dev, bool value) {
  FAR struct dw1000_gpio_dev_s *dw1000_dev =
      (FAR struct dw1000_gpio_dev_s *)dev;

  DEBUGASSERT(dw1000_dev != NULL);

  stm32_gpiowrite(GPIO_DW1000_RST, value);

  return OK;
}

static int gp_setpintype(FAR struct gpio_dev_s *dev,
                         enum gpio_pintype_e pintype) {

  FAR struct dw1000_gpio_dev_s *dw1000_dev =
      (FAR struct dw1000_gpio_dev_s *)dev;

  DEBUGASSERT(dw1000_dev != NULL);

  switch (pintype) {
  case GPIO_INPUT_PIN: {
    stm32_configgpio(GPIO_DW1000_RST | GPIO_INPUT | GPIO_CNF_INFLOAT);
  } break;
  case GPIO_OUTPUT_PIN: {
    stm32_configgpio(GPIO_DW1000_RST | GPIO_OUTPUT | GPIO_CNF_OUTPP |
                     GPIO_MODE_50MHz);
  } break;
  default:
    return ERROR;
  }

  return OK;
}

static int attach_irq(FAR struct dw1000_lower_s *lower, xcpt_t handler,
                      FAR void *arg) {
  FAR struct stm32_dw1000_lower_s *stm32_lower =
      (FAR struct stm32_dw1000_lower_s *)lower;
  DEBUGASSERT(stm32_lower);

  stm32_lower->handler = handler;
  stm32_lower->arg = arg;

  return OK;
}

static void enable_irq(FAR struct dw1000_lower_s *lower, bool state) {
  FAR struct stm32_dw1000_lower_s *stm32_lower =
      (FAR struct stm32_dw1000_lower_s *)lower;
  DEBUGASSERT(stm32_lower);

  if (state) {
    stm32_gpiosetevent(GPIO_DW1000_IRQ, true, false, true, stm32_lower->handler,
                       stm32_lower->arg);
  } else {
    stm32_gpiosetevent(GPIO_DW1000_IRQ, false, false, false, NULL, NULL);
  }

  stm32_lower->enabled = state;
}

static int stat_irq(FAR struct dw1000_lower_s *lower) {
  FAR struct stm32_dw1000_lower_s *stm32_lower =
      (FAR struct stm32_dw1000_lower_s *)lower;
  DEBUGASSERT(stm32_lower);

  return stm32_lower->enabled;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_dw1000_initialize(void) {
  int ret = OK;
  struct uwb_phy_s *phy;

  wlinfo("Configuring DW1000 \n");

  // config gpios dw1000 used
  stm32_configgpio(GPIO_DW1000_RST | GPIO_INPUT | GPIO_CNF_INFLOAT);
  stm32_configgpio(GPIO_DW1000_IRQ);
  stm32_configgpio(GPIO_DW1000_CS);

  // create spi
  g_dw1000_lower.base.spi = stm32_spibus_initialize(DW1000_SPI_ID);
  if (NULL == g_dw1000_lower.base.spi) {
    wlerr("ERROR: Failed to initialize SPI bus %d\n", DW1000_SPI_ID);
    return -ENODEV;
  }

  phy = dw1000_init((FAR struct dw1000_lower_s *)(&g_dw1000_lower));
  if (NULL == phy) {
    wlerr("ERROR: Failed to initialize dw1000 phy\n");
    return -ENODEV;
  }

  ret = uwb_phy_register(phy, 0);
  if (ret < 0) {
    wlerr("ERROR: Failed to register dw1000 phy as char device \n");
  }

  return ret;
}

#endif /* CONFIG_IEEE802154_DW1000 */
