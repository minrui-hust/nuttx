/****************************************************************************
 * include/nuttx/wireless/uwb/dw1000.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_DW1000_H
#define __INCLUDE_NUTTX_WIRELESS_DW1000_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/spi/spi.h>

struct dw1000_lower_s;

struct dw1000_irq_ops {
  int (*attach)(FAR struct dw1000_lower_s *lower, xcpt_t handler,
                FAR void *arg);
  void (*enable)(FAR struct dw1000_lower_s *lower, bool state);
  int (*stat)(FAR struct dw1000_lower_s *lower);
};

struct dw1000_lower_s {
  FAR struct spi_dev_s *spi;
  FAR struct gpio_dev_s *rst;
  FAR struct dw1000_irq_ops *irq;
};

FAR struct uwb_phy_s *dw1000_init(FAR struct dw1000_lower_s *lower);

#endif /* __INCLUDE_NUTTX_WIRELESS_UWB_DW1000_H */
