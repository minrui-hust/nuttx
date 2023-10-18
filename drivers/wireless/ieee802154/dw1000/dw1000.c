/****************************************************************************
 * drivers/wireless/ieee802154/dw1000/dw1000.c
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

#include <nuttx/kmalloc.h>

#include "dw1000.h"

extern FAR struct dw1000_dev_s *g_dw1000_dev;

extern struct uwb_phy_ops_s g_dw1000_phy_ops;

FAR struct uwb_phy_s *dw1000_init(FAR struct dw1000_lower_s *lower) {

  // alloc dev
  g_dw1000_dev = kmm_zalloc(sizeof(struct dw1000_dev_s));
  if (NULL == g_dw1000_dev) {
    return NULL;
  }

  // initialize dev
  g_dw1000_dev->lower = lower;
  g_dw1000_dev->phy.ops = &g_dw1000_phy_ops;

  nxmutex_init(&g_dw1000_dev->lock);

  // attach irq
  if (lower->irq->attach(lower, dw1000_interrupt, g_dw1000_dev) != OK) {
    goto errout;
  }

  // reset the phy
  if (g_dw1000_dev->phy.ops->reset(&g_dw1000_dev->phy) != OK) {
    goto errout;
  }

  return &g_dw1000_dev->phy;

errout:
  nxmutex_destroy(&g_dw1000_dev->lock);
  kmm_free(g_dw1000_dev);
  g_dw1000_dev = NULL;
  return NULL;
}
