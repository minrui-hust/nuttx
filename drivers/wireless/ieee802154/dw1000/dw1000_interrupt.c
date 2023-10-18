/****************************************************************************
 * drivers/wireless/ieee802154/dw1000/dw1000_phy.c
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

#include <nuttx/mm/iob.h>
#include <nuttx/mutex.h>

#include <assert.h>
#include <debug.h>
#include <stdio.h>

#include "dw1000.h"

#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"

CODE void dw1000_poll(FAR void *arg) {
  FAR struct dw1000_dev_s *dev = (FAR struct dw1000_dev_s *)arg;

  // read system status
  uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);

  // clear all flags
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_RXFCG |
                                       SYS_STATUS_ALL_RX_ERR);

  // check if receiver is needed
  if (status & (SYS_STATUS_RXPHE | SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL |
                SYS_STATUS_LDEERR)) {
    dwt_rxreset();
  }

  // do callbacks
  if (status & SYS_STATUS_TXFRS) {
    if (dev->cbs && dev->cbs->txdone) {
      dev->cbs->txdone(&dev->phy);
    }
  }
  if (status & 0x0) {
    if (dev->cbs && dev->cbs->txerror) {
      dev->cbs->txerror(&dev->phy);
    }
  }
  if (status & SYS_STATUS_RXFCG) {
    if (dev->cbs && dev->cbs->rxdone) {
      dev->cbs->rxdone(&dev->phy);
    }
  }
  if (status & SYS_STATUS_ALL_RX_ERR) {
    if (dev->cbs && dev->cbs->rxerror) {
      dev->cbs->rxerror(&dev->phy);
    }
  }
}

CODE void dw1000_isr(FAR void *arg) {
  dwt_setinterrupt(0, 2);
  dw1000_poll(arg);
}

int dw1000_interrupt(int irq, FAR void *context, FAR void *arg) {
  FAR struct dw1000_dev_s *dev = (FAR struct dw1000_dev_s *)arg;
  return work_queue(HPWORK, &dev->irq_work, dw1000_isr, dev, 0);
}
