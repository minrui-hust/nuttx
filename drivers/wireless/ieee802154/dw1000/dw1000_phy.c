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
#include <unistd.h>

#include <assert.h>
#include <debug.h>
#include <stdio.h>

#include "dw1000.h"

#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static dwt_config_t g_dw1000_def_cfg = {
    .chan = 2,
    .prf = DWT_PRF_64M,
    .txPreambLength = DWT_PLEN_128,
    .rxPAC = DWT_PAC8,
    .txCode = 9,
    .rxCode = 9,
    .nsSFD = 1,
    .dataRate = DWT_BR_6M8,
    .phrMode = DWT_PHRMODE_STD,
    .sfdTO = DWT_SFDTOC_DEF,
};

int dw1000_reset(FAR struct uwb_phy_s *phy) {
  int ret = OK;
  FAR struct dw1000_dev_s *dev = (FAR struct dw1000_dev_s *)phy;
  FAR struct dw1000_lower_s *lower = dev->lower;
  DEBUGASSERT(dev != NULL && lower != NULL);

  // lock device
  nxmutex_lock(&dev->lock);

  // disable irq
  lower->irq->enable(lower, false);

  // hard reset
  struct gpio_dev_s *rst = lower->rst;
  rst->gp_ops->go_setpintype(rst, GPIO_OUTPUT_PIN); // set pin to output
  rst->gp_ops->go_write(rst, 0);                    // set to low
  rst->gp_ops->go_setpintype(rst, GPIO_INPUT_PIN);  // set to input float
  usleep(2000);

  // initialize
  SPI_SETFREQUENCY(lower->spi, 1000000); // 1M max
  if (dwt_initialise(DWT_LOADUCODE) != DWT_SUCCESS) {
    ret = ERROR;
    goto out;
  }
  SPI_SETFREQUENCY(lower->spi, 20000000); // 20M max

  // defaut config
  dev->cfg = g_dw1000_def_cfg;
  dwt_configure(&dev->cfg);

#ifdef CONFIG_DW1000_USE_PA
  dwt_setfinegraintxseq(0);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
#endif

  // enable irq
  lower->irq->enable(lower, true);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

int dw1000_bind(FAR struct uwb_phy_s *phy, FAR struct uwb_phy_cbs_s *cbs) {
  FAR struct dw1000_dev_s *dev = (FAR struct dw1000_dev_s *)phy;
  dev->cbs = cbs;
  return OK;
}

int dw1000_irq_enable(FAR struct uwb_phy_s *phy) {
  uint32_t tx_mask = SYS_STATUS_TXFRS;
  uint32_t rx_mask = SYS_STATUS_RXDFR | SYS_STATUS_RXPHE | SYS_STATUS_RXRFSL |
                     SYS_STATUS_RXSFDTO | SYS_STATUS_LDEERR;

  dwt_setinterrupt(tx_mask | rx_mask, 2);

  return OK;
}

int dw1000_irq_check(FAR struct uwb_phy_s *phy) {
  FAR struct dw1000_dev_s *dev = (FAR struct dw1000_dev_s *)phy;
  return work_queue(HPWORK, &dev->poll_work, dw1000_poll, dev, 0);
}

int dw1000_rx_read(FAR struct uwb_phy_s *phy,
                   FAR struct uwb_phy_rxdesc_s *rxdesc) {
  uint8_t rx_len = dwt_read8bitoffsetreg(RX_FINFO_ID, 0);

  rxdesc->frame->io_pktlen = rx_len - 2;

  dwt_readrxdata(rxdesc->frame->io_data, rxdesc->frame->io_pktlen, 0);

  return OK;
}

int dw1000_rx_start(FAR struct uwb_phy_s *phy) {
  return dwt_rxenable(DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS);
}

int dw1000_rx_delay_start(FAR struct uwb_phy_s *phy, uint32_t when) {
  dwt_setdelayedtrxtime(when);
  return dwt_rxenable(DWT_START_RX_IMMEDIATE | DWT_START_RX_DELAYED |
                      DWT_NO_SYNC_PTRS);
}

int dw1000_tx_write(FAR struct uwb_phy_s *phy,
                    FAR struct uwb_phy_txdesc_s *txdesc) {
  dwt_writetxfctrl(txdesc->frame->io_pktlen + 2, 0, 1);

  return dwt_writetxdata(txdesc->frame->io_pktlen + 2, txdesc->frame->io_data,
                         0);
}

int dw1000_tx_start(FAR struct uwb_phy_s *phy) {
  return dwt_starttx(DWT_START_TX_IMMEDIATE);
}

int dw1000_tx_delay_start(FAR struct uwb_phy_s *phy, uint32_t when) {
  dwt_setdelayedtrxtime(when);
  return dwt_starttx(DWT_START_TX_DELAYED);
}

int dw1000_setattr(FAR struct uwb_phy_s *phy, int attr_id,
                   FAR void *attr_value) {
  int ret = OK;
  FAR struct dw1000_dev_s *dev = (FAR struct dw1000_dev_s *)phy;

  nxmutex_lock(&dev->lock);

  switch (attr_id) {
  case UWB_PHY_CHAN: {
    dev->cfg.chan = *((uint8_t *)attr_value);
    dwt_configure(&dev->cfg);
  } break;
  case UWB_PHY_PRF: {
    dev->cfg.prf = *((uint8_t *)attr_value);
    dwt_configure(&dev->cfg);
  } break;
  case UWB_PHY_PRL: {
    dev->cfg.txPreambLength = *((uint8_t *)attr_value);
    dwt_configure(&dev->cfg);
  } break;
  case UWB_PHY_PAC: {
    dev->cfg.rxPAC = *((uint8_t *)attr_value);
    dwt_configure(&dev->cfg);
  } break;
  case UWB_PHY_TXCODE: {
    dev->cfg.txCode = *((uint8_t *)attr_value);
    dwt_configure(&dev->cfg);
  } break;
  case UWB_PHY_RXCODE: {
    dev->cfg.rxCode = *((uint8_t *)attr_value);
    dwt_configure(&dev->cfg);
  } break;
  case UWB_PHY_DR: {
    dev->cfg.dataRate = *((uint8_t *)attr_value);
    dwt_configure(&dev->cfg);
  } break;
  case UWB_PHY_SFDTO: {
    dev->cfg.sfdTO = *((uint16_t *)attr_value);
    dwt_configure(&dev->cfg);
  } break;
  case DW1000_PHY_NSSFD: {
    dev->cfg.nsSFD = *((uint8_t *)attr_value);
    dwt_configure(&dev->cfg);
  } break;
  case DW1000_PHY_PHRMODE: {
    dev->cfg.phrMode = *((uint8_t *)attr_value);
    dwt_configure(&dev->cfg);
  } break;
  default: {
    ret = ERROR;
  } break;
  }

  nxmutex_unlock(&dev->lock);

  return ret;
}

int dw1000_getattr(FAR struct uwb_phy_s *phy, int attr_id,
                   FAR void *attr_value) {
  // TODO
  return OK;
}

struct uwb_phy_ops_s g_dw1000_phy_ops = {
    .reset = dw1000_reset,

    .bind = dw1000_bind,

    .irq_enable = dw1000_irq_enable,
    .irq_check = dw1000_irq_check,

    .rx_read = dw1000_rx_read,
    .rx_start = dw1000_rx_start,
    .rx_delay_start = dw1000_rx_delay_start,

    .tx_write = dw1000_tx_write,
    .tx_start = dw1000_tx_start,
    .tx_delay_start = dw1000_tx_delay_start,

    .setattr = dw1000_setattr,
    .getattr = dw1000_getattr,
};
