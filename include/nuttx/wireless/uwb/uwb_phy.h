/****************************************************************************
 * include/nuttx/wireless/uwb/uwb_phy.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_UWB_PHY_H
#define __INCLUDE_NUTTX_WIRELESS_UWB_PHY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>
#include <nuttx/mm/iob.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
enum uwb_phy_attr_e {
  UWB_PHY_CHAN = 0,
  UWB_PHY_PRF,    // pusle repeat frequency
  UWB_PHY_PRL,    // preamble length
  UWB_PHY_PAC,    // preamble aquisition chunk
  UWB_PHY_TXCODE, // tx preamble code
  UWB_PHY_RXCODE, // rx preamble code
  UWB_PHY_DR,     // data rate
  UWB_PHY_SFDTO,  // SFD timeout value
  UWB_PHY_BASE_ATTR_MAX,
};

union uwb_phy_attr_u {
  uint8_t chan;
  uint8_t prf;
  uint8_t prl;
  uint8_t pac;
  uint8_t txcode;
  uint8_t rxcode;
  uint8_t datarate;
  uint16_t sfd_timeout;
  uint8_t work_mode;
};

struct uwb_phy_txdesc_s {
  /* Pointer to the frame IOB */
  FAR struct iob_s *frame;
};

struct uwb_phy_rxdesc_s {
  /* Pointer to the frame IOB */
  FAR struct iob_s *frame;
};

/* UWB PHY Interface Operations **********************************/

struct uwb_phy_s;

struct uwb_phy_cbs_s {
  CODE void (*txdone)(FAR struct uwb_phy_s *phy);
  CODE void (*txerror)(FAR struct uwb_phy_s *phy);

  CODE void (*rxdone)(FAR struct uwb_phy_s *phy);
  CODE void (*rxerror)(FAR struct uwb_phy_s *phy);
};

struct uwb_phy_ops_s {
  CODE int (*reset)(FAR struct uwb_phy_s *phy);

  CODE int (*bind)(FAR struct uwb_phy_s *phy, FAR struct uwb_phy_cbs_s *cbs);

  CODE int (*irq_enable)(FAR struct uwb_phy_s *phy);
  CODE int (*irq_check)(FAR struct uwb_phy_s *phy);

  CODE int (*rx_read)(FAR struct uwb_phy_s *phy,
                      FAR struct uwb_phy_rxdesc_s *rxdesc);
  CODE int (*rx_start)(FAR struct uwb_phy_s *phy);
  CODE int (*rx_delay_start)(FAR struct uwb_phy_s *phy, uint32_t when);

  CODE int (*tx_write)(FAR struct uwb_phy_s *phy,
                       FAR struct uwb_phy_txdesc_s *txdesc);
  CODE int (*tx_start)(FAR struct uwb_phy_s *phy);
  CODE int (*tx_delay_start)(FAR struct uwb_phy_s *phy, uint32_t when);

  CODE int (*setattr)(FAR struct uwb_phy_s *phy, int attr_id,
                      FAR void *attr_value);
  CODE int (*getattr)(FAR struct uwb_phy_s *phy, int attr_id,
                      FAR void *attr_value);
};

struct uwb_phy_s {
  FAR struct uwb_phy_ops_s *ops;
};

int uwb_phy_register(FAR struct uwb_phy_s *phy, int minor);

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_UWB_PHY_H */
