/****************************************************************************
 * wireless/uwb/uwb_phy_device.c
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

#include <debug.h>
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/wireless/uwb/uwb_phy.h>

#include <sys/types.h>

#define DEVNAME_FMT "/dev/uwb%d"
#define DEVNAME_FMTLEN (8 + 1 + 1) // uwb 0~9

static struct uwb_phy_rxdesc_s g_rxdesc;

static int uwb_phy_open(FAR struct file *filep) { return OK; }

static int uwb_phy_close(FAR struct file *filep) { return OK; }

static ssize_t uwb_phy_read(FAR struct file *filep, FAR char *buffer,
                            size_t len) {
  len = g_rxdesc.frame->io_pktlen;

  memcpy(buffer, g_rxdesc.frame->io_data, len);

  g_rxdesc.frame->io_pktlen = 0;

  printf("recv pkt, len: %d\n", len);

  return len;
}

static ssize_t uwb_phy_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len) {
  int ret = OK;
  FAR struct uwb_phy_s *phy;

  phy = filep->f_inode->i_private;
  DEBUGASSERT(phy != NULL);

  struct uwb_phy_txdesc_s tx_desc;
  tx_desc.frame = iob_alloc(false);
  tx_desc.frame->io_offset = 0;
  tx_desc.frame->io_len = len;
  tx_desc.frame->io_pktlen = len;
  memcpy(&tx_desc.frame->io_data, buffer, len);

  ret = phy->ops->tx_write(phy, &tx_desc);
  ret = phy->ops->tx_start(phy);

  iob_free(tx_desc.frame);

  return ret;
}

static int uwb_phy_ioctl(FAR struct file *filep, int cmd, unsigned long arg) {
  return OK;
}

static void uwb_phy_txdone(FAR struct uwb_phy_s *phy) {}

static void uwb_phy_txerror(FAR struct uwb_phy_s *phy) {}

static void uwb_phy_rxdone(FAR struct uwb_phy_s *phy) {
  phy->ops->rx_start(phy);
  phy->ops->rx_read(phy, &g_rxdesc);
  printf("recv: %s\n", g_rxdesc.frame->io_data);
  phy->ops->irq_enable(phy);
}

static void uwb_phy_rxerror(FAR struct uwb_phy_s *phy) {
  phy->ops->rx_start(phy);
  phy->ops->irq_enable(phy);
  printf("recv err\n");
}

static struct uwb_phy_cbs_s g_uwb_phy_cbs = {
    .txdone = uwb_phy_txdone,
    .txerror = uwb_phy_txerror,
    .rxdone = uwb_phy_rxdone,
    .rxerror = uwb_phy_rxerror,
};

static const struct file_operations g_uwb_phy_fops = {
    uwb_phy_open,  /* open */
    uwb_phy_close, /* close */
    uwb_phy_read,  /* read */
    uwb_phy_write, /* write */
    NULL,          /* seek */
    uwb_phy_ioctl, /* ioctl */
};

int uwb_phy_register(FAR struct uwb_phy_s *phy, int minor) {
  char devname[DEVNAME_FMTLEN];
  int ret = OK;

  g_rxdesc.frame = iob_alloc(false);

  phy->ops->bind(phy, &g_uwb_phy_cbs);
  phy->ops->rx_start(phy);
  phy->ops->irq_enable(phy);

  snprintf(devname, DEVNAME_FMTLEN, DEVNAME_FMT, minor);
  ret = register_driver(devname, &g_uwb_phy_fops, 0666, phy);
  if (ret < 0) {
    wlerr("ERROR: register %s failed: %d\n", devname, ret);
  }

  return ret;
}
