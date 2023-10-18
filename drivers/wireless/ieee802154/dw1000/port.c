/****************************************************************************
 * drivers/wireless/ieee802154/dw1000/port.c
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

#include "decadriver/deca_device_api.h"
#include "dw1000.h"

FAR struct dw1000_dev_s *g_dw1000_dev = NULL;

int writetospi(uint16 headerLength, const uint8 *headerBuffer,
               uint32 bodylength, const uint8 *bodyBuffer) {
  if (NULL == g_dw1000_dev || NULL == g_dw1000_dev->lower) {
    return DWT_ERROR;
  }

  struct spi_dev_s *spi = g_dw1000_dev->lower->spi;
  if (NULL == spi) {
    return DWT_ERROR;
  }

  SPI_SELECT(spi, 0, true);
  SPI_SNDBLOCK(spi, headerBuffer, headerLength);
  SPI_SNDBLOCK(spi, bodyBuffer, bodylength);
  SPI_SELECT(spi, 0, false);

  return DWT_SUCCESS;
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer,
                uint32 readlength, uint8 *readBuffer) {
  if (NULL == g_dw1000_dev || NULL == g_dw1000_dev->lower) {
    return DWT_ERROR;
  }

  struct spi_dev_s *spi = g_dw1000_dev->lower->spi;
  if (NULL == spi) {
    return DWT_ERROR;
  }

  SPI_SELECT(spi, 0, true);
  SPI_SNDBLOCK(spi, headerBuffer, headerLength);
  SPI_RECVBLOCK(spi, readBuffer, readlength);
  SPI_SELECT(spi, 0, false);

  return DWT_SUCCESS;
}

decaIrqStatus_t decamutexon(void) {
  DEBUGASSERT(NULL != g_dw1000_dev && NULL != g_dw1000_dev->lower);

  struct dw1000_lower_s *lower = g_dw1000_dev->lower;

  decaIrqStatus_t s = lower->irq->stat(lower);

  if (s) {
    lower->irq->enable(lower, false);
  }

  return s;
}

void decamutexoff(decaIrqStatus_t s) {
  DEBUGASSERT(NULL != g_dw1000_dev && NULL != g_dw1000_dev->lower);

  struct dw1000_lower_s *lower = g_dw1000_dev->lower;

  if (s) {
    lower->irq->enable(lower, true);
  }
}

void deca_sleep(unsigned int time_ms) { usleep(time_ms * USEC_PER_MSEC); }
