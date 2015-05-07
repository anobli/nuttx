/*
 * Copyright (c) 2014-2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <arch/tsb/gpio.h>
#include <arch/tsb/unipro.h>
#include <nuttx/usb/apb_es1.h>
#include <apps/greybus-utils/utils.h>

#include "apbridge_backend.h"

/*
 * TODO
 * Already defined in tsb_unipro.c
 * Move them to tsb_unipro.h
 */

#define CPORTID_CDSI0    (16)
#define CPORTID_CDSI1    (17)

static int unipro_usb_to_unipro(unsigned int cportid, void *buf, size_t len)
{
    return unipro_send(cportid, buf, len);
}

static int unipro_usb_to_svc(void *buf, size_t len)
{
    return svc_handle(buf, len);
}

static int unipro_unipro_to_usb(struct apbridge_dev_s *dev,
                                unsigned int cportid, const void *buf,
                                size_t len)
{
    return unipro_to_usb(dev, buf, len);
}

static int unipro_svc_to_usb(struct apbridge_dev_s *dev,
                             void *buf, size_t len)
{
    return svc_to_usb(dev, buf, len);
}

static int unipro_recv_from_unipro(unsigned int cportid,
                                   void *buf, size_t len)
{
    return recv_from_unipro(cportid, buf, len);
}

static struct unipro_driver unipro_driver = {
    .name = "APBridge",
    .rx_handler = unipro_recv_from_unipro,
};

void apbridge_backend_init(void)
{
    int i;

    /* We have no way to to know if switch has setup route: just wait */
    sleep(2);

    /* Try to init any other cports */
    for (i = 0; i < CPORT_MAX; i++) {
        /* This cports are already allocated for display and camera */
        if (i == CPORTID_CDSI0 || i == CPORTID_CDSI1)
            continue;
        unipro_init_cport(i);
        unipro_driver_register(&unipro_driver, i);
    }
}

void apbridge_backend_register(struct apbridge_backend *apbridge_backend)
{
    apbridge_backend->usb_to_unipro = unipro_usb_to_unipro;
    apbridge_backend->unipro_to_usb = unipro_unipro_to_usb;
    apbridge_backend->usb_to_svc = unipro_usb_to_svc;
    apbridge_backend->svc_to_usb = unipro_svc_to_usb;
}
