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

#ifndef APBRIDGE_BACKEND_H
#define APBRIDGE_BACKEND_H

#include <nuttx/unipro/unipro.h>

enum ep_mapping;

typedef int (*apbridgea_intercept_handler)(unsigned int cportid, void *buf,
                                           size_t len);

struct apbridge_backend {
    int (*usb_to_unipro)(unsigned int cportid, void *buf, size_t len,
                         unipro_send_completion_t callback, void *priv);
    void (*unipro_cport_mapping)(unsigned int cportid, enum ep_mapping mapping);

    void (*init)(void);
};

int recv_from_unipro(unsigned int cportid, void *buf, size_t len);
void apbridge_backend_register(struct apbridge_backend *apbridge_backend);

#ifdef CONFIG_APBRIDGEA_INTERCEPT
int apbridgea_intercept_enable(unsigned int cportid,
                              apbridgea_intercept_handler from_usb,
                              apbridgea_intercept_handler from_unipro);
int apbridgea_intercept_disable(unsigned int cportid);
#else
static inline bool apbridgea_intercept_is_enabled(unsigned int cportid)
{
    return false;
}

static inline int apbridgea_intercept_enable(unsigned int cportid,
                                       apbridgea_intercept_handler from_usb,
                                       apbridgea_intercept_handler from_unipro)
{
    return -ENOSYS;
}

static inline int apbridgea_intercept_disable(unsigned int cportid)
{
    return -ENOSYS;
}

static inline int apbridgea_intercept_init(void)
{
    return 0;
}
#endif

#endif /* APBRIDGE_BACKEND_H */

