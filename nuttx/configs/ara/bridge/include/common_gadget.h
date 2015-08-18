#ifndef _COMMON_GADGET_H_
#define _COMMON_GADGET_H_

#include <errno.h>
#include <assert.h>
#include <stddef.h>
#include <nuttx/list.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>

#include <arch/irq.h>

typedef void (*usb_callback)(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req);

void request_pool_init(void);
int request_pool_prealloc(struct usbdev_ep_s *ep, size_t len, int n);
struct usbdev_req_s *get_request(struct usbdev_ep_s *ep,
                                 usb_callback callback,
                                 size_t len, void *priv);
void put_request(struct usbdev_req_s *req);
struct usbdev_req_s *find_request_by_priv(const void *priv);
struct usbdev_ep_s *request_to_ep(struct usbdev_req_s *req);
void request_set_priv(struct usbdev_req_s *req, void *priv);
void *request_get_priv(struct usbdev_req_s *req);

#endif /* _COMMON_GADGET_H_ */

