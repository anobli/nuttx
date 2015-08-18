#include <nuttx/config.h>

#include <string.h>
#include <nuttx/util.h>
#include <arch/board/common_gadget.h>

struct request_list {   
    struct list_head list;
    struct usbdev_ep_s *ep;
    struct usbdev_req_s *req;
    size_t len;                 /* size of allocated buffer */
    void *priv;
};

struct request_pool_s
{
    size_t len;
    struct list_head request;
    struct list_head request_in_use;
    struct list_head pool;
};
struct list_head request_pool;

struct request_pool_s *request_pool_alloc(size_t len)
{
    struct request_pool_s *pool;

    pool = kmm_malloc(sizeof(*pool));
    if (!pool) {
        return NULL;
    }
    pool->len = len;
    list_init(&pool->request);
    list_init(&pool->request_in_use);

    list_add(&request_pool, &pool->pool);

    return pool;
}

struct request_pool_s *get_request_pool(size_t len)
{
    struct list_head *iter;
    struct request_pool_s *pool;

    list_foreach(&request_pool, iter) {
        pool = list_entry(iter, struct request_pool_s, pool);
        if (pool->len == len)
            return pool;
    }
    return NULL;
}

static void init_request(struct usbdev_req_s *req,
                         usb_callback callback, size_t len, void *priv)
{
    req->len = len;
    req->priv = priv;
    req->callback = callback;
}


static struct usbdev_req_s *alloc_request(struct usbdev_ep_s *ep,
                                          usb_callback callback,
                                          size_t len, void *priv)
{
    struct usbdev_req_s *req;

    DEBUGASSERT(ep);
//    DEBUGASSERT(callback);

    req = EP_ALLOCREQ(ep);
    if (!req) {
        return NULL;
    }

    if (!len) {
        req->buf = NULL;
    } else {
        req->buf = EP_ALLOCBUFFER(ep, len);
        if (!req->buf) {
            EP_FREEREQ(ep, req);
            return NULL;
        }
    }
    init_request(req, callback, len, priv);

    return req;
}

static void free_request(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
    if (!req)
        return;

    if (req->buf != NULL) {
        EP_FREEBUFFER(ep, req->buf);
        req->buf = NULL;
        req->len = 0;
    }
    EP_FREEREQ(ep, req);
}

size_t request_len_align(size_t len)
{
    if (!len) {
        return 0;
    }
    return ((--len / 128) + 1) * 128;
}

void request_set_priv(struct usbdev_req_s *req, void *priv)
{
    struct request_list *req_list = req->priv;
    req_list->priv = priv;
}

void *request_get_priv(struct usbdev_req_s *req)
{
    struct request_list *req_list = req->priv;
    return req_list->priv;
}

struct usbdev_req_s *get_request(struct usbdev_ep_s *ep,
                                 usb_callback callback,
                                 size_t len, void *priv)
{
    irqstate_t flags;
    size_t len_aligned;
    struct list_head *list;
    struct usbdev_req_s *req = NULL;
    struct request_list *req_list;
    struct request_pool_s *pool;

    len_aligned = request_len_align(len);
    pool = get_request_pool(len_aligned);
    if (!pool) {
        pool = request_pool_alloc(len_aligned);
        if (!pool) {
            return NULL;
        }
    }

    flags = irqsave();
    if (list_is_empty(&pool->request)) {
        irqrestore(flags);
        /* Assume device driver support ep = NULL */
        req_list = kmm_malloc(sizeof(*req_list));
        if (!req_list) {
            return NULL;
        }
        req = alloc_request(ep, callback, len, req_list);
        req_list->req = req;
        req_list->len = len_aligned;
        req_list->priv = priv;
        req_list->ep = ep;
        if (!req) {
            kmm_free(req_list);
            return NULL;
        }
        list_add(&pool->request_in_use, &req_list->list);
    } else {
        list = pool->request.next;
        req_list = list_entry(list, struct request_list, list);
        req_list->priv = priv;
        req_list->ep = ep;
        req = req_list->req;
        init_request(req, callback, len, req_list);
        list_del(&req_list->list);
        list_add(&pool->request_in_use, &req_list->list);
        irqrestore(flags);
    }
    return req;
}

void put_request(struct usbdev_req_s *req)
{
    irqstate_t flags;
    size_t len_aligned;
    struct request_list *req_list;
    struct request_pool_s *pool;

    DEBUGASSERT(req);
    req_list = (struct request_list *)req->priv;
    len_aligned = request_len_align(req_list->len);
    pool = get_request_pool(len_aligned);
    DEBUGASSERT(pool);

    flags = irqsave();
    list_del(&req_list->list);
    list_add(&pool->request, &req_list->list);
    irqrestore(flags);
}

struct usbdev_req_s *find_request_by_priv(const void *priv)
{
    struct list_head *pool_iter, *req_iter;
    struct request_pool_s *pool;
    struct request_list *req_list;

    list_foreach(&request_pool, pool_iter) {
        pool = list_entry(pool_iter, struct request_pool_s, pool);
        list_foreach(&pool->request_in_use, req_iter) {
            req_list = list_entry(req_iter, struct request_list, list);
            if (req_list->priv == priv) {
                return req_list->req;
            }
        }
    }
    return NULL;
}

struct usbdev_ep_s *request_to_ep(struct usbdev_req_s *req)
{
    struct request_list *req_list = req->priv;

    return req_list->ep;
}

int request_pool_prealloc(struct usbdev_ep_s *ep, size_t len, int n)
{
    int i;
    int ret = 0;
    struct usbdev_req_s *req[n];

    for (i = 0; i < n; i++) {
        req[i] = get_request(ep, NULL, len, NULL);
        if (!req[i]) {
            ret = -ENOMEM;
            break;
        }
    }
    for (i--; i >= 0; i--) {
        put_request(req[i]);
    }
    return ret;
}

void request_pool_init(void)
{
    list_init(&request_pool);
}

