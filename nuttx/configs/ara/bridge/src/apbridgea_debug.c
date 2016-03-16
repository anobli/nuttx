#include <stdio.h>
#include <string.h>
#include <nuttx/list.h>
#include <nuttx/bufram.h>
#include <apps/greybus-utils/utils.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/greybus_timestamp.h>
#include <nuttx/unipro/unipro.h>
#include <arch/board/common_gadget.h>
#include <arch/board/apbridgea_gadget.h>
#include <arch/board/apbridgea_unipro.h>

struct loopback_rx {
    struct list_head list;
    unsigned int cportid;
    void *buf;
    int len;
    void *priv;
};

#undef LATENCY_TAG
#undef LOOPBACK
#define FAKE_DATA
#define FAKE_RESPONSE

#ifdef LATENCY_TAG
static struct gb_timestamp *ts;
#endif
#ifdef LOOPBACK
static struct list_head loopback_rx_list;
#endif

#if defined(LOOPBACK) || defined(FAKE_DATA)
static pthread_t g_debug_thread;
static sem_t debug_lock;
#endif

#ifdef LATENCY_TAG
static int tag_tx(unsigned int cportid, void *buf, size_t len, void *priv)
{
	gb_timestamp_tag_entry_time(&ts[cportid], cportid);

	return unipro_tx_transfer(cportid, buf, len, priv);
}

static int tag_rx(unsigned int cportid, void *buf, size_t len, void *priv)
{
	struct gb_operation_hdr *gbhdr;

    gbhdr = (struct gb_operation_hdr *)buf;
    gb_timestamp_tag_exit_time(&ts[cportid], cportid);

    /* Skip the timestamping if it's not a response from GPB to AP. */
    if (gbhdr->type & GB_TYPE_RESPONSE_FLAG) {
        gb_timestamp_log(&ts[cportid], cportid,
                         buf, len, GREYBUS_FW_TIMESTAMP_APBRIDGE);
    }

	return usb_rx_transfer(cportid, buf, len, priv);
}
#endif

#ifdef LOOPBACK
static int loopback_tx(unsigned int cportid, void *buf, size_t len, void *priv)
{
    struct loopback_rx *rx;

    rx = kmm_malloc(sizeof(*rx));
    if (!rx) {
        lowsyslog("DTC\n");
        return -ENOMEM;
    }

    rx->cportid = cportid;
    rx->buf = buf;
    rx->len = len;
    rx->priv = priv;
    list_init(&rx->list);
    list_add(&loopback_rx_list, &rx->list);
    sem_post(&debug_lock);

    return 0;
}

static void *debug_fn(void *p_data)
{
    irqstate_t flags;
    void *rxbuf;
    struct loopback_rx *rx;
    struct gb_operation_hdr *gbhdr;

    while(1) {
        sem_wait(&debug_lock);

        do {
            rxbuf = bufram_page_alloc(bufram_size_to_page_count(CPORT_BUF_SIZE));
            if (!rxbuf) {
                usleep(1);
            }
        } while (!rxbuf);

        flags = irqsave();
        rx = list_entry(loopback_rx_list.next, struct loopback_rx, list);
        list_del(&rx->list);
        irqrestore(flags);

        memcpy(rxbuf, rx->buf, rx->len);
        usb_release_buffer(NULL, rx->buf);

        gbhdr = (struct gb_operation_hdr *)rxbuf;
        gbhdr->type |= 0x80;
        usb_rx_transfer(rx->cportid, rxbuf, rx->len, rx->priv);

        kmm_free(rx);
    }
    return NULL;
}
#endif /* LOOPBACK */

#ifdef FAKE_DATA
static int fake_tx(unsigned int cportid, void *buf, size_t len, void *priv)
{
    usb_release_buffer(NULL, buf);
    return 0;
}

static void *debug_fn(void *p_data)
{
    int id = 0;
    irqstate_t flags;
    void *rxbuf;
    struct gb_operation_hdr *gbhdr;

    sem_wait(&debug_lock);
    while(1) {
        do {
            rxbuf = bufram_page_alloc(bufram_size_to_page_count(CPORT_BUF_SIZE));
            if (!rxbuf) {
                usleep(1);
            }
        } while (!rxbuf);

        gbhdr = (struct gb_operation_hdr *)rxbuf;

#ifdef FAKE_RESPONSE
        gbhdr->type = 0x82;
#else
        gbhdr->type = 0x02;
#endif
        gbhdr->size = 128 + sizeof(*gbhdr);
        gbhdr->result = 0;
        gbhdr->id = id++;

        usb_rx_transfer(5, rxbuf, gbhdr->size, get_apbridge_dev());
    }
    return NULL;
}
#endif /* FAKE_DATA */

static int latency_tag_en_vendor_request_out(struct usbdev_s *dev, uint8_t req,
                                             uint16_t index, uint16_t value,
                                             void *buf, uint16_t len)
{
    int ret = -EINVAL;
    struct apbridge_dev_s *priv = usbdev_to_apbridge(dev);

    if (value < unipro_cport_count()) {
#if defined(LATENCY_TAG)
        ret = register_cport_callback(priv, value, tag_rx, tag_tx);
#elif defined(LOOPBACK)
        ret = register_cport_callback(priv, value, usb_rx_transfer, loopback_tx);
#elif defined(FAKE_DATA)
        ret = register_cport_callback(priv, value, usb_rx_transfer, fake_tx);
#endif
        lldbg("enable tagging for cportid %d\n", value);
#ifdef FAKE_DATA
        sem_post(&debug_lock);
#endif
    }
    return ret;
}

static int latency_tag_dis_vendor_request_out(struct usbdev_s *dev, uint8_t req,
                                              uint16_t index, uint16_t value,
                                              void *buf, uint16_t len)
{
    int ret = -EINVAL;
    struct apbridge_dev_s *priv = usbdev_to_apbridge(dev);

    if (value < unipro_cport_count()) {
        ret = unregister_cport_callback(priv, value);
        lldbg("disable tagging for cportid %d\n", value);
    }
    return ret;
}

int apbridgea_debug_init(void)
{
    int ret = 0;

#ifdef LATENCY_TAG
    ts = kmm_malloc(sizeof(struct gb_timestamp) * unipro_cport_count());
    if (!ts) {
        return -ENOMEM;
    }
    gb_timestamp_init();
#endif

    if (register_vendor_request(APBRIDGE_ROREQUEST_LATENCY_TAG_EN, VENDOR_REQ_OUT,
                                latency_tag_en_vendor_request_out))
        printf("Fail to register APBRIDGE_ROREQUEST_LATENCY_TAG_EN"
               " vendor request\n");
    if (register_vendor_request(APBRIDGE_ROREQUEST_LATENCY_TAG_DIS, VENDOR_REQ_OUT,
                                latency_tag_dis_vendor_request_out))
        printf("Fail to register APBRIDGE_ROREQUEST_LATENCY_TAG_DIS"
               " vendor request\n");

#ifdef LOOPBACK
    list_init(&loopback_rx_list);
#endif
#if defined(LOOPBACK) || defined(FAKE_DATA)
    sem_init(&debug_lock, 0, 0);
    ret = pthread_create(&g_debug_thread, NULL,
                         debug_fn, NULL);
#endif
    return ret;
}

void apbridgea_debug_free(void)
{
#ifdef LATENCY_TAG
	kmm_free(ts);
#endif
}