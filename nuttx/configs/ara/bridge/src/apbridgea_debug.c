#include <stdio.h>
#include <apps/greybus-utils/utils.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/greybus_timestamp.h>
#include <nuttx/unipro/unipro.h>
#include <arch/board/common_gadget.h>
#include <arch/board/apbridgea_gadget.h>
#include <arch/board/apbridgea_unipro.h>

static struct gb_timestamp *ts;

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

static int latency_tag_en_vendor_request_out(struct usbdev_s *dev, uint8_t req,
                                             uint16_t index, uint16_t value,
                                             void *buf, uint16_t len)
{
    int ret = -EINVAL;
    struct apbridge_dev_s *priv = usbdev_to_apbridge(dev);

    if (value < unipro_cport_count()) {
        ret = register_cport_callback(priv, value, tag_rx, tag_tx);
        lldbg("enable tagging for cportid %d\n", value);
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
    ts = kmm_malloc(sizeof(struct gb_timestamp) * unipro_cport_count());
    if (!ts) {
        return -ENOMEM;
    }
    gb_timestamp_init();

    if (register_vendor_request(APBRIDGE_ROREQUEST_LATENCY_TAG_EN, VENDOR_REQ_OUT,
                                latency_tag_en_vendor_request_out))
        printf("Fail to register APBRIDGE_ROREQUEST_LATENCY_TAG_EN"
               " vendor request\n");
    if (register_vendor_request(APBRIDGE_ROREQUEST_LATENCY_TAG_DIS, VENDOR_REQ_OUT,
                                latency_tag_dis_vendor_request_out))
        printf("Fail to register APBRIDGE_ROREQUEST_LATENCY_TAG_DIS"
               " vendor request\n");

    return 0;
}

void apbridgea_debug_free(void)
{
	kmm_free(ts);
}