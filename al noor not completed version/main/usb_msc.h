#ifndef USB_MSC_H
#define USB_MSC_H

#include <stdbool.h>
#include "sdmmc_cmd.h"

bool usb_msc_start(sdmmc_card_t *card);
bool usb_msc_is_active(void);
bool usb_msc_is_connected(void);
const char* usb_msc_get_status(void);

#endif // USB_MSC_H
