/**
 * usb_msc.h
 * USB Mass Storage Class (MSC) for Noor Audio Player
 * 
 * This module handles:
 * - USB MSC initialization to expose SD card as USB storage
 * - Composite USB device (UART + MSC)
 * - Non-blocking USB operations
 * - No interference with existing UART/audio workflow
 */

#ifndef USB_MSC_H
#define USB_MSC_H

#include <stdbool.h>
#include <stdint.h>
#include "sdmmc_cmd.h"

/**
 * Initialize USB MSC and expose SD card as USB storage device
 * 
 * @return true if USB MSC initialized successfully, false on error
 * 
 * Note: This initializes a composite USB device with:
 * - CDC (Communication Device Class) for UART logging
 * - MSC (Mass Storage Class) for SD card access
 * 
 * The function is non-blocking and returns immediately.
 */
bool usb_msc_init(void);

/**
 * Check if USB is currently connected
 * 
 * @return true if USB host is connected, false otherwise
 */
bool usb_msc_is_connected(void);

/**
 * Check if USB MSC is mounted (SD card accessible via USB)
 * 
 * @return true if USB MSC is mounted and ready, false otherwise
 */
bool usb_msc_is_mounted(void);

/**
 * Get current USB MSC status as string
 * 
 * @return Status string (e.g., "Connected", "Mounted", "Disconnected")
 */
const char* usb_msc_get_status(void);

/**
 * Register SD card with USB MSC for file access
 * Call this after SD card is successfully mounted at /sdcard
 * 
 * @param card Pointer to sdmmc_card_t structure from SD card initialization
 */
void usb_msc_set_sd_card(sdmmc_card_t *card);

#endif // USB_MSC_H
