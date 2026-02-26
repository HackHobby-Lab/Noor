/**
 * usb_manager.h
 * USB CDC + MSC composite device for AL-NOOR-PLAYER
 *
 * One USB-C cable presents:
 *   - A COM port (CDC ACM) for serial debugging / firmware flashing
 *   - A removable USB storage drive (MSC) backed by the SD card
 *
 * IMPORTANT: usb_manager_init() must be called AFTER sd_card_init()
 * (needs the sdmmc_card_t handle) and AFTER the audio task is created
 * (needs the task handle for USB MSC change notifications).
 *
 * usb_manager_init() performs the initial FAT mount of /sdcard via
 * tinyusb_msc_storage_mount(). sd_card_init() initializes hardware only.
 * ALL future FAT mount/unmount operations are managed by TinyUSB MSC storage.
 */

#ifndef USB_MANAGER_H
#define USB_MANAGER_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * Notification bit sent to the audio task when USB MSC mount state changes.
 * When USB host releases the SD card, the audio task handles folder rescan.
 */
#define NOTIFY_USB_MSC_CHANGED_BIT  (1u << 29)

/**
 * Initialize TinyUSB CDC+MSC composite device.
 * Mounts /sdcard via tinyusb_msc_storage_mount() (initial FAT mount).
 *
 * @param audio_task  Handle of the audio task (for USB MSC notifications)
 */
void usb_manager_init(TaskHandle_t audio_task);

/**
 * Returns true while the USB host has the SD card mounted (MSC active).
 * Audio playback and SD file operations must be suspended during this time.
 */
bool usb_manager_msc_active(void);

/**
 * Reboot the ESP32-S3 into ROM USB serial download mode.
 * Triggered by holding Home + VolUp for 3 seconds.
 * The device re-enumerates as the Espressif ROM download device,
 * compatible with: esptool.py --chip esp32s3 -p COMx write_flash ...
 * This function does not return.
 */
void usb_manager_reboot_to_download(void);

/**
 * Signal that boot audio (welcome.wav + rotate.wav) has finished.
 * Call this from the audio task after announce_play_boot_greetings() returns.
 * _premount_cb waits for this signal before yielding the SD card to the PC,
 * ensuring boot audio always plays fully on first USB connect.
 */
void usb_manager_set_boot_audio_done(void);

#endif /* USB_MANAGER_H */
