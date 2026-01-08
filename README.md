# Noor
This is repository for Noor the educational toy

# Pin Configuration (Initial Draft)

This section lists the **initial GPIO assignments** for the ESP32‑S3 N16R8 module used in this project.  
Please note: these mappings are **provisional** and may be updated based on hardware validation and testing results.

---

## I2S Audio (MAX98357 Amplifier)

| Signal      | GPIO   | Notes                                |
|-------------|--------|--------------------------------------|
| BCLK        | GPIO6  | Bit clock                            |
| LRCLK / WS  | GPIO7  | Word select (left/right clock)       |
| DIN         | GPIO8  | Data input                           |
| GAIN        | GPIO9  | Configures amplifier gain            |

---

## SD Card (SPI Mode)

| Signal | GPIO   | Notes                        |
|--------|--------|------------------------------|
| CS     | GPIO10 | Chip select                  |
| MOSI   | GPIO11 | Master out, slave in         |
| SCK    | GPIO12 | Clock                        |
| MISO   | GPIO13 | Master in, slave out         |

---

## Rotary Encoders

| Encoder   | Signal | GPIO   |
|-----------|--------|--------|
| Encoder 1 | CLK    | GPIO1  |
| Encoder 1 | DT     | GPIO2  |
| Encoder 1 | SW     | GPIO4  |

*(Additional encoders can be mapped similarly in future revisions.)*

---

## Pushbuttons

| Function     | GPIO   |
|--------------|--------|
| Play / Pause | GPIO14 |
| Back / Home  | GPIO15 |
| Volume +     | GPIO16 |
| Volume –     | GPIO17 |

---

## Reserved Pins (Do Not Use)

| Pin Group     | GPIOs                        | Reason                                  |
|---------------|------------------------------|-----------------------------------------|
| Strapping     | GPIO0, GPIO3, GPIO45, GPIO46 | Affect boot mode and startup config      |
| PSRAM (Octal) | GPIO35, GPIO36, GPIO37       | Connected to embedded PSRAM              |
| Debug UART    | GPIO43, GPIO44               | Default UART for debugging               |
| USB           | GPIO19, GPIO20               | Dedicated to USB‑OTG functionality       |
| JTAG          | GPIO39, GPIO40, GPIO41, GPIO42 | Reserved for JTAG debugging            |
| RGB LED       | GPIO47                       | On‑module RGB LED                        |

---

## Notes

- All button and encoder inputs are configured for **interrupt handling** with internal pull‑ups enabled.  
- I2S and SPI pins are routed through the ESP32‑S3 GPIO matrix and can be reassigned if conflicts arise.  
- Reserved pins listed above must not be used for general I/O.  
- These assignments are **initial** and may change during integration and testing. Always refer to the latest revision of this README for updates.
