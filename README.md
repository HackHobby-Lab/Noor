# Noor
This is repository for Noor the educational toy

# Pin Configuration (Initial Draft)

This section lists the **initial GPIO assignments** for the ESP32‑S3 N16R8 module used in this project.  
Please note: these mappings are **provisional** and may be updated based on hardware validation and testing results.

---

## I2S Audio (MAX98357 Amplifier)

| Signal      | GPIO    | Notes                                   |
|-------------|---------|-----------------------------------------|
| BCLK        | GPIO18  | Bit clock                               |
| LRCLK / WS  | GPIO17  | Word select (left/right clock)          |
| DIN         | GPIO16  | Data input                              |
| GAIN        | GPIO9   | Configures amplifier gain(not used yet) |


Connections Test Successful
---

## SD Card (SPI Mode)

| Signal | GPIO   | Notes                        |
|--------|--------|------------------------------|
| CS     | GPIO10 | Chip select                  |
| MOSI   | GPIO11 | Master out, slave in         |
| SCK    | GPIO12 | Clock                        |
| MISO   | GPIO13 | Master in, slave out         |

Connections Test Successful
---

## Rotary Encoders

| Encoder   | Signal | GPIO   |
|-----------|--------|--------|
| Encoder 1 | CLK    | GPIO1  |
| Encoder 1 | DT     | GPIO2  |
| Encoder 1 | SW     | GPIO21 |

*(Additional encoders can be mapped similarly in future revisions.)*

Connections Test Successful
---

## Pushbuttons

| Function     | GPIO   |
|--------------|--------|
| Play / Pause | GPIO14 |
| Back / Home  | GPIO15 |
| Volume +     | GPIO4  |
| Volume –     | GPIO5  |

Connections Test Successful
---

## Audio Jack

| Connection | Pin Description |
|------------|-----------------|
| Speaker + | SW (switch second pin from left when jack face is away) |
| Speaker - | MAX OUT - |
| MAX OUT + | TIP (most left pin from back when jack face is away) |
| GND | SLEEVE of audio jack (single pin of audio jack) |
| GPIO 48 | jack switch to detect headphones|

Connections Test Successful
---
## Reserved Pins (Do Not Use)

| Pin Group     | GPIOs                          | Reason                                  |
|---------------|--------------------------------|-----------------------------------------|
| Strapping     | GPIO0, GPIO3, GPIO45, GPIO46   | Affect boot mode and startup config      |
| PSRAM (Octal) | GPIO35, GPIO36, GPIO37         | Connected to embedded PSRAM              |
| Debug UART    | GPIO43, GPIO44                 | Default UART for debugging               |
| USB           | GPIO19, GPIO20                 | Dedicated to USB‑OTG functionality       |
| JTAG          | GPIO39, GPIO40, GPIO41, GPIO42 | Reserved for JTAG debugging            |
| RGB LED       | GPIO47                         | On‑module RGB LED                        |

---

## Notes

- All button and encoder inputs are configured for **interrupt handling** with internal pull‑ups enabled.  
- I2S and SPI pins are routed through the ESP32‑S3 GPIO matrix and can be reassigned if conflicts arise.  
- Reserved pins listed above must not be used for general I/O.  
- These assignments are **initial** and may change during integration and testing. Always refer to the latest revision of this README for updates.




#  SD Card Structure 

Simple guide to organize your SD card for the Noor Audio Player.

---

##  Quick Overview

 SD card has **two types of files**:

1. ** Announcement files** (in root) - Tell user what they're selecting
2. ** Story folders** - Contain the actual audio content

---

##  Complete File Structure

```
/sdcard/
│
├──  SYSTEM AUDIO
│   ├── welcome.wav   # Plays at boot
│   └── ROTATE.wav    # Home screen message
│
├──  FOLDER ANNOUNCEMENTS
│   ├── STORIES.wav               # Announces "Stories of Prophets" folder
│   └── TA.wav                    # Announces "TAWID" folder
│
├──  PROPHET ANNOUNCEMENTS
│   ├── ADAM.wav
│   ├── MUHAMMAD.wav
│   ├── NUH.wav
│   └── ... (more prophets)
│
├──  STORY ANNOUNCEMENTS (for StoriesofProphets)
│   ├── INSMUH.wav
│   ├── mstoryone.wav
│   ├── mstorytwo.wav
│   └── ... (up to mstoryseven.wav)
│
├──  TAWID ANNOUNCEMENTS
│   ├── TT1.wav                              # Announces t1.wav
│   ├── TT2.wav                              # Announces t2.wav
│   └── TT3.wav                              # Announces t3.wav
│
├──  FOLDERS
│   │
│   ├── StoriesofProphets/
|   |   |── ADAM
|   |   |── DOUD
|   |   |── IBRAHIM
|   |   |── ISMAEEL
|   |   |── ISSA
│   │   └── MUHAMMAD/
│   │   |    ├── m1.wav
│   │   |    ├── m2.wav
|   |   |    |   .
|   |   |    |   .
|   |   |    |   .
│   │   |    └── m7.wav
|   |   |── NUH
|   |   |── YAQOOB
|   |   |── YOSUF
│   │
│   └── TAWID/
│       ├── t1.wav
│       ├── t2.wav
│       └── t3.wav
```

---



##  File Naming Rules

###  **DO:**
- Use **no spaces**: `StoriesofProphets` 
- Use **no underscores**: `HazratMuhammad`  
- Keep names **short and clear**

###  **DON'T:**
- Use spaces: `Stories of Prophets` 
- Use underscores: `Stories_of_Prophets` 

---

##  Audio Requirements

| Property | Value |
|----------|-------|
| Format | WAV |
---


##  Adding New Content

### **Add New Prophet:**

**Step 1:** Create announcement
```
Yousuf.wav  ← Record "Prophet Yusuf"
```

**Step 2:** Create folder
```
STORIES/YOUSUF/
```

**Step 3:** Add stories
```
Tousuf/
├── y1.wav
├── y2.wav
└── y3.wav

Simmilarly for other prophets
```

### **Add More Tawid Lessons:**

**Step 1:** Create announcement
```
TT4.wav  ← Record "Lesson Four"
```

**Step 2:** Add story file
```
TAWID/t4.wav
```
