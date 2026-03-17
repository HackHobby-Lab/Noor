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
├── announcements/                        ← 150 files total
│   │
│   ├── welcome.wav                       Boot greeting
│   ├── rotate.wav                        "Rotate to navigate"
│   ├── stories.wav                       STORIES folder name
│   │
│   ├── ADAM.WAV                          Prophet folder name announcements
│   ├── DAOUD.WAV
│   ├── IBRAHIM.WAV
│   ├── ISMAEEL.WAV
│   ├── ISSA.WAV
│   ├── MOUSSA.WAV
│   ├── MUHAMMAD.WAV
│   ├── NUH.WAV
│   ├── YAQOOB.WAV
│   └── YOUSUF.WAV
│   ├── INSMUH.WAV                        "Entering Muhammad folder" sound
│   │
│   ├── TA.wav                            TAWID folder name announcement
│   ├── TT1.wav  TT2.wav  ...  TT20.wav   Lesson announcements (played on scroll/select)
│   │
│   ├── A1ANN.WAV  ...  A10ANN.WAV        Adam story track announcements
│   ├── D1ANN.WAV  ...  D10ANN.WAV        Daoud
│   ├── IB1ANN.WAV ...  IB10ANN.WAV       Ibrahim
│   ├── IS1ANN.WAV ...  IS10ANN.WAV       Ismaeel
│   ├── I1ANN.WAV  ...  I10ANN.WAV        Issa
│   ├── MS1ANN.WAV ...  MS10ANN.WAV       Moussa
│   ├── M1ANN.WAV  ...  M10ANN.WAV        Muhammad
│   ├── N1ANN.WAV  ...  N10ANN.WAV        Nuh
│   ├── YQ1ANN.WAV ...  YQ10ANN.WAV       Yaqoob
│   ├── Y1ANN.WAV  ...  Y10ANN.WAV        Yousuf
│   │
│   ├── v1.wav  v2.wav  ...  v10.wav      Volume level feedback
│   ├── correct.wav                       Quiz correct answer sound
│   ├── wrong.wav                         Quiz wrong answer sound
│   ├── b1.wav  b2.wav  b3.wav            Battery alert sounds
│
├── STORIES/
│   │
│   ├── ADAM/                             ← 100 files
│   │   ├── a1.wav  a2.wav  ...  a10.wav        Story audio (shown in navigation)
│   │   ├── sa1_1.wav  crra1_1.wav  wra1_1.wav  Story 1 — Quiz 1
│   │   ├── sa1_2.wav  crra1_2.wav  wra1_2.wav  Story 1 — Quiz 2
│   │   ├── sa1_3.wav  crra1_3.wav  wra1_3.wav  Story 1 — Quiz 3
│   │   └── ... same pattern for stories 2–10
│   │       (sa{N}_{Q}.wav / crra{N}_{Q}.wav / wra{N}_{Q}.wav  N=1-10, Q=1-3)
│   │
│   ├── DAOUD/                            ← 100 files
│   │   ├── d1.wav  ...  d10.wav
│   │   └── sd{N}_{Q}.wav / crrd{N}_{Q}.wav / wrd{N}_{Q}.wav  (N=1-10, Q=1-3)
│   │
│   ├── IBRAHIM/                          ← 100 files
│   │   ├── ib1.wav  ...  ib10.wav
│   │   └── sib{N}_{Q}.wav / crrib{N}_{Q}.wav / wrib{N}_{Q}.wav
│   │
│   ├── ISMAEEL/                          ← 100 files
│   │   ├── is1.wav  ...  is10.wav
│   │   └── sis{N}_{Q}.wav / crris{N}_{Q}.wav / wris{N}_{Q}.wav
│   │
│   ├── ISSA/                             ← 100 files
│   │   ├── i1.wav  ...  i10.wav
│   │   └── si{N}_{Q}.wav / crri{N}_{Q}.wav / wri{N}_{Q}.wav
│   │
│   ├── MOUSSA/                           ← 100 files
│   │   ├── ms1.wav  ...  ms10.wav
│   │   └── sms{N}_{Q}.wav / crrms{N}_{Q}.wav / wrms{N}_{Q}.wav
│   │
│   ├── MUHAMMAD/                         ← 100 files
│   │   ├── m1.wav  ...  m10.wav
│   │   └── sm{N}_{Q}.wav / crrm{N}_{Q}.wav / wrm{N}_{Q}.wav
│   │
│   ├── NUH/                              ← 100 files
│   │   ├── n1.wav  ...  n10.wav
│   │   └── sn{N}_{Q}.wav / crrn{N}_{Q}.wav / wrn{N}_{Q}.wav
│   │
│   ├── YAQOOB/                           ← 100 files
│   │   ├── yq1.wav  ...  yq10.wav
│   │   └── syq{N}_{Q}.wav / crryq{N}_{Q}.wav / wryq{N}_{Q}.wav
│   │
│   └── YOUSUF/                           ← 100 files
│       ├── y1.wav  ...  y10.wav
│       └── sy{N}_{Q}.wav / crry{N}_{Q}.wav / wry{N}_{Q}.wav
│
├── TAWID/                                ← 320 files
│   │
│   ├── T1.wav   T2.wav   ...  T20.wav    Lesson audio (shown in navigation)
│   │
│   ├── st1_1.wav   crrt1_1.wav   wrt1_1.wav    Lesson 1 — Quiz 1
│   ├── st1_2.wav   crrt1_2.wav   wrt1_2.wav    Lesson 1 — Quiz 2
│   ├── st1_3.wav   crrt1_3.wav   wrt1_3.wav    Lesson 1 — Quiz 3
│   ├── st1_4.wav   crrt1_4.wav   wrt1_4.wav    Lesson 1 — Quiz 4
│   ├── st1_5.wav   crrt1_5.wav   wrt1_5.wav    Lesson 1 — Quiz 5
│   └── ... same pattern for lessons 2–20
│       (st{N}_{Q}.wav / crrt{N}_{Q}.wav / wrt{N}_{Q}.wav  N=1-20, Q=1-5)
│
└── update.bin                            Optional OTA firmware update
```

---
---
 
## File Count Summary
 
| Folder | Navigable files | Hidden quiz files | Total |
|---|---|---|---|
| `announcements/` | 150 | — | **150** |
| `STORIES/ADAM/` | 10 | 90 | **100** |
| `STORIES/DAOUD/` | 10 | 90 | **100** |
| `STORIES/IBRAHIM/` | 10 | 90 | **100** |
| `STORIES/ISMAEEL/` | 10 | 90 | **100** |
| `STORIES/ISSA/` | 10 | 90 | **100** |
| `STORIES/MOUSSA/` | 10 | 90 | **100** |
| `STORIES/MUHAMMAD/` | 10 | 90 | **100** |
| `STORIES/NUH/` | 10 | 90 | **100** |
| `STORIES/YAQOOB/` | 10 | 90 | **100** |
| `STORIES/YOUSUF/` | 10 | 90 | **100** |
| `TAWID/` | 20 | 300 | **320** |
| **TOTAL** | **220** | **1,200** | **1,470** |
 
---
 
## File Naming Conventions
 
### Prophet Stories
 
| Pattern | Example | Meaning |
|---|---|---|
| `{p}{N}.wav` | `a1.wav` | Story N for prophet with prefix p |
| `s{p}{N}_{Q}.wav` | `sa1_1.wav` | Story N, quiz Q — question audio |
| `crr{p}{N}_{Q}.wav` | `crra1_1.wav` | Story N, quiz Q — correct answer |
| `wr{p}{N}_{Q}.wav` | `wra1_1.wav` | Story N, quiz Q — wrong answer |
 
**Prophet prefix table:**
 
| Prophet | Prefix | Story files | Track announcement |
|---|---|---|---|
| Adam | `a` | `a1.wav … a10.wav` | `A1ANN.WAV … A10ANN.WAV` |
| Daoud | `d` | `d1.wav … d10.wav` | `D1ANN.WAV … D10ANN.WAV` |
| Ibrahim | `ib` | `ib1.wav … ib10.wav` | `IB1ANN.WAV … IB10ANN.WAV` |
| Ismaeel | `is` | `is1.wav … is10.wav` | `IS1ANN.WAV … IS10ANN.WAV` |
| Issa | `i` | `i1.wav … i10.wav` | `I1ANN.WAV … I10ANN.WAV` |
| Moussa | `ms` | `ms1.wav … ms10.wav` | `MS1ANN.WAV … MS10ANN.WAV` |
| Muhammad | `m` | `m1.wav … m10.wav` | `M1ANN.WAV … M10ANN.WAV` |
| Nuh | `n` | `n1.wav … n10.wav` | `N1ANN.WAV … N10ANN.WAV` |
| Yaqoob | `yq` | `yq1.wav … yq10.wav` | `YQ1ANN.WAV … YQ10ANN.WAV` |
| Yousuf | `y` | `y1.wav … y10.wav` | `Y1ANN.WAV … Y10ANN.WAV` |
 
### TAWID
 
| Pattern | Example | Meaning |
|---|---|---|
| `T{N}.wav` | `T5.wav` | Lesson N content (shown in navigation) |
| `TT{N}.wav` | `TT5.wav` | Lesson N announcement (in `announcements/`) |
| `st{N}_{Q}.wav` | `st5_1.wav` | Lesson N, quiz Q — question audio |
| `crrt{N}_{Q}.wav` | `crrt5_1.wav` | Lesson N, quiz Q — correct answer |
| `wrt{N}_{Q}.wav` | `wrt5_1.wav` | Lesson N, quiz Q — wrong answer |
 
> Quiz support files (any `.wav` containing `_`) are automatically hidden from navigation by the firmware.
 
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


# Instructions

https://docs.google.com/document/d/1RYAxxgP-j9_PKx_9E5XExqXV5vHtyEG8tLyEzbizOWY/edit?tab=t.0
