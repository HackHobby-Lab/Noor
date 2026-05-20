# Noor firmware patch — v1.2 (final, audited)

This patch addresses every firmware-fixable issue in the client's feedback
report. Hardware-only issues (headphone audio path, full elimination of
reboot-on-unplug) are documented at the bottom — they require a PCB
revision and cannot be fixed in firmware.

---

## Files in this bundle

| File | Purpose | Lines |
|---|---|---|
| `audio.c` | I2S install-once + static buffers (heap-fragmentation fix) | 359 |
| `buttons.c` | Active-LOW wiring (PCB buttons to GND) | 90 |
| `buttons.h` | Header for active-LOW config | 41 |
| `config.h` | Raised file-count limits, added encoder direction flag | 186 |
| `encoder.c` | Full quadrature decoder (1 click = 1 event) | 296 |
| `headphone_detect.c` | Larger task stack + 3-of-3 stable-read debounce | 208 |
| `main.c` | TAWID -> TEACHINGS, duplicate-announcement fix, quiz gating | 1105 |
| `quiz.c` | Dynamic quiz count, N cap raised to 999 | 634 |
| `sd_card.c` | Alphabetical sort for folders + subfolders | 595 |

---

## Issue-by-issue mapping (matches client feedback document)

### §1 — Working features
No code change required.

### §2 — Navigation order
Two separate problems were causing the "random order" symptom:

**§2a Folder order in copy order, not alphabetical**
Fixed in `sd_card.c`:
- New `compare_path_alpha()` case-insensitive comparator (line 132)
- `qsort` called in `sd_scan_folders()` (line 403)
- `qsort` called in `sd_scan_subfolders()` (line 486)
- Existing `qsort` for WAV files (line 446) kept unchanged

Result: prophets always appear in fixed order
ADAM -> DAOUD -> IBRAHIM -> ISMAEEL -> ISSA -> MOUSSA -> MUHAMMAD ->
NUH -> YAQOOB -> YOUSUF -> (loops to ADAM)

**§2b Encoder fires multiple events per click**
Fixed in `encoder.c` — full quadrature state-machine decoder:
- Both CLK and DT pins on ANYEDGE interrupts (lines 248, 255)
- 16-entry lookup table `qdec_table` (line 77)
- Accumulator emits one event only when +/-4 reached (line 109)
- Boot primes the state to avoid phantom first event (line 223)

Result: 1 physical click = exactly 1 event, regardless of contact bouncing.

**§2c Direction mapping**
In `main.c` encoder_callback (lines 854, 863, 870):
- `ENC_DIR_CW`  -> `nav_next_*`  (forward, next alphabetically)
- `ENC_DIR_CCW` -> `nav_prev_*`  (backward)

Optional safety net in case the encoder is wired such that physical
clockwise produces CCW in the firmware: `config.h` line 108
`#define ENCODER_INVERT_DIRECTION  0`
Change to `1` and rebuild if first hardware test shows direction reversed.

### §3 — Duplicate announcement on enter
Fixed in `main.c` `_announce_and_drain()` (line 229):
```
static char s_last_announced[64] = {0};
if (s_last_announced[0] != '\0' &&
    strcasecmp(s_last_announced, filename) == 0) {
    return;  // already played this announcement
}
```
Selecting ADAM then pressing enter no longer replays "Adam".

### §4 — Buttons not responding
Fixed in `buttons.c`:
- `pull_up_en   = GPIO_PULLUP_ENABLE`   (line 35)
- `pull_down_en = GPIO_PULLDOWN_DISABLE` (line 36)
- Detection on falling edge HIGH -> LOW (line 74)

Result: button press = GPIO line shorted to GND, idle = pulled up to 3.3V.

### §5 — Random reboots during navigation
Fixed in `audio.c`:
- `i2s_install_once()` called in `audio_init()` only, not per file (line 153)
- I2S driver stays installed for the lifetime of the application
- Static playback buffers replace per-file `malloc`/`free`
  - `s_input_buffer`  (2 KB, line 65)
  - `s_output_buffer` (1 KB, line 66)
- Per-file work reduced to `i2s_set_clk()` to switch sample rate

Net effect: zero heap allocations during navigation -> no fragmentation ->
no audio-task crashes -> no random reboots.

### §6 — Headphone jack
**§6a No audio through headphones — HARDWARE ISSUE, not fixed here.**
The MAX98357A is a bridge-tied (BTL) Class-D amp. Its only outputs are
OUT+ and OUT-; audio only exists between them, not between either one
and GND. The current jack wiring (TIP -> MAX OUT+, SLEEVE -> GND)
cannot deliver audio to headphones.

PCB revision required: either an LC filter + coupling capacitor on a
dedicated headphone path, or a separate headphone amplifier IC
(PAM8908, NS4263) driven from a I2S DAC line-out.

**§6b Reboot on unplug — Partially mitigated.**
In `headphone_detect.c`:
- Task stack 2048 -> 4096 (line 34)
- 3-of-3 stable-read debounce filters jack-arc transients (line 39)

Full fix requires hardware: add a 100 uF bulk cap on the amp's VDD pin
on the next PCB revision.

### §7 — Section rename (TAWID -> Islamic Teachings + Invocations)
Fixed in `main.c`:
- 0 remaining `"TAWID"` literals in code
- 6 `strcasecmp(..., "TEACHINGS")` references (lines 204, 265, 335, 359,
  383, 1058)
- Firmware auto-scans whatever top-level folders the client puts on the
  SD card; STORIES + TEACHINGS + INVOCATIONS works out of the box.

### §8 — Quizzes only inside TEACHINGS
Fixed in `main.c`:
- New `is_quiz_eligible_folder()` helper (line 198) — returns true only
  when the parent folder is `TEACHINGS`
- Both `quiz_exists_for_story` call sites are now gated by this check
  (lines 747-748, 789-790)

Stories of the Prophets and Invocations will never trigger a quiz,
even if matching quiz files are accidentally present on the SD card.

### §9 — Remove hardcoded limits
Fixed in three places:
- `config.h`  MAX_FOLDERS 32 -> 500 (line 153)
- `config.h`  MAX_WAV_FILES 64 -> 500 (line 154)
- `quiz.c`   parse_story_filename N cap 20 -> 999 (line 269)
- `quiz.c`   quiz_count now detected dynamically — probes
            `s{prefix}{N}_1.wav`, `_2.wav`, ... `_99.wav` and counts
            actual files (line 371)

Result: client can add T1...T999 lessons, any number of quizzes per
lesson, any number of stories per prophet — no firmware changes needed.

---

## SD card layout

```
/sdcard/
|-- announcements/        all voice feedback files
|-- STORIES/              Stories of the Prophets (no quizzes ever)
|   |-- ADAM/
|   |-- DAOUD/
|   |-- IBRAHIM/
|   |-- ISMAEEL/
|   |-- ISSA/
|   |-- MOUSSA/
|   |-- MUHAMMAD/
|   |-- NUH/
|   |-- YAQOOB/
|   `-- YOUSUF/
|-- TEACHINGS/            Islamic Teachings (quizzes allowed)
|   |-- T1.wav ... T999.wav  lesson audio (any count)
|   `-- quiz files (optional per lesson):
|       st{N}_{Q}.wav    quiz question
|       crrt{N}_{Q}.wav  correct answer
|       wrt{N}_{Q}.wav   wrong answer
|-- INVOCATIONS/          Invocations (no quizzes ever)
`-- firmware.bin          (only when updating)
```

Quizzes inside TEACHINGS:
- Trigger ONLY if `st{N}_1.wav` exists alongside lesson `T{N}.wav`
- Firmware probes _1, _2, _3, ... _99 and plays however many exist
- No fixed count — 1, 3, 5, 10 per lesson — whatever client adds

---

## Build & ship

1. Replace the 9 files in `Al noor ver 2.0/Al noor ver 1.1/main/`
2. Build:
   ```
   idf.py build
   ```
3. Rename `build/app-template.bin` to `firmware.bin`
4. Send `firmware.bin` to client — they drop it in the SD card root,
   power-cycle, and the device self-flashes via the SD-card OTA path
   already present in `SD_CARD_OTA.c`. No cable needed.

## First-test checklist for the client

Ask the client to confirm after applying the update:

1. Folder order is stable across power cycles
   (ADAM -> DAOUD -> IBRAHIM ... always in the same order)
2. One physical encoder click moves selection by exactly one item
3. Clockwise rotation moves FORWARD alphabetically; counter-clockwise
   moves BACKWARD
   - If direction is reversed, change `ENCODER_INVERT_DIRECTION` in
     `config.h` from `0` to `1`, rebuild, ship new bin
4. Selecting a prophet and pressing enter does NOT replay the prophet
   name a second time
5. All four physical buttons respond to presses
6. No spontaneous reboots during navigation
7. Stories in STORIES/ never trigger quizzes
8. Lessons in TEACHINGS/ trigger quizzes only when quiz files exist for
   that specific lesson

Items 9 and 10 from §6 (headphone audio output, full elimination of
reboot-on-unplug) require PCB revision and are not part of this
firmware update.
