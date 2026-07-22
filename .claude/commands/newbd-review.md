---
description: Emulate Henry's review of a new-board ArduPilot PR (README/hwdef vs template + house rules)
argument-hint: <ardupilot-PR-number> [more PR numbers...]
allowed-tools: Bash, Read
---

Produce a review of a **new autopilot board** PR on **ArduPilot/ardupilot**, emulating the
reviewing style and standards of Henry Wurzburg (@Hwurzburg). The checklist below was
distilled from ~40 of his new-board reviews. Goal: catch the same things he catches, in
roughly the same order and voice, so he can post/adjust it — **not** to auto-approve.

PR numbers to review: **$ARGUMENTS**

## 0. Read the reference standards first

Always read these current-version files before reviewing (the requirements evolve):

- `misc/readme_template.md` — the canonical README structure/section order/wording.
- `dev/source/docs/readme_file.rst` — the "HWDEF Readme.md Guide" (required sections and
  the exact suggested wording for Compass, Firmware, Loading Firmware, Battery, etc.).

The README under review should match the template's sections, order, headings, and the
stock wording for the boilerplate sections.

## 1. Gather everything about the PR

For each PR number:

```bash
gh api repos/ArduPilot/ardupilot/pulls/<N> --jq '.title'
# files + status (added/modified) — identify the hwdef dir and whether README/images/bootloader are present
gh api repos/ArduPilot/ardupilot/pulls/<N>/files --paginate --jq '.[] | "\(.status)\t\(.filename)"'
# commit structure (Henry checks this every time)
gh api repos/ArduPilot/ardupilot/pulls/<N>/commits --paginate --jq '.[].commit.message' | head -50
# CI status
gh pr checks <N> --repo ArduPilot/ardupilot 2>/dev/null | head -40
# the full diff (read the README and hwdef closely)
gh pr diff <N> --repo ArduPilot/ardupilot
```

Fetch the **raw** files (the diff alone hides unchanged context you need to cross-check):

```bash
DIR=libraries/AP_HAL_ChibiOS/hwdef/<DIR>
for f in README.md Readme.md hwdef.dat hwdef-bl.dat defaults.parm; do
  echo "===== $f ====="; gh api "repos/ArduPilot/ardupilot/contents/$DIR/$f?ref=<HEADSHA>" \
    --jq '.content' 2>/dev/null | base64 -d
done
```

(Use the PR head sha/branch. Get it from `gh api .../pulls/<N> --jq '.head.sha'`.)

The single most important cross-check: **does the README describe what the hwdef actually
does?** Henry's most frequent findings are README-vs-hwdef mismatches (protocols, DMA,
BIDIR, pin names).

## 2. Preliminary gatekeeping (state these FIRST, as blockers)

Henry often posts a short "preliminary review" listing blockers before doing detailed
work. Check and report, in this order:

1. **Pinout image present & adequate.** A clear image (silkscreen or photo) labeling every
   user-facing pad/connector is **required** — without it he will not review ("I consider
   it vaporware"). Images must be committed in the hwdef dir and **relative-linked**
   (`![x](x.png)`, not an absolute/remote URL) or CI can't find them. Detect the image
   files and whether the README links them:

   ```bash
   # image files added/modified in the PR
   gh api repos/ArduPilot/ardupilot/pulls/<N>/files --paginate \
     --jq '.[] | select(.filename|test("\\.(png|jpg|jpeg|gif)$";"i")) | "\(.status)\t\(.filename)"'
   ```

   Report: is there **at least one image**? Is at least one a **pinout/wiring** image
   (not just a product beauty shot)? Are they **relative-linked** in the README (flag any
   `![...](http...)` remote link)? Are any committed images **not** referenced by the
   README (or vice-versa)? If there is no pinout image, this is a hard blocker — stop and
   say a full review waits on it, exactly as he does. Then run §2b.
2. **Where-to-buy / purchase link present.** Without it the board won't be added to the
   wiki and is "invisible to ArduPilot users" — he treats this as a hard requirement, and
   existing boards without one are at risk of removal.
3. **Bootloader binary committed** for the new board.
4. **Commit structure:** exactly **one commit per library/subsystem changed**, each
   message prefixed with that library, e.g. `AP_HAL_ChibiOS: add <board>`,
   `Tools: add <board> bootloader` (note: it's `Tools:`, not `AP_Tools:`), bootloader
   binaries commit for `AP_Bootloader`/`bootloaders`. **No merge commits — rebase only.**
5. **CI passing.** He generally will not do a full review until CI is green (markdown
   linting especially). Note the failing checks.
6. **Board ID:** allocated in the correct range, and spelled identically in
   `Tools/AP_Bootloader/board_types.txt` and the hwdef. ODID variants need **two** board
   defs/ids (`<BOARD>` and `<BOARD>-ODID`), and shouldn't default ODID on unless a DroneCAN
   ODID module is integrated. (Board-ID reservation is best done as its own quick PR first.)
7. **English only** — no non-English characters in hwdef/README.
8. **Scope:** a new board + a new driver + a core change should be **split into separate
   PRs**. Lab-only/experimental boards not offered for sale don't belong in master.

## 2b. Pinout image ↔ README consistency check

This automates one of Henry's most frequent manual findings ("there is no PC8 in the
pinout image", "M5-9 are not the image labels", "labels should match the image"). Download
**every committed image** (`.png`/`.jpg`/etc.) and **read each one visually** with the Read
tool, then cross-check. **Read images the README does not link, too** — an unreferenced
image is often exactly the extra pinout you need (a second/back-side board face, a
through-hole/castellated pad map, a connector close-up). Do not stop after the one image the
README embeds.

**Download the images** (the PR head is usually a fork — use `head.repo.full_name` and the
head sha, and the contents API `download_url`):

```bash
HEAD_REPO=$(gh api repos/ArduPilot/ardupilot/pulls/<N> --jq '.head.repo.full_name')
HEAD_SHA=$(gh api repos/ArduPilot/ardupilot/pulls/<N> --jq '.head.sha')
DIR=libraries/AP_HAL_ChibiOS/hwdef/<DIR>
for img in <image filenames from step 2>; do
  url=$(gh api "repos/$HEAD_REPO/contents/$DIR/$img?ref=$HEAD_SHA" --jq '.download_url')
  curl -sL "$url" -o "/tmp/board_$img"
done
```

Then `Read` each `/tmp/board_*` image (the Read tool renders images visually) and build an
inventory of the **labels actually printed on the board/connectors** — pad names
(`TX1`, `RX6`, `SBUS`, `LED`, `5V`, `VTX`, `CAM`, `M1..Mx`, `A1..Ax`, `RSSI`, `CURR`,
`SDA/SCL`, connector names, etc.).

**Cross-check both directions:**
- **README → image:** every pin/pad the README names (in the UART table, PWM/GPIO tables,
  RC/OSD/VTX/battery sections) should appear on an image. Flag any the image doesn't show —
  especially raw CPU pin names (`PA10`, `PC8`) that should never be user-facing.
  **Do NOT conclude "not pinned out" from image-absence alone.** A connector/top-face pinout
  routinely omits signals exposed on **through-hole / castellated / bottom-face solder pads**
  (a spare "USER" UART, extra I2C/ADC, boot pads) — these are user-accessible even when they
  never appear in the connector diagram. When the README names a port/pin that no image
  shows, distinguish: (a) genuinely not exposed → say remove it from the hwdef or mark it
  "not pinned out"; (b) plausibly on unshown solder pads → treat it as an **incomplete
  pinout** and ask for those pads to be shown/labeled in an image, rather than asserting it
  is unpinned. A user-solderable UART with no documented location isn't usable, so this is a
  real finding either way — but phrase it as "not shown in any image / needs labeling," not
  "does not exist."
- **image → README:** every user-facing labeled pad/connector on the image should be
  documented somewhere in the README. Flag anything shown but unexplained (e.g. an `SB`,
  `SW1-4`, `PVDD`, or `EXT` pad with no description).
- **naming consistency:** the README should use the **image's** label for a pin, and UART
  pins should carry the UART number (`TX3/RX3`), matching what the image shows. Flag
  mismatches (image says `TEL`/UART2 but README calls it UART1, etc.).

Because OCR/vision on dense or low-res pinout images is imperfect, present these as
findings **to verify**, quoting the label as you read it, rather than hard assertions —
and note if an image was too low-resolution/rotated to read reliably (Henry also rejects
unreadable pinouts and asks for split/clearer images).

## 3. README format & markdown

- Sections, order, and headings match `misc/readme_template.md`. **A `## Compass`
  section is always required** (even if just "no built-in compass, attach external via
  I2C on SDA/SCL"). Standardized heading names and `##`/`###` levels are enforced.
- Markdown-lint clean: **no embedded HTML**, markdown tables only (no HTML tables), no
  leftover rst (`:ref:`, `.. note::`, triple-backtick "block highlighting", rst
  underlines), no stray/duplicate blank lines, no bare URLs, correct heading levels,
  a single top-level `#` title.
- **Remove the "how to build firmware" section** — users who can build don't need it.
- **Firmware** section states the firmware server URL and the **folder name** to look
  under. **Loading Firmware**: if it ships with an AP bootloader → update via GCS/`.apj`;
  if not → DFU instructions (and where the boot button is).
- Remove generic "Pixhawk standard" filler and "further information" dumps; a single
  pointer to the vendor doc is enough.

## 4. Content correctness — the bulk of the review

Cross-check each against the hwdef, the pinout image labels, and ArduPilot behavior:

**Pins & labels** (see §2b for the image cross-check that feeds this)
- **Remove all CPU pin references** (e.g. `PA10`, bus/address numbers) from the README —
  they mean nothing to users. Use the **pad/connector labels from the pinout image**, and
  every pin named in the README must match the image (and vice-versa) — report the
  mismatches found in §2b here.
- Only document **pinned-out / user-accessible** pins. If the hwdef defines UARTs,
  outputs, GPIOs, analog inputs, RSSI, etc. that aren't pinned out, either remove them
  from the hwdef or, if kept, mark them "not pinned out."

**UART Mapping**
- Table must show the **default protocol** (connector/label name is fine *in addition*),
  and note **DMA** capability per UART. TX/RX pin names should carry the UART number
  (`TX3`/`RX3`), not the serial-port number. Drop a raw "DMA stream" column — it just
  confuses users.
- **README protocols must match the hwdef.** A port marked "TELEM3" with no protocol
  assigned → either set its default protocol (e.g. MAVLink2) in hwdef or relabel it.
- **README DMA column must match the hwdef.** For every UART, check its TX/RX pin lines in
  the hwdef for a `NODMA` tag: if either pin is `NODMA`, that UART does **not** have DMA and
  the table must say so (`No`/✘). Flag any row that claims DMA (`DMA Enabled`/✔) for a UART
  whose hwdef pins are `NODMA` — and vice-versa (table says no DMA but the pins are not
  tagged `NODMA`). Report the specific serial/UART number and the hwdef line.
- **Unpinned UART left in SERIAL_ORDER** → set its default protocol to `NONE` (or mark the
  slot `EMPTY`), otherwise ArduPilot hunts for a GPS on it and never checks the next port.
  (Exception: a UART deliberately defaulted to RCIN — `DEFAULT_SERIALx_PROTOCOL
  SerialProtocol_RCIN` — is the UART-based RC input and is correct as-is; see **RC Input**.
  Do not flag it or ask for `NONE`.)
- **First GPS must be on the first GPS-protocol serial port.** If the wiring shows the GPS
  on a later UART, set the earlier GPS-default UART to `NONE`.
- Note flow-control pins where present.

**RC Input** (his most-rewritten section — match the template wording)
- **A board can have TWO RC inputs, and having both is correct — do not flag either as an
  error.** Detect what the hwdef actually configures before reviewing:
  - **Timer-based** RC input — a pin tagged `RCININT` (e.g. `PI5 TIM8_CH1 TIM8 RCININT ...`;
    `grep -n RCININT hwdef.dat`). This is the "RCIN"/"SBUS" pad and handles **all
    unidirectional protocols including PPM** (and SBUS).
  - **UART-based** RC input — a UART whose default protocol is RCIN, i.e.
    `DEFAULT_SERIALx_PROTOCOL SerialProtocol_RCIN` (or protocol `23`;
    `grep -n 'DEFAULT_SERIAL.*RCIN' hwdef.dat`). This handles the **bidirectional /
    telemetry** protocols (CRSF/ELRS, SRXL2, IRC Ghost, FPort).
- **When a UART is defaulted to RCIN, that is intentional — treat it as the UART RC input,
  not a fault.** Specifically: do **not** report it as an "unassigned/mismatched protocol,"
  do **not** tell them to set it to `NONE` (the unpinned-UART rule below does NOT apply to a
  RCIN-defaulted UART — it is in use), and `RCIN` is a valid entry in the table's protocol
  column (the connector's image label, e.g. `Uart8`, is the *label*; `RCIN` is the
  *protocol*). The only things to verify for a UART RC input: it is a **full UART with DMA**
  (bidirectional/CRSF needs DMA — flag if its pins are `NODMA`), and the README documents it.
- If **only** a timer pin exists (no UART defaulted to RCIN), then note that bidirectional
  protocols require the user to repurpose a spare full UART to protocol `23` (RCIN), and give
  the `SERIALx_OPTIONS` values (FPort `15`, CRSF/ELRS `0`, SRXL2 `4` + TX only). Either way,
  link `common-rc-systems` and give the `SERIALx_OPTIONS` values for the RC UART in play.
- **F4 boards:** SBUS on a bare UART needs an inverter/IOMCU — otherwise SBUS goes to the
  RCIN/timer pin. Don't claim CRSF on an alternate-config UART (alt configs have no DMA).
- Remove instructions to set baud rate (auto-detected) and to set inversion options on H7
  (auto-detected).

**PWM Output**
- State output count, IOMCU vs FMU split (MAIN vs AUX labels matching the case), **timer
  groupings** with the warning that **all outputs in a group must use the same protocol**,
  which outputs support DShot / bidirectional DShot, and which are **PWM-only (no DMA)**.
  Call out any serial-LED output.
- hwdef **BIDIR tags:** only one channel per timer *pair* plus CH3 should be tagged; remove
  redundant BIDIR where CH3 already carries it; the `TIM4_CH4` quirk needs CH3 tagged for
  DMA. Encourage enabling BDShot on the ESC-connector outputs.
- **Serial LED:** if a LED/NeoPixel pin exists, set it up for plug-and-play — `SERVOx_FUNCTION 120`
  in `defaults.parm` **and** `define DEFAULT_NTF_LED_TYPES 455` (455 also drives many
  external-GPS NTF LEDs) in hwdef. Don't disable the serial-LED pin.

**Compass** — always a section. If no internal compass, remove the `COMPASS` probe lines
from hwdef and use the "no built-in compass, use external I2C on SDA/SCL" wording. If it
has one, use the stock "has a built-in compass … usually disabled … external as part of a
GPS/Compass combo" wording. Never preset an external compass's orientation.

**Battery Monitoring** — list the defaults actually set in the hwdef (`BATT_MONITOR`,
`BATT_VOLT_PIN`, `BATT_CURR_PIN`, `BATT_VOLT_MULT`, `BATT_AMP_PERVLT`) and the voltage
range. **Battery defaults belong in the hwdef, not `defaults.parm`.** Note if a power
module is included and that scales need adjusting for other monitors. Don't set scales to
`1` and call it done.

**OSD / DisplayPort** — if `OSD_TYPE2 5` is defaulted, the DisplayPort UART's default
protocol must be set to MSP DisplayPort in the hwdef; note simultaneous analog + HD-VTX.

**VTX power / Camera switch** — GPIO number, board pin name, and which RELAY controls it;
add the `RELAYn` define in hwdef and the matching README section (template has both). VTX
power should default **off** at boot.

**GPIOs / RSSI / Airspeed / Analog** — provide a GPIO-number table; assign GPIO numbers to
all outputs; only list pinned-out pins. Prefer an analog RSSI input where an ADC pin is
available; give `RSSI_ANA_PIN`/`ARSPD_PIN` numbers.

**defaults.parm / hwdef cleanup**
- Remove anything already an ArduPilot default (log dirs, IMU mask, 2MB-board defaults,
  baud rates, RC reversals, RSSI presets). Frame-type default → hwdef, not defaults.parm.
- Remove superfluous defines/comments, HW-versioning defines (AP doesn't use them), and
  duplicate/redefined outputs. Enable CAN ports in defaults.parm only if intended
  (`CAN_P1_DRIVER 1`).
- DMA allocation: IMU SPIs should out-prioritize timers; verify BIDIR outputs don't share
  DMA with an IMU.

**Warnings** — flag anything that can damage hardware or fail silently: non-5V pins near a
buzzer/peripheral, USB-VBUS not isolated from the vehicle BEC, an always-on BT module tied
to a pinned-out UART RX (makes that UART unusable), etc.

## 5. Produce the review

Output in Henry's format and voice (direct, practical, terse; explains the *why* for the
user's benefit; offers concrete `suggestion` replacements where wording is the issue).

**The posted review only contains things the author needs to act on. It is not a report of
your review process.** Concretely:

1. **Preliminary / blockers** — run through §2 (images, buy link, bootloader, commit
   structure, CI, board id) while you work, but **only include this section in the output
   if at least one of them is actually a blocker.** If everything in §2 passes, drop the
   section entirely — do not list "images: OK / buy link: OK / CI: OK" etc. Henry doesn't
   confirm what's already fine, he only flags what isn't.
2. **Per-file findings** — grouped by file (`README.md`, `hwdef.dat`, `hwdef-bl.dat`,
   `defaults.parm`), each as an actionable item anchored to the relevant line/section.
   Provide a ```suggestion``` block with the corrected text whenever the fix is wording.
   **Only include a file/section heading if it has at least one finding under it.** Never
   write filler like "nothing notable here" / "looks fine" / "no issues found" for a file,
   a section (e.g. Compass), or a closing "everything else checks out" catch-all line —
   if there's nothing to flag, that file or topic just doesn't appear in the output at all.
3. **Closing line** — skip the formal "Summary verdict" (no bolded
   CHANGES_REQUESTED/COMMENTED/APPROVED label, no recap paragraph of what's good and what's
   not). End with a single short, direct line telling the author what to do next, in
   Henry's own voice, e.g. *"RE-request my review when these changes are made."* or
   *"request my review again when changes are done.."* — one sentence, not a summary.

**Always save the draft** to `~/Desktop/ardupilot/review-drafts/pr<N>_review.md` (create the
directory if it doesn't exist) — this is the standard output location for every run, not
something to ask about. Use that exact naming convention (lowercase `pr`, underscore, no
board name) to match the drafts already there.

Then **ask** whether to post it as a review comment on the PR. Default posting method is a
**single block comment** — the whole draft file as one review body, via
`gh pr review <N> --repo ArduPilot/ardupilot --request-changes/--comment --body-file <path>`
— not split into per-line inline comments (that would need the REST `pulls/<N>/comments`
API with path+line for each finding, which is unnecessary noise for this workflow; only do
that if Henry explicitly asks for inline/anchored comments instead). Do not post without
confirmation.

## Notes / limits

- Some checks need Henry's judgment or his local python hwdef checker (exact DMA sharing,
  timer-group legality, whether a specific board rev matches an image). Flag these as
  "verify" rather than asserting them.
- These criteria reflect reviews from ~Jan–Jul 2026; if the template or dev guide has
  changed, the files read in §0 win over this list.
