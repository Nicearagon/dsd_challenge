# DeepSea CAN Diagnostic Tool

A receive-only diagnostic tool for the charger's internal CAN bus. It shows each
power module's live readings and fault codes, reassembles the multi-frame
identification strings, ignores unrelated traffic, and keeps working when the
bus misbehaves.

- Python 3, standard library only (`socket` with `AF_CAN`/`SOCK_RAW`/`CAN_RAW`, and `struct`).
- Never transmits a frame.
- Single file: `main.py`.

## Usage

```
python3 main.py --iface vcan0            # in-place terminal dashboard
python3 main.py --iface vcan0 --grader   # NDJSON event stream on stdout (ADAPTER.md)
```

Exit codes: `0` normal stop, `1` the CAN interface could not be opened, `2` the
platform has no SocketCAN support.

## How a frame flows through the tool

```
vcan0 ──► kernel filter (CAN_RAW_FILTER) ──► recv() ──► classify() ──► decoder / reassembler ──► sink
          noise is dropped here              16 bytes   second filter                          NDJSON or dashboard
```

1. **`open_can_socket()`** opens a raw CAN socket with kernel-side filters.
2. **`run()`** reads one `struct can_frame` at a time (`"=IB3x8s"`: id, dlc, padding, 8 data bytes) with a 0.2 s timeout, so periodic output keeps flowing on a quiet bus.
3. **`Monitor.handle()`** classifies the frame, counts it, and routes it to the telemetry decoder, the fault decoder or the reassembler.
4. A **sink** turns events into output: `GraderSink` writes NDJSON, `DashboardSink` redraws the screen.

Everything except `open_can_socket()` is platform-independent and takes bytes
and a clock as input, which is what made it testable away from the device.

## Filtering

### Choice: kernel filter first, user-space check second

**1. Kernel acceptance filters (`CAN_RAW_FILTER`).** The socket installs three
`(can_id, can_mask)` pairs. A frame is delivered when
`(rx_id & mask) == (can_id & mask)` for any of them:

| Filter ID | Mask | Accepts |
|---|---|---|
| `0x100` | `0x7FC` + EFF + RTR | `0x100`–`0x103` (telemetry) |
| `0x1F0` | `0x7FF` + EFF + RTR | `0x1F0` only (fault codes) |
| `0x6F0` | `0x7FC` + EFF + RTR | `0x6F0`–`0x6F3` (identification strings) |

- **`0x7FC`** ignores the two low bits, so one rule covers four consecutive IDs. This works because both `0x100` and `0x6F0` end in binary `00`.
- **`0x7FF`** compares all 11 bits, so the fault rule matches exactly one ID. A narrower mask such as `0x3FF` would also accept `0x5F0`.
- **The EFF and RTR flag bits are part of every mask.** Without them, the mask only looks at the low 11 bits:
  - An extended 29-bit frame such as `0x80000100` is a different message from another protocol, yet it would alias module 0's telemetry.
  - A remote request for `0x100` carries no data and would decode as a bogus 0 V / 0 A / −40 °C reading.

  With the flags in the mask, the kernel requires them to be clear, as they are in the filter IDs.
- **Error frames need no mask bit.** They are only delivered when requested with `CAN_RAW_ERR_FILTER`, which the tool never sets.
- **The filters are installed before `bind()`,** so no unfiltered frame can reach the queue in between.

**Why in the kernel.** Noise never reaches the process: no copy, no wake-up and
no Python work per discarded frame. More importantly, noise does not compete for
space in the socket's receive queue. Noise rises to 150 frames/s halfway through
the run, and if that queue overflows, the kernel silently drops frames. A dropped
frame would directly break the `frames_processed` count, the telemetry freshness
check and reassembly. Kernel filtering is also SocketCAN's native mechanism, so
it needs no dependency.

This was confirmed on the device: across a full 60 s generator run, including
the high-noise second half, the tool's own count of frames that reached user
space but were rejected by `classify()` stayed at **0**. All noise was dropped
by the kernel.

**2. User-space check (`classify()`).** The same rule is repeated in Python: it
rejects any frame with the EFF, RTR or ERR flag and maps IDs to
telemetry / fault / diag / ignore. This is defense in depth: the tool stays
correct even if the kernel filter were missing or wrong. It is also the only
filter when frames come from a non-CAN source, such as a replay. A test checks
that both layers accept exactly the same set of IDs over the whole 11-bit range,
with and without flags.

### Alternatives considered

- **Filtering only in Python:** correct, but every noise frame wakes the process and takes queue space.
- **One socket per message type:** more moving parts, and it loses the single, ordered view of the bus.
- **BPF socket filters:** more power than three ID ranges need.
- **An inverted filter (`CAN_INV_FILTER`):** a single rule could reject the noise range `0x200`–`0x2FF` instead of accepting the three wanted ranges. It works against the noise we know about, but any other unrelated traffic that appears on the bus would get through. An allow-list fails closed; a deny-list fails open.
- **A broadcast manager socket (`CAN_BCM`):** the kernel's subsystem for cyclic messages. It can deliver a frame only when its content differs from the previous frame on the same ID, and it can report an ID that stops arriving, which would be a kernel-side "stale" signal. Here it does not pay off:
  - **Content-change filtering would lose real frames.** Telemetry and diagnostic frames always differ from the previous frame on their ID (the sequence counter and PCI bytes change), but fault frames do not: two consecutive faults with the same module and code are identical. That happened in one of three seeded replays of the generator. BCM would suppress the second one, dropping a real fault and undercounting `frames_processed`. Every frame has to be seen.
  - **With that filtering off,** only the silence timeout is left, in exchange for a more complex message format and one subscription per ID (nine of them).

### What `frames_processed` counts

Every frame whose ID is in one of the three ranges, **including** frames that
are later discarded: orphan Consecutive Frames, oversized First Frames, short
payloads. This matches the generator's `real_frame_count`, which counts frames
sent, not valid messages. Noise is never counted.

## Reassembly state management

### Keying: one slot per CAN ID

`Reassembler` holds a dictionary `{0x6F0: slot, 0x6F1: slot, 0x6F2: slot, 0x6F3: slot}`.
Each module has its own ID and at most one message in flight, so the CAN ID
identifies the in-progress message unambiguously. The four modules' frames
arrive interleaved frame by frame; separate slots mean no attempt can see or
corrupt another module's bytes.

### Bounding: fixed at startup

- All four slots are created once, at construction. Each one holds a preallocated `bytearray(64)` plus four integers: declared length, bytes filled, expected sequence nibble and the time of the last accepted frame.
- **Nothing is allocated per message.** A new First Frame resets the existing slot and overwrites its buffer.
- **Frames for any ID outside the set create no state:** `dict.get()` returns `None` and the frame is ignored.

So repeated abandonment (situation 5) cannot grow memory: an abandoned First
Frame is simply overwritten by the next one. The bound is 4 × 64 bytes of
payload state no matter how long the tool runs or what the bus does. On the
device, process memory stayed at exactly the same value throughout a 10-minute
run (see Validation). The rest of
the application state is bounded the same way: latest telemetry per module
(4 entries), last completed string per ID (4 entries), and a `deque(maxlen=8)`
of recent faults.

### Expiry: replacement, plus a lazy timeout

- **The normal expiry mechanism is replacement.** A module that gives up on a message eventually sends a new First Frame, which discards the old attempt. This needs no timer.
- **As a safety net,** an attempt is also dropped if more than 1 s passes between two of its frames (`DIAG_TIMEOUT_NS`, the N_Cr value from ISO 15765-2). This prevents a stale First Frame from being completed by a Consecutive Frame that arrives much later.
- **The timeout is checked lazily,** when the next frame for that ID arrives, so there is no timer thread and no locking.

### State machine

Each frame is dispatched on the high nibble of byte 0 (the PCI type):

| Frame | Action |
|---|---|
| **First Frame** (`0x1`) | Any attempt in progress is dropped. The 12-bit length is read; a length `< 8` (should have been a Single Frame) or `> 64` is rejected. Otherwise the first 6 bytes are stored and the expected sequence becomes 1. |
| **Consecutive Frame** (`0x2`) | Ignored if no attempt is in progress. The attempt is abandoned if the sequence nibble is not the expected one. Otherwise the next `min(7, remaining)` bytes are copied (bytes past the declared length are padding) and the expected nibble advances mod 16 (1…15, 0, 1…). When the declared length is reached, the message is returned and the slot is freed. |
| **Anything else** (Single Frame, Flow Control, reserved) | Ignored, without touching the attempt in progress. |

One deliberate decision: **a First Frame always drops the previous attempt,
even if the new First Frame is itself invalid** (oversized or truncated). A new
First Frame means the sender has abandoned what it was sending, so keeping the
old attempt alive could only lead to completing a message the sender no longer
stands behind.

### The five situations

| Situation | What happens | Counter |
|---|---|---|
| 1. Orphan Consecutive Frame | No attempt in progress → ignored | `orphan_cf` |
| 2. Restarting First Frame | Old attempt dropped, new one started | `restarted` |
| 3. Oversized length claim | First Frame rejected; nothing allocated | `rejected_len` |
| 4. Out-of-order sequence | Attempt abandoned; its remaining frames become orphans | `out_of_order` |
| 5. Repeated abandonment | Each new First Frame overwrites the same slot | `restarted` |

No output is ever produced for an abandoned attempt. Because state is per ID,
none of these affect another module's message in progress. Completed strings
are decoded as ASCII with `errors="replace"`, so unexpected bytes cannot crash
the tool.

The counters do not affect grader output. They are shown on the dashboard and
were used to verify that each situation was detected for the right reason.

## Decoding

- **Telemetry** (`"<HHBBH"`): voltage `raw × 0.1`, current `raw × 0.01`, temperature `raw − 40`, status bits enabled / fault / derated, and `seq`. Values are rounded with the same expression the generator uses to log ground truth (`round(raw * 0.1, 2)`), which avoids float artifacts such as `412.30000000000007`. The module number comes from the CAN ID.
- **Fault codes** (`"BB6x"`): module ID and fault code. The full 8-byte payload defined by the spec is required.
- **Short payloads** are counted as processed frames but not decoded or reported.

## Output

**Grader mode** (`--grader`):
- One JSON object per line on stdout, flushed after every line, emitted as events happen: `telemetry`, `fault`, `diag_complete` (with `ts_ns` = `time.monotonic_ns()` at the moment of completion).
- `stats` is emitted every second and once more on exit.
- Diagnostics go to stderr only.

**Dashboard mode:**
- In-place redraw with plain ANSI escape codes (no `curses`), at a fixed 5 Hz regardless of frame rate, so drawing never falls behind the bus.
- Shows readings, identification strings, recent faults with their names, frame counts and reassembly counters.
- A module with no telemetry for 2 s is marked `STALE`.

**Shutdown:**
- SIGINT, SIGTERM and SIGHUP only set a flag. The loop notices within 0.2 s (the socket timeout), exits cleanly, and the final `stats` line is always written.
- If stdout's reader goes away (e.g. `| head`), the broken pipe is detected, output stops, and the process exits quietly instead of printing a traceback.

## Validation

### On the Raspberry Pi (`vcan0`, the official generator)

A 60 s run with the dashboard started **before** the generator ended with exactly the expected counters:

```
completed=928 restarted=925 rejected_len=1 orphan_cf=1 out_of_order=1 timed_out=0
```

- **928 messages** = 232 cycles of 0.25 s × 4 modules: every clean message was reassembled.
- **`restarted` is 925 and not 928** because on three of the special cycles (orphan, oversized, out-of-order) there is no pending abandoned First Frame to restart.
- **`rejected_len`, `orphan_cf` and `out_of_order` are each exactly 1:** each situation was detected once, for the right reason, and no frame was lost mid-message.
- **`ignored: 0`:** no noise frame reached the process, so the kernel filter is doing the filtering, with `classify()` as a backstop that never had to act.

A 90 s run in **grader mode**, stopped with SIGTERM after the generator
finished, was compared line by line against the generator's
`ground_truth.jsonl`:

```
PASS telemetry - 720 reported / 720 sent
PASS faults - [(2, 3), (0, 3), (2, 2)] / [(2, 3), (0, 3), (2, 2)]
PASS frames_processed - 7766 / 7766 (104 stats lines)
PASS diag 0x6f0 - 352 completed / 352 sent
PASS diag 0x6f1 - 352 completed / 352 sent
PASS diag 0x6f2 - 352 completed / 352 sent
PASS diag 0x6f3 - 352 completed / 352 sent
PASS last line is stats - stats
situations: [('out_of_order', '0x6f0'), ('restart_ff', '0x6f1'), ('orphan_cf', '0x6f3'), ('oversized_len', '0x6f2')]
```

- **Telemetry** lines are identical to the send log, in order, so the last line per module is also the freshest.
- **Each ID** completed exactly the clean messages that were sent, in order. That includes the recovery message after each situation, with no completion from a broken attempt.
- **The final `stats`** equals the generator's real-frame count, and it is the last line written, after SIGTERM.
- **Stats cadence:** 104 `stats` lines, one per second for the whole time the tool ran: the 90 s of traffic, plus the time before the generator started and between its end and SIGTERM.

**Memory over a 10-minute run.** The tool ran in grader mode for 727 s against a
600 s generator run: about 2,400 abandonment cycles per module. All 9,568
messages were reassembled. The process RSS (`VmRSS` from `/proc/<pid>/status`)
was sampled every 20 s, starting about two minutes in, so at least the last six
minutes of live traffic (over 1,400 abandonment cycles per module) fall inside
the sampled window. Every sample read the same value:

```
VmRSS:     13504 kB   (25 of 25 samples)
```

The 13.5 MB is essentially the Python interpreter itself; the tool's own state
is a few kilobytes and is allocated once, as described above.

**Closed output.** With live traffic, `python3 main.py --iface vcan0 --grader | head -3`
printed three NDJSON lines and returned immediately. The tool detected the broken
pipe, stopped, and exited with status 0, with no traceback.

**Starting mid-burst is a real case, and it is handled.** In an earlier run the
tool was started roughly 7 s after the generator. It reported
`completed=820 restarted=818 rejected_len=1 orphan_cf=7 out_of_order=0`:
- 108 messages (about 27 cycles) were sent before the tool was listening.
- The out-of-order situation most likely fell inside that window.
- The tool came up in the middle of a burst, after some modules' First Frames. Their Consecutive Frames were correctly treated as orphans: ignored, with no incorrect message completed and no error.

### During development (away from the device)

- **Unit tests** cover decoding and rounding, every reassembly situation, four interleaved IDs, the 15 → 0 nibble wrap, truncated frames, the timeout, and the run loop and sinks. A bounded-memory test runs 50,000 abandonment cycles under `tracemalloc` and checks that `main.py`'s allocations do not grow.
- **The official generator was replayed** under a virtual clock, with its socket swapped for an in-memory capture, across several random seeds. The NDJSON output matched its `ground_truth.jsonl` exactly: telemetry values, freshness, faults, `frames_processed` against `real_frame_count`, and the exact sequence of completed strings per ID.
- **A 90 s real-time simulation** (generator → local UDP → the same `serve()` loop) passed all of those checks, with 7766 of 7766 real frames counted.
- **The tests themselves were checked** by injecting typical bugs into a copy of the code: a buffer shared across IDs, counting noise, accepting lengths over 64, a per-message allocation, not resetting on a restarting First Frame, and not abandoning on a bad sequence. The tests caught each one.

## Limitations and what I would do with more time

- **Kernel drops are not detected.** If the socket queue overflows, frames are lost silently. I would enable `SO_RXQ_OVFL` and read the drop counter through `recvmsg()`, and also flag gaps in each module's telemetry `seq`.
- **`ts_ns` is processing time, not arrival time.** If the process falls behind, completion timestamps drift. `SO_TIMESTAMP` would give the kernel's receive time.
- **Single thread.** In grader mode, a slow stdout reader blocks the read loop through `flush()`. A writer thread with a bounded queue would decouple the two.
- **The timeout is lazy.** An attempt on an ID that goes permanently silent stays "in progress" forever. This costs no memory (the slot exists anyway) and cannot produce a wrong result, but the state is never cleaned up on its own.
- **Fault payloads are not validated beyond their length.** The module ID is not range-checked and the reserved bytes are not required to be zero. I chose to report a fault with odd padding rather than drop it, on high-voltage equipment.
- **Basic dashboard.** It does not adapt to terminal size or handle resizes.
- **No CAN FD.** 72-byte FD frames are rejected by `parse_frame()`.

---

This project was developed with AI assistance.
