#!/usr/bin/env python3
"""DeepSea CAN diagnostic tool.

Receive-only monitor for the charger's internal CAN bus. It decodes
power-module telemetry and fault frames, and reassembles the ISO 15765-2-style
identification strings each module sends across several frames.

    python3 main.py --iface vcan0            # live terminal dashboard
    python3 main.py --iface vcan0 --grader   # NDJSON event stream (ADAPTER.md)

Standard library only. Everything that touches AF_CAN lives in
open_can_socket(), so the rest of this module imports and runs on any
platform. The tests rely on that to exercise the whole pipeline without a
CAN interface.
"""
import argparse
import collections
import json
import os
import signal
import socket
import struct
import sys
import time

# --- Wire format -------------------------------------------------------------

FRAME_FMT = "=IB3x8s"  # struct can_frame: can_id, dlc, 3 pad bytes, data[8]
FRAME_SIZE = struct.calcsize(FRAME_FMT)  # 16

CAN_EFF_FLAG = 0x80000000  # extended (29-bit) frame
CAN_RTR_FLAG = 0x40000000  # remote transmission request
CAN_ERR_FLAG = 0x20000000  # error frame

NUM_MODULES = 4
TELEMETRY_BASE = 0x100
FAULT_ID = 0x1F0
DIAG_BASE = 0x6F0
DIAG_IDS = tuple(range(DIAG_BASE, DIAG_BASE + NUM_MODULES))

MAX_DIAG_LEN = 64
MIN_DIAG_LEN = 8  # anything shorter would have been a Single Frame
DIAG_TIMEOUT_NS = 1000000000  # ISO 15765-2 N_Cr: max gap between consecutive frames

KIND_TELEMETRY = "telemetry"
KIND_FAULT = "fault"
KIND_DIAG = "diag"

FAULT_NAMES = {1: "overtemp", 2: "overvoltage", 3: "undervoltage", 4: "isolation_fault"}

# Kernel acceptance filters as (can_id, can_mask): a frame passes when
# (rx_id & mask) == (can_id & mask), and the filters are OR-ed together.
# can_id is the reference value; the mask has a 1 for every bit that must
# match and a 0 for every bit that is ignored. For example:
#
#   0x7FC = 111 1111 1100  -> ignore the 2 low bits: 4 consecutive IDs, as long
#                             as the base ends in binary 00 (0x100, 0x6F0 do)
#     telemetry filter, can_id 0x100:
#     0x103 & 0x7FC = 0x100 == 0x100 & 0x7FC   pass   (so do 0x100, 0x101, 0x102)
#     0x104 & 0x7FC = 0x104 != 0x100           reject
#     0x0FF & 0x7FC = 0x0FC != 0x100           reject
#     diag filter, can_id 0x6F0:
#     0x6F2 & 0x7FC = 0x6F0 == 0x6F0 & 0x7FC   pass   (so do 0x6F0, 0x6F1, 0x6F3)
#     0x6F4 & 0x7FC = 0x6F4 != 0x6F0           reject
#     0x250 & 0x7FC = 0x250 != 0x6F0           reject (noise)
#   0x7FF = 111 1111 1111  -> compare all 11 bits: exactly one ID
#     fault filter, can_id 0x1F0:
#     0x1F0 & 0x7FF = 0x1F0                    pass
#     0x5F0 & 0x7FF = 0x5F0 != 0x1F0           reject
#   A narrower mask such as 0x3FF would skip bit 10 and also accept 0x5F0,
#   which is why an exact match always uses 0x7FF whatever the ID value.
#
# The ID masks alone (0x7FC / 0x7FF) only cover the low 11 bits, so on their
# own they would let through frames that merely alias these IDs:
#   - EFF: an extended 29-bit frame such as 0x80000100 is a different message
#     from another protocol, but its low bits match module 0's telemetry.
#   - RTR: a remote request for 0x100 asks for telemetry and carries no data;
#     decoding it would report a bogus 0 V / 0 A / -40 C reading.
# Adding both flag bits to every mask makes the kernel compare them too, and
# since the filter IDs have them clear, matching frames must have them clear.
# ERR needs no mask bit: error frames are only delivered when requested with
# CAN_RAW_ERR_FILTER, which this tool never sets. classify() rejects all three
# again in user space.
_FLAG_BITS = CAN_EFF_FLAG | CAN_RTR_FLAG
KERNEL_FILTERS = (
    (TELEMETRY_BASE, 0x7FC | _FLAG_BITS),  # 0x100-0x103
    (FAULT_ID, 0x7FF | _FLAG_BITS),        # 0x1F0
    (DIAG_BASE, 0x7FC | _FLAG_BITS),       # 0x6F0-0x6F3
)

_FRAME = struct.Struct(FRAME_FMT)
_TELEMETRY = struct.Struct("<HHBBH")
_FAULT = struct.Struct("BB6x")  # module_id, fault_code, 6 reserved bytes


def parse_frame(raw):
    """Split a raw struct can_frame into (can_id, data), or None if malformed."""
    if len(raw) != FRAME_SIZE:
        return None
    can_id, dlc, data = _FRAME.unpack(raw)
    return can_id, data[:min(dlc, 8)]


def classify(can_id):
    """Return the kind of frame for IDs we care about, None for everything else.

    Mirrors KERNEL_FILTERS in user space: a second line of defence, and the only
    filter when frames come from somewhere other than a filtered CAN socket.
    """
    if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
        return None
    if TELEMETRY_BASE <= can_id < TELEMETRY_BASE + NUM_MODULES:
        return KIND_TELEMETRY
    if can_id == FAULT_ID:
        return KIND_FAULT
    if DIAG_BASE <= can_id < DIAG_BASE + NUM_MODULES:
        return KIND_DIAG
    return None


# --- Decoding ----------------------------------------------------------------

def decode_telemetry(module, data):
    """Decode an 8-byte telemetry payload, or None if it is too short."""
    if len(data) < _TELEMETRY.size:
        return None
    v_raw, c_raw, t_raw, status, seq = _TELEMETRY.unpack_from(data)
    return {
        "module": module,
        "seq": seq,
        "voltage": round(v_raw * 0.1, 2),
        "current": round(c_raw * 0.01, 2),
        "temp_c": t_raw - 40,
        "enabled": bool(status & 0x01),
        "fault": bool(status & 0x02),
        "derated": bool(status & 0x04),
    }


def decode_fault(data):
    """Decode an 8-byte fault-code payload, or None if it is too short."""
    if len(data) < _FAULT.size:
        return None
    module, code = _FAULT.unpack_from(data)
    return {"module": module, "code": code}


# --- Multi-frame reassembly --------------------------------------------------

class _Slot:
    """In-progress message for one CAN ID. Allocated once, reused forever."""

    __slots__ = ("buf", "length", "filled", "next_seq", "last_ns")

    def __init__(self, max_len):
        self.buf = bytearray(max_len)
        self.reset()

    def reset(self):
        self.length = 0  # 0 means idle
        self.filled = 0
        self.next_seq = 1
        self.last_ns = 0

    @property
    def active(self):
        return self.length != 0


class Reassembler:
    """ISO 15765-2-style receive state machine, one slot per known CAN ID.

    Slots are created up front for a fixed set of IDs and each holds a
    preallocated max_len buffer, so memory does not depend on traffic: an
    abandoned First Frame is simply overwritten by the next one. Frames for
    IDs outside the set are ignored and never create state.
    """

    def __init__(self, can_ids, max_len=MAX_DIAG_LEN, timeout_ns=DIAG_TIMEOUT_NS):
        self.max_len = max_len
        self.timeout_ns = timeout_ns
        self._slots = {can_id: _Slot(max_len) for can_id in can_ids}
        self.counters = dict.fromkeys(
            ("completed", "restarted", "rejected_len", "orphan_cf",
             "out_of_order", "timed_out", "malformed", "ignored_pci"), 0)

    @property
    def slot_count(self):
        return len(self._slots)

    def in_progress(self, can_id):
        slot = self._slots.get(can_id)
        return slot is not None and slot.active

    def feed(self, can_id, data, now_ns):
        """Process one frame. Returns the complete message bytes or None."""
        slot = self._slots.get(can_id)
        if slot is None or not data:
            return None

        if slot.active and now_ns - slot.last_ns > self.timeout_ns:
            self.counters["timed_out"] += 1
            slot.reset()

        pci = data[0] >> 4
        if pci == 0x1:
            self._first_frame(slot, data, now_ns)
            return None
        if pci == 0x2:
            return self._consecutive_frame(slot, data, now_ns)
        # Single Frame / Flow Control / reserved: not part of this protocol.
        # Ignored without touching the message in progress.
        self.counters["ignored_pci"] += 1
        return None

    def _first_frame(self, slot, data, now_ns):
        # Any First Frame means the sender gave up on whatever it was sending,
        # so the old attempt is dropped even if the new one turns out invalid.
        if slot.active:
            self.counters["restarted"] += 1
            slot.reset()
        if len(data) < 8:
            self.counters["malformed"] += 1
            return
        length = ((data[0] & 0x0F) << 8) | data[1]
        if length < MIN_DIAG_LEN or length > self.max_len:
            self.counters["rejected_len"] += 1
            return
        slot.buf[0:6] = data[2:8]
        slot.length = length
        slot.filled = 6
        slot.next_seq = 1
        slot.last_ns = now_ns

    def _consecutive_frame(self, slot, data, now_ns):
        if not slot.active:
            self.counters["orphan_cf"] += 1
            return None
        if data[0] & 0x0F != slot.next_seq:
            self.counters["out_of_order"] += 1
            slot.reset()
            return None
        take = min(7, slot.length - slot.filled)
        chunk = data[1:1 + take]  # bytes past the declared length are padding
        if len(chunk) < take:
            self.counters["malformed"] += 1
            slot.reset()
            return None
        slot.buf[slot.filled:slot.filled + take] = chunk
        slot.filled += take
        slot.next_seq = (slot.next_seq + 1) & 0x0F
        slot.last_ns = now_ns
        if slot.filled < slot.length:
            return None
        message = bytes(slot.buf[:slot.length])
        slot.reset()
        self.counters["completed"] += 1
        return message


# --- Application state -------------------------------------------------------

class Monitor:
    """Routes classified frames to decoders and keeps the latest known state.

    Every container here is fixed-size or bounded (four modules, four diag IDs,
    a capped fault history), so the process footprint stays flat however long
    it runs.
    """

    def __init__(self, sink, clock=time.monotonic_ns, recent_faults=8):
        self.sink = sink
        self.clock = clock
        self.frames_processed = 0  # telemetry + fault + diag frames, never noise
        self.frames_ignored = 0
        self.telemetry = [None] * NUM_MODULES
        self.telemetry_ns = [0] * NUM_MODULES
        self.recent_faults = collections.deque(maxlen=recent_faults)
        self.diag_strings = {can_id: None for can_id in DIAG_IDS}
        self.reassembler = Reassembler(DIAG_IDS)

    def handle(self, can_id, data):
        kind = classify(can_id)
        if kind is None:
            self.frames_ignored += 1
            return
        self.frames_processed += 1

        if kind == KIND_TELEMETRY:
            module = can_id - TELEMETRY_BASE
            reading = decode_telemetry(module, data)
            if reading is not None:
                self.telemetry[module] = reading
                self.telemetry_ns[module] = self.clock()
                self.sink.on_telemetry(reading)
        elif kind == KIND_FAULT:
            fault = decode_fault(data)
            if fault is not None:
                self.recent_faults.append(fault)
                self.sink.on_fault(fault)
        else:
            message = self.reassembler.feed(can_id, data, self.clock())
            if message is not None:
                completed_ns = self.clock()
                text = message.decode("ascii", errors="replace")
                self.diag_strings[can_id] = text
                self.sink.on_diag(can_id, text, completed_ns)


# --- Output ------------------------------------------------------------------

class GraderSink:
    """NDJSON event stream on stdout, as specified in ADAPTER.md."""

    tick_interval_ns = 1000000000  # stats line every second

    def __init__(self, out=None):
        self.out = out if out is not None else sys.stdout
        self.closed = False

    def _emit(self, obj):
        if self.closed:
            return
        try:
            self.out.write(json.dumps(obj, separators=(",", ":")) + "\n")
            self.out.flush()
        except BrokenPipeError:
            self.closed = True  # reader went away; run() stops on this

    def on_start(self, monitor):
        pass

    def on_telemetry(self, reading):
        line = {"type": "telemetry"}
        line.update(reading)
        self._emit(line)

    def on_fault(self, fault):
        self._emit({"type": "fault", "module": fault["module"], "code": fault["code"]})

    def on_diag(self, can_id, text, ts_ns):
        self._emit({"type": "diag_complete", "can_id": hex(can_id), "string": text, "ts_ns": ts_ns})

    def on_tick(self, monitor):
        self._emit({"type": "stats", "frames_processed": monitor.frames_processed})

    def on_exit(self, monitor):
        self.on_tick(monitor)


def render_dashboard(monitor, now_ns=None, stale_after_ns=2000000000):
    """Return the dashboard as plain text (no escape codes)."""
    if now_ns is None:
        now_ns = monitor.clock()
    rule = "-" * 72
    lines = ["DeepSea CAN Diagnostic Tool", rule]
    for module, t in enumerate(monitor.telemetry):
        if t is None:
            lines.append("Module %d:  (no telemetry yet)" % module)
            continue
        stale = now_ns - monitor.telemetry_ns[module] > stale_after_ns
        lines.append(
            "Module %d: %6.1fV %7.2fA %4dC  enabled=%-5s fault=%-5s derated=%-5s seq=%-5d%s"
            % (module, t["voltage"], t["current"], t["temp_c"], t["enabled"], t["fault"],
               t["derated"], t["seq"], "  STALE" if stale else ""))
    lines += [rule, "Identification strings:"]
    for can_id, text in monitor.diag_strings.items():
        lines.append("  %s: %s" % (hex(can_id), text if text is not None else "(not yet received)"))
    lines += [rule, "Recent faults:"]
    if monitor.recent_faults:
        for f in reversed(monitor.recent_faults):
            name = FAULT_NAMES.get(f["code"], "unknown")
            lines.append("  module %d, code %d (%s)" % (f["module"], f["code"], name))
    else:
        lines.append("  (none)")
    c = monitor.reassembler.counters
    lines += [
        rule,
        "frames processed: %d   ignored: %d" % (monitor.frames_processed, monitor.frames_ignored),
        "reassembly: completed=%d restarted=%d rejected_len=%d orphan_cf=%d "
        "out_of_order=%d timed_out=%d"
        % (c["completed"], c["restarted"], c["rejected_len"], c["orphan_cf"],
           c["out_of_order"], c["timed_out"]),
        "Ctrl+C to exit",
    ]
    return "\n".join(lines)


class DashboardSink:
    """In-place terminal dashboard drawn with plain ANSI escape codes."""

    tick_interval_ns = 200000000  # redraw at 5 Hz, independent of frame rate

    def __init__(self, out=None):
        self.out = out if out is not None else sys.stdout
        self.closed = False

    def _write(self, text):
        try:
            self.out.write(text)
            self.out.flush()
        except BrokenPipeError:
            self.closed = True

    def on_start(self, monitor):
        self._write("\x1b[?25l\x1b[2J")  # hide cursor, clear screen

    def on_telemetry(self, reading):
        pass

    def on_fault(self, fault):
        pass

    def on_diag(self, can_id, text, ts_ns):
        pass

    def on_tick(self, monitor):
        body = render_dashboard(monitor).replace("\n", "\x1b[K\n")
        self._write("\x1b[H" + body + "\x1b[K\n\x1b[J")  # home, redraw, clear the rest

    def on_exit(self, monitor):
        self.on_tick(monitor)
        self._write("\x1b[?25h")  # show cursor again


# --- Main loop -----------------------------------------------------------------

def run(sock, monitor, sink, should_stop=lambda: False):
    """Read frames until asked to stop, then let the sink emit its final output.

    `sock` only needs recv(); it should have a timeout so periodic output
    (stats, redraws) keeps flowing on a quiet bus. recv() returning b"" means
    end of stream, which a CAN socket never does but replay sources can.
    """
    sink.on_start(monitor)
    next_tick = monitor.clock()
    try:
        while not should_stop() and not sink.closed:
            try:
                raw = sock.recv(FRAME_SIZE)
            except socket.timeout:
                raw = None
            if raw == b"":
                break
            if raw:
                frame = parse_frame(raw)
                if frame is not None:
                    monitor.handle(*frame)
            now = monitor.clock()
            if now >= next_tick:
                sink.on_tick(monitor)
                next_tick = now + sink.tick_interval_ns
    finally:
        sink.on_exit(monitor)


def open_can_socket(iface, timeout=0.2, rcvbuf=1 << 20):
    """Open a receive-only raw CAN socket with kernel-side ID filtering (Linux)."""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        filters = b"".join(struct.pack("=II", can_id, mask) for can_id, mask in KERNEL_FILTERS)
        # Filters go on before bind() so no unfiltered frame is ever queued.
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        except OSError:
            pass  # kernel caps it at net.core.rmem_max; the default still works
        sock.bind((iface,))
        sock.settimeout(timeout)
    except BaseException:
        sock.close()
        raise
    return sock


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="DeepSea CAN bus diagnostic tool (receive only).")
    ap.add_argument("--iface", default="vcan0", help="CAN interface to listen on (default: vcan0)")
    ap.add_argument("--grader", action="store_true", help="emit NDJSON events on stdout instead of the dashboard")
    return ap.parse_args(argv)


def serve(sock, grader, out=None):
    """Run the tool on an already-open socket until a signal or end of stream.

    Split from main() so any socket-like source (a replay, a UDP bridge in
    dev/sim.py) goes through exactly the same sink, signal and shutdown path.
    """
    sink = GraderSink(out) if grader else DashboardSink(out)
    monitor = Monitor(sink)

    stop = []

    def request_stop(signum, frame):
        stop.append(signum)

    for name in ("SIGINT", "SIGTERM", "SIGHUP"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), request_stop)

    try:
        run(sock, monitor, sink, should_stop=lambda: bool(stop))
    finally:
        if sink.closed and sink.out is sys.stdout:
            # stdout is a dead pipe: point it at /dev/null so the interpreter's
            # final flush doesn't raise again on the way out.
            devnull = os.open(os.devnull, os.O_WRONLY)
            os.dup2(devnull, sys.stdout.fileno())
    return monitor


def main(argv=None):
    args = parse_args(argv)
    if not hasattr(socket, "AF_CAN"):
        print("error: this Python has no AF_CAN support (SocketCAN requires Linux)", file=sys.stderr)
        return 2
    try:
        sock = open_can_socket(args.iface)
    except OSError as exc:
        print("error: cannot open CAN interface %r: %s" % (args.iface, exc), file=sys.stderr)
        return 1

    print("[candiag] listening on %s (%s mode)" % (args.iface, "grader" if args.grader else "dashboard"),
          file=sys.stderr)
    try:
        serve(sock, args.grader)
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
