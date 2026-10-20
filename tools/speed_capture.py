"""FPGA V6 duplex control/telemetry worker; 1.5Mbps/1kHz and V2/V3/V5 offline compatibility."""
import binascii
import csv
import json
import threading
import time
from collections import defaultdict, deque
from itertools import islice
from pathlib import Path
from speed_statistics import SpeedStatistics, STAT_FIELDS

FIELDS = ['time_s', 'host_s', 'host_unix_s', 'sequence', 'flags', 'start', 'drive', 'run',
          'indexed', 'calibrated', 'saturated', 'state', 'fault', 'speed_state', 'target_rpm',
          'ramp_rpm', 'speed_rpm', 'id_ma', 'iq_ma', 'id_ref_ma', 'iq_ref_ma', 'iq_limit_ma', 'position',
          'protocol_version', 'sample_period_s', 'speed_error_rpm', 'speed_error_pct',
          'position_mode', 'position_ready', 'position_settled', 'position_target_limited',
          'position_range_fault', 'position_mode_pending', 'position_counts', 'position_target_counts',
          'position_error_counts', 'position_turns', 'position_target_turns', 'position_error_deg',
          'position_degrees', 'position_target_degrees', 'remote_armed', 'host_alive',
          'requested_position_mode', 'ack_sequence', 'ack_opcode', 'ack_status', 'ack_key', 'ack_value',
          'speed_kp_q16', 'speed_ki_q16', 'current_kp_q16', 'current_ki_q16', 'position_kp_q16',
          'adc_frames', 'rx_good', 'rx_errors']


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0
        self.discarded = 0

    def feed(self, data):
        self.buffer.extend(data)
        rows = []
        while len(self.buffer) >= 4:
            if self.buffer[:2] != b'\xa5\x5a' or self.buffer[2] not in (2, 3, 5, 6) or self.buffer[3] != ({2:32,3:32,5:48,6:96}.get(self.buffer[2])):
                del self.buffer[0]
                self.discarded += 1
                continue
            length = self.buffer[3]
            if len(self.buffer) < length:
                break
            b = self.buffer[:length]
            if binascii.crc_hqx(b[:-2], 0xffff) != int.from_bytes(b[-2:], 'big'):
                del self.buffer[0]
                self.bad_crc += 1
                continue
            del self.buffer[:length]
            u16 = lambda n: int.from_bytes(b[n:n+2], 'big')
            i16 = lambda n: int.from_bytes(b[n:n+2], 'big', signed=True)
            i32 = lambda n: int.from_bytes(b[n:n+4], 'big', signed=True)
            position_flags = b[54] if b[2] == 6 else (b[42] if b[2] == 5 else 0)
            if b[2] == 6:
                angles = [int.from_bytes(b[offset:offset+8], 'big', signed=True) for offset in (30,38,46)]
                # Millidegree rounding is far smaller than one encoder count.
                counts = [((abs(value)*128+2812)//5625) * (-1 if value<0 else 1) for value in angles]
                position_counts, position_target, position_error = counts
                actual_degrees, target_degrees, error_degrees = [value/1000 for value in angles]
            else:
                position_counts, position_target, position_error = (i32(30),i32(34),i32(38)) if b[2] == 5 else (0,0,0)
                actual_degrees, target_degrees, error_degrees = [value*360/8192 for value in (position_counts,position_target,position_error)]
            rows.append(dict(
                sequence=u16(4), flags=b[6], start=b[6] & 1,
                drive=(b[6] >> 1) & 1, run=(b[6] >> 2) & 1,
                indexed=(b[6] >> 3) & 1, calibrated=(b[6] >> 4) & 1,
                saturated=(b[6] >> 7) & 1, state=b[7], fault=b[8], speed_state=b[9],
                target_rpm=u16(10), ramp_rpm=i16(12) if b[2] in (5,6) else u16(12),
                speed_rpm=int.from_bytes(b[14:18], 'big', signed=True) / 256.0,
                id_ma=i16(18), iq_ma=i16(20), id_ref_ma=i16(22), iq_ref_ma=i16(24),
                iq_limit_ma=u16(26), position=u16(28), protocol_version=b[2],
                sample_period_s=0.005 if b[2] == 2 else 0.001,
                position_mode=position_flags & 1, position_ready=(position_flags >> 1) & 1,
                position_settled=(position_flags >> 2) & 1, position_target_limited=(position_flags >> 3) & 1,
                position_range_fault=(position_flags >> 4) & 1, position_mode_pending=(position_flags >> 5) & 1,
                position_counts=position_counts, position_target_counts=position_target, position_error_counts=position_error,
                position_turns=actual_degrees/360, position_target_turns=target_degrees/360,
                position_degrees=actual_degrees, position_target_degrees=target_degrees,
                position_error_deg=error_degrees,
                remote_armed=(b[55]&1) if b[2]==6 else 0,
                host_alive=((b[55]>>1)&1) if b[2]==6 else ((b[6]>>6)&1),
                requested_position_mode=((b[55]>>2)&1) if b[2]==6 else (position_flags&1),
                ack_sequence=u16(56) if b[2]==6 else None,
                ack_opcode=b[58] if b[2]==6 else None, ack_status=b[59] if b[2]==6 else None,
                ack_key=b[60] if b[2]==6 else None,
                ack_value=(i32(62) if b[60]==4 else int.from_bytes(b[62:66],'big')) if b[2]==6 else None,
                speed_kp_q16=int.from_bytes(b[66:70],'big') if b[2]==6 else None,
                speed_ki_q16=int.from_bytes(b[70:74],'big') if b[2]==6 else None,
                current_kp_q16=int.from_bytes(b[74:78],'big') if b[2]==6 else None,
                current_ki_q16=int.from_bytes(b[78:82],'big') if b[2]==6 else None,
                position_kp_q16=int.from_bytes(b[82:86],'big') if b[2]==6 else None,
                adc_frames=int.from_bytes(b[86:90],'big') if b[2]==6 else None,
                rx_good=u16(90) if b[2]==6 else None, rx_errors=u16(92) if b[2]==6 else None))
        return rows


class CaptureSession:
    """Serial/file I/O stays on one worker; plotting never blocks data recording."""
    def __init__(self, port, output, source=None, seconds=0, baud=1500000, stats_window=5, settle_seconds=1, control=False):
        self.control_enabled = bool(control and source is None)
        if self.control_enabled:
            from motor_protocol import MotorClient
            self.client = MotorClient()
        else:
            self.client = None
        self.port = port
        self.base = Path(output)
        self.source = Path(source) if source else None
        self.seconds = seconds
        self.baud = int(baud)
        self.speed_statistics = SpeedStatistics(stats_window, settle_seconds)
        self.decoder = Decoder()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.done = threading.Event()
        self.rows = deque(maxlen=62000)
        self.plot_rows = deque(maxlen=62000)
        self.by_target = defaultdict(deque)
        self.started = time.monotonic()
        self.epoch_started = time.time()
        self.count = self.missing = self.resets = self.elapsed_seq = 0
        self.elapsed_time = 0.0
        self.previous_seq = None
        self.faults = set()
        self.error = None
        self.connected = False
        self.last_received = None
        self.last_frame = None
        self.thread = threading.Thread(target=self._run, name='foc-serial-capture', daemon=True)

    def start(self):
        self.base.parent.mkdir(parents=True, exist_ok=True)
        self.started = time.monotonic()
        self.epoch_started = time.time()
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=11 if self.client else 3)
        if self.thread.is_alive():
            raise RuntimeError('Serial acquisition did not finish controlled shutdown')

    def snapshot(self, include_rows=True):
        with self.lock:
            return dict(rows=list(self.rows) if include_rows else [], count=self.count,
                        missing=self.missing, resets=self.resets, bad_crc=self.decoder.bad_crc,
                        discarded=self.decoder.discarded, connected=self.connected, error=self.error,
                        last_received=self.last_received, last_frame=self.last_frame,
                        speed_stats=self.speed_statistics.snapshot(),
                        done=self.done.is_set(), control=self.client.snapshot() if self.client else None)

    def plot_snapshot(self, window_s):
        with self.lock:
            latest = self.last_frame
            if not latest:
                return []
            count = min(len(self.plot_rows), int(window_s/latest['sample_period_s'])+8)
            values = list(islice(reversed(self.plot_rows), count))
        values.reverse()
        return values

    def reset_stats(self):
        with self.lock:
            self.speed_statistics.reset()

    def set_stats_window(self, seconds):
        with self.lock:
            self.speed_statistics.window_s = float(seconds)

    def _append(self, row, now):
        seq = row['sequence']
        if self.previous_seq is not None:
            delta = (seq-self.previous_seq) & 0xffff
            reset = seq <= 16 and seq < self.previous_seq and delta > 16 and not row['calibrated']
            if reset or delta >= 32768:
                self.resets += 1
                self.elapsed_seq += 1
                self.elapsed_time += row['sample_period_s']
                self.speed_statistics.reset(clear_last=False)
            else:
                self.elapsed_seq += delta
                self.elapsed_time += delta*row['sample_period_s']
                self.missing += max(0, delta-1)
        self.previous_seq = seq
        row['time_s'] = self.elapsed_time
        row['speed_error_rpm'] = row['speed_rpm']-(row['ramp_rpm'] if row['position_mode'] else row['target_rpm'])
        row['speed_error_pct'] = 100*row['speed_error_rpm']/row['target_rpm'] if row['target_rpm'] and not row['position_mode'] else None
        row['host_s'] = round(now-self.started, 6)
        row['host_unix_s'] = round(self.epoch_started+now-self.started, 6)
        self.rows.append(row)
        self.plot_rows.append(tuple(row[k] for k in ('time_s','target_rpm','ramp_rpm','speed_rpm',
                                                    'speed_error_rpm','id_ma','iq_ma','iq_ref_ma',
                                                    'position_turns','position_target_turns','position_error_deg','position_mode')))
        self.speed_statistics.update(row)
        self.last_frame = row
        self.last_received = now
        self.count += 1
        self.faults.add(row['fault'])
        if not row['position_mode'] and row['speed_state'] == 2 and row['ramp_rpm'] == row['target_rpm']:
            values = self.by_target[row['target_rpm']]
            values.append((row['time_s'],row['speed_rpm']))
            while values and values[0][0] <= row['time_s']-1+1e-9:
                values.popleft()

    def _run(self):
        serial_port = source = None
        try:
            if self.source:
                source = self.source.open('rb')
            else:
                import serial
                serial_port = serial.Serial(port=None, baudrate=self.baud, timeout=0.02, write_timeout=.1)
                serial_port.dtr = False
                serial_port.rts = False
                serial_port.port = self.port
                serial_port.open()
            with self.lock:
                self.connected = True
            with self.base.with_suffix('.csv').open('w', newline='', encoding='utf-8-sig') as out, \
                    self.base.with_suffix('.bin').open('wb') as raw, \
                    self.base.with_name(self.base.name+'_speed_stats.csv').open('w',newline='',encoding='utf-8-sig') as stats_out:
                writer = csv.DictWriter(out, fieldnames=FIELDS)
                writer.writeheader()
                stats_writer = csv.DictWriter(stats_out, fieldnames=STAT_FIELDS)
                stats_writer.writeheader()
                last_flush = time.monotonic()
                last_stats_time = -1.0
                while True:
                    now = time.monotonic()
                    closing = self.stop_event.is_set() or (self.seconds and now-self.started >= self.seconds)
                    if self.client:
                        if closing:
                            self.client.request_shutdown(now)
                        outgoing = self.client.poll(now)
                        if outgoing:
                            serial_port.write(outgoing)
                        if self.client.shutdown_complete(now):
                            break
                    elif closing:
                        break
                    data = source.read(16384) if source else serial_port.read(min(max(serial_port.in_waiting, 1), 16384))
                    if source and not data:
                        break
                    raw.write(data)
                    now = time.monotonic()
                    decoded = self.decoder.feed(data)
                    with self.lock:
                        for row in decoded:
                            self._append(row, now)
                    if self.client:
                        for row in decoded:
                            self.client.on_frame(row, now)
                    writer.writerows(decoded)
                    if decoded and decoded[-1]['time_s']-last_stats_time >= 1:
                        with self.lock:
                            stats = self.speed_statistics.values()
                        if stats:
                            stats_writer.writerow(stats)
                        last_stats_time = decoded[-1]['time_s']
                    if now-last_flush >= 1:
                        out.flush()
                        raw.flush()
                        stats_out.flush()
                        last_flush = now
        except Exception as exc:
            with self.lock:
                self.error = f'{type(exc).__name__}: {exc}'
        finally:
            if serial_port:
                serial_port.close()
            if source:
                source.close()
            with self.lock:
                self.connected = False
            self.done.set()
            try:
                self.base.with_suffix('.summary.json').write_text(json.dumps(self.summary(), indent=2), encoding='utf-8')
            except OSError as exc:
                with self.lock:
                    self.error = f'Cannot save summary: {exc}'

    def summary(self):
        with self.lock:
            stats = {str(target): dict(samples=len(v), mean=sum(x[1] for x in v)/len(v), minimum=min(x[1] for x in v), maximum=max(x[1] for x in v))
                     for target, v in self.by_target.items() if v}
            return dict(capture_started_unix_s=self.epoch_started, valid_frames=self.count,
                        bad_crc=self.decoder.bad_crc, discarded_bytes=self.decoder.discarded,
                        missing_sequence_frames=self.missing, detected_resets=self.resets,
                        fault_codes=sorted(self.faults), last_frame=self.last_frame,
                        error=self.error, last_one_second_by_target=stats,
                        baud=self.baud, speed_statistics=self.speed_statistics.snapshot(),
                        control=self.client.snapshot() if self.client else None)
