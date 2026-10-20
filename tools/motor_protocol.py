"""V1 command packets and a single-writer, ACK-driven V6 motor client.

GUI code only queues requests. The acquisition worker owns the serial port and
calls poll/on_frame, so heartbeats and STOP never depend on drawing speed.
"""
import binascii
import math
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from decimal import Decimal, ROUND_HALF_UP

HEARTBEAT, WRITE, READ, RESET = 1, 2, 3, 4
START, MODE, RPM, ANGLE, IQ = 1, 2, 3, 4, 5
SPEED_KP, SPEED_KI, CURRENT_KP, CURRENT_KI, POSITION_KP = range(0x10, 0x15)
LIMITS = {START: (0, 1), MODE: (0, 1), RPM: (0, 3000), ANGLE: (-2**31, 2**31-1),
          IQ: (0, 200), SPEED_KP: (0, 655360), SPEED_KI: (0, 65536),
          CURRENT_KP: (0, 1048576), CURRENT_KI: (0, 65536), POSITION_KP: (0, 655360)}
NAMES = {START: '启停', MODE: '模式', RPM: '转速/限速', ANGLE: '角度目标', IQ: 'Iq上限',
         SPEED_KP: '速度Kp', SPEED_KI: '速度Ki', CURRENT_KP: '电流Kp',
         CURRENT_KI: '电流Ki', POSITION_KP: '位置Kp'}
RESULTS = {0: '已执行', 1: '未知命令', 2: '未知参数', 3: '参数超范围',
           4: '请先停止电机', 5: '尚未重新使能，请先停止', 6: '驱动尚未就绪',
           7: '驱动存在故障', 8: '控制器忙', 255: '暂无回执'}


def encode_command(sequence, opcode, key=0, value=0):
    if not 0 <= int(sequence) <= 65535 or not 0 <= int(opcode) <= 255 or not 0 <= int(key) <= 255:
        raise ValueError('Invalid command identity')
    if not -2**31 <= int(value) <= 2**32-1:
        raise ValueError('Command value does not fit32bits')
    body = struct.pack('>2sBBHBBIH', b'\x5a\xa5', 1, 16, int(sequence), int(opcode), int(key), int(value)&0xffffffff, 0)
    return body + struct.pack('>H', binascii.crc_hqx(body, 0xffff))


def degrees_to_mdeg(degrees):
    value = Decimal(str(degrees))
    if not value.is_finite():
        raise ValueError('角度必须是有限数值')
    result = int((value*1000).quantize(Decimal('1'), rounding=ROUND_HALF_UP))
    if not LIMITS[ANGLE][0] <= result <= LIMITS[ANGLE][1]:
        raise ValueError('角度超出可发送范围')
    return result


def gain_to_q16(gain, key):
    value = float(gain)
    if not math.isfinite(value):
        raise ValueError('系数必须是有限数值')
    result = int((Decimal(str(gain))*65536).quantize(Decimal('1'), rounding=ROUND_HALF_UP))
    low, high = LIMITS[key]
    if not low <= result <= high:
        raise ValueError(f'{NAMES[key]}超范围')
    return result


@dataclass
class Command:
    sequence: int
    opcode: int
    key: int
    value: int
    frame: bytes
    attempts: int = 0
    sent_at: float = 0


class MotorClient:
    ACK_TIMEOUT = .18
    HEARTBEAT_PERIOD = .2
    FRESH_TIMEOUT = .5
    MAX_ATTEMPTS = 3
    SHUTDOWN_TIMEOUT = 8.5

    def __init__(self, sequence=None):
        self.lock = threading.RLock()
        self.sequence = (time.monotonic_ns() if sequence is None else sequence) & 0xffff
        self.queue = deque()
        self.pending = None
        self.events = deque(maxlen=40)
        self.latest = None
        self.last_received = None
        self.initialized = False
        self.generation = 0
        self.stale = False
        self.next_heartbeat = 0.0
        self.shutdown_requested = False
        self.shutdown_deadline = None
        self.shutdown_confirmed = False
        self.last_result = None
        self.sent_commands = self.retries = 0

    def _event(self, text):
        self.events.append(time.strftime('%H:%M:%S')+'  '+text)

    def _new(self, opcode, key=0, value=0):
        self.sequence = (self.sequence+1) & 0xffff
        return Command(self.sequence, opcode, key, value, encode_command(self.sequence, opcode, key, value))

    def _check_ready(self):
        if not self.initialized or self.stale or not self.latest or self.latest['protocol_version'] != 6:
            raise ConnectionError('等待V6控制固件和有效串口数据')
        if self.shutdown_requested:
            raise ConnectionError('正在停止并退出')

    def _add(self, commands):
        self._check_ready()
        if len(self.queue)+len(commands)>64:
            raise RuntimeError('待发命令过多，请等待回执')
        result=[]
        for opcode,key,value in commands:
            if opcode == WRITE and (key not in LIMITS or not LIMITS[key][0] <= int(value) <= LIMITS[key][1]):
                raise ValueError('参数超出允许范围')
            result.append((opcode,key,int(value)))
        ids=[]
        for opcode,key,value in result:
            command=self._new(opcode,key,value)
            self.queue.append(command)
            ids.append(command.sequence)
        return ids

    def write(self, key, value):
        with self.lock:
            return self._add([(WRITE,key,value)])[0]

    def read(self, key):
        with self.lock:
            return self._add([(READ,key,0)])[0]

    def write_many(self, values):
        with self.lock:
            return self._add([(WRITE,key,value) for key,value in values])

    def target(self, mode, rpm, degrees=0, iq_limit=200, start=False):
        with self.lock:
            self._check_ready()
            running = bool(self.latest['start'] or self.latest['drive'])
            actual_mode = int(self.latest['requested_position_mode'])
            if running and int(mode) != actual_mode:
                raise ValueError('切换模式前请先停止电机')
            if start and running:
                raise ValueError('电机已开启，请使用发送目标')
            values=[]
            if not running:
                values.append((MODE,int(mode)))
            values += [(IQ,int(iq_limit)), (RPM,int(rpm))]
            if mode:
                values.append((ANGLE,degrees_to_mdeg(degrees)))
            if start:
                if not self.latest['remote_armed'] or not self.latest['host_alive'] or self.latest['fault'] or self.latest['state'] != 1:
                    raise ValueError('驱动尚未就绪或需要先停止/复位')
                if not rpm or not iq_limit:
                    raise ValueError('启动时转速/限速和Iq上限必须大于0')
                values.append((START,1))
            return self._add([(WRITE,key,value) for key,value in values])

    def _stop(self, reason):
        self.queue.clear()
        self.pending=None
        command=self._new(WRITE,START,0)
        self.queue.append(command)
        self._event(reason+'：请求停止，清除待发指令')
        return command.sequence

    def request_stop(self, reason='手动停止'):
        with self.lock:
            return self._stop(reason)

    def request_reset(self):
        with self.lock:
            self._check_ready()
            if self.latest['drive'] or self.latest['start']:
                raise ValueError('请等待电机停止后再复位')
            self._stop('复位前停机')
            self.queue.append(self._new(RESET))
            self.queue.append(self._new(WRITE,START,0))

    def request_shutdown(self, now=None):
        with self.lock:
            if not self.shutdown_requested:
                now=time.monotonic() if now is None else now
                self.shutdown_requested=True
                self.shutdown_deadline=now+self.SHUTDOWN_TIMEOUT
                self._stop('关闭控制台')

    def on_frame(self, row, now):
        with self.lock:
            previous=self.latest
            reboot=bool(previous and previous['protocol_version']==6 and row['protocol_version']==6
                        and row['ack_status']==255 and previous['ack_status']!=255
                        and row['adc_frames']<previous['adc_frames'] and not row['remote_armed'])
            self.latest=row
            self.last_received=now
            if row['protocol_version'] != 6:
                return
            if not self.initialized or self.stale or reboot:
                self.initialized=True
                self.generation+=1
                self.stale=False
                self.next_heartbeat=now
                self._stop('串口接入/恢复')
            command=self.pending
            if command and (row['ack_sequence'],row['ack_opcode'],row['ack_key']) == (command.sequence,command.opcode,command.key):
                status=row['ack_status']
                self.last_result=dict(sequence=command.sequence,opcode=command.opcode,key=command.key,
                                      status=status,value=row['ack_value'],requested_value=command.value)
                self._event(f"#{command.sequence} {NAMES.get(command.key,'复位')}：{RESULTS.get(status,str(status))}")
                self.pending=None
                if status:
                    had_start = command.key==START and command.value==1 or any(c.key==START and c.value==1 for c in self.queue)
                    self.queue.clear()
                    if had_start:
                        self._stop('启动序列被拒绝')
                elif command.opcode==WRITE and command.key==START and command.value==0 and self.shutdown_requested:
                    self.shutdown_confirmed=True

    def poll(self, now):
        with self.lock:
            if not self.initialized or not self.latest or self.latest['protocol_version'] != 6:
                return b''
            fresh = self.last_received is not None and now-self.last_received <= self.FRESH_TIMEOUT
            if not fresh and not self.stale:
                self.stale=True
                self._stop('遥测超时')
            frames=[]
            if self.pending and now-self.pending.sent_at>=self.ACK_TIMEOUT:
                if self.pending.attempts>=self.MAX_ATTEMPTS:
                    failed=self.pending
                    self.pending=None
                    self.queue.clear()
                    self._event(f'#{failed.sequence} 未收到回执')
                    if not (failed.opcode==WRITE and failed.key==START and failed.value==0):
                        self._stop('命令超时')
                else:
                    self.pending.sent_at=now
                    self.pending.attempts+=1
                    self.retries+=1
                    frames.append(self.pending.frame)
            if not self.pending and self.queue:
                self.pending=self.queue.popleft()
                self.pending.attempts=1
                self.pending.sent_at=now
                self.sent_commands+=1
                frames.append(self.pending.frame)
            if fresh and now>=self.next_heartbeat:
                frames.append(encode_command(0,HEARTBEAT))
                self.next_heartbeat=now+self.HEARTBEAT_PERIOD
            return b''.join(frames)

    def shutdown_complete(self, now):
        with self.lock:
            if not self.shutdown_requested:
                return False
            if not self.initialized:
                return True
            if self.shutdown_confirmed and self.latest and not self.latest['start'] and not self.latest['drive']:
                return True
            return now>=self.shutdown_deadline

    def snapshot(self, now=None):
        with self.lock:
            now=time.monotonic() if now is None else now
            fresh=self.last_received is not None and now-self.last_received<=self.FRESH_TIMEOUT
            return dict(available=self.initialized and fresh and not self.stale and not self.shutdown_requested and bool(self.latest and self.latest['protocol_version']==6),
                        initialized=self.initialized,generation=self.generation,stale=self.stale or not fresh,
                        pending=(dict(sequence=self.pending.sequence,key=self.pending.key,attempts=self.pending.attempts) if self.pending else None),
                        queued=len(self.queue),events=list(self.events),last_result=self.last_result,
                        shutdown_requested=self.shutdown_requested,shutdown_confirmed=self.shutdown_confirmed,
                        sent_commands=self.sent_commands,retries=self.retries)
