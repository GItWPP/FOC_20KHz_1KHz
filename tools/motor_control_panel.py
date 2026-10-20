"""Qt motor controls. All serial operations are queued to CaptureSession's worker."""
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
from motor_protocol import (SPEED_KP, SPEED_KI, CURRENT_KP, CURRENT_KI, POSITION_KP,
                            NAMES, gain_to_q16)


class MotorControlPanel(QtWidgets.QWidget):
    def __init__(self, session, parent=None):
        super().__init__(parent)
        self.session=session
        self.client=session.client
        self.loaded=False
        self.generation=None
        self.local_error=''
        self.last_events=None
        self.setMinimumWidth(285)
        self.setMaximumWidth(365)
        layout=QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(5,5,10,5)
        heading=QtWidgets.QLabel('运动控制')
        heading.setStyleSheet('font-size:20px; font-weight:600; color:#0f172a;')
        layout.addWidget(heading)
        self.link=QtWidgets.QLabel(f'{session.port} · {session.baud:,} baud\n正在连接，电机保持停止')
        self.link.setWordWrap(True)
        layout.addWidget(self.link)

        motion=QtWidgets.QGroupBox('启停与目标')
        grid=QtWidgets.QGridLayout(motion)
        grid.addWidget(QtWidgets.QLabel('模式'),0,0)
        self.mode=QtWidgets.QComboBox()
        self.mode.addItems(['速度控制','角度定位'])
        self.mode.currentIndexChanged.connect(self._mode_label)
        grid.addWidget(self.mode,0,1)
        self.rpm_label=QtWidgets.QLabel('目标转速')
        grid.addWidget(self.rpm_label,1,0)
        self.rpm=QtWidgets.QSpinBox()
        self.rpm.setRange(0,3000)
        self.rpm.setValue(300)
        self.rpm.setSuffix(' rpm')
        grid.addWidget(self.rpm,1,1)
        self.angle_label=QtWidgets.QLabel('目标角度')
        self.angle=QtWidgets.QDoubleSpinBox()
        self.angle.setRange(-2147483.647,2147483.647)
        self.angle.setDecimals(3)
        self.angle.setSuffix(' °')
        self.angle.setSingleStep(1)
        grid.addWidget(self.angle_label,2,0)
        grid.addWidget(self.angle,2,1)
        grid.addWidget(QtWidgets.QLabel('Iq 上限'),3,0)
        self.iq=QtWidgets.QSpinBox()
        self.iq.setRange(0,200)
        self.iq.setValue(200)
        self.iq.setSuffix(' mA')
        grid.addWidget(self.iq,3,1)
        self.apply=QtWidgets.QPushButton('发送目标')
        self.apply.clicked.connect(lambda:self._action(self._send_target))
        grid.addWidget(self.apply,4,0,1,2)
        self.start=QtWidgets.QPushButton('启动')
        self.start.setMinimumHeight(42)
        self.start.setStyleSheet('QPushButton {background:#15803d; color:white; font-size:16px; font-weight:600;} QPushButton:disabled {background:#cbd5e1; color:#64748b;}')
        self.start.clicked.connect(lambda:self._action(self._start))
        self.stop=QtWidgets.QPushButton('停止  [Esc]')
        self.stop.setMinimumHeight(42)
        self.stop.setStyleSheet('background:#b91c1c; color:white; font-size:16px; font-weight:600;')
        self.stop.clicked.connect(self.request_stop)
        grid.addWidget(self.start,5,0)
        grid.addWidget(self.stop,5,1)
        self.reset=QtWidgets.QPushButton('故障 / 控制复位')
        self.reset.clicked.connect(lambda:self._action(self.client.request_reset))
        grid.addWidget(self.reset,6,0,1,2)
        self.note=QtWidgets.QLabel('位置以启动对齐完成处为0°，支持正反多圈。停止会清除旧位置目标。\n角度分辨率约0.043945°；断线后需重新启动。')
        self.note.setWordWrap(True)
        self.note.setStyleSheet('color:#64748b; padding-top:5px;')
        grid.addWidget(self.note,7,0,1,2)
        layout.addWidget(motion)

        advanced=QtWidgets.QGroupBox('环路参数 · 停机应用')
        form=QtWidgets.QFormLayout(advanced)
        self.gains={}
        for key,title,maximum,default in [(SPEED_KP,'速度 Kp',10,1),(SPEED_KI,'速度 Ki',1,655/65536),
                                          (CURRENT_KP,'电流 Kp',16,2),(CURRENT_KI,'电流 Ki',1,.0625),
                                          (POSITION_KP,'位置 Kp /s',10,4)]:
            field=QtWidgets.QDoubleSpinBox()
            field.setRange(0,maximum)
            field.setDecimals(6)
            field.setSingleStep(.01 if key in (SPEED_KI,CURRENT_KI) else .1)
            field.setValue(default)
            field.setStyleSheet('QDoubleSpinBox:disabled {color:#475569; background:#f1f5f9;}')
            self.gains[key]=field
            form.addRow(title,field)
        self.advanced_apply=QtWidgets.QPushButton('应用环路参数')
        self.advanced_apply.clicked.connect(lambda:self._action(self._apply_gains))
        form.addRow(self.advanced_apply)
        self.sync=QtWidgets.QPushButton('载入设备当前参数')
        self.sync.clicked.connect(lambda:self._action(self.sync_from_device))
        form.addRow(self.sync)
        gain_note=QtWidgets.QLabel('显示基准系数；原低速增益调度保持。')
        gain_note.setWordWrap(True)
        gain_note.setStyleSheet('color:#64748b;')
        form.addRow(gain_note)
        self.advanced=advanced
        layout.addWidget(advanced)
        self.result=QtWidgets.QLabel('等待设备回执')
        self.result.setWordWrap(True)
        layout.addWidget(self.result)
        self.errors=QtWidgets.QLabel('')
        self.errors.setWordWrap(True)
        self.errors.setStyleSheet('color:#b91c1c;')
        layout.addWidget(self.errors)
        self.events=QtWidgets.QPlainTextEdit()
        self.events.setReadOnly(True)
        self.events.setMaximumBlockCount(40)
        self.events.setMinimumHeight(90)
        self.events.setMaximumHeight(135)
        self.events.setStyleSheet('font-size:11px; color:#475569; background:#f8fafc;')
        layout.addWidget(self.events,1)
        self.shortcut=QtWidgets.QShortcut(QtGui.QKeySequence('Escape'),parent or self)
        self.shortcut.activated.connect(self.request_stop)
        self._mode_label()
        self.update_state(session.snapshot(False))

    def _action(self, action):
        try:
            self.local_error=''
            action()
        except (ValueError,ConnectionError,RuntimeError) as exc:
            self.local_error=str(exc)
        self.errors.setText(self.local_error)

    def _mode_label(self, value=None):
        position=bool(self.mode.currentIndex())
        self.rpm_label.setText('运动限速' if position else '目标转速')
        self.angle_label.setVisible(position)
        self.angle.setVisible(position)

    def _send_target(self):
        self.client.target(self.mode.currentIndex(),self.rpm.value(),self.angle.value(),self.iq.value())

    def _start(self):
        self.client.target(self.mode.currentIndex(),self.rpm.value(),self.angle.value(),self.iq.value(),start=True)

    def request_stop(self):
        self.local_error=''
        self.client.request_stop()
        self.angle.setValue(0)

    def _apply_gains(self):
        values=[(key,gain_to_q16(field.value(),key)) for key,field in self.gains.items()]
        self.client.write_many(values)

    def sync_from_device(self, initial=False):
        row=self.session.snapshot(False)['last_frame']
        if not row or row['protocol_version']!=6:
            raise ConnectionError('等待V6设备数据')
        self.mode.setCurrentIndex(row['requested_position_mode'])
        self.rpm.setValue(row['target_rpm'])
        self.iq.setValue(row['iq_limit_ma'])
        self.angle.setValue(0 if initial else row['position_target_degrees'])
        fields={SPEED_KP:'speed_kp_q16',SPEED_KI:'speed_ki_q16',CURRENT_KP:'current_kp_q16',
                CURRENT_KI:'current_ki_q16',POSITION_KP:'position_kp_q16'}
        for key,name in fields.items():
            self.gains[key].setValue(row[name]/65536)
        self.loaded=True

    def update_state(self, snapshot):
        control=snapshot.get('control') or {}
        row=snapshot['last_frame']
        available=bool(control.get('available') and snapshot['connected'] and not snapshot['error'])
        running=bool(row and (row['start'] or row['drive']))
        queued=bool(control.get('pending') or control.get('queued'))
        if self.generation!=control.get('generation'):
            self.generation=control.get('generation')
            self.loaded=False
        if available and row and row['remote_armed'] and row['host_alive'] and not self.loaded:
            self.sync_from_device(initial=True)
        self.mode.setEnabled(available and not running and not queued)
        for widget in (self.rpm,self.angle,self.iq,self.apply):
            widget.setEnabled(available and self.loaded)
        self.start.setEnabled(available and self.loaded and not running and not queued and row['remote_armed']
                              and row['host_alive'] and not row['fault'] and row['state']==1)
        self.stop.setEnabled(bool(control.get('initialized')))
        self.reset.setEnabled(available and not running and not queued)
        for field in self.gains.values():
            field.setEnabled(available and not running and not queued)
        self.advanced_apply.setEnabled(available and not running and not queued)
        self.sync.setEnabled(available)
        if control.get('shutdown_requested'):
            state='正在停止电机，驱动关闭后退出'
        elif not row:
            state='等待串口数据'
        elif row['protocol_version']!=6:
            state='当前固件只支持监测，请下载串口控制版'
        elif not available:
            state='连接中断，停止命令已优先发送'
        elif not row['host_alive']:
            state='等待FPGA确认接收心跳，请检查RX接线'
        elif row['fault']:
            state=f"故障 0x{row['fault']:02X}，排除原因后复位"
        elif not row['remote_armed']:
            state='请先停止，再启动'
        else:
            state='串口控制已连接' + (' · 电机开启' if running else ' · 电机停止')
        self.link.setText(f'{self.session.port} · {self.session.baud:,} baud\n{state}')
        if row and row['protocol_version']==6:
            self.result.setText(f"待确认 {int(bool(control.get('pending')))+control.get('queued',0)} 条  "
                                f"FPGA收帧 {row['rx_good']} / 错误 {row['rx_errors']}\n"
                                f"设备设定 {row['target_rpm']}rpm · Iq上限 {row['iq_limit_ma']}mA")
        self.errors.setText(self.local_error)
        events=control.get('events',[])
        if events!=self.last_events:
            self.events.setPlainText('\n'.join(events))
            self.events.verticalScrollBar().setValue(self.events.verticalScrollBar().maximum())
            self.last_events=events
