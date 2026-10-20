"""Fast Qt/pyqtgraph frontend; full-rate acquisition/statistics stay in the worker."""
import time
import traceback
from collections import deque
from datetime import datetime
from pathlib import Path
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets


class LivePlot:
    def __init__(self, session, window=10, snapshot=None, refresh_hz=60):
        self.session = session
        self.window = float(window)
        self.refresh_hz = int(refresh_hz)
        self.snapshot_path = Path(snapshot) if snapshot else None
        self.paused = False
        self.position_mode = False
        self.auto_y = True
        self.clear_before = -1.0
        self.closed = False
        self.error = None
        self.rendered_frames = self.last_display_count = self.paint_count = 0
        self.update_times = deque(maxlen=1200)
        self.last_header = self.last_rate_time = time.monotonic()
        self.last_rate_count = self.last_render_count = 0
        self.rate = self.actual_fps = 0.0
        self.backend = 'pyqtgraph '+pg.__version__
        pg.setConfigOptions(antialias=False, background='w', foreground='#30343b')
        self.app = pg.mkQApp('FPGA FOC Live Curves')
        self.app.setFont(QtGui.QFont('Microsoft YaHei',9))
        parent = self

        class Window(QtWidgets.QWidget):
            def closeEvent(self, event):
                if parent.close():
                    event.accept()
                else:
                    event.ignore()

        class PaintCounter(QtCore.QObject):
            def eventFilter(self, obj, event):
                if event.type() == QtCore.QEvent.Paint:
                    parent.paint_count += 1
                return False

        self.widget = Window()
        self.widget.setWindowTitle(f'FPGA FOC 电机控制台 - {session.port}')
        self.widget.resize(1620 if session.client else 1250,950)
        self.widget.setStyleSheet('QGroupBox {font-weight:600; border:1px solid #cbd5e1; border-radius:5px; margin-top:10px; padding-top:12px;} QGroupBox::title {subcontrol-origin:margin; left:10px;} QPushButton {padding:5px;}')
        outer = QtWidgets.QHBoxLayout(self.widget)
        self.control_panel = None
        if session.client:
            from motor_control_panel import MotorControlPanel
            self.control_panel = MotorControlPanel(session,self.widget)
            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
            scroll.setWidget(self.control_panel)
            scroll.setMinimumWidth(300)
            scroll.setMaximumWidth(380)
            outer.addWidget(scroll)
        right = QtWidgets.QWidget()
        outer.addWidget(right,1)
        layout = QtWidgets.QVBoxLayout(right)
        layout.setContentsMargins(0,0,0,0)
        title = QtWidgets.QLabel('FPGA FOC · 实时状态与曲线')
        title.setStyleSheet('font-size:20px; font-weight:600; padding:3px;')
        layout.addWidget(title)
        self.connection = QtWidgets.QLabel('正在连接串口…')
        self.values = QtWidgets.QLabel('等待第一帧有效数据')
        self.values.setStyleSheet('font-size:15px; padding:3px;')
        self.detail = QtWidgets.QLabel('')
        for label in (self.connection,self.values,self.detail):
            layout.addWidget(label)
        stats_box = QtWidgets.QGroupBox('稳态转速误差 / 波动（全量样本，不平滑）')
        self.stats_box = stats_box
        grid = QtWidgets.QGridLayout(stats_box)
        self.stats_status = QtWidgets.QLabel('到达目标后等待1秒，再计算最近5秒')
        grid.addWidget(self.stats_status,0,0,1,4)
        self.stats_labels = {}
        for index,(key,title) in enumerate([('mean','平均转速'),('bias','平均偏差'),('pp','波动峰峰值'),('std','标准差'),
                                             ('range','转速范围'),('max','最大绝对误差'),('rmse','RMSE'),('coverage','样本 / 覆盖')]):
            cell = QtWidgets.QLabel(title+'：--')
            cell.setStyleSheet('font-size:13px; padding:3px;')
            grid.addWidget(cell,1+index//4,index%4)
            self.stats_labels[key] = cell
        self.position_check = QtWidgets.QLabel('位置累计均速：--（与反馈均值使用同一统计窗）')
        self.position_check.setStyleSheet('font-size:13px; padding:4px;')
        grid.addWidget(self.position_check,3,0,1,4)
        layout.addWidget(stats_box)
        self.graphics = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics,1)
        self.paint_filter = PaintCounter(self.graphics)
        self.graphics.viewport().installEventFilter(self.paint_filter)
        self.plots = []
        self.lines = {}
        specs = [[('target_rpm',1,'设定','#ea580c'),('ramp_rpm',2,'斜坡','#16a34a'),('speed_rpm',3,'实际','#2563eb')],
                 [('speed_error_rpm',4,'实际 - 设定','#7c3aed')],
                 [('id_ma',5,'Id','#16a34a'),('iq_ma',6,'Iq','#2563eb'),('iq_ref_ma',7,'Iq参考','#ea580c')]]
        self.columns = {}
        for row,group in enumerate(specs):
            p = self.graphics.addPlot(row=row,col=0)
            p.showGrid(x=True,y=True,alpha=.18)
            p.addLegend(offset=(10,5),labelTextSize='9pt')
            p.setLabel('left',['转速','跟踪误差','电流'][row],units=['rpm','rpm','mA'][row])
            p.getAxis('left').enableAutoSIPrefix(False)
            p.setMouseEnabled(x=False,y=True)
            p.setXRange(-window,0,padding=0)
            if row:
                p.setXLink(self.plots[0])
            self.plots.append(p)
            for key,column,label,color in group:
                line = p.plot(pen=pg.mkPen(color,width=1),name=label,connect='all')
                line.setDownsampling(auto=True,method='peak')
                line.setClipToView(True)
                self.lines[key] = line
                self.columns[key] = column
                if key == 'speed_error_rpm':
                    self.error_legend_label = p.legend.items[-1][1]
        self.plots[2].setLabel('bottom','相对当前时刻 / s（最新数据 = 0）')
        self.plots[1].addItem(pg.InfiniteLine(pos=0,angle=0,pen=pg.mkPen('#9ca3af',width=1)))
        controls = QtWidgets.QHBoxLayout()
        self.pause_button = QtWidgets.QPushButton('暂停曲线 [空格]')
        self.pause_button.clicked.connect(self.toggle_pause)
        self.y_button = QtWidgets.QPushButton('自动Y轴：开')
        self.y_button.clicked.connect(self.toggle_y)
        self.clear_button = QtWidgets.QPushButton('清空曲线')
        self.clear_button.clicked.connect(self.clear)
        self.save_button = QtWidgets.QPushButton('保存截图')
        self.save_button.clicked.connect(self.save)
        self.reset_stats_button = QtWidgets.QPushButton('重置统计')
        self.reset_stats_button.clicked.connect(self.reset_stats)
        for button in (self.pause_button,self.y_button,self.clear_button,self.save_button,self.reset_stats_button):
            controls.addWidget(button)
        controls.addWidget(QtWidgets.QLabel('显示窗(s)'))
        self.window_spin = QtWidgets.QSpinBox()
        self.window_spin.setRange(5,60)
        self.window_spin.setValue(int(window))
        self.window_spin.valueChanged.connect(self.set_window)
        controls.addWidget(self.window_spin)
        controls.addWidget(QtWidgets.QLabel('统计窗(s)'))
        self.stats_spin = QtWidgets.QSpinBox()
        self.stats_spin.setRange(1,30)
        self.stats_spin.setValue(int(session.speed_statistics.window_s))
        self.stats_spin.valueChanged.connect(session.set_stats_window)
        controls.addWidget(self.stats_spin)
        layout.addLayout(controls)
        self.footer = QtWidgets.QLabel('暂停只暂停曲线；采集与心跳持续。关闭控制台会先停止电机。1kHz数据完整保存在CSV。' if session.client else '只读监测；暂停/关闭不发送控制命令。数据完整保存在CSV。')
        layout.addWidget(self.footer)
        self.shortcut = QtWidgets.QShortcut(QtGui.QKeySequence('Space'),self.widget)
        self.shortcut.activated.connect(self.toggle_pause)
        self.timer = QtCore.QTimer(self.widget)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.setInterval(max(5,round(1000/refresh_hz)))
        self.timer.timeout.connect(self.update_safe)

    def toggle_pause(self, checked=False):
        self.paused = not self.paused
        self.pause_button.setText('继续滚动 [空格]' if self.paused else '暂停曲线 [空格]')
        for p in self.plots:
            p.setMouseEnabled(x=self.paused,y=True)

    def toggle_y(self, checked=False):
        self.auto_y = not self.auto_y
        self.y_button.setText('自动Y轴：开' if self.auto_y else '自动Y轴：关')
        for p in self.plots:
            p.enableAutoRange(axis='y',enable=self.auto_y)

    def clear(self, checked=False):
        r = self.session.snapshot(False)['last_frame']
        self.clear_before = r['time_s'] if r else -1
        for line in self.lines.values():
            line.setData([],[])

    def set_window(self, value):
        self.window = float(value)
        self.plots[0].setXRange(-self.window,0,padding=0)

    def reset_stats(self, checked=False):
        self.session.reset_stats()

    def save(self, checked=False):
        path = self.session.base.parent/(self.session.base.name+'_view_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.png')
        self.widget.grab().save(str(path))
        self.footer.setText(f'截图已保存：{path}')

    def close(self):
        self.session.stop_event.set()
        if self.session.client and not self.session.done.is_set():
            self.session.client.request_shutdown()
            self.footer.setText('正在停止电机并等待驱动关闭…')
            return False
        self.closed = True
        self.timer.stop()
        return True

    def _show_stats(self, snapshot):
        s = snapshot['speed_stats']
        current = s['current']
        v = current or s['last_valid']
        remaining = s['settling_remaining_s'] or 0.0
        note = {'collecting':'正在统计','settling':f"等待稳速 {remaining:.1f}s",
                'ramping':'加减速中，排除瞬态','not_running':'停机 / 未运行'}.get(s['status'],s['status'])
        if not current and v:
            note += '；保留上次稳态结果'
        self.stats_status.setText(f"最近 {s['window_s']:g}s，目标变化 / 斜坡到达后等待 {s['settle_s']:g}s  |  {note}")
        texts = {'mean':'平均转速：--','bias':'平均偏差：--','pp':'波动峰峰值：--','std':'标准差：--',
                 'range':'转速范围：--','max':'最大绝对误差：--','rmse':'RMSE：--','coverage':'样本 / 覆盖：--'}
        if v:
            texts.update(mean=f"平均转速：{v['mean_rpm']:.3f} rpm（设定 {v['target_rpm']}）",
                         bias=f"平均偏差：{v['bias_rpm']:+.3f} rpm / {v['bias_pct']:+.3f}%",
                         pp=f"波动峰峰值：{v['peak_to_peak_rpm']:.3f} rpm / {v['peak_to_peak_pct']:.3f}%",
                         std=f"标准差：{v['std_rpm']:.3f} rpm",
                         range=f"转速范围：{v['min_rpm']:.3f} ～ {v['max_rpm']:.3f} rpm",
                         max=f"最大绝对误差：{v['max_abs_error_rpm']:.3f} rpm / {v['max_abs_error_pct']:.3f}%",
                         rmse=f"RMSE：{v['rmse_rpm']:.3f} rpm",
                         coverage=f"样本 / 覆盖：{v['samples']} / {v['span_s']:.2f}s")
        for key,text in texts.items():
            self.stats_labels[key].setText(text)
        if v and v.get('position_mean_rpm') is not None:
            difference = v['feedback_position_difference_rpm']
            self.position_check.setText(f"位置累计均速：{v['position_mean_rpm']:.3f} rpm    "
                                       f"位置均速偏差：{v['position_bias_rpm']:+.3f} rpm / {v['position_bias_pct']:+.3f}%    "
                                       f"反馈均值−位置均速：{difference:+.3f} rpm（同窗对照）")
            color = '#b91c1c' if abs(difference)>max(.2,.02*v['target_rpm']) else '#166534'
            self.position_check.setStyleSheet(f'font-size:13px; padding:4px; color:{color};')
        else:
            self.position_check.setText('位置累计均速：--（收集同窗位置数据；长缺帧时不推测圈数）')
            self.position_check.setStyleSheet('font-size:13px; padding:4px;')

    def update_safe(self):
        try:
            self.update()
        except Exception as exc:
            self.error = f'{type(exc).__name__}: {exc}'
            traceback.print_exc()
            self.widget.close()

    def update(self):
        begin = time.perf_counter()
        now = time.monotonic()
        snap = self.session.snapshot(False)
        r = snap['last_frame']
        if now-self.last_header >= .1:
            if now-self.last_rate_time >= 1:
                self.rate = (snap['count']-self.last_rate_count)/(now-self.last_rate_time)
                self.actual_fps = (self.rendered_frames-self.last_render_count)/(now-self.last_rate_time)
                self.last_rate_count,self.last_render_count = snap['count'],self.rendered_frames
                self.last_rate_time = now
            state = '采集中' if snap['connected'] else '采集已停止'
            if snap['last_received'] is None or now-snap['last_received']>.5:
                state = '等待数据'
            if self.session.source:
                state = '离线回放'
            if snap['error']:
                state = snap['error']
            self.connection.setText(f"{self.session.port} | {self.session.baud} / 8N1 | {state} | "
                                    f"接收 {self.rate:.0f}Hz / 绘图 {self.actual_fps:.0f}fps | "
                                    f"帧 {snap['count']}  CRC错误 {snap['bad_crc']}  缺帧 {snap['missing']}")
            if r:
                pct = '--' if r['speed_error_pct'] is None else f"{r['speed_error_pct']:+.3f}%"
                self.values.setText(f"设定 {r['target_rpm']} rpm    实际 {r['speed_rpm']:.3f} rpm    "
                                    f"偏差 {r['speed_error_rpm']:+.3f} rpm / {pct}    Iq {r['iq_ma']} mA")
                if r['position_mode']:
                    self.values.setText(f"目标 {r['position_target_degrees']:.3f}°    实际 {r['position_degrees']:.3f}°    "
                                        f"位置误差 {r['position_error_deg']:+.3f}°    "
                                        f"{'已到位保持' if r['position_settled'] else ('定位中' if r['position_ready'] else '等待启动建零')}")
                self.detail.setText(f"start={r['start']}  驱动={r['drive']}  状态={r['state']}/{r['speed_state']}  "
                                    f"故障=0x{r['fault']:02X}  {'暂停显示，持续记录' if self.paused else '实时滚动'}")
                if r['position_mode']:
                    self.detail.setText(self.detail.text()+f"  转速={r['speed_rpm']:.3f}rpm  上限={r['target_rpm']}rpm"
                                        +f"  Iq={r['iq_ma']}mA  模式待切换={r['position_mode_pending']}")
                if bool(r['position_mode']) != self.position_mode:
                    self.position_mode = bool(r['position_mode'])
                    self.stats_box.setVisible(not self.position_mode)
                    self.plots[0].setLabel('left','位置' if self.position_mode else '转速',units='°' if self.position_mode else 'rpm')
                    self.plots[1].setLabel('left','位置误差' if self.position_mode else '跟踪误差',units='°' if self.position_mode else 'rpm')
                    self.error_legend_label.setText('目标 - 实际' if self.position_mode else '实际 - 设定')
            if self.control_panel:
                self.control_panel.update_state(snap)
            self._show_stats(snap)
            self.last_header = now
        if not self.paused and r and snap['count'] != self.last_display_count:
            values = self.session.plot_snapshot(self.window)
            if values:
                data = np.asarray(values,dtype=np.float64)
                data = data[data[:,0]>=max(r['time_s']-self.window,self.clear_before)]
                data = data[data[:,11] == r['position_mode']]
                xs = data[:,0]-r['time_s']
                position_columns = {'target_rpm':9,'speed_rpm':8,'speed_error_rpm':10}
                for key,line in self.lines.items():
                    if r['position_mode'] and key == 'ramp_rpm':
                        line.setData([],[])
                    else:
                        column = position_columns.get(key,self.columns[key]) if r['position_mode'] else self.columns[key]
                        line.setData(xs,data[:,column]*(360 if r['position_mode'] and key in ('target_rpm','speed_rpm') else 1),skipFiniteCheck=True)
                self.last_display_count = snap['count']
                self.rendered_frames += 1
            self.update_times.append((time.perf_counter()-begin)*1000)
        if self.session.client and self.session.client.shutdown_requested and self.session.done.is_set():
            self.widget.close()
        if self.session.seconds and now-self.session.started >= self.session.seconds:
            self.widget.close()

    def run(self):
        self.widget.show()
        self.started = time.monotonic()
        self.last_rate_time = self.started
        self.timer.start()
        pg.exec()
        self.timer.stop()
        if self.snapshot_path:
            self.snapshot_path.parent.mkdir(parents=True,exist_ok=True)
            self.widget.grab().save(str(self.snapshot_path))
        elapsed = max(.001,time.monotonic()-self.started)
        return dict(backend=self.backend,rendered_frames=self.rendered_frames,paint_events=self.paint_count,
                    plot_fps=self.rendered_frames/elapsed,paint_fps=self.paint_count/elapsed,
                    update_ms_p50=float(np.percentile(self.update_times,50)) if self.update_times else None,
                    update_ms_p95=float(np.percentile(self.update_times,95)) if self.update_times else None,
                    last_display_count=self.last_display_count,closed=self.closed,error=self.error)
