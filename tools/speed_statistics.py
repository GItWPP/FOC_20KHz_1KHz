"""Rolling steady-speed metrics, using every unfiltered telemetry sample."""
import math
from collections import deque

STAT_FIELDS = ['time_s', 'target_rpm', 'samples', 'span_s', 'mean_rpm', 'min_rpm', 'max_rpm',
               'bias_rpm', 'bias_pct', 'peak_to_peak_rpm', 'peak_to_peak_pct', 'std_rpm',
               'rmse_rpm', 'max_abs_error_rpm', 'max_abs_error_pct', 'position_mean_rpm',
               'position_bias_rpm', 'position_bias_pct', 'feedback_position_difference_rpm']


class SpeedStatistics:
    def __init__(self, window_s=5.0, settle_s=1.0):
        self.window_s = float(window_s)
        self.settle_s = float(settle_s)
        self.last_valid = None
        self.reset()

    def reset(self, clear_last=True):
        self.samples = deque()
        self.minimum = deque()
        self.maximum = deque()
        self.total = self.total_sq = 0.0
        self.number = 0
        self.position_total = self.position_epoch = 0
        self.previous_position = self.position_time = None
        self.target = None
        self.since = None
        self.now = 0.0
        self.status = 'not_running'
        if clear_last:
            self.last_valid = None

    def update(self, row):
        t, speed, target = row['time_s'], row['speed_rpm'], row['target_rpm']
        eligible = (not row.get('position_mode', 0) and target > 0 and row['start'] and row['drive'] and row['state'] == 6
                    and row['speed_state'] == 2 and not row['fault'] and row['ramp_rpm'] == target)
        if not eligible:
            self.reset(clear_last=False)
            self.now = t
            self.status = 'position_mode' if row.get('position_mode', 0) else ('ramping' if row['drive'] else 'not_running')
            return
        if self.target != target:
            self.reset(clear_last=False)
            self.target = target
            self.since = t
        self.now = t
        if t-self.since+1e-9 < self.settle_s:
            self.status = 'settling'
            return
        self.status = 'collecting'
        position = row.get('position')
        if position is None:
            self.position_epoch += 1
            self.previous_position = None
        else:
            if self.previous_position is not None:
                dt = t-self.position_time
                # At3000rpm10ms may hide half a revolution: do not guess across gaps.
                if not 0 < dt < .009999:
                    self.position_epoch += 1
                    self.position_total = 0
                else:
                    self.position_total += (position-self.previous_position+4096)%8192-4096
            else:
                self.position_total = 0
            self.previous_position, self.position_time = position,t
        while self.samples and self.samples[0][0] <= t-self.window_s+1e-9:
            _, old_number, old, _, _ = self.samples.popleft()
            self.total -= old
            self.total_sq -= old*old
            if self.minimum and self.minimum[0][0] == old_number:
                self.minimum.popleft()
            if self.maximum and self.maximum[0][0] == old_number:
                self.maximum.popleft()
        self.number += 1
        self.samples.append((t, self.number, speed,self.position_total,self.position_epoch))
        self.total += speed
        self.total_sq += speed*speed
        while self.minimum and self.minimum[-1][1] >= speed:
            self.minimum.pop()
        while self.maximum and self.maximum[-1][1] <= speed:
            self.maximum.pop()
        self.minimum.append((self.number, speed))
        self.maximum.append((self.number, speed))

    def values(self):
        n = len(self.samples)
        if not n:
            return None
        mean = self.total/n
        variance = max(0.0, self.total_sq/n-mean*mean)
        lo, hi = self.minimum[0][1], self.maximum[0][1]
        bias = mean-self.target
        peak = max(abs(lo-self.target), abs(hi-self.target))
        span = self.samples[-1][0]-self.samples[0][0]
        position_mean = None
        if span > 0 and self.previous_position is not None and self.samples[-1][4] == self.samples[0][4]:
            position_mean = (self.samples[-1][3]-self.samples[0][3])*60/(8192*span)
        result = dict(time_s=self.now, target_rpm=self.target, samples=n,
                      span_s=span, mean_rpm=mean, min_rpm=lo, max_rpm=hi,
                      bias_rpm=bias, bias_pct=100*bias/self.target,
                      peak_to_peak_rpm=hi-lo, peak_to_peak_pct=100*(hi-lo)/self.target,
                      std_rpm=math.sqrt(variance), rmse_rpm=math.sqrt(variance+bias*bias),
                      max_abs_error_rpm=peak, max_abs_error_pct=100*peak/self.target,
                      position_mean_rpm=position_mean,
                      position_bias_rpm=position_mean-self.target if position_mean is not None else None,
                      position_bias_pct=100*(position_mean-self.target)/self.target if position_mean is not None else None,
                      feedback_position_difference_rpm=mean-position_mean if position_mean is not None else None)
        self.last_valid = result
        return result

    def snapshot(self):
        current = self.values()
        return dict(status=self.status, window_s=self.window_s, settle_s=self.settle_s,
                    settling_remaining_s=max(0,self.settle_s-(self.now-self.since)) if self.since is not None else None,
                    current=current, last_valid=self.last_valid)
