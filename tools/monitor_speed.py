"""FPGA FOC V6 serial control console; V2/V3/V5 offline compatibility.

Default live use opens the curve window. --no-plot selects text/CSV only.
"""
import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from speed_capture import CaptureSession, Decoder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM11')
    parser.add_argument('--read-only', action='store_true', help='disable all serial commands and heartbeats')
    parser.add_argument('--baud', type=int, default=1500000)
    parser.add_argument('--refresh-hz', type=int, default=60)
    parser.add_argument('--stats-window', type=float, default=5)
    parser.add_argument('--settle-seconds', type=float, default=1)
    parser.add_argument('--seconds', type=float, default=0, help='0 = until window close/Ctrl+C')
    parser.add_argument('--output', help='output prefix; default uses a unique timestamp')
    parser.add_argument('--input', type=Path, help='decode a saved V2/V3/V5/V6 binary file')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--plot', dest='plot', action='store_true')
    mode.add_argument('--no-plot', dest='plot', action='store_false')
    parser.set_defaults(plot=None)
    parser.add_argument('--window', type=float, default=10, help='rolling window, 5..60 seconds')
    parser.add_argument('--snapshot', type=Path, help='save final window image on exit')
    args = parser.parse_args()
    if not 5 <= args.window <= 60:
        parser.error('--window must be between 5 and 60 seconds')
    if args.seconds < 0:
        parser.error('--seconds must be nonnegative')
    if not 1 <= args.refresh_hz <= 144 or not 1 <= args.stats_window <= 30 or args.settle_seconds < 0:
        parser.error('refresh 1..144Hz, statistics window 1..30s, settle >=0 required')
    plot = args.plot if args.plot is not None else not bool(args.input)
    output = args.output or str(Path(__file__).resolve().parent/'monitor_output'/('speed_'+datetime.now().strftime('%Y%m%d_%H%M%S')))
    if args.input and args.input.resolve() == Path(output).with_suffix('.bin').resolve():
        parser.error('Input binary and output binary must be different files')
    session = CaptureSession(args.port, output, source=args.input, seconds=args.seconds,baud=args.baud,
                             stats_window=args.stats_window,settle_seconds=args.settle_seconds,
                             control=bool(plot and not args.input and not args.read_only))
    viewer = None
    gui_result = None
    if plot:
        from speed_live_plot import LivePlot
        viewer = LivePlot(session, window=args.window, snapshot=args.snapshot,refresh_hz=args.refresh_hz)
    print(f'FOC capture: {args.port}, {args.baud}/8N1, V3/V5/V6=1000Hz; output={output}', flush=True)
    print('Control GUI closes with STOP and drive-off readback; --no-plot/--read-only never sends commands.', flush=True)
    session.start()
    try:
        if viewer:
            gui_result = viewer.run()
        else:
            last_print = 0
            while not session.done.wait(.05):
                snap = session.snapshot(False)
                r = snap['last_frame']
                now = time.monotonic()
                if r and now-last_print >= .5:
                    print(f"t={r['time_s']:8.3f}s start={r['start']} state={r['state']}/{r['speed_state']} "
                          f"set={r['target_rpm']:4d} ramp={r['ramp_rpm']:4d} "
                          f"speed={r['speed_rpm']:8.2f}rpm Id={r['id_ma']:5d} "
                          f"Iq={r['iq_ma']:5d}/{r['iq_ref_ma']:5d}mA fault={r['fault']:02x}", flush=True)
                    if r['position_mode']:
                        print(f"  angle={r['position_degrees']:.3f}deg target={r['position_target_degrees']:.3f}deg "
                              f"error={r['position_error_counts']}count/{r['position_error_deg']:+.4f}deg "
                              f"ready={r['position_ready']} settled={r['position_settled']}",flush=True)
                    last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        session.stop()
    summary = session.summary()
    if gui_result:
        summary['live_plot'] = gui_result
        session.base.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps(summary, indent=2))
    return 1 if summary['error'] or (gui_result and gui_result.get('error')) else 0


if __name__ == '__main__':
    raise SystemExit(main())
