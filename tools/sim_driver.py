# Runs a line_follower.py (given by path) with its default values, presses Start,
# prints every change of the robot's action, then Ctrl+C.
#   python3 sim_driver.py T0 MODULE_PATH DURATION
#   SET="attr=val,attr=val" changes UI boxes before Start (e.g. SET="h_min=95,h_max=125")
import importlib.util
import os
import signal
import sys
import time

T0, MOD, DURATION = float(sys.argv[1]), sys.argv[2], float(sys.argv[3])
spec = importlib.util.spec_from_file_location('lf_under_test', MOD)
lf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lf)
orig_run = lf.ControlPanel.run


def run_with_script(self):
    def at(t, fn):
        self.root.after(max(0, int((T0 + t - time.time()) * 1000)), fn)

    node, last = self.node, [None]
    orig_drive = node.drive

    def drive(lin, ang, action):
        if action != last[0]:
            print(f'[{time.time() - T0:5.1f}s] {action}  (v={lin:+.2f}, w={ang:+.2f})', flush=True)
            last[0] = action
        orig_drive(lin, ang, action)

    orig_stop = node.stop_robot

    def stop_robot():
        if last[0] != 'หยุด':
            print(f'[{time.time() - T0:5.1f}s] หยุด', flush=True)
            last[0] = 'หยุด'
        orig_stop()

    node.drive, node.stop_robot = drive, stop_robot
    for kv in filter(None, os.environ.get('SET', '').split(',')):
        k, v = kv.split('=')
        next(b for b, attr, *_ in self.boxes if attr == k).set(v)
    at(1.0, self.start_button.invoke)
    at(DURATION, lambda: os.kill(os.getpid(), signal.SIGINT))
    orig_run(self)


lf.ControlPanel.run = run_with_script
lf.main()
