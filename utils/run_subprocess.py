"""
- Continue update transformation and observe on rviz
Tips: we can subcribe x,y,z,rx,ry,rz from another rostopic or from mobile apps
Run 3 terminals:
roscore
rviz.  Remember to show tf
python2 run_subprocess.py
"""

import subprocess
import os
import numpy as np
import signal

x, y, z, rx, ry, rz = 0, 1, 0, 0, 0, 0
tf_process = None

# Run with shell = True, we need to run command in string
def run_subprocess_sol1(x, y, z, rx, ry, rz):
    global tf_process
    cmd = "rosrun tf static_transform_publisher {} {} {} {} {} {} /map /uwb 100".format(
        x, y, z, rx, ry, rz
    )
    tf_process = subprocess.Popen([cmd], shell=True)


# Run subprocess with list, we dont need to config shell
def run_subprocess_sol2(x, y, z, rx, ry, rz):
    global tf_process
    cmd = [
        "rosrun",
        "tf",
        "static_transform_publisher",
        "{}".format(x),
        "{}".format(y),
        "{}".format(z),
        "{}".format(rx),
        "{}".format(ry),
        "{}".format(rz),
        "/map",
        "/uwb",
        "100",
    ]
    tf_process = subprocess.Popen(cmd)


print("Subprocess status:", tf_process)
run_subprocess_sol2(x, y, z, rx, ry, rz)
while True:
    a = raw_input("Nhap input: ")
    print(a)
    if a == "c":
        x, y, z, rx, ry, rz = x + 1, y + 0, z + 0, rx + 0, ry + 0, rz + 0
        # check process is killed
        poll = tf_process.poll()
        if poll is None:
            # killprocess-solution 1: terminate POpen process
            # tf_process.terminate()

            # killprocess-solution2: similar with ctrl + C
            os.kill(tf_process.pid,signal.SIGINT)

            # killprocess-solution3: kill process using pid
            # os.system("kill -9 {pid}".format(pid=tf_process.pid))
        run_subprocess_sol2(x, y, z, rx, ry, rz)
    else:
        poll = tf_process.poll()
        if poll is None:
            os.kill(tf_process.pid, signal.SIGINT)
        break
