#!/usr/bin/env python
import os
import subprocess
import rospy

if __name__ == "__main__":
    # shell tuc la run bang terminal

    # subprocess is faster and newer than os.popen
    # shell tuc la run bang terminalist
    # nodes = os.popen('rosnode list').readlines()   SLOWER
    nodes = subprocess.check_output("rosnode list", shell=True)
    nodes = nodes.split("\n")
    cmd_vel_muxs = [True for node in nodes if "cmd_vel_mux" in node]
    is_mux_tool_initialed = any(cmd_vel_mux == True for cmd_vel_mux in cmd_vel_muxs)

    rospy.loginfo("mux_seleted is initialied in switch_manual_tele.py")

    # print("mux_seleted is initialied:", is_mux_tool_initialed)
    if not is_mux_tool_initialed:
        subprocess.Popen(
            "rosrun  topic_tools mux cmd_vel auto_cmd_vel manual_cmd_vel mux:=mux_cmd_vel",
            shell=True,
        )
    # chay xong la thoat luon terminal
    subprocess.Popen(
        "rosrun topic_tools mux_select mux_cmd_vel manual_cmd_vel", shell=True
    )

    # TODO: khi thoat file .py nay thi can kill node cmd_vel_mux do process1 tao nen. Co the sd kill process hoac kill node. Kill process thi tot hon
