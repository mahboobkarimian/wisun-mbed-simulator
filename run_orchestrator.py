"""
Author: Eric Tran <eric.tran@heig-vd.ch>
"""

import subprocess
import os
import shutil
import glob
from pathlib import Path
import signal
import time
import sys
import argparse

os.environ["DBUS_SYSTEM_BUS_ADDRESS"] = "unix:path=/var/run/dbus/system_bus_socket"

all_processes = [] # Track all running processes

def cleanup():
    print("Perform cleanup")
    while all_processes:
        proc = all_processes.pop()
        print(f"Clean PID: {proc.pid}")
        proc.terminate()
        proc.wait()
    # wssimserver respawn with a new PID, can't track process
    process_wssim = 'wssimserver'
    process_wsbrd = 'wsbrd'
    process_wshwsim = 'wshwsim'
    process_wsnode = 'wsnode'

    # Ensure all wssimserver processes are cleaned
    try:
        all_pids = subprocess.check_output(['pgrep', process_wssim]).decode().strip().split('\n')
        for pid in all_pids:
            print(f"Find new wssimserver with pid : {pid}. Perform cleanup")
            os.kill(int(pid), signal.SIGTERM)
    except subprocess.CalledProcessError:
        print("No wssimserver running. No cleanup required")

    # Ensure all wshwsim processes are cleaned
    try:
        all_pids = subprocess.check_output(['pgrep', process_wshwsim]).decode().strip().split('\n')
        for pid in all_pids:
            print(f"Find new wshwsim with pid : {pid}. Perform cleanup")
            os.kill(int(pid), signal.SIGTERM)
    except subprocess.CalledProcessError:
        print("No wshwsim running. No cleanup required")

    # Ensure all wsbrd processes are cleaned
    try:
        all_pids = subprocess.check_output(['pgrep', process_wsbrd]).decode().strip().split('\n')
        for pid in all_pids:
            print(f"Find new wsbrd with pid : {pid}. Perform cleanup")
            os.kill(int(pid), signal.SIGTERM)
    except subprocess.CalledProcessError:
        print("No wsbrd running. No cleanup required")

    # Ensure all wsnode processes are cleaned
    try:
        all_pids = subprocess.check_output(['pgrep', process_wsnode]).decode().strip().split('\n')
        for pid in all_pids:
            print(f"Find new wsnode with pid : {pid}. Perform cleanup")
            os.kill(int(pid), signal.SIGTERM)
    except subprocess.CalledProcessError:
        print("No wsnode running. No cleanup required")

def handle_exit(signum, frame):
    print(f"Detect signal: {signum}")
    cleanup()
    sys.exit(0)

DIR = Path().resolve()
TPG_STR = "-g 0,1:0.71 -g 0,2:0.71 -g 0,3:0.65 -g 2,4:0.85 -g 2,5:0.7 -g 4,6:0.99 -g 0,7:0.6 -g 1,7:0.71 -g 4,7:0.68 -g 0,8:0.9 -g 3,8:0.8 -g 5,8:0.6 -g 7,8:0.95 -g 3,9:0.7 -g 4,9:0.8 -g 7,9:0.75 -g 8,9:0.8 -g 7,10:0.77 -g 8,11:0.86 -g 9,12:0.8 -g 9,13:0.67 -g 10,14:0.75"
TPG = ["0,1:0.71","0,2:0.71","0,3:0.65","2,4:0.85","2,5:0.7","4,6:0.99","0,7:0.6","1,7:0.71","4,7:0.68","0,8:0.9","3,8:0.8","5,8:0.6","7,8:0.95","3,9:0.7","4,9:0.8","7,9:0.75","8,9:0.8","7,10:0.77","8,11:0.86","9,12:0.8","9,13:0.67","10,14:0.75"]
TMP_DIR = Path("/tmp")
WSBRD_DIR = TMP_DIR / "wsbrd"
SIM_SOCKET_FILE = TMP_DIR / "sim_socket"
PAE_FILE = TMP_DIR / "*_pae_*"

def run_cmd(cmd, verbose:bool = False):
    process = subprocess.Popen(
        cmd,
        stdout=None if verbose else subprocess.DEVNULL, #None if verbose else subprocess.DEVNULL,# None to directly redirect on terminal stdout and not subprocess stdout
        stderr=subprocess.STDOUT,
        text=True,
        shell=True
    )
    all_processes.append(process)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-g', action='append', type=str, help="Link between two nodes with link quality. (e.g: -g 0,1:0.71)") 
    parser.add_argument('-v', '--verbose', action=argparse.BooleanOptionalAction)
    args = parser.parse_args()
    # Default value with argparse is not the expected behavior with action append. Append value to default list instead of replacing it with provided value. 
    # Need to use workaround (https://bugs.python.org/issue16399)
    if args.g is None:
        args.g = TPG
    tpg_params = " ".join(f"-g{link}" for link in args.g)
    all_nodes = set()
    for edge in args.g:
        nodes = edge[:edge.index(":")].split(',')
        all_nodes.update(nodes)
    # Delete wsbrd
    if WSBRD_DIR.exists(): 
        shutil.rmtree(WSBRD_DIR)
        print(f"Deleted folder {WSBRD_DIR.name}")
    else:
        print(f"Folder {WSBRD_DIR.name} does not exist")
    WSBRD_DIR.mkdir()
    
    # Delete sim_socket file
    if SIM_SOCKET_FILE.exists():
        SIM_SOCKET_FILE.unlink()
        print(f"Delete file {SIM_SOCKET_FILE.name}")
    else:
        print(f"File {SIM_SOCKET_FILE.name} does not exist")

    # Delete pae files
    for file_path in glob.glob(str(PAE_FILE)):
        os.remove(file_path)
        print(f"Deleted file {file_path}")

    # Compile latest changes
    result_run = subprocess.run("ninja")
    result_run.check_returncode() # Fail run if return code is not 0

    # Write sudoers rule file
    subprocess.run([f"{DIR}/config_sudoers.sh"]) # Dirty trick to perform privileged operation. Must be executed once

    # Write wsbrd dbus config
    subprocess.run([f"{DIR}/config_dbus.sh"]) # Dirty trick to perform privileged operation. Must be executed once

    # Map signal
    signal.signal(signal.SIGINT, handle_exit) # Handle Ctrl+C
    signal.signal(signal.SIGTERM, handle_exit) # Handle kill
    futures = set()
    # Parse topology parameters
    try:
        # Run sim server
        wssimserver_cmd = f"{DIR}/wssimserver {tpg_params} {str(SIM_SOCKET_FILE)} --dump"
        run_cmd(wssimserver_cmd, verbose=True)
        time.sleep(2.)

        n_nodes = len(all_nodes)
        # Run MAC
        print("Create MAC instances")
        for i in range(n_nodes):
            hex_id = format(i, '02x')
            mac_cmd = f"{DIR}/wshwsim -m 01:02:03:04:05:06:00:{hex_id} /tmp/uart{i} {str(SIM_SOCKET_FILE)}"
            run_cmd(mac_cmd)
            time.sleep(.1)
        # Run Nodes
        print("Create node instances")
        for i in range(1, n_nodes):
            node_cmd = f"{DIR}/wsnode -F {DIR}/examples/wsnode.conf -u $(readlink \"/tmp/uart{i}\") -o storage_prefix=/tmp/n{i}_"
            run_cmd(node_cmd)
    
        # Run BR node 
        print("Create BR node instance")
        br_cmd = f'sudo {DIR}/wsbrd -F {DIR}/examples/wsbrd.conf -u $(readlink /tmp/uart0)'
        run_cmd(br_cmd)

        i = 0
        while True:
            if args.verbose:
                i =  (i + 1) % 4 
                print(f"Run in progress{'.' * i}   ", end='\r')
            time.sleep(1.)  

    except Exception as e:
        print(f"Error: {e}")
    finally:
        cleanup()