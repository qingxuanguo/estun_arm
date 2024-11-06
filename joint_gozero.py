import os
import sys
import time
import ctypes
import pysoem
import threading
import data_store_pdo as dsp

import tty, termios
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

ifname = 'enp2s0'
pd_thread_stop_event = threading.Event()
ch_thread_stop_event = threading.Event()
global actual_wkc
actual_wkc = 0
master = pysoem.Master()
master.in_op = False
ec_TIMEOUT = 500 # us
cycle_time = 500 # us

joint_pdo = dsp.Pdo_stream(0)

class PDO_Error(Exception):
    def __init__(self, message):
        super().__init__(message)
        self.message = message

def COBOT_setup_func(slave_pos):
    device = master.slaves[slave_pos]

    device.sdo_write(0x1C12, 0, bytes(ctypes.c_uint8(0)))
    device.sdo_write(0x1C13, 0, bytes(ctypes.c_uint8(0)))

    device.sdo_write(0x1A00, 0, bytes(ctypes.c_uint8(0)))
    device.sdo_write(0x1A00, 1, bytes(ctypes.c_uint32(0x60410010)))
    device.sdo_write(0x1A00, 2, bytes(ctypes.c_uint32(0x60610008)))
    device.sdo_write(0x1A00, 3, bytes(ctypes.c_uint32(0x60640020)))
    device.sdo_write(0x1A00, 0, bytes(ctypes.c_uint8(3)))

    device.sdo_write(0x1A01, 0, bytes(ctypes.c_uint8(0)))
    device.sdo_write(0x1A01, 1, bytes(ctypes.c_uint32(0x60410010)))
    device.sdo_write(0x1A01, 2, bytes(ctypes.c_uint32(0x60610008)))
    device.sdo_write(0x1A01, 3, bytes(ctypes.c_uint32(0x60640020)))
    device.sdo_write(0x1A01, 4, bytes(ctypes.c_uint32(0x606c0020)))
    device.sdo_write(0x1A01, 0, bytes(ctypes.c_uint8(4)))

    device.sdo_write(0x1A02, 0, bytes(ctypes.c_uint8(0)))
    device.sdo_write(0x1A02, 1, bytes(ctypes.c_uint32(0x60410010)))
    device.sdo_write(0x1A02, 2, bytes(ctypes.c_uint32(0x60610008)))
    device.sdo_write(0x1A02, 3, bytes(ctypes.c_uint32(0x603F0010)))
    device.sdo_write(0x1A02, 4, bytes(ctypes.c_uint32(0x60640020)))
    device.sdo_write(0x1A02, 5, bytes(ctypes.c_uint32(0x606C0020)))
    device.sdo_write(0x1A02, 6, bytes(ctypes.c_uint32(0x60770010)))
    device.sdo_write(0x1A02, 7, bytes(ctypes.c_uint32(0x300A0010)))
    device.sdo_write(0x1A02, 8, bytes(ctypes.c_uint32(0x30210110)))
    device.sdo_write(0x1A02, 9, bytes(ctypes.c_uint32(0x30050120)))
    device.sdo_write(0x1A02, 10, bytes(ctypes.c_uint32(0x30060120)))
    device.sdo_write(0x1A02, 0, bytes(ctypes.c_uint8(10)))

    device.sdo_write(0x1A03, 0, bytes(ctypes.c_uint8(0)))
    device.sdo_write(0x1A03, 1, bytes(ctypes.c_uint32(0x60410010)))
    device.sdo_write(0x1A03, 0, bytes(ctypes.c_uint8(1)))

    device.sdo_write(0x1600, 0, bytes(ctypes.c_uint8(0)))
    device.sdo_write(0x1600, 1, bytes(ctypes.c_uint32(0x60400010)))
    device.sdo_write(0x1600, 2, bytes(ctypes.c_uint32(0x60600008)))
    device.sdo_write(0x1600, 3, bytes(ctypes.c_uint32(0x607A0020)))
    device.sdo_write(0x1600, 0, bytes(ctypes.c_uint8(3)))

    device.sdo_write(0x1601, 0, bytes(ctypes.c_uint8(0)))
    device.sdo_write(0x1601, 1, bytes(ctypes.c_uint32(0x60400010)))
    device.sdo_write(0x1601, 2, bytes(ctypes.c_uint32(0x60600008)))
    device.sdo_write(0x1601, 3, bytes(ctypes.c_uint32(0x607A0020)))
    device.sdo_write(0x1601, 4, bytes(ctypes.c_uint32(0x60FF0020)))
    device.sdo_write(0x1601, 0, bytes(ctypes.c_uint8(4)))

    device.sdo_write(0x1602, 0, bytes(ctypes.c_uint8(0)))
    device.sdo_write(0x1602, 1, bytes(ctypes.c_uint32(0x60400010)))
    device.sdo_write(0x1602, 2, bytes(ctypes.c_uint32(0x60600008)))
    device.sdo_write(0x1602, 3, bytes(ctypes.c_uint32(0x607A0020)))
    device.sdo_write(0x1602, 4, bytes(ctypes.c_uint32(0x60FF0020)))
    device.sdo_write(0x1602, 5, bytes(ctypes.c_uint32(0x60710010)))
    device.sdo_write(0x1602, 6, bytes(ctypes.c_uint32(0x607E0008)))
    device.sdo_write(0x1602, 7, bytes(ctypes.c_uint32(0x60B10020)))
    device.sdo_write(0x1602, 8, bytes(ctypes.c_uint32(0x60B20010)))
    device.sdo_write(0x1602, 9, bytes(ctypes.c_uint32(0x30240020)))
    device.sdo_write(0x1602, 10, bytes(ctypes.c_uint32(0x30250020)))
    device.sdo_write(0x1602, 0, bytes(ctypes.c_uint8(10)))

    device.sdo_write(0x1603, 0, bytes(ctypes.c_uint8(0)))
    device.sdo_write(0x1603, 1, bytes(ctypes.c_uint32(0x60400010)))
    device.sdo_write(0x1603, 0, bytes(ctypes.c_uint8(1)))

    device.sdo_write(0x1C12, 1, bytes(ctypes.c_uint16(0x1602)))
    device.sdo_write(0x1C12, 0, bytes(ctypes.c_uint8(1)))

    device.sdo_write(0x1C13, 1, bytes(ctypes.c_uint16(0x1A02)))
    device.sdo_write(0x1C13, 0, bytes(ctypes.c_uint8(1)))

    # device.sdo_write(0x6060, 0, bytes(ctypes.c_uint8(8)))

def _processdata_thread():
    global actual_wkc
    while not pd_thread_stop_event.is_set():
        t0 = time.time_ns()
        master.send_processdata()
        actual_wkc = master.receive_processdata(timeout = ec_TIMEOUT)
        if not actual_wkc == master.expected_wkc:
            print("incorrect wkc", actual_wkc, master.expected_wkc)
        tspan_ns = (time.time_ns() - t0)
        if (tspan_ns < cycle_time*1000):
            time.sleep( (cycle_time*1_000 - tspan_ns)/1_000_000_000)

def _pdo_update_loop():
    master.in_op = True
    is_in_init = 1
    index_init = 0
    index = 0

    joint = master.slaves[0]

    try:
        while 1:
            t0 = time.time_ns()
            joint_pdo.decode_Tx_PDO(joint.input)
            
            if index_init < 100:
                joint_pdo.Controlword = 0
            elif index_init < 200:
                joint_pdo.Controlword = 6
            elif index_init < 300:
                joint_pdo.Controlword = 7
            elif index_init < 400:
                joint_pdo.Controlword = 15
                joint_pdo.ModeOperation = 8
            elif index_init == 400:
                is_in_init = 0
            if index > 20000:
                joint_pdo.Controlword = 0

            p0 = joint_pdo.PositionActualValue
            p1 = -5000
            if (index<=10000):
                joint_pdo.TargetPosition = int((p1-p0)/(10000)*index+p0)
            else:
                joint_pdo.TargetPosition = p1
            
            if joint_pdo.ErrorCode == 16392:
                joint_pdo.Controlword = 128
                index = 0
                print("WRITE error out--------------------------")

            print('position',p0,'target position', joint_pdo.TargetPosition)
            joint.output = joint_pdo.encode_Rx_PDO()

            if is_in_init:
                index_init += 1
            else:
                index += 1

            tspan_ns = (time.time_ns() - t0)
            if (tspan_ns < cycle_time*1000):
                time.sleep( (cycle_time*1_000 - tspan_ns)/1_000_000_000)

    except KeyboardInterrupt:
        joint_pdo.TargetPosition = joint_pdo.PositionActualValue
        joint_pdo.TargetVelocity = 0
        joint_pdo.TargetTorque = 0
        joint.output = joint_pdo.encode_Rx_PDO()
        time.sleep(2*cycle_time/1_000_000_000)
        joint_pdo.Controlword = 0

        joint.output = joint_pdo.encode_Rx_PDO()
        print("stopped")

def _check_slave(slave, slave_pos):
    if slave.state == (pysoem.SAFEOP_STATE + pysoem.STATE_ERROR):
        print(f"ERROR : slave {slave_pos} is in SAFE_OP + ERROR, attempting ack.")
        slave.state = pysoem.SAFEOP_STATE + pysoem.STATE_ACK
        slave.write_state()
    elif slave.state == pysoem.SAFEOP_STATE:
        print(f"WARNING : slave {slave_pos} is in SAFE_OP, try change to OPERATIONAL.")
        slave.state = pysoem.OP_STATE
        slave.write_state()
    elif slave.state > pysoem.NONE_STATE:
        if slave.reconfig():
            slave.is_lost = False
            print(f"MESSAGE : slave {slave_pos} reconfigured")
    elif not slave.is_lost:
        slave.state_check(pysoem.OP_STATE)
        if slave.state == pysoem.NONE_STATE:
            slave.is_lost = True
            print(f"ERROR : slave {slave_pos} lost")
    if slave.is_lost:
        if slave.state == pysoem.NONE_STATE:
            if slave.recover():
                slave.is_lost = False
                print(f"MESSAGE : slave {slave_pos} recovered")
        else:
            slave.is_lost = False
            print(f"MESSAGE : slave {slave_pos} found")

def _check_thread():
    global actual_wkc
    while not ch_thread_stop_event.is_set():
        if master.in_op and ((actual_wkc < master.expected_wkc) ):
            master.do_check_state = False
            master.read_state()
            for i, slave in enumerate(master.slaves):
                if slave.state != pysoem.OP_STATE:
                    master.do_check_state = True
                    _check_slave(slave, i)
            if not master.do_check_state:
                print("OK : all slaves resumed OPERATIONAL.")
        time.sleep(0.001)


master.open(ifname)
if not master.config_init() > 0:
    master.close()
    raise PDO_Error("no slave found")
for i, slave in enumerate(master.slaves):
    slave.config_func = COBOT_setup_func(i)
    slave.is_lost = False

master.config_map()

if master.state_check(pysoem.SAFEOP_STATE, timeout = 50_000) != pysoem.SAFEOP_STATE:
    master.close()
    raise PDO_Error("not all slaves reached SAFEOP state")

slave.dc_sync(act = True, sync0_cycle_time = cycle_time*1_000)

master.state = pysoem.OP_STATE

check_thread = threading.Thread(target = _check_thread)
check_thread.start()
proc_thread = threading.Thread(target = _processdata_thread)
proc_thread.start()

master.send_processdata()
master.receive_processdata(timeout = ec_TIMEOUT)

master.write_state()

all_slaves_reached_op_state = False
for i in range(40):
    master.state_check(pysoem.OP_STATE, timeout = 50_000)
    if master.state == pysoem.OP_STATE:
        all_slaves_reached_op_state = True
        break

joint = master.slaves[0]
joint_pdo.decode_Tx_PDO(joint.input)
print('joint position', joint_pdo.PositionActualValue)

print("Press any key to continue! (or press ESC to quit!)")
if getch() != chr(0x1b):
    try: 
        if all_slaves_reached_op_state:
            _pdo_update_loop()
    except KeyboardInterrupt: 


        pd_thread_stop_event.set()
        ch_thread_stop_event.set()
        proc_thread.join()
        check_thread.join()
        master.state = pysoem.INIT_STATE
        master.write_state()
        master.close

        if not all_slaves_reached_op_state:
            raise PDO_Error("not all slaves in OP")