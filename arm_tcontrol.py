import os
import sys
import math
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

control_cycle_sec = 0.0006
lpf_cutoff_frequency = 10
lpf_cutoff_frequency_torque_ctrl = 50

Joint1_pdo = dsp.Pdo_stream(0)
Joint1_Max_Torque_Output = 4000 # 0.1Nm
Joint1_velocity_lpf      = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency)
Joint1_torque_sensor_lpf = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency)
Joint1_torque_ctrl       = dsp.PIDcontroller(control_cycle_sec, Kp = 0.025, Ki = 0.003, Kd = 0.004)
Joint1_torque_ctrl_lpf   = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency_torque_ctrl)

Joint2_pdo = dsp.Pdo_stream(1)
Joint2_Max_Torque_Output = 4000 # 0.1Nm
Joint2_velocity_lpf      = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency)
Joint2_torque_sensor_lpf = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency)
Joint2_torque_ctrl       = dsp.PIDcontroller(control_cycle_sec, Kp = 0.025, Ki = 0.003, Kd = 0.003)
Joint2_torque_ctrl_lpf   = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency_torque_ctrl)

Joint3_pdo = dsp.Pdo_stream(2)
Joint3_Max_Torque_Output = 2000 # 0.1Nm
Joint3_velocity_lpf      = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency)
Joint3_torque_sensor_lpf = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency)
Joint3_torque_ctrl       = dsp.PIDcontroller(control_cycle_sec, Kp = 0.018, Ki = 0.0005, Kd = 0.0008)
Joint3_torque_ctrl_lpf   = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency_torque_ctrl)

Joint4_pdo = dsp.Pdo_stream(3)
Joint4_Max_Torque_Output = 600 # 0.1Nm
Joint4_velocity_lpf      = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency)
Joint4_torque_sensor_lpf = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency)
Joint4_torque_ctrl       = dsp.PIDcontroller(control_cycle_sec, Kp = 0.016, Ki = 0.0006, Kd = 0.0004)
Joint4_torque_ctrl_lpf   = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency_torque_ctrl)

Joint5_pdo = dsp.Pdo_stream(4)
Joint5_Max_Torque_Output = 600 # 0.1Nm
Joint5_velocity_lpf      = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency)
Joint5_torque_sensor_lpf = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency)
Joint5_torque_ctrl       = dsp.PIDcontroller(control_cycle_sec, Kp = 0.016, Ki = 0.0006, Kd = 0.0004)
Joint5_torque_ctrl_lpf   = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency_torque_ctrl)

Joint6_pdo = dsp.Pdo_stream(5)
Joint6_Max_Torque_Output = 600 # 0.1Nm
Joint6_velocity_lpf      = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency)
Joint6_torque_sensor_lpf = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency)
Joint6_torque_ctrl       = dsp.PIDcontroller(control_cycle_sec, Kp = 0.016, Ki = 0.0006, Kd = 0.0004)
Joint6_torque_ctrl_lpf   = dsp.LowPassFilter(control_cycle_sec, lpf_cutoff_frequency_torque_ctrl)


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

def sign(x):
    return -1 if x < 0 else (1 if x > 0 else 0)

def _pdo_update_loop():
    master.in_op = True
    is_in_init = 1
    index_init = 0
    index = 0
    task_span = 80000 ###
    ###  TARGET POS HERE  ###
    target = [0,0,0,0,0,-10000] # linear end target
    Joint_Position = [0,0,0,0,0,0]
    Joint_Position_target = [0,0,0,0,0,0]
    Joint_Position_error = [0,0,0,0,0,0]
    # Joint_Velovity_filtered = [0,0,0,0,0,0]
    Joint_Torque_sensor_filtered = [0,0,0,0,0,0]
    Joint_Torque_ctrl_output = [0,0,0,0,0,0]
    Joint_Torque_ctrl_output_filtered = [0,0,0,0,0,0]
    Joint_Torque_ctrl_target = [0,0,0,0,0,0]
    
    Joint1 = master.slaves[0]
    Joint2 = master.slaves[1]
    Joint3 = master.slaves[2]
    Joint4 = master.slaves[3]
    Joint5 = master.slaves[4]
    Joint6 = master.slaves[5]

    try:
        while 1:
            t0 = time.time_ns()
            Joint1_pdo.decode_Tx_PDO(Joint1.input)
            Joint2_pdo.decode_Tx_PDO(Joint2.input)
            Joint3_pdo.decode_Tx_PDO(Joint3.input)
            Joint4_pdo.decode_Tx_PDO(Joint4.input)
            Joint5_pdo.decode_Tx_PDO(Joint5.input)
            Joint6_pdo.decode_Tx_PDO(Joint6.input)
            
            Joint_Position[0] = Joint1_pdo.PositionActualValue
            Joint_Position[1] = Joint2_pdo.PositionActualValue
            Joint_Position[2] = Joint3_pdo.PositionActualValue
            Joint_Position[3] = Joint4_pdo.PositionActualValue
            Joint_Position[4] = Joint5_pdo.PositionActualValue
            Joint_Position[5] = Joint6_pdo.PositionActualValue
            
            Joint_Torque_sensor_filtered[0] = Joint1_torque_sensor_lpf.getFilteredOutput(Joint1_pdo.TorqueSensorValue) 
            Joint_Torque_sensor_filtered[1] = Joint2_torque_sensor_lpf.getFilteredOutput(Joint2_pdo.TorqueSensorValue)
            Joint_Torque_sensor_filtered[2] = Joint3_torque_sensor_lpf.getFilteredOutput(Joint3_pdo.TorqueSensorValue) 
            Joint_Torque_sensor_filtered[3] = Joint4_torque_sensor_lpf.getFilteredOutput(Joint4_pdo.TorqueSensorValue)
            Joint_Torque_sensor_filtered[4] = Joint5_torque_sensor_lpf.getFilteredOutput(Joint5_pdo.TorqueSensorValue) 
            Joint_Torque_sensor_filtered[5] = Joint6_torque_sensor_lpf.getFilteredOutput(Joint6_pdo.TorqueSensorValue)
            
            
            if index_init < 100:
                Joint1_pdo.Controlword = 0
                Joint2_pdo.Controlword = 0
                Joint3_pdo.Controlword = 0
                Joint4_pdo.Controlword = 0
                Joint5_pdo.Controlword = 0
                Joint6_pdo.Controlword = 0
            elif index_init < 200:
                Joint1_pdo.Controlword = 6
                Joint2_pdo.Controlword = 6
                Joint3_pdo.Controlword = 6
                Joint4_pdo.Controlword = 6
                Joint5_pdo.Controlword = 6
                Joint6_pdo.Controlword = 6
            elif index_init < 300:
                Joint1_pdo.Controlword = 7
                Joint2_pdo.Controlword = 7
                Joint3_pdo.Controlword = 7
                Joint4_pdo.Controlword = 7
                Joint5_pdo.Controlword = 7
                Joint6_pdo.Controlword = 7
            elif index_init < 400:
                Joint1_pdo.Controlword = 15
                Joint2_pdo.Controlword = 15
                Joint3_pdo.Controlword = 15
                Joint4_pdo.Controlword = 15
                Joint5_pdo.Controlword = 15
                Joint6_pdo.Controlword = 15
                
                Joint1_pdo.ModeOperation = 10
                Joint2_pdo.ModeOperation = 10
                Joint3_pdo.ModeOperation = 10
                Joint4_pdo.ModeOperation = 10
                Joint5_pdo.ModeOperation = 10
                Joint6_pdo.ModeOperation = 10
            elif index_init == 400:
                is_in_init = 0
            if index > (task_span+10000):
                Joint1_pdo.Controlword = 0
                Joint2_pdo.Controlword = 0
                Joint3_pdo.Controlword = 0
                Joint4_pdo.Controlword = 0
                Joint5_pdo.Controlword = 0
                Joint6_pdo.Controlword = 0

            ###  LINEAR TRAJECTORY  ###
            if (index<=task_span):
                ###  TRAJECTORY GENERATION   ###
                
                # Joint_Position_target[0] = int((target[0]-
                #                                  Joint_Position[0])/(10000)*index+Joint_Position[0])  
                Joint_Position_target[0] = int(20000*math.sin((2*math.pi)/(task_span/6)*index))
                # Joint_Position_target[1] = int((target[1]-
                #                                  Joint_Position[1])/(10000)*index+Joint_Position[1])
                Joint_Position_target[1] = int(20000*math.sin((2*math.pi)/(task_span/6)*index))
                Joint_Position_target[2] = int((target[2]-
                                                 Joint_Position[2])/(10000)*index+Joint_Position[2])
                Joint_Position_target[3] = int((target[3]-
                                                 Joint_Position[3])/(10000)*index+Joint_Position[3])
                Joint_Position_target[4] = int((target[4]-
                                                 Joint_Position[4])/(10000)*index+Joint_Position[4])
                Joint_Position_target[5] = int((target[5]-
                                                 Joint_Position[5])/(10000)*index+Joint_Position[5])
                ###                          ###
                
                Joint1_pdo.TargetPosition = Joint_Position_target[0]
                Joint2_pdo.TargetPosition = Joint_Position_target[1]
                Joint3_pdo.TargetPosition = Joint_Position_target[2]
                Joint4_pdo.TargetPosition = Joint_Position_target[3]
                Joint5_pdo.TargetPosition = Joint_Position_target[4]
                Joint6_pdo.TargetPosition = Joint_Position_target[5]
                
                Joint_Position_error[0] = Joint_Position_target[0] - Joint_Position[0]
                Joint_Position_error[1] = Joint_Position_target[1] - Joint_Position[1]
                Joint_Position_error[2] = Joint_Position_target[2] - Joint_Position[2]
                Joint_Position_error[3] = Joint_Position_target[3] - Joint_Position[3]
                Joint_Position_error[4] = Joint_Position_target[4] - Joint_Position[4]
                Joint_Position_error[5] = Joint_Position_target[5] - Joint_Position[5]
                
                Joint_Torque_ctrl_output[0] = Joint1_torque_ctrl.getControllerOutput(Joint_Position_error[0]) 
                Joint_Torque_ctrl_output[1] = Joint2_torque_ctrl.getControllerOutput(Joint_Position_error[1])
                Joint_Torque_ctrl_output[2] = Joint3_torque_ctrl.getControllerOutput(Joint_Position_error[2])
                Joint_Torque_ctrl_output[3] = Joint4_torque_ctrl.getControllerOutput(Joint_Position_error[3])
                Joint_Torque_ctrl_output[4] = Joint5_torque_ctrl.getControllerOutput(Joint_Position_error[4])
                Joint_Torque_ctrl_output[5] = Joint6_torque_ctrl.getControllerOutput(Joint_Position_error[5])
                
                # Joint_Torque_ctrl_output[0] = Joint_Torque_ctrl_output[0]
                
                Joint_Torque_ctrl_output_filtered[0] = Joint1_torque_ctrl_lpf.getFilteredOutput(Joint_Torque_ctrl_output[0])
                Joint_Torque_ctrl_output_filtered[1] = Joint2_torque_ctrl_lpf.getFilteredOutput(Joint_Torque_ctrl_output[1])
                Joint_Torque_ctrl_output_filtered[2] = Joint3_torque_ctrl_lpf.getFilteredOutput(Joint_Torque_ctrl_output[2])
                Joint_Torque_ctrl_output_filtered[3] = Joint4_torque_ctrl_lpf.getFilteredOutput(Joint_Torque_ctrl_output[3])
                Joint_Torque_ctrl_output_filtered[4] = Joint5_torque_ctrl_lpf.getFilteredOutput(Joint_Torque_ctrl_output[4])
                Joint_Torque_ctrl_output_filtered[5] = Joint6_torque_ctrl_lpf.getFilteredOutput(Joint_Torque_ctrl_output[5])

                Joint_Torque_ctrl_target[0] = int(Joint_Torque_ctrl_output_filtered[0]/0.1)
                Joint_Torque_ctrl_target[1] = int(Joint_Torque_ctrl_output_filtered[1]/0.1)
                Joint_Torque_ctrl_target[2] = int(Joint_Torque_ctrl_output_filtered[2]/0.1)
                Joint_Torque_ctrl_target[3] = int(Joint_Torque_ctrl_output_filtered[3]/0.1)
                Joint_Torque_ctrl_target[4] = int(Joint_Torque_ctrl_output_filtered[4]/0.1)
                Joint_Torque_ctrl_target[5] = int(Joint_Torque_ctrl_output_filtered[5]/0.1)
                
                if (abs(Joint_Torque_ctrl_target[0])>Joint1_Max_Torque_Output):
                    Joint_Torque_ctrl_target[0] = sign(Joint_Torque_ctrl_target[0])*Joint1_Max_Torque_Output
                if (abs(Joint_Torque_ctrl_target[1])>Joint2_Max_Torque_Output):
                    Joint_Torque_ctrl_target[1] = sign(Joint_Torque_ctrl_target[1])*Joint2_Max_Torque_Output
                if (abs(Joint_Torque_ctrl_target[2])>Joint3_Max_Torque_Output):
                    Joint_Torque_ctrl_target[2] = sign(Joint_Torque_ctrl_target[2])*Joint3_Max_Torque_Output
                if (abs(Joint_Torque_ctrl_target[3])>Joint4_Max_Torque_Output):
                    Joint_Torque_ctrl_target[3] = sign(Joint_Torque_ctrl_target[3])*Joint4_Max_Torque_Output
                if (abs(Joint_Torque_ctrl_target[4])>Joint5_Max_Torque_Output):
                    Joint_Torque_ctrl_target[4] = sign(Joint_Torque_ctrl_target[4])*Joint5_Max_Torque_Output
                if (abs(Joint_Torque_ctrl_target[5])>Joint6_Max_Torque_Output):
                    Joint_Torque_ctrl_target[5] = sign(Joint_Torque_ctrl_target[5])*Joint6_Max_Torque_Output
                    
                Joint1_pdo.TargetTorque = Joint_Torque_ctrl_target[0]
                Joint2_pdo.TargetTorque = Joint_Torque_ctrl_target[1]
                Joint3_pdo.TargetTorque = Joint_Torque_ctrl_target[2]
                Joint4_pdo.TargetTorque = Joint_Torque_ctrl_target[3]
                Joint5_pdo.TargetTorque = Joint_Torque_ctrl_target[4]
                Joint6_pdo.TargetTorque = Joint_Torque_ctrl_target[5]
            else:
                Joint_Position_target[0] = int(target[0])
                Joint_Position_target[1] = int(target[1])
                Joint_Position_target[2] = int(target[2])
                Joint_Position_target[3] = int(target[3])
                Joint_Position_target[4] = int(target[4])
                Joint_Position_target[5] = int(target[5])
                Joint1_pdo.TargetPosition = Joint_Position_target[0]
                Joint2_pdo.TargetPosition = Joint_Position_target[1]
                Joint3_pdo.TargetPosition = Joint_Position_target[2]
                Joint4_pdo.TargetPosition = Joint_Position_target[3]
                Joint5_pdo.TargetPosition = Joint_Position_target[4]
                Joint6_pdo.TargetPosition = Joint_Position_target[5]
                
                Joint_Torque_ctrl_target[0] = 0
                Joint_Torque_ctrl_target[1] = 0
                Joint_Torque_ctrl_target[2] = 0
                Joint_Torque_ctrl_target[3] = 0
                Joint_Torque_ctrl_target[4] = 0
                Joint_Torque_ctrl_target[5] = 0
                
                Joint1_pdo.TargetTorque = Joint_Torque_ctrl_target[0]
                Joint2_pdo.TargetTorque = Joint_Torque_ctrl_target[1]
                Joint3_pdo.TargetTorque = Joint_Torque_ctrl_target[2]
                Joint4_pdo.TargetTorque = Joint_Torque_ctrl_target[3]
                Joint5_pdo.TargetTorque = Joint_Torque_ctrl_target[4]
                Joint6_pdo.TargetTorque = Joint_Torque_ctrl_target[5]
            
            # if (index<=task_span):
            #     Joint1_pdo.TargetPosition = int(50000*math.sin((2*math.pi)/(task_span/6)*index))
            # else: 
            #     Joint1_pdo.TargetPosition = 0
            
            if Joint1_pdo.ErrorCode == 16392:
                Joint1_pdo.Controlword = 128
                index = 0
                print("WRITE error out--------------------------")
            if Joint2_pdo.ErrorCode == 16392:
                Joint2_pdo.Controlword = 128
                index = 0
                print("WRITE error out--------------------------")
            if Joint3_pdo.ErrorCode == 16392:
                Joint3_pdo.Controlword = 128
                index = 0
                print("WRITE error out--------------------------")
            if Joint4_pdo.ErrorCode == 16392:
                Joint4_pdo.Controlword = 128
                index = 0
                print("WRITE error out--------------------------")
            if Joint5_pdo.ErrorCode == 16392:
                Joint5_pdo.Controlword = 128
                index = 0
                print("WRITE error out--------------------------")
            if Joint6_pdo.ErrorCode == 16392:
                Joint6_pdo.Controlword = 128
                index = 0
                print("WRITE error out--------------------------")

            print('p1:%07d,t1:%05d,p2:%07d,t2:%05d,p3:%07d,t3:%05d,p4:%07d,t4:%05d,p5:%07d,t5:%05d,p6:%07d,t6:%05d' 
                  % (Joint_Position[0],Joint_Torque_ctrl_target[0], 
                    Joint_Position[1],Joint_Torque_ctrl_target[1], 
                    Joint_Position[2],Joint_Torque_ctrl_target[2], 
                    Joint_Position[3],Joint_Torque_ctrl_target[3], 
                    Joint_Position[4],Joint_Torque_ctrl_target[4], 
                    Joint_Position[5],Joint_Torque_ctrl_target[5]))
            # print(Joint_Position_error[1], Joint_Torque_ctrl_output[1],Joint_Torque_ctrl_target[1])
            
            Joint1.output = Joint1_pdo.encode_Rx_PDO()
            Joint2.output = Joint2_pdo.encode_Rx_PDO()
            Joint3.output = Joint3_pdo.encode_Rx_PDO()
            Joint4.output = Joint4_pdo.encode_Rx_PDO()
            Joint5.output = Joint5_pdo.encode_Rx_PDO()
            Joint6.output = Joint6_pdo.encode_Rx_PDO()

            if is_in_init:
                index_init += 1
            else:
                index += 1

            tspan_ns = (time.time_ns() - t0)
            if (tspan_ns < cycle_time*1000):
                time.sleep( (cycle_time*1_000 - tspan_ns)/1_000_000_000)

    except KeyboardInterrupt:
        Joint1_pdo.TargetPosition = Joint1_pdo.PositionActualValue
        Joint2_pdo.TargetPosition = Joint2_pdo.PositionActualValue
        Joint3_pdo.TargetPosition = Joint3_pdo.PositionActualValue
        Joint4_pdo.TargetPosition = Joint4_pdo.PositionActualValue
        Joint5_pdo.TargetPosition = Joint5_pdo.PositionActualValue
        Joint6_pdo.TargetPosition = Joint6_pdo.PositionActualValue
        # Joint1_pdo.TargetVelocity = 0
        Joint1_pdo.TargetTorque = 0
        Joint2_pdo.TargetTorque = 0
        Joint3_pdo.TargetTorque = 0
        Joint4_pdo.TargetTorque = 0
        Joint5_pdo.TargetTorque = 0
        Joint6_pdo.TargetTorque = 0
        Joint1.output = Joint1_pdo.encode_Rx_PDO()
        Joint2.output = Joint2_pdo.encode_Rx_PDO()
        Joint3.output = Joint3_pdo.encode_Rx_PDO()
        Joint4.output = Joint4_pdo.encode_Rx_PDO()
        Joint5.output = Joint5_pdo.encode_Rx_PDO()
        Joint6.output = Joint6_pdo.encode_Rx_PDO()
        time.sleep(2*cycle_time/1_000_000_000)
        Joint1_pdo.Controlword = 0
        Joint2_pdo.Controlword = 0
        Joint3_pdo.Controlword = 0
        Joint4_pdo.Controlword = 0
        Joint5_pdo.Controlword = 0
        Joint6_pdo.Controlword = 0

        Joint1.output = Joint1_pdo.encode_Rx_PDO()
        Joint2.output = Joint2_pdo.encode_Rx_PDO()
        Joint3.output = Joint3_pdo.encode_Rx_PDO()
        Joint4.output = Joint4_pdo.encode_Rx_PDO()
        Joint5.output = Joint5_pdo.encode_Rx_PDO()
        Joint6.output = Joint6_pdo.encode_Rx_PDO()
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

Joint1 = master.slaves[0]
Joint2 = master.slaves[1]
Joint3 = master.slaves[2]
Joint4 = master.slaves[3]
Joint5 = master.slaves[4]
Joint6 = master.slaves[5]
Joint1_pdo.decode_Tx_PDO(Joint1.input)
Joint2_pdo.decode_Tx_PDO(Joint2.input)
Joint3_pdo.decode_Tx_PDO(Joint3.input)
Joint4_pdo.decode_Tx_PDO(Joint4.input)
Joint5_pdo.decode_Tx_PDO(Joint5.input)
Joint6_pdo.decode_Tx_PDO(Joint6.input)
print('J1:%07d,J2:%07d,J3:%07d,J4:%07d,J5:%07d,J6:%07d,' 
                  % (Joint1_pdo.PositionActualValue,
                     Joint2_pdo.PositionActualValue,
                     Joint3_pdo.PositionActualValue,
                     Joint4_pdo.PositionActualValue,
                     Joint5_pdo.PositionActualValue,
                     Joint6_pdo.PositionActualValue))

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
else:
    pd_thread_stop_event.set()
    ch_thread_stop_event.set()
    proc_thread.join()
    check_thread.join()
    master.state = pysoem.INIT_STATE
    master.write_state()
    master.close