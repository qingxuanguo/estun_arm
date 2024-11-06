import type_conversion as tc

M_PI = 3.1415926535

class PIDcontroller: 
    def __init__(self, control_cycle_sec, Kp, Ki, Kd):
        self.control_cycle_sec_ = control_cycle_sec
        self.Kp_ = Kp
        self.Ki_ = Ki
        self.Kd_ = Kd

        self.e = 0
        self.de = 0
        self.dde = 0
        self.prev_output_ = 0

    def getControllerOutput(self, present_error):
        self.dde = self.de
        self.de = self.e
        self.e = present_error
        
        K1 = self.Kp_ + self.Kd_/self.control_cycle_sec_ + self.Ki_*self.control_cycle_sec_
        K2 = -self.Kp_ - 2*self.Kd_/self.control_cycle_sec_
        K3 = self.Kd_/self.control_cycle_sec_

        self.prev_output_ = self.prev_output_ + self.e*K1 + self.de*K2 + self.dde*K3
        return self.prev_output_ 

class LowPassFilter:
    def __init__(self, control_cycle_sec, cutoff_frequency):
        self.cutoff_freq_ = cutoff_frequency
        self.control_cycle_sec_ = control_cycle_sec
        self.prev_output_ = 0

        if (cutoff_frequency > 0):
            self.alpha_ = (2.0*M_PI*self.cutoff_freq_*self.control_cycle_sec_)/(1.0+2.0*M_PI*self.cutoff_freq_*self.control_cycle_sec_)
        else:
            self.alpha_ = 1
    
    def getFilteredOutput(self, present_raw_value):
        self.prev_output_ = self.alpha_*present_raw_value + (1.0 - self.alpha_)*self.prev_output_
        return self.prev_output_

class Pdo_stream:
    def __init__(self, slave_id_):
        self.slave_id = slave_id_

        #TxPDO
        self.Statusword            = 0 
        self.ModesOperationDisplay = 0 
        self.ErrorCode             = 0 
        self.PositionActualValue   = 0 
        self.VelocityActualValue   = 0
        self.TorqueActualValue     = 0 
        self.TorqueSensorValue     = 0 
        self.CollisionFlags        = 0 
        self.SemiEncoderSPI        = 0 
        self.FullEncoderSPI        = 0

        #RxPDO
        self.Controlword                  = 0              
        self.ModeOperation                = 0               
        self.TargetPosition               = 0
        self.TargetVelocity               = 0
        self.TargetTorque                 = 0
        self.Polarity                     = 0
        self.VelocityOffset               = 0
        self.TorqueOffset                 = 0
        self.GravityCompensation          = 0
        self.GravityCommandedCompensation = 0
    
    def decode_Tx_PDO(self, tx_pkt):
        self.Statusword            = tc.byte_2_2_UINT(tx_pkt[0:2])
        self.ModesOperationDisplay = tc.byte_1_2_SINT(tx_pkt[2:3])
        self.ErrorCode             = tc.byte_2_2_UINT(tx_pkt[3:5]) 
        self.PositionActualValue   = tc.byte_4_2_DINT(tx_pkt[5:9])  
        self.VelocityActualValue   = tc.byte_4_2_DINT(tx_pkt[9:13])  
        self.TorqueActualValue     = tc.byte_2_2_INT(tx_pkt[13:15])   
        self.TorqueSensorValue     = tc.byte_2_2_INT(tx_pkt[15:17])   
        self.CollisionFlags        = tc.byte_2_2_INT(tx_pkt[17:19])  
        self.SemiEncoderSPI        = tc.byte_4_2_UDINT(tx_pkt[19:23])  
        self.FullEncoderSPI        = tc.byte_4_2_UDINT(tx_pkt[23:27])  
    
    def encode_Rx_PDO(self):
        rx_pkt = bytearray(28)
        rx_pkt[0:2]   = tc.UINT_2_byte_2(self.Controlword)
        rx_pkt[2:3]   = tc.SINT_2_byte_1(self.ModeOperation)
        rx_pkt[3:7]   = tc.DINT_2_byte_4(self.TargetPosition)
        rx_pkt[7:11]  = tc.DINT_2_byte_4(self.TargetVelocity)
        rx_pkt[11:13] = tc.INT_2_byte_2(self.TargetTorque)
        rx_pkt[13:14] = tc.USINT_2_byte_1(self.Polarity)
        rx_pkt[14:18] = tc.DINT_2_byte_4(self.VelocityOffset)
        rx_pkt[18:20] = tc.INT_2_byte_2(self.TorqueOffset)
        rx_pkt[20:24] = tc.DINT_2_byte_4(self.GravityCompensation)
        rx_pkt[24:28] = tc.DINT_2_byte_4(self.GravityCommandedCompensation)
        return bytes(rx_pkt)
    
    def StatuswordInterprate(self):
        statusword = self.Statusword
        if statusword == 0:
            return "NotReadyToSwitchOn"
        elif statusword == 64:
            return "Disabled"
        elif statusword == 33:
            return "ReadyToSwitchOn"
        elif statusword == 563:
            return "SwitchedOn"
        elif statusword == 567:
            return "Operating"
        elif statusword == 7:
            return "QuickStopActive"
        elif statusword == 8:
            return "Fault"

    # def Pos2Deg(self):
    #     return (self.PositionActualValue)/524288*360
    
    # def Vel2DegSec(self):
    #     return (self.VelocityActualValue)*0.001/(self.PI())*180
    
    # def Tau2Nm(self):
    #     return (self.TorqueActualValue)*0.1
    
    # def Tau_s2Nm(self):
    #     return (self.TorqueSensorValue)*0.1