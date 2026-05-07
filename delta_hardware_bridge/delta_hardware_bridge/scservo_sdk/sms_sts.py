#!/usr/bin/env python

from .scservo_def import *
from .protocol_packet_handler import *
from .group_sync_read import *
from .group_sync_write import *

# Baudrate definition
SMS_STS_1M = 0
SMS_STS_0_5M = 1
SMS_STS_250K = 2
SMS_STS_128K = 3
SMS_STS_115200 = 4
SMS_STS_76800 = 5
SMS_STS_57600 = 6
SMS_STS_38400 = 7

# Memory table definition
#-------EPROM(Read Only)--------
SMS_STS_MODEL_L = 3
SMS_STS_MODEL_H = 4

#-------EPROM(Read/Write)--------
SMS_STS_ID = 5
SMS_STS_BAUD_RATE = 6
SMS_STS_MIN_ANGLE_LIMIT_L = 9
SMS_STS_MIN_ANGLE_LIMIT_H = 10
SMS_STS_MAX_ANGLE_LIMIT_L = 11
SMS_STS_MAX_ANGLE_LIMIT_H = 12
SMS_STS_CW_DEAD = 26
SMS_STS_CCW_DEAD = 27
SMS_STS_OFS_L = 31
SMS_STS_OFS_H = 32
SMS_STS_MODE = 33

#-------SRAM(Read/Write)--------
SMS_STS_TORQUE_ENABLE = 40
SMS_STS_ACC = 41
SMS_STS_GOAL_POSITION_L = 42
SMS_STS_GOAL_POSITION_H = 43
SMS_STS_GOAL_TIME_L = 44
SMS_STS_GOAL_TIME_H = 45
SMS_STS_GOAL_SPEED_L = 46
SMS_STS_GOAL_SPEED_H = 47
SMS_STS_LOCK = 55

#-------SRAM(Read Only)--------
SMS_STS_PRESENT_POSITION_L = 56
SMS_STS_PRESENT_POSITION_H = 57
SMS_STS_PRESENT_SPEED_L = 58
SMS_STS_PRESENT_SPEED_H = 59
SMS_STS_PRESENT_LOAD_L = 60
SMS_STS_PRESENT_LOAD_H = 61
SMS_STS_PRESENT_VOLTAGE = 62
SMS_STS_PRESENT_TEMPERATURE = 63
SMS_STS_MOVING = 66
SMS_STS_PRESENT_CURRENT_L = 69
SMS_STS_PRESENT_CURRENT_H = 70

class sms_sts(protocol_packet_handler):
    def __init__(self, portHandler):
        protocol_packet_handler.__init__(self, portHandler, 0)
        self.groupSyncWrite = GroupSyncWrite(self, SMS_STS_ACC, 7)

    def WritePosEx(self, scs_id, position, speed, acc):
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxRx(scs_id, SMS_STS_ACC, len(txpacket), txpacket)

    def WritePosExTxOnly(self, scs_id, position, speed, acc):
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxOnly(scs_id, SMS_STS_ACC, len(txpacket), txpacket)

    def ReadPos(self, scs_id):
        scs_present_position, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, SMS_STS_PRESENT_POSITION_L)
        return self.scs_tohost(scs_present_position, 15), scs_comm_result, scs_error

    def ReadSpeed(self, scs_id):
        scs_present_speed, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, SMS_STS_PRESENT_SPEED_L)
        return self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadPosSpeed(self, scs_id):
        scs_present_position_speed, scs_comm_result, scs_error = self.read4ByteTxRx(scs_id, SMS_STS_PRESENT_POSITION_L)
        scs_present_position = self.scs_loword(scs_present_position_speed)
        scs_present_speed = self.scs_hiword(scs_present_position_speed)
        return self.scs_tohost(scs_present_position, 15), self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadMoving(self, scs_id):
        moving, scs_comm_result, scs_error = self.read1ByteTxRx(scs_id, SMS_STS_MOVING)
        return moving, scs_comm_result, scs_error

    def SyncWritePosEx(self, scs_id, position, speed, acc):
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.groupSyncWrite.addParam(scs_id, txpacket)

    def RegWritePosEx(self, scs_id, position, speed, acc):
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.regWriteTxRx(scs_id, SMS_STS_ACC, len(txpacket), txpacket)

    def RegAction(self):
        return self.action(BROADCAST_ID)

    def WheelMode(self, scs_id):
        return self.write1ByteTxRx(scs_id, SMS_STS_MODE, 1)

    def WriteSpec(self, scs_id, speed, acc):
        speed = self.scs_toscs(speed, 15)
        txpacket = [acc, 0, 0, 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxRx(scs_id, SMS_STS_ACC, len(txpacket), txpacket)

    def LockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, SMS_STS_LOCK, 1)

    def unLockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, SMS_STS_LOCK, 0)

    def ReadCurrent(self, scs_id):
        """
        Read real-time current value (unit: mA)
        Returns: current value (signed), comm result, error code
        """
        # Read two bytes of raw current data (address 69-70)
        raw_current, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, SMS_STS_PRESENT_CURRENT_L)
        
        # Assume 6.5mA/bit conversion ratio (refer to manual for details)
        current = self.scs_tohost(raw_current, 15) * 6.5  # 15 indicates 16-bit signed number
        return current, scs_comm_result, scs_error

    def ReadVoltage(self, scs_id):
        """
        Read real-time voltage (unit: 0.1V)
        Returns: voltage value, comm result, error code
        """
        voltage, result, error = self.read1ByteTxRx(scs_id, SMS_STS_PRESENT_VOLTAGE)
        return voltage * 0.1, result, error  # Adjust according to actual conversion ratio in manual

    def ReadTemperature(self, scs_id):
        """
        Read real-time temperature (unit: Celsius)
        Returns: temperature value, comm result, error code
        """
        return self.read1ByteTxRx(scs_id, SMS_STS_PRESENT_TEMPERATURE)

    def ReadLoad(self, scs_id):
        """
        Read real-time load (unit: %)
        Returns: load percentage (signed), comm result, error code
        """
        raw_load, result, error = self.read2ByteTxRx(scs_id, SMS_STS_PRESENT_LOAD_L)
        # Convert to signed percentage (positive=CCW load, negative=CW load)
        load_percent = self.scs_tohost(raw_load, 15) * 0.1  # Adjust scale factor according to manual
        return load_percent, result, error

    def ReadID(self):
        """
        Read Servo ID
        Parameters:
            scs_id: Servo ID, defaults to Broadcast ID (0xFE)
        Returns: ID value, comm result, error code
        """
        return self.scan_ids()

    def ReadAngleLimits(self, scs_id):
        """
        Read servo max/min angle limits
        Returns: min angle, max angle, comm result, error code
        """
        # Read min angle limit (2 bytes)
        min_angle, result1, error1 = self.read2ByteTxRx(scs_id, SMS_STS_MIN_ANGLE_LIMIT_L)
        
        # Read max angle limit (2 bytes) 
        max_angle, result2, error2 = self.read2ByteTxRx(scs_id, SMS_STS_MAX_ANGLE_LIMIT_L)

        # Return the more serious comm error
        if result1 != COMM_SUCCESS:
            return min_angle, max_angle, result1, error1
        elif result2 != COMM_SUCCESS:
            return min_angle, max_angle, result2, error2
            
        return min_angle, max_angle, COMM_SUCCESS, 0
    
    def WriteAngleLimits(self, scs_id, min_angle, max_angle):
        """
        Set servo max/min angle limits
        Parameters:
            scs_id: Servo ID
            min_angle: Min angle limit value (0~4095)
            max_angle: Max angle limit value (0~4095)
        Returns: comm result, error code
        """
        # Write min angle limit (2 bytes)
        result1, error1 = self.write2ByteTxRx(scs_id, SMS_STS_MIN_ANGLE_LIMIT_L, min_angle)
        
        # Write max angle limit (2 bytes)
        result2, error2 = self.write2ByteTxRx(scs_id, SMS_STS_MAX_ANGLE_LIMIT_L, max_angle)
        
        # Return the more serious comm error
        if result1 != COMM_SUCCESS:
            return result1, error1
        return result2, error2
    
    def TorqueEnable(self, scs_id, enable):
        """
        Set servo torque switch
        Parameters:
            scs_id: Servo ID
            enable: 0=Turn off torque output, 1=Turn on torque output, 128=Calibrate current position to 2048
        Returns: comm result, error code
        """
        return self.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, enable)