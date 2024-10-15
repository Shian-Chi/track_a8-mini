"""
Python implementation of SIYI SDK
ZR10 webpage: http://en.siyi.biz/en/Gimbal%20Camera/ZR10/overview/
Author : Mohamed Abdelkader
Email: mohamedashraf123@gmail.com
Copyright 2022

"""
import socket
from a8mini_PyAPI.siyi_sdk.siyi_message import *
from time import sleep, time
import logging
from a8mini_PyAPI.siyi_sdk.utils import toInt
import threading

import math

class A8Mini_Controller:
    class DataExpiredError(Exception):
        pass

    def __init__(self, server_ip="192.168.144.25", port=37260, debug=False):
        """
        
        Params
        --
        - server_ip [str] IP address of the camera
        - port: [int] UDP port of the camera
        """
        self._debug= debug # print debug messages
        LOG_FORMAT=' [%(levelname)s] %(asctime)s [SIYISDK::%(funcName)s] :\t%(message)s'
        logging.basicConfig(format=LOG_FORMAT, level=logging.INFO if not self._debug else logging.DEBUG)
        self._logger = logging.getLogger(self.__class__.__name__)

        # Message sent to the camera
        self._out_msg = SIYIMESSAGE(debug=self._debug)
        
        # Message received from the camera
        self._in_msg = SIYIMESSAGE(debug=self._debug)        

        self._server_ip = server_ip
        self._port = port

        self._BUFF_SIZE=1024

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rcv_wait_t = 2 # Receiving wait time
        self._socket.settimeout(self._rcv_wait_t)
        self._connected = False

        self._fw_msg = FirmwareMsg()                    # cmd:0x01
        self._hw_msg = HardwareIDMsg()                  # cmd:0x02
        self._manualZoom_msg=ManualZoomMsg()            # cmd:0x05
        self._gimbalSpeed_msg=GimbalSpeedMsg()          # cmd:0x07
        self._center_msg=CenterMsg()                    # cmd:0x08
        self._record_msg=RecordingMsg()                 # cmd:0x0a
        self._mountDir_msg=MountDirMsg()                # cmd:0x0a
        self._motionMode_msg=MotionModeMsg()            # cmd:0x0a
        self._funcFeedback_msg=FuncFeedbackInfoMsg()    # cmd:0x0b
        self._att_msg=AttitdueMsg()                     # cmd:0x0d
        self._absoluteZoomLevel_msg=AbsoluteZoomLevelMsg()
        self._gimbalAngle_msg=GimbalAngleMsg()

        self._recv_thread_stop = False
        # self._g_att_thread_stop = False

    # def startGimbalAttLoop(self, req_interval=0.1):
    #     self._g_att_thread_stop = False
    #     self._g_att_thread = threading.Thread(target=self.gimbalAttLoop, args=(req_interval,))
    #     self._g_att_thread.start()
    
    # def stopGimbalAttLoop(self):
    #     self._g_att_thread_stop = True

    # def gimbalAttLoop(self, t):
        # while not self._g_att_thread_stop :
        #     print(self.requestGimbalAttitude())
        #     sleep(t)

    def sendMsg(self, msg):
        """
        Sends a message to the camera

        Params
        --
        msg [str] Message to send
        """
        b = bytes.fromhex(msg)
        try:
            self._socket.sendto(b, (self._server_ip, self._port))
            self._logger.debug(f'send msg: {msg}')
            return True
        except Exception as e:
            self._logger.error("Could not send bytes")
            return False

    def startRecvLoop(self):
        self._recv_thread_stop = False
        self._recv_thread = threading.Thread(target=self.recvLoop)
        self._recv_thread.start()

    def stopRecvLoop(self):
        self._recv_thread_stop = True

    def recvLoop(self):
        self._logger.debug("Started data receiving thread")
        while not self._recv_thread_stop:
            try:
                self.bufferCallback()
            except TimeoutError :
                # self.stopRecvLoop()
                # self._logger.error('receive timeout')
                pass
            except Exception as e:
                self._logger.error(e)
        self._logger.debug("Exiting data receiving thread")
    
    def bufferCallback(self):
        """
        Receives messages and parses its content
        """
        buff,addr = self._socket.recvfrom(self._BUFF_SIZE)

        buff_str = buff.hex()
        self._logger.debug("Buffer: %s", buff_str)

        # 10 bytes: STX+CTRL+Data_len+SEQ+CMD_ID+CRC16
        #            2 + 1  +    2   + 2 +   1  + 2
        MINIMUM_DATA_LENGTH=10*2

        HEADER='5566'
        # Go through the buffer
        while(len(buff_str)>=MINIMUM_DATA_LENGTH):
            if buff_str[0:4]!=HEADER:
                # Remove the 1st element and continue 
                tmp=buff_str[1:]
                buff_str=tmp
                continue

            # Now we got minimum amount of data. Check if we have enough
            # Data length, bytes are reversed, according to SIYI SDK
            low_b = buff_str[6:8] # low byte
            high_b = buff_str[8:10] # high byte
            data_len = high_b+low_b
            data_len = int('0x'+data_len, base=16)
            char_len = data_len*2

            # Check if there is enough data (including payload)
            if(len(buff_str) < (MINIMUM_DATA_LENGTH+char_len)):
                # No useful data
                buff_str=''
                break
            
            packet = buff_str[0:MINIMUM_DATA_LENGTH+char_len]
            buff_str = buff_str[MINIMUM_DATA_LENGTH+char_len:]

            # Finally decode the packet!
            val = self._in_msg.decodeMsg(packet)
            if val is None:
                continue
            
            data, data_len, cmd_id, seq = val[0], val[1], val[2], val[3]

            if cmd_id==COMMAND.ACQUIRE_FW_VER:
                self.parseFirmwareMsg(data, seq)
            elif cmd_id==COMMAND.ACQUIRE_HW_ID:
                self.parseHardwareIDMsg(data, seq)
            elif cmd_id==COMMAND.ACQUIRE_GIMBAL_INFO:
                self.parseGimbalInfoMsg(data, seq)
            elif cmd_id==COMMAND.ACQUIRE_GIMBAL_ATT:
                self.parseAttitudeMsg(data, seq)
            elif cmd_id==COMMAND.FUNC_FEEDBACK_INFO:
                self.parseFunctionFeedbackMsg(data, seq)
            elif cmd_id==COMMAND.GIMBAL_ROT:
                self.parseGimbalSpeedMsg(data, seq)
            elif cmd_id==COMMAND.MANUAL_FOCUS:
                self.parseManualFocusMsg(data, seq)
            elif cmd_id==COMMAND.MANUAL_ZOOM:
                self.parseZoomMsg(data, seq)
            elif cmd_id==COMMAND.CENTER:
                self.parseGimbalCenterMsg(data, seq)
            elif cmd_id==COMMAND.ABSOLUTE_ZOOM_LEVEL:
                self.parseAbsoluteZoomLevelMsg(data, seq)
            elif cmd_id==COMMAND.GIMBAL_ANGLE_SET:
                self.parseGimbalAngleMsg(data, seq)
            else:
                self._logger.warning("CMD ID is not recognized")
        
        return None
    
    ##################################################
    #               Request functions                #
    ##################################################    
    def requestFirmwareVersion(self):
        """
        Sends request for firmware version

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.firmwareVerMsg()
        if not self.sendMsg(msg):
            return False
        return True
    
    def requestHardwareID(self):
        """
        Sends request for Hardware ID

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.hwIdMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestGimbalAttitude(self):
        """
        Sends request for gimbal attitude

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.gimbalAttMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestGimbalInfo(self):
        """
        Sends request for gimbal information

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.gimbalInfoMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestFunctionFeedback(self):
        """
        Sends request for function feedback msg

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.funcFeedbackMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestPhoto(self):
        """
        Sends request for taking photo
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.takePhotoMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestRecording(self):
        """
        Sends request for toglling video recording
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.recordMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestFPVMode(self):
        """
        Sends request for setting FPV mode
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.fpvModeMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestLockMode(self):
        """
        Sends request for setting Lock mode
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.lockModeMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestFollowMode(self):
        """
        Sends request for setting Follow mode
        
        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.followModeMsg()
        if not self.sendMsg(msg):
            return False
        return True
# -------------
    def requestZoomIn(self):
        """
        Sends request for zoom in

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.zoomInMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomOut(self):
        """
        Sends request for zoom out

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.zoomOutMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestZoomHold(self):
        """
        Sends request for stopping zoom

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.stopZoomMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestAbsoluteZoomLevel(self, int_val, float_val):
        msg = self._out_msg.absoluteZoomLevelMsg(int_val, float_val)
        if not self.sendMsg(msg):
            return False
        return True

    def requestCenterGimbal(self):
        """
        Sends request for gimbal centering

        Returns
        --
        [bool] True: success. False: fail
        """
        msg = self._out_msg.centerMsg()
        if not self.sendMsg(msg):
            return False
        return True

    def requestGimbalSpeed(self, yaw_speed:int, pitch_speed:int):
        """
        Sends request for gimbal centering

        Params
        --
        yaw_speed [int] -100~0~100. away from zero -> fast, close to zero -> slow. Sign is for direction
        pitch_speed [int] Same as yaw_speed
        
        Returns
        --
        [bool] True: success. False: fail
        []
        """
        msg = self._out_msg.gimbalSpeedMsg(yaw_speed, pitch_speed)
        if not self.sendMsg(msg):
            return False
        return True

    def requestGimbalAngle(self, yaw_angle:float, pitch_angle:float):
        msg = self._out_msg.gimbalAngleSetMsg(yaw_angle, pitch_angle)
        if not self.sendMsg(msg):
            return False
        return True

    ####################################################
    #                Parsing functions                 #
    ####################################################
    def parseFirmwareMsg(self, msg:str, seq:int):
        try:
            self._fw_msg.gimbal_firmware_ver= msg[8:16]
            self._fw_msg.seq=seq
            self._fw_msg._last_update_time=time()

            self._logger.debug("Firmware version: %s", self._fw_msg.gimbal_firmware_ver)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseHardwareIDMsg(self, msg:str, seq:int):
        try:
            self._hw_msg.seq=seq
            self._hw_msg.id = msg
            self._hw_msg._last_update_time=time()

            self._logger.debug("Hardware ID: %s", self._hw_msg.id)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseAttitudeMsg(self, msg:str, seq:int):   
        try:
            self._att_msg.seq=seq
            self._att_msg.yaw = toInt(msg[2:4]+msg[0:2]) /10.
            self._att_msg.pitch = toInt(msg[6:8]+msg[4:6]) /10.
            self._att_msg.roll = toInt(msg[10:12]+msg[8:10]) /10.
            self._att_msg.yaw_speed = toInt(msg[14:16]+msg[12:14]) /10.
            self._att_msg.pitch_speed = toInt(msg[18:20]+msg[16:18]) /10.
            self._att_msg.roll_speed = toInt(msg[22:24]+msg[20:22]) /10.
            self._att_msg._last_update_time=time()

            self._logger.debug("(yaw, pitch, roll= (%s, %s, %s)", 
                                    self._att_msg.yaw, self._att_msg.pitch, self._att_msg.roll)
            self._logger.debug("(yaw_speed, pitch_speed, roll_speed= (%s, %s, %s)", 
                                    self._att_msg.yaw_speed, self._att_msg.pitch_speed, self._att_msg.roll_speed)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalInfoMsg(self, msg:str, seq:int):
        try:
            self._record_msg.seq=seq
            self._motionMode_msg.seq=seq
            self._mountDir_msg.seq=seq
            
            self._record_msg.state = int('0x'+msg[6:8], base=16)
            self._motionMode_msg.mode = int('0x'+msg[8:10], base=16)
            self._mountDir_msg.dir = int('0x'+msg[10:12], base=16)
            self._record_msg._last_update_time=time()
            self._motionMode_msg._last_update_time=time()
            self._mountDir_msg._last_update_time=time()
            

            self._logger.debug("Recording state %s", self._record_msg.state)
            self._logger.debug("Mounting direction %s", self._mountDir_msg.dir)
            self._logger.debug("Gimbal motion mode %s", self._motionMode_msg.mode)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseZoomMsg(self, msg:str, seq:int):
        try:
            self._manualZoom_msg.seq=seq
            self._manualZoom_msg.level = int('0x'+msg[2:4]+msg[0:2], base=16) /10.
            self._manualZoom_msg._last_update_time = time()
            
            self._logger.debug("Zoom level %s", self._manualZoom_msg.level)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalSpeedMsg(self, msg:str, seq:int):
        
        try:
            self._gimbalSpeed_msg.seq=seq
            self._gimbalSpeed_msg.success = bool(int('0x'+msg, base=16))
            self._gimbalSpeed_msg._last_update_time = time()

            self._logger.debug("Gimbal speed success: %s", self._gimbalSpeed_msg.success)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseGimbalCenterMsg(self, msg:str, seq:int):
        
        try:
            self._center_msg.seq=seq
            self._center_msg.success = bool(int('0x'+msg, base=16))
            self._center_msg._last_update_time = time()
            
            self._logger.debug("Gimbal center success: %s", self._center_msg.success)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False

    def parseFunctionFeedbackMsg(self, msg:str, seq:int):
        
        try:
            self._funcFeedback_msg.seq=seq
            self._funcFeedback_msg.info_type = int('0x'+msg, base=16)
            self._funcFeedback_msg._last_update_time = time()
            
            self._logger.debug("Function Feedback Code: %s", self._funcFeedback_msg.info_type)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False
    
    def parseAbsoluteZoomLevelMsg(self, msg:str, seq:int):
        try:
            self._absoluteZoomLevel_msg.seq=seq
            self._absoluteZoomLevel_msg.success = bool(int('0x'+msg, base=16))
            self._absoluteZoomLevel_msg._last_update_time = time()

            self._logger.debug("Absolute Zoom Code: %s", self._funcFeedback_msg.info_type)

            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False
    
    def parseGimbalAngleMsg(self, msg:str, seq:int):
        try:
            self._gimbalAngle_msg.seq=seq
            self._gimbalAngle_msg.yaw = toInt(msg[2:4]+msg[0:2]) /10.
            self._gimbalAngle_msg.pitch = toInt(msg[6:8]+msg[4:6]) /10.
            self._gimbalAngle_msg.roll = toInt(msg[10:12]+msg[8:10]) /10.
            self._gimbalAngle_msg._last_update_time = time()

            self._logger.debug("(yaw, pitch, roll= (%s, %s, %s)", 
                                    self._att_msg.yaw, self._att_msg.pitch, self._att_msg.roll)
            return True
        except Exception as e:
            self._logger.error("Error %s", e)
            return False 

    ##################################################
    #                   Get functions                #
    ##################################################
    def getAttitude(self, exp_time=math.inf):
        if not hasattr(self._att_msg, '_last_update_time'): return None, None
        if self._att_msg._last_update_time + exp_time < time() : raise A8Mini_Controller.DataExpiredError('_att_msg is expired')
        
        data = {'yaw':self._att_msg.yaw,
                'pitch':self._att_msg.pitch,
                'roll':self._att_msg.roll,
                'yaw_speed':self._att_msg.yaw_speed,
                'pitch_speed':self._att_msg.pitch_speed,
                'roll_speed':self._att_msg.roll_speed}
        data_age = time()-self._att_msg._last_update_time
        return data, data_age

    def getFirmwareVersion(self, exp_time=math.inf):
        if not hasattr(self._fw_msg, '_last_update_time'): return None, None
        if self._fw_msg._last_update_time + exp_time < time() : raise A8Mini_Controller.DataExpiredError('_fw_msg is expired')

        data = {'gimbal_firmware_ver':self._fw_msg.gimbal_firmware_ver}
        data_age = time()-self._fw_msg._last_update_time
        return data, data_age

    def getHardwareID(self, exp_time=math.inf):
        if not hasattr(self._hw_msg, '_last_update_time'): return None, None
        if self._hw_msg._last_update_time + exp_time < time() : raise A8Mini_Controller.DataExpiredError('_hw_msg is expired')

        data = {'id':self._hw_msg.id}
        data_age = time()-self._hw_msg._last_update_time
        return data, data_age

    def getRecordingState(self, exp_time=math.inf):
        if not hasattr(self._record_msg, '_last_update_time'): return None, None
        if self._record_msg._last_update_time + exp_time < time() : raise A8Mini_Controller.DataExpiredError('_record_msg is expired')

        data = {'state':self._record_msg.state}
        data_age = time()-self._record_msg._last_update_time
        return data, data_age

    def getMotionMode(self, exp_time=math.inf):
        if not hasattr(self._motionMode_msg, '_last_update_time'): return None, None
        if self._motionMode_msg._last_update_time + exp_time < time() : raise A8Mini_Controller.DataExpiredError('_motionMode_msg is expired')

        data = {'mode':self._motionMode_msg.mode}
        data_age = time()-self._motionMode_msg._last_update_time
        return data, data_age

    def getMountingDirection(self, exp_time=math.inf):
        if not hasattr(self._mountDir_msg, '_last_update_time'): return None, None
        if self._mountDir_msg._last_update_time + exp_time < time() : raise A8Mini_Controller.DataExpiredError('_mountDir_msg is expired')

        data = {'dir':self._mountDir_msg.dir}
        data_age = time()-self._mountDir_msg._last_update_time
        return data, data_age

    def getFunctionFeedback(self, exp_time=math.inf):
        if not hasattr(self._funcFeedback_msg, '_last_update_time'): return None, None
        if self._funcFeedback_msg._last_update_time + exp_time < time() : raise A8Mini_Controller.DataExpiredError('_funcFeedback_msg is expired')

        data = {'info_type':self._funcFeedback_msg.info_type}
        data_age = time()-self._funcFeedback_msg._last_update_time
        return data, data_age

    # TODO
    # def getZoomLevel(self):
    #     pass
    

def test():
    cam=A8Mini_Controller(debug=False)

    cam.startRecvLoop()
    

    i = 0
    cam.requestGimbalAttitude()
    try:
        while True:
            try: 
                att, data_age = cam.getAttitude(exp_time=0.1)
                if att is None: continue
                print(f'{data_age*1000:.2f} ms, {att}')
            except A8Mini_Controller.DataExpiredError: 
                print('.',end='')
            if i%5==0: cam.requestGimbalAttitude()
            i+=1
            sleep(0.1)
    except KeyboardInterrupt :
        pass
    except Exception as e:
        print(e)
    cam.stopRecvLoop()
    


if __name__=="__main__":
    test()
