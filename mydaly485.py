##############################################################
#                                                            #
# thanks to  https://github.com/dreadnought/python-daly-bms  #
#                                                            #
##############################################################


'''

#python3
#pyserial==3.5
'''

import serial
import struct
import time
import math
import logging
import json
import logging
import sys
import re
import time
from paho.mqtt import client as mqtt_client
import random
import sys
import os

#################### change this
version="mydaly485 v1.0"
# 4 for serial conn
myaddress = 4 
broker = "192.168.2.11"
usethisname = "user"
usethispassword = "pass"
port = 1883

###################
print(version)
print("USE python3 mydaly.py USB0")
print("USB0  daly is /dev/ttyUSB0")
print("A     charging_mos    0=NOT CHANGE 1=ON 2=OFF")
print("B     discharging_mos 0=NOT CHANGE 1=ON 2=OFF")
print("C     set soc         0=NOT CHANGE 1-100=SET")
print("D     BMS restart     0=NOT CHANGE 1=RESTART" )

if (len(sys.argv)==6):
   arg1= str(sys.argv[1])
   arg1=arg1.strip()
else:
   arg1=""

if (arg1 == ""):
   print(version)
   print("USE   python3 mydaly485.py USB0 A B C D")
   print("USB0  daly is /dev/ttyUSB0")
   print("A     charging_mos    0=NOT CHANGE 1=ON 2=OFF")
   print("B     discharging_mos 0=NOT CHANGE 1=ON 2=OFF")
   print("C     set soc         0=NOT CHANGE 1-100=SET")
   print("D     BMS restart     0=NOT CHANGE 1=RESTART" )
   
   print("exit")
   sys.exit(1)

mydevice="/dev/tty"+arg1

esiste=os.system("ls "+ mydevice)

if esiste==0:
    print(mydevice+" OK")
else:
   print(version)
   print("USE python3 mydaly.py USB0")
   print("USB0  daly is /dev/ttyUSB0")
   print("A     charging_mos    0=NOT CHANGE 1=ON 2=OFF")
   print("B     discharging_mos 0=NOT CHANGE 1=ON 2=OFF")
   print("C     set soc         0=NOT CHANGE 1-100=SET")
   print("D     BMS restart     0=NOT CHANGE 1=RESTART" )
   print(" ")
   print("your choice "+mydevice+" is invalid")
   print("exit")
   sys.exit(1)



arg2=str(sys.argv[2])
arg2=arg2.strip()
set_charg_mos=0
if arg2=="1":
   set_charg_mos=1
   print("request: SET charging mos ON")
elif arg2=="2":
   set_charg_mos=2
   print("request: SET charging mos OFF")
else:
   set_charg_mos=0
   print("request: don't change charging mos status")

arg3=str(sys.argv[3])
arg3=arg3.strip()
set_discharg_mos=0
if arg3=="1":
   set_discharg_mos=1
   print("request: SET discharging mos ON")
elif arg3=="2":
   set_discharg_mos=2
   print("request: SET discharging mos OFF")
else:
   set_discharg_mos=0
   print("request: don't change discharging mos status")

arg4=str(sys.argv[4])
arg4=arg4.strip()
set_soc=0
if arg4.isdigit():
   set_soc=int(arg4)
else:
   set_soc=0

if set_soc>100:
   print("request error: SET soc MAX 100") 
   set_soc=0

if set_soc>0:
   print("request: SET soc "+str(set_soc)+"%")
else:
   print("request: don't change soc value")    

arg5=str(sys.argv[5])
arg5=arg5.strip()
set_reset=0
if arg5=="1":
   set_reset=1
   print("request: RESET BMS")
else:
   set_reset=0
   print("request: don't RESET BMS")






myretries=3
log_format = '%(levelname)-8s [%(filename)s:%(lineno)d] %(message)s'
level = logging.ERROR
logging.basicConfig(level=level, format=log_format, datefmt='%H:%M:%S')
logger = logging.getLogger()



ERROR_CODES = {
    0: [
        "one stage warning of unit over voltage",
        "one stage warning of unit over voltage",
        "one stage warning of unit over voltage",
        "two stage warning of unit over voltage",
        "Total voltage is too high One alarm",
        "Total voltage is too high Level two alarm",
        "Total voltage is too low One alarm",
        "Total voltage is too low Level two alarm"
    ],
    1: ["Charging temperature too high. One alarm",
        "Charging temperature too high. Level two alarm",
        "Charging temperature too low. One alarm",
        "Charging temperature's too low. Level two alarm",
        "Discharge temperature is too high. One alarm",
        "Discharge temperature is too high. Level two alarm",
        "Discharge temperature is too low. One alarm",
        "Discharge temperature is too low. Level two alarm",
        ],
    2: ["Charge over current. Level one alarm",
        "Charge over current, level two alarm",
        "Discharge over current. Level one alarm",
        "Discharge overcurrent, level two alarm",
        "SOC is too high an alarm",
        "SOC is too high. Alarm Two",
        "SOC is too low. level one alarm",
        "SOC is too low. level two alarm",
        ],
3: ["Excessive differential pressure level one alarm",
        "Excessive differential pressure level two alarm",
        "Excessive temperature difference level one alarm",
        "Excessive temperature difference level two alarm",
        ],
4: ["charging  MOS overtemperature warning",
        "discharge MOS overtemperature warning",
        "charging MOS temperature detection sensor failure",
        "discharge MOS temperature detection sensor failure",
        "charging MOS adhesion failure",
        "discharge MOS adhesion failure",
        "charging MOS breaker failure",
        "discharge MOS breaker failure",
        ],
5: ["AFE acquisition chip malfunction",
        "monomer collect drop off",
        "Single Temperature Sensor Fault",
        "EEPROM storage failures",
        "RTC clock malfunction",
        "Precharge Failure",
        "vehicle communications malfunction",
        "intranet communication module malfunction",
        ],
6: ["Current Module Failure",
        "main pressure detection module",
        "Short circuit protection failure",
        "Low Voltage No Charging",
        "RESERVED",
        "RESERVED",
        "RESERVED",
        "RESERVED",
        ],
}



class DalyBMS:
    def __init__(self, request_retries=3, address=4, logger=None):
        """

        :param request_retries: How often read requests should get repeated in case that they fail (Default: 3).
        :param address: Source address for commands sent to the BMS (4 for RS485, 8 for UART/Bluetooth)
        :param logger: Python Logger object for output (Default: None)
        """
        self.status = None
        if logger:
            self.logger = logger
        else:
            self.logger = logging.getLogger(__name__)
        self.request_retries = request_retries
        self.address = address  # 4 = USB, 8 = Bluetooth

    def connect(self, device):
        """
        Connect to a serial device

        :param device: Serial device, e.g. /dev/ttyUSB0
        """
        self.serial = serial.Serial(
            port=device,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5,
            xonxoff=False,
            writeTimeout=0.5
        )
        self.get_status()

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

    @staticmethod
    def _calc_crc(message_bytes):
        """
        Calculate the checksum of a message

        :param message_bytes: Bytes for which the checksum should get calculated
        :return: Checksum as bytes
        """
        return bytes([sum(message_bytes) & 0xFF])

    def _format_message(self, command, extra=""):
        """
        Takes the command ID and formats a request message

        :param command: Command ID ("90" - "98")
        :return: Request message as bytes
        """
        # 95 -> a58095080000000000000000c2
        message = "a5%i0%s08%s" % (self.address, command, extra)
        message = message.ljust(24, "0")
        message_bytes = bytearray.fromhex(message)
        message_bytes += self._calc_crc(message_bytes)
        self.logger.debug("w %s" % message_bytes.hex())
        return message_bytes

    def _read_request(self, command, extra="", max_responses=1, return_list=False):
        """
        Sends a read request to the BMS and reads the response. In case it fails, it retries 'max_responses' times.

        :param command: Command ID ("90" - "98")
        :param max_responses: For how many response packages it should wait (Default: 1).
        :return: Request message as bytes or False
        """
        response_data = None
        x = None
        for x in range(0, self.request_retries):
            response_data = self._read(
                command=command,
                extra=extra,
                max_responses=max_responses,
                return_list=return_list)
            if not response_data:
                self.logger.debug("%x. try failed, retrying..." % (x + 1))
                time.sleep(0.2)
            else:
                break
        if not response_data:
            self.logger.error('%s failed after %s tries' % (command, x + 1))
            return False
        return response_data

    def _read(self, command, extra="", max_responses=1, return_list=False):
        self.logger.debug("-- %s ------------------------" % command)
        if not self.serial.is_open:
            self.serial.open()
        message_bytes = self._format_message(command, extra=extra)

        # clear all buffers, in case something is left from a previous command that failed
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        if not self.serial.write(message_bytes):
            self.logger.error("serial write failed for command" % command)
            return False
        x = 0
        response_data = []
        while True:
            b = self.serial.read(13)
            if len(b) == 0:
                self.logger.debug("%i empty response for command %s" % (x, command))
                break
            self.logger.debug("%i %s %s" % (x, b.hex(), len(b)))
            x += 1
            response_crc = self._calc_crc(b[:-1])
            if response_crc != b[-1:]:
                self.logger.debug("response crc mismatch: %s != %s" % (response_crc.hex(), b[-1:].hex()))
            header = b[0:4].hex()
            # todo: verify  more header fields
            if header[4:6] != command:
                self.logger.debug("invalid header %s: wrong command (%s != %s)" % (header, header[4:6], command))
                continue
            data = b[4:-1]
            response_data.append(data)
            if x == max_responses:
                break

        if return_list or len(response_data) > 1:
            return response_data
        elif len(response_data) == 1:
            return response_data[0]
        else:
            return False

    def get_soc(self, response_data=None):
        # SOC of Total Voltage Current
        if not response_data:
            response_data = self._read_request("90")
        if not response_data:
            return False

        parts = struct.unpack('>h h h h', response_data)
        
        mymsg2='"total_voltage": "'+str( parts[0] / 10 )+'", '
        mymsg2=mymsg2+'"current": "'+str( ( parts[2] - 30000) / 10 )+'", '
        mymsg2=mymsg2+'"soc_percent": "'+str( parts[3] / 10 )+'", '
        return mymsg2
        '''
        data = {
            "total_voltage": parts[0] / 10,
            # "x_voltage": parts[1] / 10, # always 0
            "current": (parts[2] - 30000) / 10,  # negative=charging, positive=discharging
            "soc_percent": parts[3] / 10
        }
        return data,mymsg2
        '''
    def get_cell_voltage_range(self, response_data=None):
        # Cells with the maximum and minimum voltage
        if not response_data:
            response_data = self._read_request("91")
        if not response_data:
            return False

        parts = struct.unpack('>h b h b 2x', response_data)
        
        mymsg2='"highest_voltage": "'+str( parts[0] / 1000 )+'", '
        mymsg2=mymsg2+'"highest_cell": "'+str( parts[1] )+'", '
        mymsg2=mymsg2+'"lowest_voltage": "'+str( ( parts[2] ) / 1000 )+'", '
        mymsg2=mymsg2+'"lowest_cell": "'+str( parts[3] )+'", '
        mymsg2=mymsg2+'"volt_diff": "'+str( (int(parts[0])-int(parts[2])) /1000 )+'", '
        return mymsg2
        '''
        data = {
            "highest_voltage": parts[0] / 1000,
            "highest_cell": parts[1],
            "lowest_voltage": parts[2] / 1000,
            "lowest_cell": parts[3],
            "volt_diff": (int(parts[0])-int(parts[2]))/1000,
        }
        return data
        ''' 
    def get_temperature_range(self, response_data=None):
        # Temperature in degrees celsius
        if not response_data:
            response_data = self._read_request("92")
        if not response_data:
            return False
        parts = struct.unpack('>b b b b 4x', response_data)
        
        mymsg2='"highest_temperature": "'+str( parts[0] -40 )+'", '
        mymsg2=mymsg2+'"highest_sensor": "'+str( parts[1] )+'", '
        mymsg2=mymsg2+'"lowest_temperature": "'+str( parts[2] -40 )+'", '
        mymsg2=mymsg2+'"lowest_sensor": "'+str( parts[3] )+'", '
        return mymsg2
        '''
        data = {
            "highest_temperature": parts[0] - 40,
            "highest_sensor": parts[1],
            "lowest_temperature": parts[2] - 40,
            "lowest_sensor": parts[3],
        }
        return data
        '''
    def get_mosfet_status(self, response_data=None):
        # Charge/discharge, MOS status
        if not response_data:
            response_data = self._read_request("93")
        if not response_data:
            return False
        # todo: implement
        self.logger.debug(response_data.hex())

        parts = struct.unpack('>b ? ? B l', response_data)

        if parts[0] == 0:
            mode = "stationary"
        elif parts[0] == 1:
            mode = "charging"
        else:
            mode = "discharging"

        
        mymsg2='"mode": "'+str( mode )+'", '
        mymsg2=mymsg2+'"charging_mosfet": "'+str( parts[1] )+'", '
        mymsg2=mymsg2+'"discharging_mosfet": "'+str( parts[2] )+'", '
        #mymsg2=mymsg2+'"bms_cycles": "'+str( parts[3] )+'", '  # unstable result
        mymsg2=mymsg2+'"capacity_ah": "'+str( parts[4] / 1000 )+'", '
        return mymsg2
        '''
        data = {
            "mode": mode,
            "charging_mosfet": parts[1],
            "discharging_mosfet": parts[2],
            # "bms_cycles": parts[3], unstable result
            "capacity_ah": parts[4] / 1000,
        }
        return data
        '''
    def get_status(self, response_data=None):
        if not response_data:
            response_data = self._read_request("94")
        if not response_data:
            return False

        parts = struct.unpack('>b b ? ? b h x', response_data)
        state_bits = bin(parts[4])[2:]
        state_names = ["DI1", "DI2", "DI3", "DI4", "DO1", "DO2", "DO3", "DO4"]
        states = {}
        state_index = 0
        ele_sta=""
        for bit in reversed(state_bits):
            if len(state_bits) == state_index:
                break
            states[state_names[state_index]] = bool(int(bit))
            ele_sta+='"'+str(state_names[state_index])+'": "'+str(bool(int(bit)))+'", '
            state_index += 1
            
        data = {
            "cells": parts[0],  # number of cells
            "temperature_sensors": parts[1],  # number of sensors
            "charger_running": parts[2],
            "load_running": parts[3],
            # "state_bits": state_bits,
            "states": states,
            "cycles": parts[5],  # number of charge/discharge cycles
        }
        self.status = data
        mymsg2='"cells": "'+str( parts[0] )+'", '
        mymsg2=mymsg2+'"temperature_sensors": "'+str( parts[1] )+'", '
        mymsg2=mymsg2+'"charger_running": "'+str( parts[2] )+'", '
        mymsg2=mymsg2+'"load_running": "'+str( parts[3] )+'", '
        mymsg2=mymsg2+str(ele_sta)
        mymsg2=mymsg2+'"cycles": "'+str( parts[5] )+'", '
        return mymsg2
        #return data

    def _calc_num_responses(self, status_field, num_per_frame):
        if not self.status:
            self.logger.error("get_status has to be called at least once before calling get_cell_voltages")
            return False

        # each response message includes 3 cell voltages
        if self.address == 8:
            # via Bluetooth the BMS returns all frames, even when they don't have data
            if status_field == 'cell_voltages':
                max_responses = 16
            elif status_field == 'temperatures':
                max_responses = 3
            else:
                self.logger.error("unkonwn status_field %s" % status_field)
                return False
        else:
            # via UART/USB the BMS returns only frames that have data
            max_responses = math.ceil(self.status[status_field] / num_per_frame)
        return max_responses

    def _split_frames(self, response_data, status_field, structure):
        values = {}
        x = 1
        for response_bytes in response_data:
            parts = struct.unpack(structure, response_bytes)
            if parts[0] != x:
                self.logger.warning("frame out of order, expected %i, got %i" % (x, response_bytes[0]))
                continue
            for value in parts[1:]:
                values[len(values) + 1] = value
                if len(values) == self.status[status_field]:
                    return values
            x += 1

    def get_cell_voltages(self, response_data=None):
        if not response_data:
            max_responses = self._calc_num_responses(status_field="cells", num_per_frame=3)
            if not max_responses:
                return
            response_data = self._read_request("95", max_responses=max_responses, return_list=True)
        if not response_data:
            return False

        cell_voltages = self._split_frames(response_data=response_data, status_field="cells", structure=">b 3h x")
        
        ele_sta=""
        for id in cell_voltages:
            cell_voltages[id] = cell_voltages[id] / 1000
            if id<10:
               ele_sta+='"V0'+str(id)+'": "'+str( cell_voltages[id] )+'", '
            else:
               ele_sta+='"V'+str(id)+'": "'+str( cell_voltages[id] )+'", '
        #return cell_voltages
        return ele_sta

    def get_temperatures(self, response_data=None):
        # Sensor temperatures
        if not response_data:
            max_responses = self._calc_num_responses(status_field="temperature_sensors", num_per_frame=7)
            if not max_responses:
                return
            response_data = self._read_request("96", max_responses=max_responses, return_list=True)
        if not response_data:
            return False

        temperatures = self._split_frames(response_data=response_data, status_field="temperature_sensors", structure=">b 7b")
        ele_sta=""
        for id in temperatures:
            temperatures[id] = temperatures[id] - 40
            if id<10:
              ele_sta+='"T0'+str(id)+'": "'+str( temperatures[id] )+'", '
            else:  
              ele_sta+='"T'+str(id)+'": "'+str( temperatures[id] )+'", '
        #return temperatures
        return ele_sta

    def get_balancing_status(self, response_data=None):
        # Cell balancing status
        if not response_data:
            response_data = self._read_request("97")
        if not response_data:
            return False
        self.logger.info(response_data.hex())
        bits = bin(int(response_data.hex(), base=16))[2:].zfill(48)
        self.logger.info(bits)
        cells = {}
        ele_sta=""
        for cell in range(1, self.status["cells"] + 1):
            cells[cell] = bool(int(bits[cell * -1]))
            if cell<10:
              ele_sta+='"B0'+str(cell)+'": "'+str( bool(int(bits[cell * -1])) )+'", '
            else:
              ele_sta+='"B'+str(cell)+'": "'+str( bool(int(bits[cell * -1])) )+'", '        
        
        self.logger.info(cells)
        # todo: get sample data and verify result
        #return {"error": "not implemented"}
        return ele_sta

    def get_errors(self, response_data=None):
        # Battery failure status
        if not response_data:
            response_data = self._read_request("98")
        if int.from_bytes(response_data, byteorder='big') == 0:
            return []

        byte_index = 0
        errors = []
        for b in response_data:
            if b == 0:
                byte_index += 1
                continue
            bits = bin(b)[2:]
            bit_index = 0
            for bit in reversed(bits):
                if bit == "1":
                    errors.append(ERROR_CODES[byte_index][bit_index])

                bit_index += 1

            self.logger.debug("%s %s %s" % (byte_index, b, bits))
            byte_index += 1
        return errors

    def get_all(self):
        mymsg=""
        timestamp=int(time.time())
        mymsg='{"timestamp": "'+str(timestamp)+'", '
        mymsg=mymsg+self.get_soc()
        mymsg=mymsg+self.get_cell_voltage_range()
        mymsg=mymsg+self.get_temperature_range()
        mymsg=mymsg+self.get_mosfet_status()
        mymsg=mymsg+self.get_status()
        mymsg=mymsg+self.get_cell_voltages()
        mymsg=mymsg+self.get_temperatures()
        mymsg=mymsg+self.get_balancing_status()        
        mymsg2,mystr0=self.get_batx()
        mymsg=mymsg+mymsg2
        mymsg2=str(self.get_errors())
        mymsg2=mymsg2.strip()
        mymsg2 = re.sub(r"\[", "", mymsg2)
        mymsg2 = re.sub(r"\]", "", mymsg2)
        if mymsg2=="":
           mymsg2="nn"
        
        mymsg=mymsg+'"Errors": "'+mymsg2+'"}'
        return mymsg,mystr0
        '''
        return {
            "soc": self.get_soc(),
            "cell_voltage_range": self.get_cell_voltage_range(),
            "temperature_range": self.get_temperature_range(),
            "mosfet_status": self.get_mosfet_status(),
            "status": self.get_status(),
            "cell_voltages": self.get_cell_voltages(),
            "temperatures": self.get_temperatures(),
            "balancing_status": self.get_balancing_status(),
            "batt":self.get_batx(),
            "errors": self.get_errors()
        }
        ''' 
    def set_charge_mosfet(self, on=True, response_data=None):
        if on:
            extra = "01"
        else:
            extra = "00"
        if not response_data:
            response_data = self._read_request("da", extra=extra)
        if not response_data:
            return False
        self.logger.info(response_data.hex())
        # on response
        # 0101000002006cbe
        # off response
        # 0001000002006c44

    def set_discharge_mosfet(self, on=True, response_data=None):
        if on:
            extra = "01"
        else:
            extra = "00"
        if not response_data:
            response_data = self._read_request("d9", extra=extra)
        if not response_data:
            return False
        self.logger.info(response_data.hex())
        # on response
        # 0101000002006cbe
        # off response
        # 0001000002006c44


    # Set SoC. Value is float from 0.0 to 100.0
    def set_soc(self, value):
        v = round(value*10.0)
        if v > 1000 : v = 1000
        if v < 0 : v = 0
        extra='000000000000%0.4X' % v
        response_data = self._read_request("21", extra=extra)
        self.logger.info(response_data.hex())

    def restart(self, response_data=None):
        response_data = self._read("00","",1,False)
        
        
    def get_batx(self, response_data=None):
        # bat A of Total Voltage Current
        if not response_data:
            response_data = self._read_request("54")
        if not response_data:
            mystr = "nn"

        #b'20231107'
        mystr=response_data.decode()
        mystr1=mystr.strip()
     
        response_data=None
        if not response_data:
            max_responses = 2
            if not max_responses:
                return
            response_data = self._read_request("63", max_responses=max_responses, return_list=True)
        if not response_data:
            mystr="nn"
        
        mystr=str(response_data[0].decode())+str(response_data[1].decode())
        mystr=mystr.strip()
        #[b'\x01BMS-ST1', b'\x0203-303E']
        mystr = re.sub(r"\[", "", mystr)
        mystr = re.sub(r"\]", "", mystr)
        mystr = re.sub(r"'b", "", mystr)
        mystr = re.sub(r"'", "", mystr)
        mystr = re.sub(r"\x01", "", mystr)
        mystr2 = re.sub(r"\x02", "", mystr)
        
        response_data=None
        if not response_data:
            max_responses = 2
            if not max_responses:
                return
            response_data = self._read_request("62", max_responses=max_responses, return_list=True)
        if not response_data:
            mystr="nn"
        mystr=str(response_data[0].decode())+str(response_data[1].decode())
        mystr=mystr.strip()
        #[b'\x0112_2310', b'\x0225_001T']
        mystr = re.sub(r"\[", "", mystr)
        mystr = re.sub(r"\]", "", mystr)
        mystr = re.sub(r"'b", "", mystr)
        mystr = re.sub(r"'", "", mystr)
        mystr = re.sub(r"\x01", "", mystr)
        mystr3 = re.sub(r"\x02", "", mystr)        
        
        mymsg2='"bat_serial": "'+str( mystr1 )+'", '
        mymsg2=mymsg2+'"bat_hard_v": "'+str( mystr2 )+'", '
        mymsg2=mymsg2+'"bat_soft_v": "'+str( mystr3 )+'", '
        
        mystr0=str( mystr1 )+"_"+str( mystr2 )+"_"+str( mystr3 )
        
        return mymsg2,mystr0 
        '''
        data = {
            "bat_serial": mystr1,
            "bat_hard_v": mystr2,
            "bat_soft_v": mystr3,
        }
        return data
        '''
        
###################  INIZIO

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected to MQTT Broker with result: {rc}")
        client.publish(topic, msg)
        print (topic)
        print (msg)
        client.disconnect()
        print()
        print("========================================")
        print(f"Gracefully disconnected from MQTT Broker")
        time.sleep(sleeptime)
    else:
        print("Failed to connect to Broker, return code = ", rc)


def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected MQTT Broker disconnection!")
        time.sleep(1)



       
bms = DalyBMS(request_retries=myretries, address=myaddress, logger=logger)
bms.connect(device=mydevice)
result = False
result,mystr0 = bms.get_all()        

print(result)
print(mystr0)


topic="D_"+mystr0+"/status"
msg=str(result)
#sleeptime serve per mqtt
sleeptime = 1
client_id = f'python-mqtt-{random.randint(0, 1000)}'
client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, client_id)

#client.username_pw_set(username=usethisname, password=usethispassword)

client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.connect(broker, port, 60)



if set_charg_mos==1:
   print ("set charge mosfet=ON")
   time.sleep(2)
   on = True
   result = False
   result = bms.set_charge_mosfet(on=on)

if set_charg_mos==2:
   print ("set charge mosfet=OFF")
   time.sleep(2)
   on = False
   result = False
   result = bms.set_charge_mosfet(on=on)

if set_discharg_mos==1:
   print ("set discharge mosfet=ON")
   time.sleep(2)
   on = True
   result = False
   result = bms.set_discharge_mosfet(on=on)
   print(result)

if set_discharg_mos==2:
   print ("set discharge mosfet=OFF")
   time.sleep(2)
   on = False
   result = False
   result = bms.set_discharge_mosfet(on=on)
   print(result)

if set_soc>0:
   print ("set soc="+str(set_soc)+"%")
   time.sleep(2)
   v=set_soc
   result = False
   result = bms.set_soc(v)
   print(result)

if set_reset==1:
   print ("reset BMS")
   time.sleep(2)
   result = False
   result = bms.restart()
   print(result)




if __name__ == "__main__":
    client.loop_forever()