from __future__ import division
## Libraries

import serial # Library required for serial declarations and comunicatiaon  
import struct # Library required for managing binary data
import time

##

from  Shimmer3_Serial_Commands import Shimmer3_Serial_Commands

class Shimmer3_sensor(object):

    def __init__(self, Serial_Port=None, Conf=0):

        #Serial Configuration
        self.Serial = serial.Serial() # Declaration of the serial attibute with the serial library
        self.Serial.port = Serial_Port # Serial Port declaration
        #self.Serial.baudrate = 115200 # Serial baud rate declaration
        self.Serial.baudrate = 115200
        self.Serial.bytesize = serial.EIGHTBITS # Serial byte size declaration (8 bits)
        self.Serial.parity = serial.PARITY_NONE # Bit parity declaration 
        self.Serial.stopbits = serial.STOPBITS_ONE # Stop bits declaration 
        self.Serial.timeout = 1 # Serial time out declaration
        self.Serial.xonxoff = False     
        self.Serial.rtscts = False     
        self.Serial.dsrdtr = False       
        self.Serial.writeTimeout = 2

        #Serial Shimmer Values
        self.Allowed_baud_rates = {'115200': 0,
                                   '1200': 1,
                                   '2400': 2,
                                   '4800': 3,
                                   '9600': 4,
                                   '19200': 5,
                                   '38400': 6,
                                   '57600': 7,
                                   '230400': 8,
                                   '460800': 9,
                                   '921600': 10} # The aviable Serial Baud rates in the Shimmer3 module and the byte number for configurating it

        #Serial Shimmer Values
        self.Allowed_accelerometer_ranges = {'2g': 0,
                                             '4g': 1,
                                             '8g': 2,
                                             '16g': 3} # The aviable accelerometer ranges    

        # shimmer hardware parameters
        self.shimmer_code_name = None

        # Shimmer commands configurations
        self.time_wait_write_command = 0.5
        self.streaming_state = False

        # Bluetooth configurations
        self.Bluetooth_serial_str = 'BTHENUM\\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&0045\\8&566DAAE&0&000666DD'

        # Sample frequency configurations
        self.Max_Sample_frequency = 32768

    def shimmer3_COM_aviable(self, show=False):
        import serial.tools.list_ports
        COM_list = list(serial.tools.list_ports.comports())
        Shimer_COM_List = []

        for com_duple in COM_list:
            ref = com_duple[2].find(self.Bluetooth_serial_str)
            if ref != -1:
                Shimer_serial_name = com_duple[2].replace(self.Bluetooth_serial_str,'')
                Shimer_COM_List.append((com_duple[0], Shimer_serial_name))
                
            else:
                if show:
                    print('The port ' + com_duple[0] +  'is not a shimmer bluetooth serial: ' + com_duple[2])
        if show:
            for x in range (0,len(Shimer_COM_List)):
                print('Shimmer' + str(x+1) + ': ' + str(Shimer_COM_List[x]))
        return Shimer_COM_List

    def open_port(self,show=False):

        if self.Serial.isOpen():
            print ('The serial port ' + self.Serial.port + ' is already opened')

        else:
            if self.Serial.port != None:

                Shimer3_serial_list = self.shimmer3_COM_aviable()
                for port in Shimer3_serial_list:
                    if port[0] == self.Serial.port:
                        self.shimmer_code_name = port[1] # See if the associated serial port is related to a shimmer bluetooth device
                        break

                if self.shimmer_code_name == None:
                    print('The Port ' + self.Serial.port + ' is not linked to a shimmer bluetooth device')
                    
                
                try: 
                    self.Serial.open() # Open the serial port according to the "self.serial.port" attribute
                    self.Serial.flushInput() # flush input buffer, discarding all its contents
                    self.Serial.flushOutput()# flush output buffer, aborting current output and discard all that is in buffer
                    if show:
                        print ('The shimmer was successfully connected to the port ' + self.Serial.port)
                    return True
                
                except Exception, e:
                    print ("error at opening serial port: ") + str(e)
                    return False
                    #exit()
            else:
                print('open_port method: The Shimmer3 object does not have associated a serial COM port')

    def close_port(self,show=False):

        if self.Serial.isOpen():
            try: 
                self.Serial.close() # Close searial port
                if show:
                    print ('The shimmer was successfully disconnected from the port ' + self.Serial.port)
                return True
            except Exception, e:
                print ("error at clossing serial port: " + str(e) )
                return False
                #exit()

        else:
            print ('close_port method: The serial port ' + self.Serial.port + ' is NOT opened')

    def read_port(self,Num_frames,show=False):
        # Num_frames => Number of serial Bytes that are required to read

        if self.Serial.isOpen():
            Serial_data = '' # Variable used for saving the received data
            numbytes = 0 # Variable used for determining when all data have been received 

            while numbytes < Num_frames: # Loop to make sure that the whole data will be received
                Serial_data += self.Serial.read(Num_frames) # Function for reading "Num_frames" in the UART RX
                #print(struct.unpack('B'*framesize,ddata))
                numbytes = len(Serial_data) # Update received data length
                
            if show:
                print('Data received: ')
                print(Serial_data)
                
            return Serial_data

        else:
            print ('read_port method: The serial port ' + self.Serial.port + ' is NOT opened')
            return False

    def write_port(self,Data_list,show=False):
        # Data_list => List (or equivalent) which contains the bytes (Data) required to send

        if self.Serial.isOpen():
            self.Serial.flushOutput()
            Data_bin_sent = ''; # Variable used for packaging the Data Bytes
        
            for byte in Data_list: # Loop to package the whole bytes in "Data_list" in order to be sent
                Data_bin_sent += struct.pack('B', byte) # Function for packaging "Data_list" in binary code in bytes ('B' unsigned char) 

            # 
            try:                                                                               
                self.Serial.write(Data_bin_sent) # Function for sending the binary data by the UART TX
                ACK_response = self.wait_for_serial_ack() # Wait for the character 0xff, which is the ACK and determines the end of a data frame
                time.sleep(self.time_wait_write_command) # Wait 'time_wait_write_command' seconds
                if ACK_response:
                    if show:
                        print('Data sent: ')
                        print(struct.unpack('B'*len(Data_bin_sent), Data_bin_sent))
                    return True
                else:
                    print('write_port method: Error at receiving ACK')
                    return False

            except Exception, e:
                print('Error at sending serial Data: ' + str(e))
                return False

        else:
            print ('write_port method: The serial port ' + self.Serial.port + ' is NOT opened')
            return False
        
    def wait_for_serial_ack(self):
        
        if self.Serial.isOpen():
            data = ""
            ack = struct.pack('B', 0xff) 
            while data != ack:
                data = self.Serial.read(1)
                #print(data)

            return True

        else:
            print ('wait_for_serial_ack method: The serial port ' + self.Serial.port + ' is NOT opened')
            return False

# Set methods -- Set methods -- Set methods -- Set methods -- Set methods

    def set_serial_baud_rate(self, baud = '115200', show=False):
        #0 (115200 - default), 1 (1200), 2 (2400), 
        #3 (4800), 4 (9600), 5 (19200), 6 (38400),
        #7 (57600), 8 (230400), 9 (460800) and 10 (921600)

        baud = str(baud) # Change the value of the Baud rate input as a string 
        if self.Allowed_baud_rates.has_key(baud): # Detect if the input baud rate is allowed 

            if self.Serial.isOpen():
                
                self.Serial.write(struct.pack( 'BB', Shimmer3_Serial_Commands['SET_BT_COMMS_BAUD_RATE']
                                                   , self.Allowed_baud_rates[baud] ) ) # Send the command to congurate the serial Baud rate and the corresponding number

                self.Serial.setBaudrate(int(baud)) # Set also the baud rate of the serial communication chanel
                ACK_response = self.wait_for_serial_ack() # Wait for acknowledge response
                time.sleep(self.time_wait_write_command) # Wait 'time_wait_write_command' seconds
                if ACK_response:
                    if show:
                        print('The serial baud rate was configured to ' + str(baud) + 'bps')
                    return True
                else:
                    print ('set_serial_baud_rate method: It was not possible to set the serial baud rate of the simmer (' + self.Serial.port + ')')
                    return False
            
            else:
                print ('set_serial_baud_rate method: The serial port ' + self.Serial.port + ' is NOT opened')
                return False

        else:
            print ('The Baud Rate ' + baud + ' is NOT allowed in the Shimmmer3 module')
            return False

    def set_sample_frequency(self, Freq_Hz=128, show=False):

        Num_frequency = int(self.Max_Sample_frequency/Freq_Hz) # the number required to be sent to the shimmer for setting the sample frequency, it must be devided in two bytes
        Frequency_bytes = self.decimal_2_decimal_list_bytes(Num_frequency) # Getting the two bytes for setting the sample frequency required

        if len(Frequency_bytes) == 1:
            response = sensor.write_port([ Shimmer3_Serial_Commands['SET_SAMPLING_RATE_COMMAND'], Frequency_bytes[0], 0])# Sending the command to set the sample frequency and the two bytes that contains the value
        else:
            response = sensor.write_port([ Shimmer3_Serial_Commands['SET_SAMPLING_RATE_COMMAND'], Frequency_bytes[0], Frequency_bytes[1] ])# Sending the command to set the sample frequency and the two bytes that contains the value
            
        if response:
            if show:
                 print ('The sample frequency was set at ' + str(self.Max_Sample_frequency/Num_frequency) + 'Hz')
            return True
        else:
            print ('It was NOT possible to set the sample frequency at ' + str(Freq_Hz) + 'Hz')
            return False

    def decimal_2_decimal_list_bytes(self, decimal_number, show=False):

        if type(decimal_number) == type(1):
            Num_bytes = int((decimal_number / 256))# Estimating the number or bytes required to represent the decimal number, taking into account that with one byte it is possible to reprsent 256 numbers (0 - 255)
            Hex_presentation = "{0:x}".format(decimal_number) # Getting the Hexadecimal representation of the decimal number, as a string
            Data_list = [] # List which will contain the bytes of the decimal number
            Hex_length = len(Hex_presentation)
        
            if Num_bytes == 0:
                Data_list.append(decimal_number)
            else:
                x=Hex_length
                while x > 1:
                    segment = Hex_presentation[x-2:x]
                    byte = int(segment,16) # estimating the decimal value of the corresponding byte, in order to use the struct library
                    Data_list.append(byte) # inserting the corresponding byte to the list 
                    x = x-2

                if x==1: # checking if there is a last byte that need to be transformed
                    byte = int(Hex_presentation[0],16) 
                    Data_list.append(byte)
            if show:
                print('Decimal recived: ' + str(decimal_number))
                print('Number of bytes required: ' + str(Num_bytes))
                print('Hexadecimal representation: ' + Hex_presentation)
                print('Bytes representation: ' + str(Data_list))
                
            return Data_list
            
        else:
            print ('decimal_2_list_bytes method: The input value must be an integrer number')
            return False

    def disenable_sensors(self, show=False):

        response = self.write_port([ Shimmer3_Serial_Commands['SET_SENSORS_COMMAND'], 0x00, 0x00, 0x00 ]) # Sending the byte command to configure sensors and disnable all sensors with 0x00 bytes
        if response:
            if show:
                print('The sensors were successfully disenabled')
            return True
        else:
            print('disenable_sensors method: It was NOT possible to disenable the sensors')
            return False

    def set_sensors(self, show=False):

        response = self.write_port([ Shimmer3_Serial_Commands['SET_SENSORS_COMMAND'], 0xE0, 0x00, 0x00]) # Sending the byte command to configure sensors and disnable all sensors with 0x00 bytes
        if response:
            if show:
                print('The sensors were successfully enabled')
            return True
        else:
            print('set_sensors method: It was NOT possible set the sensors')
            return False

    def set_accelerometer_range(self, acc_range='16g',show=False):
        # Shimmer 3 are 0 (+/- 2g), 1 (+/- 4g), 2 (+/- 8g) and 3 (+/- 16g)

        if self.Allowed_accelerometer_ranges.has_key(acc_range): # Detect if the input accelerometer range is allowed 

            if self.Serial.isOpen():
                response = self.write_port([ Shimmer3_Serial_Commands['SET_ACCEL_RANGE_COMMAND'],
                                             self.Allowed_accelerometer_ranges[acc_range] ]) # Sending the byte command for configuring the accelerometer range

                if response:
                    if show:
                        print('The serial accelerometer range was configured to ' + acc_range )
                    return True
                else:
                    print ('set_accelerometer_range method: It was not possible to set the accelerometer range of the shimmer (' + self.Serial.port + ')')
                    return False
            
            else:
                print ('set_serial_baud_rate method: The serial port ' + self.Serial.port + ' is NOT opened')
                return False

# Get methods -- Get methods -- Get methods -- Get methods -- Get methods
    def get_sample_frequency(self, show=False):

        if self.streaming_state:
            print ('The shimmer (' + self.Serial.port + ') is currently streaming, it is NOT recomended trying to ask it for configuration parameters')
            return False
        else:
            self.write_port([ Shimmer3_Serial_Commands['GET_SAMPLING_RATE_COMMAND'] ]) # sent the comand to ask for the current sample rate
            sample_rate_bytes_response = self.read_port(3) # read the response of the sensor that is compoused by 3 bytes (sample rate response comand, LOW byte sample rate, HIGH byte sample rate)
            

            if len(sample_rate_bytes_response) == 3:
                sample_rate_response_command = struct.unpack('B', sample_rate_bytes_response[0]) # Separate the bytes 
            
                if sample_rate_response_command[0] == Shimmer3_Serial_Commands['SAMPLING_RATE_RESPONSE']: # check if it is the response of the sample rate
                    sample_rate_div_value = struct.unpack('h', sample_rate_bytes_response[1:3])
                    sample_rate_div_value = sample_rate_div_value[0]
                    sample_rate = self.Max_Sample_frequency/sample_rate_div_value

                    if show:
                        print ('The sample rate gotten of the shimmer (' + self.Serial.port + ') is ' + str(sample_rate) + 'Hz')
                    
                    return sample_rate
                
                else:
                    print('The serial command received is NOT the expected for the sample rate')
                    print('Expected: ' + str(Shimmer3_Serial_Commands['SAMPLING_RATE_RESPONSE']) + '    Received: ' + str(sample_rate_response_command[0]))
                    return False

            else:
                print('It was expectet 3 bytes for the response of the current sample rate, however it was recieved: ' + str(len(sample_rate_response)))
                return False
            
    def get_serial_baud_rate(self, show=False):

        if self.streaming_state:
            print ('The shimmer (' + self.Serial.port + ') is currently streaming, it is NOT recomended trying to ask it for configuration parameters')
            return False
        else:
            self.write_port([ Shimmer3_Serial_Commands['GET_BT_COMMS_BAUD_RATE'] ]) # sent the comand to ask for the current serial baud rate
            baud_rate_bytes_response = self.read_port(2) # read the response of the sensor that is compoused by 2 bytes (serial baud rate response comand, byte serial baud rate)
            

            if len(baud_rate_bytes_response) == 2:
                baud_rate_response_command = struct.unpack('B', baud_rate_bytes_response[0]) # Separate the bytes 
            
                if baud_rate_response_command[0] == Shimmer3_Serial_Commands['BT_COMMS_BAUD_RATE_RESPONSE']: # check if it is the response of the baud rate
                    baud_rate_byte_value = struct.unpack('B', baud_rate_bytes_response[1])
                    baud_rate_byte_value = baud_rate_byte_value[0]

                    baud_rate_list = self.Allowed_baud_rates.keys()
                    baud_rate = None
                    for value_baud_rate in baud_rate_list:
                        if self.Allowed_baud_rates[value_baud_rate] == baud_rate_byte_value:
                            baud_rate = value_baud_rate
                            break

                    if baud_rate == None:
                        print('The byte ' + str(baud_rate_byte_value) + ' is not considered in the allowed baud rates')
                        return False
                    else:
                        if show:
                            print ('The Baud rate gotten of the shimmer (' + self.Serial.port + ') is ' + baud_rate + 'bpm')
                        return baud_rate
                
                else:
                    print('The serial command received is NOT the expected for the baud rate request')
                    print('Expected: ' + str(Shimmer3_Serial_Commands['BT_COMMS_BAUD_RATE_RESPONSE']) + '    Received: ' + str(baud_rate_response_command[0]))
                    return False

            else:
                print('It was expectet 2 bytes for the response of the current baud rate, however it was recieved: ' + str(len(baud_rate_bytes_response)))
                return False

    def get_accelerometer_range(self, show=False):

        if self.streaming_state:
            print ('The shimmer (' + self.Serial.port + ') is currently streaming, it is NOT recomended trying to ask it for configuration parameters')
            return False
        else:
            self.write_port([ Shimmer3_Serial_Commands['GET_ACCEL_RANGE_COMMAND'] ]) # sent the comand to ask for the current accelerometer range
            acc_range_bytes_response = self.read_port(2) # read the response of the sensor that is compoused by 2 bytes (accelerometer range command response, byte accelerometer range)
            

            if len(acc_range_bytes_response) == 2:
                acc_range_response_command = struct.unpack('B', acc_range_bytes_response[0]) # Separate the bytes 
            
                if acc_range_response_command[0] == Shimmer3_Serial_Commands['ACCEL_RANGE_RESPONSE']: # check if it is the response of the baud rate
                    acc_range_byte_value = struct.unpack('B', acc_range_bytes_response[1])
                    acc_range_byte_value = acc_range_byte_value[0]

                    acc_range_list = self.Allowed_accelerometer_ranges.keys()
                    acc_range = None
                    for value_acc_range in acc_range_list:
                        if self.Allowed_accelerometer_ranges[value_acc_range] == acc_range_byte_value:
                            acc_range = value_acc_range
                            break

                    if acc_range == None:
                        print('The byte ' + str(acc_range_byte_value) + ' is not considered in the allowed accelerometers ranges')
                        return False
                    else:
                        if show:
                            print ('The accelerometer range gotten from the shimmer (' + self.Serial.port + ') is ' + acc_range )
                        return acc_range
                
                else:
                    print('The serial command received is NOT the expected for the accelerometer range request')
                    print('Expected: ' + str(Shimmer3_Serial_Commands['ACCEL_RANGE_RESPONSE']) + '    Received: ' + str(acc_range_response_command[0]))
                    return False

            else:
                print('It was expectet 2 bytes for the response of the current accelerometer range, however it was recieved: ' + str(len(acc_range_bytes_response)))
                return False
        
# Streaming methods -- Streaming methods -- Streaming methods -- Streaming methods -- Streaming methods
    def start_streaming(self, show=False):

        if self.streaming_state: # check if the shimmer is in the streaming state
            print('The shimmer (' + self.Serial.port + ') is already streaming')
        else:
            
            if show:
                print('Inicializing streaming Sensor port: ' + self.Serial.port)

            response = self.write_port([ Shimmer3_Serial_Commands['START_STREAMING_COMMAND'] ]) # Send the start treaming serial command
            if response:
                if show:
                    print('The shimmer3 (' +  self.Serial.port + ') has started to stream')
                self.streaming_state = True
                return True
            else:
                print('start_streaming method: It was NOT possible to start the streaming of the shimmer (' + self.Serial.port + ')')
                return False

    def stop_streaming(self, show=False):
        
        if self.streaming_state:
            #response = True
            #sensor.Serial.write(struct.pack('B', 0x20))
            response = self.write_port([ Shimmer3_Serial_Commands['STOP_STREAMING_COMMAND'] ])
            if show:
                print('Finishing streaming Sensor port: ' + self.Serial.port)
        
            if response:

                if show:
                    print('The shimmer3 (' +  self.Serial.port + ') has stoped to stream')
                self.streaming_state = False

                remaining_data = ''
                while self.Serial.inWaiting() > 0:
                    print(self.Serial.inWaiting())
                    remaining_data = remaining_data + self.read_port(self.Serial.inWaiting())
                return remaining_data
            
            else:
                print('start_streaming method: It was NOT possible to stop the streaming of the shimmer (' + self.Serial.port + ')')
                return False
        else:
            print('The Shimmer (' + self.Serial.port + ') is not streaming')
        
# Trasnform values methods -- Trasnform values methods -- Trasnform values methods -- Trasnform values methods

    def map_value(self, value, ini_min, ini_max, end_min, end_max):
        # Figure out how 'wide' each range is
        ini_range = ini_max - ini_min
        end_range = end_max - end_min

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - ini_min) / float(ini_range)

        # Convert the 0-1 range into a value in the right range.
        return end_min + (valueScaled * end_range)

if __name__ == "__main__":
    Sesnor_Data = []
    
    sensor = Shimmer3_sensor('COM18');
    sensor.open_port(show=True)
    #sensor.set_serial_baud_rate(baud='921600',show=True)

    sensor.disenable_sensors(show=True)

    sensor.set_sensors(show=True)

    #sensor.write_port([0x08, 0x00, 0x00, 0x00])
    #sensor.write_port([0x08, 0xE0, 0x00, 0x00])
    #sensor.write_port([0x05, 0x00, 0x19])
    sensor.set_sample_frequency(5.12,show=True)
    sensor.set_accelerometer_range(acc_range='16g',show=True)
    

    print('ini')
    #sensor.write_port([0x07])
    
    sensor.start_streaming(show=True)
    #sensor.start_streaming()
    data_size = 22
    #sensor.write_port([0x20])
    #time.sleep(5)
    #a = sensor.Serial.read(sensor.Serial.inWaiting())
    #sensor.write_port([0x20])
    #sensor.stop_streaming(show=True)
    #sensor.stop_streaming()
    #sensor.Serial.write(struct.pack('B', 0x20))
    #sensor.stop_streaming()
    #print('fin')
     
    print ("TimeStamp: AccelX   AccelY   AccelZ |   GyroX   GyroY   GyroZ |   MagX   MagY   MagZ")
    try:
        while 1:
            
            data = sensor.read_port(data_size)
            data_1 = data[0:data_size]
            data_2 = data[data_size:]
        
            packettype = struct.unpack('B', data[0:1])
            (timestamp0, timestamp1, timestamp2) = struct.unpack('BBB', data[1:4])
            (accelx, accely, accelz, gyrox, gyroy, gyroz, magx, magy, magz) = struct.unpack('HHHHHHhhh', data[4:data_size])
            timestamp = timestamp0 + timestamp1*256 + timestamp2*65536
            Sesnor_Data.append(timestamp)
            print ( "0x%02x,%5d,\t%4f,%4f,%4f,\t%4d,%4d,%4d,\t%4d,%4d,%4d" % (packettype[0], timestamp, sensor.map_value(accelx, 0, (2**12)-1, -2, 2), sensor.map_value(accely, 0, (2**12)-1, -2, 2), sensor.map_value(accelz, 0, (2**12)-1, -2, 2), gyrox, gyroy, gyroz, magx, magy, magz) )
    except KeyboardInterrupt:
        remaining_data = sensor.stop_streaming(show=True)
        #sensor.write_port([0x20])
        #sensor.close_port(show=True)
    print('fin')
        

