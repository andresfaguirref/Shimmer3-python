

Shimmer3_Serial_Commands = {

    'ACK_RESPONSE'                      : 255, # The acknowledgemnt byte is the first byte value returned from the Shimmer after each command the Shimmer receives
    'DATA_PACKET_START_BYTE'            : 0, # First byte value in a Data Packet
    'INQUIRY_COMMAND'                   : 0x01, #char(1)  # Inquiry command value sent to the shimmer in order to receive an inquiry response
    'INQUIRY_RESPONSE'                  : 0x02, #char(2) # First byte value in the Inquiry Response packet
    'GET_SAMPLING_RATE_COMMAND'         : 0x03, #char(3) # Get sampling rate command sent to the shimmer in order to receive the sampling rate response
    'SAMPLING_RATE_RESPONSE'            : 0x04, #char(4) # First byte value received from the shimmer in the sampling rate response, it is followed by the byte value defining the setting
    'SET_SAMPLING_RATE_COMMAND'         : 0x05, #char(5) # First byte sent to the shimmer when implementing a set sampling rate operation, it is followed by the byte value defining the setting
    'TOGGLE_LED_COMMAND'                : 0x06, #char(6) # This byte value is sent in order to toggle the red LED
    'START_STREAMING_COMMAND'           : 0x07, #char(7) # This byte value is sent in order to start data streaming from the Shimmer
    'SET_SENSORS_COMMAND'               : 0x08, #char(8) # First byte sent to the shimmer when implementing a set enabled sensors operation, it is followed by the 2 byte values defining the setting
    'SET_ACCEL_RANGE_COMMAND'           : 0x09, #char(9) # First byte sent to the shimmer when implementing a set accelerometer range operation, it is followed by the byte value defining the setting
    'ACCEL_RANGE_RESPONSE'              : 0x0A, #char(10) # First byte value received from the shimmer in the accel range response, it is followed by the byte value defining the setting
    'GET_ACCEL_RANGE_COMMAND'           : 0x0B, #char(11) # Get accelerometer range command sent to the shimmer in order to receive the accelerometer range response
    'SET_5V_REG_COMMAND'                : 0x0C, #char(12); # First byte sent to the shimmer when implementing a set 5 volt Regulator operation, it is followed by the bit value defining the setting
    'SET_PMUX_COMMAND'                  : 0x0D, #char(13) # First byte sent to the shimmer when implementing a set PMux operation, it is followed by the bit value defining the setting
    'SET_CONFIG_BYTE0_COMMAND'          : 0x0E, #char(14) # First byte sent to the shimmer when implementing a set config byte0 operation, it is followed by the byte value defining the setting
    'CONFIG_BYTE0_RESPONSE'             : 0x0F, #char(15); # First byte value received from the shimmer in the config byte0 response, it is followed by the byte value defining the setting
    'GET_CONFIG_BYTE0_COMMAND'          : 0x10, #char(16) # Get config byte0 command sent to the shimmer in order to receive the config byte0 response (Get config bytes byte0, byte1, byte2, byte3 For Shimmer3.) 

    'STOP_STREAMING_COMMAND'            : 0x20, #char(32) # This byte value is sent in order to stop data streaming from the Shimmer
    'SET_GSR_RANGE_COMMAND'             : 0x21, #char(33) # First byte sent to the shimmer when implementing a set gsr range operation, it is followed by the byte value defining the setting
    'GSR_RANGE_RESPONSE'                : 0x22, #char(34) # First byte value received from the shimmer in the config byte0 response, it is followed by the byte value defining the setting
    'GET_GSR_RANGE_COMMAND'             : 0x23, #char(35) # Get gsr range command sent to the shimmer in order to receive the gsr range response
    'GET_SHIMMER_VERSION_COMMAND'       : 0x24, #char(36) # Get shimmer version command sent to the shimmer in order to receive the shimmer version response
    'SHIMMER_VERSION_RESPONSE'          : 0x25, #char(37) # First byte value received from the shimmer in the shimmer version response, it is followed by the byte value defining the setting

    'SET_EMG_CALIBRATION_COMMAND'      : 0x26, #char(38)
    'EMG_CALIBRATION_RESPONSE'         : 0x27, #char(39)
    'GET_EMG_CALIBRATION_COMMAND'      : 0x28, #char(40)
    'SET_ECG_CALIBRATION_COMMAND'      : 0x29, #char(41)
    'ECG_CALIBRATION_RESPONSE'         : 0x2A, #char(42)
    'GET_ECG_CALIBRATION_COMMAND'      : 0x2B, #char(43)
    'GET_ALL_CALIBRATION_COMMAND'      : 0x2C, #char(44)
    'ALL_CALIBRATION_RESPONSE'         : 0x2D, #char(45)
    'GET_FW_VERSION_COMMAND'           : 0x2E, #char(46)
    'FW_VERSION_RESPONSE'              : 0x2F, #char(47)
    'SET_BLINK_LED'                    : 0x30, #char(48)
    'BLINK_LED_RESPONSE'               : 0x31, #char(49)
    'GET_BLINK_LED'                    : 0x32, #char(50)
    'SET_GYRO_TEMP_VREF_COMMAND'       : 0x33, #char(51)
    'SET_BUFFER_SIZE_COMMAND'          : 0x34, #char(52)
    'BUFFER_SIZE_RESPONSE'             : 0x35, #char(53)
    'GET_BUFFER_SIZE_COMMAND'          : 0x36, #char(54)
    
    'SET_MAG_GAIN_COMMAND'                        : 0x37, #char(hex2dec('37'))
    'MAG_GAIN_RESPONSE'                           : 0x38, #char(hex2dec('38'))
    'GET_MAG_GAIN_COMMAND'                        : 0x39, #char(hex2dec('39'))
    'SET_MAG_SAMPLING_RATE_COMMAND'               : 0x3A, #char(hex2dec('3A'))
    'MAG_SAMPLING_RATE_RESPONSE'                  : 0x3B, #char(hex2dec('3B'))
    'GET_MAG_SAMPLING_RATE_COMMAND'               : 0x3C, #char(hex2dec('3C')) 

    'GET_SHIMMER_VERSION_COMMAND_NEW'               : 0x3F, #char(hex2dec('3F')) 
    'SET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND'    : 0x40, #char(hex2dec('40'))   
    'LSM303DLHC_ACCEL_SAMPLING_RATE_RESPONSE'	    : 0x41, #char(hex2dec('41'))  
    'GET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND'    : 0x42, #char(hex2dec('42')) 
    'SET_LSM303DLHC_ACCEL_LPMODE_COMMAND'	    : 0x43, #char(hex2dec('43'))         
    'LSM303DLHC_ACCEL_LPMODE_RESPONSE'              : 0x44, #char(hex2dec('44'))            

    'GET_LSM303DLHC_ACCEL_LPMODE_COMMAND'	        : 0x45, #char(hex2dec('45'))        
    'SET_LSM303DLHC_ACCEL_HRMODE_COMMAND'	        : 0x46, #char(hex2dec('46'))         
    'LSM303DLHC_ACCEL_HRMODE_RESPONSE'	        : 0x47, #char(hex2dec('47'))         
    'GET_LSM303DLHC_ACCEL_HRMODE_COMMAND'	        : 0x48, #char(hex2dec('48'))         
    'SET_MPU9150_GYRO_RANGE_COMMAND'	        : 0x49, #char(hex2dec('49'))
    'MPU9150_GYRO_RANGE_RESPONSE'                 : 0x4A, #char(hex2dec('4A'))
    'GET_MPU9150_GYRO_RANGE_COMMAND'              : 0x4B, #char(hex2dec('4B'))
    'SET_MPU9150_SAMPLING_RATE_COMMAND'           : 0x4C, #char(hex2dec('4C'))
    'MPU9150_SAMPLING_RATE_RESPONSE'              : 0x4D, #char(hex2dec('4D'))
    'GET_MPU9150_SAMPLING_RATE_COMMAND'           : 0x4E, #char(hex2dec('4E'))

    'SET_BMP180_PRES_RESOLUTION_COMMAND'          : 0x52, #char(hex2dec('52'))
    'BMP180_PRES_RESOLUTION_RESPONSE'             : 0x53, #char(hex2dec('53'))
    'GET_BMP180_PRES_RESOLUTION_COMMAND'          : 0x54, #char(hex2dec('54'))
    #'SET_BMP180_PRES_CALIBRATION_COMMAND'	: 0x55, char(hex2dec('55'))
    #'BMP180_PRES_CALIBRATION_RESPONSE'           : 0x56, char(hex2dec('56'))
    #'GET_BMP180_PRES_CALIBRATION_COMMAND'        : 0x57, char(hex2dec('57'))
    'BMP180_CALIBRATION_COEFFICIENTS_RESPONSE'    : 0x58, #char(hex2dec('58'))
    'GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND' : 0x59, #char(hex2dec('59'))
    
    'BMP280_CALIBRATION_COEFFICIENTS_RESPONSE'    : 0x9F, #char(hex2dec('9F'))
    'GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND' : 0xA0, #char(hex2dec('A0'))

    'RESET_TO_DEFAULT_CONFIGURATION_COMMAND'      : 0x5A, #char(hex2dec('5A'))
    'RESET_CALIBRATION_VALUE_COMMAND'             : 0x5B, #char(hex2dec('5B'))
    'MPU9150_MAG_SENS_ADJ_VALS_RESPONSE'          : 0x5C, #char(hex2dec('5C'))
    'GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND'       : 0x5D, #char(hex2dec('5D'))
    'SET_INTERNAL_EXP_POWER_ENABLE_COMMAND'       : 0x5E, #char(hex2dec('5E'))
    'INTERNAL_EXP_POWER_ENABLE_RESPONSE'          : 0x5F, #char(hex2dec('5F'))
    'GET_INTERNAL_EXP_POWER_ENABLE_COMMAND'       : 0x60, #char(hex2dec('60'))
    'SET_EXG_REGS_COMMAND'                        : 0x61, #char(hex2dec('61'))
    'EXG_REGS_RESPONSE'                           : 0x62, #char(hex2dec('62'))
    'GET_EXG_REGS_COMMAND'                        : 0x63, #char(hex2dec('63'))
    
    'DAUGHTER_CARD_ID_RESPONSE'                   : 0x65, #char(hex2dec('65'))
    'GET_DAUGHTER_CARD_ID_COMMAND'                : 0x66, #char(hex2dec('66'))

    'SET_BT_COMMS_BAUD_RATE'                      : 0x6A, #char(hex2dec('6A'))
    'BT_COMMS_BAUD_RATE_RESPONSE'                 : 0x6B, #char(hex2dec('6B'))
    'GET_BT_COMMS_BAUD_RATE'                      : 0x6C, #char(hex2dec('6C'))
        
    'START_SDBT_COMMAND'                : 0x70, #char(hex2dec('70'))
    'STATUS_RESPONSE'                   : 0x71, #char(hex2dec('71'))
    'GET_STATUS_COMMAND'                : 0x72, #char(hex2dec('72'))
    
    'DIR_RESPONSE'                      : 0x88, #char(hex2dec('88'))
    'GET_DIR_COMMAND'                   : 0x89, #char(hex2dec('89'))
    'INSTREAM_CMD_RESPONSE'             : 0x8A, #char(hex2dec('8A'))
        
    'SET_RWC_COMMAND'                 : 0x8F, #char(hex2dec('8F'))
    
    'GET_RWC_COMMAND'                 : 0x91, #char(hex2dec('91'))
    'RWC_RESPONSE'                    : 0x90, #char(hex2dec('90'))
        
    'START_LOGGING_ONLY_COMMAND'      : 0x92, #char(hex2dec('92'))
    'STOP_LOGGING_ONLY_COMMAND'       : 0x93, #char(hex2dec('93'))
    
    'STOP_SDBT_COMMAND'             : 0x97, #char(hex2dec('97'))

    'GET_VBATT_COMMAND'               : 0x95, #char(hex2dec('95'))
    'VBATT_RESPONSE'              : 0x94, #char(hex2dec('94'))
        
    'SET_VBATT_FREQ_COMMAND'          : 0x98, #char(hex2dec('98'))
                 
                                  
    'GET_ACCEL_CALIBRATION_PARAMETERS_COMMAND'    : 0x19, #char(19)# Command to get accelerometer calibration parameters from the shimmer device
    'GET_GYRO_CALIBRATION_PARAMETERS_COMMAND'     : 0x22, #char(22) # Command to get gyroscope calibration parameters from the shimmer device
    'GET_MAGNE_CALIBRATION_PARAMETERS_COMMAND'    : 0x23, #char(25) # Command to get magnetometer calibration parameters from the shimmer device


    }
