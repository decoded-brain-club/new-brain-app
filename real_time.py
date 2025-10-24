import spidev  # Provides SPI interface to communicate with the ADC chips
import time  # Allows insertion of delays where needed in the acquisition loop
#from RPi import GPIO  # Legacy import kept commented in case Raspberry Pi GPIO is preferred
from matplotlib import pyplot as plt  # Used to draw real-time plots of the captured data
from scipy.ndimage import gaussian_filter1d  # Imported for potential smoothing (unused but kept for parity)
from scipy import signal  # Supplies signal-processing helpers such as Butterworth filters
import gpiod  # Grants access to GPIO lines through the libgpiod interface

#GPIO.setwarnings(False)  # Original GPIO warning suppression (commented because RPi.GPIO is unused)
#GPIO.setmode(GPIO.BOARD)  # Original GPIO numbering mode selection (commented for the same reason)

button_pin_1 = 26  # GPIO line used to read the primary button state (mapped from BOARD pin 26)
button_pin_2 = 13  # Secondary button line kept from the original design (unused but retained)
cs_pin = 19  # GPIO line that acts as manual chip-select for the secondary SPI bus
#chip = gpiod.Chip("gpiochip4")  # Alternate constructor kept from early experiments
chip = gpiod.chip("/dev/gpiochip4")  # Open the gpiochip device that exposes the required lines
#chip = gpiod.chip("0")  # Backup attempt referencing the chip by index instead of path
#cs_line = chip.get_line(19)  # GPIO19  # Another historical snippet kept for context
#cs_line.request(consumer="SPI_CS", type=gpiod.LINE_REQ_DIR_OUT)  # Earlier API usage style
cs_line = chip.get_line(cs_pin)  # Grab the chip-select line handle using the configured pin number
cs_line_out = gpiod.line_request()  # Prepare a line request structure for configuring direction
cs_line_out.consumer = "SPI_CS"  # Name the consumer for easier debugging via gpioinfo
cs_line_out.request_type = gpiod.line_request.DIRECTION_OUTPUT  # Request the line as an output
cs_line.request(cs_line_out)  # Apply the prepared request to obtain control of the CS line

#cs_line.request(consumer="SPI_CS", type=gpiod.line_request.DIRECTION_OUTPUT)  # Legacy API call left for reference
cs_line.set_value(1)  # Set the chip-select line high so the ADC starts deselected


#button_line_1 = chip.get_line(button_pin_1)  # Older direct request pattern for the button line
#button_line_1.request(consumer = "Button", type = gpiod.LINE_REQ_DIR_IN)  # Legacy API style for input configuration

line_1 = chip.get_line(button_pin_1)  # Acquire the first button line that is polled for DRDY transitions

#line_2 = chip.get_line(button_pin_2)  # Placeholder for a second button input (unused but preserved)

button_line_1 = gpiod.line_request()  # Allocate a line request to configure the button input
button_line_1.consumer = "Button"  # Name the consumer as "Button" for diagnostics
button_line_1.request_type = gpiod.line_request.DIRECTION_INPUT  # Set the button line direction to input
line_1.request(button_line_1)  # Apply the request so the script can read the button state

#button_line_2 = chip.get_line(button_pin_2)  # Retained placeholder for a second button line
#button_line_2.request(consumer = "Button", type = gpiod.LINE_REQ_DIR_IN)  # Original request style
#button_line_2 = gpiod.line_request()  # Would allocate a second request structure (unused)
#button_line_2.consumer = "Button"  # Would name the second button consumer
#button_line_2.request_type = gpiod.line_request.DIRECTION_INPUT  # Would set the direction to input
#line_2.request(button_line_2)  # Would apply the input configuration to the second button line

spi = spidev.SpiDev()  # Instantiate an SPI device object for the first ADC chain
spi.open(0,0)  # Bind the first SPI device to bus 0, chip-select 0
spi.max_speed_hz  = 4000000  # Set the SPI clock to 4 MHz for the primary ADC
spi.lsbfirst=False  # Ensure bits are shifted MSB-first as required by the ADS1299
spi.mode=0b01  # Configure SPI mode 1 (CPOL=0, CPHA=1) for the ADC
spi.bits_per_word = 8  # Use 8-bit transfers when exchanging bytes with the ADC

spi_2 = spidev.SpiDev()  # Instantiate the second SPI device object for the additional ADC chain
spi_2.open(0,1)  # Bind the secondary SPI device to bus 0, chip-select 1
spi_2.max_speed_hz=4000000  # Match the SPI clock to 4 MHz on the secondary channel
spi_2.lsbfirst=False  # Maintain MSB-first communication for the second ADC
spi_2.mode=0b01  # Use the same SPI mode 1 on the secondary ADC
spi_2.bits_per_word = 8  # Keep 8-bit word transfers for the second SPI device

who_i_am=0x00  # Register address used to verify device identity (unused but left intact)
config1=0x01  # Config register 1 address for data rate and power settings
config2=0X02  # Config register 2 address for bias and lead-off control
config3=0X03  # Config register 3 address for reference buffer configuration

reset=0x06  # Command byte that resets the ADC to default register values
stop=0x0A  # Command byte that halts conversion and places the ADC in standby
start=0x08  # Command byte that resumes conversions in normal operation
sdatac=0x11  # Command byte that stops continuous data read mode
rdatac=0x10  # Command byte that starts continuous data read mode
wakeup=0x02  # Command byte that wakes the ADC from standby
rdata = 0x12  # Command byte that performs a single data read (unused in streaming mode)

ch1set=0x05  # Channel 1 configuration register address
ch2set=0x06  # Channel 2 configuration register address
ch3set=0x07  # Channel 3 configuration register address
ch4set=0x08  # Channel 4 configuration register address
ch5set=0x09  # Channel 5 configuration register address
ch6set=0x0A  # Channel 6 configuration register address
ch7set=0x0B  # Channel 7 configuration register address
ch8set=0x0C  # Channel 8 configuration register address

data_test= 0x7FFFFF  # Reference value used to detect sign-extension boundaries in samples
data_check=0xFFFFFF  # Maximum 24-bit value used when correcting overflowed samples

def read_byte(register):  # Function that reads a single register from the first ADC
 write=0x20  # Base command for register reads
 register_write=write|register  # Combine the base command with the target register address
 data = [register_write,0x00,register]  # Build the payload specifying register and byte count
 read_reg=spi.xfer(data)  # Perform the SPI transfer and capture the returned bytes
 print ("data", read_reg)  # Log the raw data for debugging purposes
 
def send_command(command):  # Function that sends a simple command to the first ADC
 send_data = [command]  # Wrap the command byte in a list
 com_reg=spi.xfer(send_data)  # Transmit the command over SPI
 
def write_byte(register,data):  # Function that writes a single register on the first ADC
 write=0x40  # Base command for register writes
 register_write=write|register  # Combine the base command with the register address
 data = [register_write,0x00,data]  # Build the payload with the register, count, and data byte
 print (data)  # Print the payload for traceability
 spi.xfer(data)  # Execute the SPI transfer to write the register

def read_byte_2(register):  # Function that reads a register from the second ADC
 write=0x20  # Base command for reads
 register_write=write|register  # Combine base command with register address
 data = [register_write,0x00,register]  # Construct the read payload
 cs_line.set_value(0)  # Pull chip-select low to choose the second ADC
 read_reg=spi.xfer(data)  # Perform the SPI transfer using the primary interface
 cs_line.set_value(1)  # Release chip-select after the transfer
 print ("data", read_reg)  # Output the returned register contents
 
def send_command_2(command):  # Function that sends a command to the second ADC
 send_data = [command]  # Wrap the command byte for transfer
 cs_line.set_value(0)  # Assert chip-select for the second ADC
 spi_2.xfer(send_data)  # Send the command over the secondary SPI interface
 cs_line.set_value(1)  # Release chip-select to end the transaction
 
def write_byte_2(register,data):  # Function that writes a register on the second ADC
 write=0x40  # Base command for writes
 register_write=write|register  # Combine the base command with the target register
 data = [register_write,0x00,data]  # Construct the payload for the write
 print (data)  # Display the payload for debugging insight

 cs_line.set_value(0)  # Assert chip-select before the write
 spi_2.xfer(data)  # Execute the SPI transfer on the secondary interface
 cs_line.set_value(1)  # Release chip-select after the write

 

send_command (wakeup)  # Wake the first ADC from standby
send_command (stop)  # Place the first ADC in standby to allow configuration
send_command (reset)  # Reset the first ADC registers to defaults
send_command (sdatac)  # Stop any ongoing continuous read mode on the first ADC

write_byte (0x14, 0x80) #GPIO 80  # Configure GPIO settings on the first ADC
write_byte (config1, 0x96)  # Set configuration register 1 on the first ADC
write_byte (config2, 0xD4)  # Set configuration register 2 on the first ADC
write_byte (config3, 0xFF)  # Set configuration register 3 on the first ADC
write_byte (0x04, 0x00)  # Reset misc register 0x04
write_byte (0x0D, 0x00)  # Reset misc register 0x0D
write_byte (0x0E, 0x00)  # Reset misc register 0x0E
write_byte (0x0F, 0x00)  # Reset misc register 0x0F
write_byte (0x10, 0x00)  # Reset misc register 0x10
write_byte (0x11, 0x00)  # Reset misc register 0x11
write_byte (0x15, 0x20)  # Configure misc register 0x15
#
write_byte (0x17, 0x00)  # Set offset register 0x17
write_byte (ch1set, 0x00)  # Configure channel 1 settings
write_byte (ch2set, 0x00)  # Configure channel 2 settings
write_byte (ch3set, 0x00)  # Configure channel 3 settings
write_byte (ch4set, 0x00)  # Configure channel 4 settings
write_byte (ch5set, 0x00)  # Configure channel 5 settings
write_byte (ch6set, 0x00)  # Configure channel 6 settings
write_byte (ch7set, 0x01)  # Configure channel 7 settings
write_byte (ch8set, 0x01)  # Configure channel 8 settings

send_command (rdatac)  # Start continuous data read mode on the first ADC
send_command (start)  # Start conversions on the first ADC


send_command_2 (wakeup)  # Wake the second ADC from standby
send_command_2 (stop)  # Place the second ADC in standby for configuration
send_command_2 (reset)  # Reset the second ADC registers to defaults
send_command_2 (sdatac)  # Stop any continuous read in progress on the second ADC

write_byte_2 (0x14, 0x80) #GPIO 80  # Configure GPIO settings on the second ADC
write_byte_2 (config1, 0x96)  # Set configuration register 1 on the second ADC
write_byte_2 (config2, 0xD4)  # Set configuration register 2 on the second ADC
write_byte_2 (config3, 0xFF)  # Set configuration register 3 on the second ADC
write_byte_2 (0x04, 0x00)  # Reset misc register 0x04 on the second ADC
write_byte_2 (0x0D, 0x00)  # Reset misc register 0x0D on the second ADC
write_byte_2 (0x0E, 0x00)  # Reset misc register 0x0E on the second ADC
write_byte_2 (0x0F, 0x00)  # Reset misc register 0x0F on the second ADC
write_byte_2 (0x10, 0x00)  # Reset misc register 0x10 on the second ADC
write_byte_2 (0x11, 0x00)  # Reset misc register 0x11 on the second ADC
write_byte_2 (0x15, 0x20)  # Configure misc register 0x15 on the second ADC
#
write_byte_2 (0x17, 0x00)  # Set offset register 0x17 on the second ADC
write_byte_2 (ch1set, 0x00)  # Configure channel 1 on the second ADC
write_byte_2 (ch2set, 0x00)  # Configure channel 2 on the second ADC
write_byte_2 (ch3set, 0x00)  # Configure channel 3 on the second ADC
write_byte_2 (ch4set, 0x00)  # Configure channel 4 on the second ADC
write_byte_2 (ch5set, 0x00)  # Configure channel 5 on the second ADC
write_byte_2 (ch6set, 0x00)  # Configure channel 6 on the second ADC
write_byte_2 (ch7set, 0x01)  # Configure channel 7 on the second ADC
write_byte_2 (ch8set, 0x01)  # Configure channel 8 on the second ADC

send_command_2 (rdatac)  # Start continuous data read mode on the second ADC
send_command_2 (start)  # Start conversions on the second ADC

DRDY=1  # Initialize the data-ready flag

result=[0]*27  # Buffer to hold parsed channel data from the first ADC
result_2=[0]*27  # Buffer to hold parsed channel data from the second ADC


data_1ch_test = []  # Storage for channel 1 samples from ADC 1
data_2ch_test = []  # Storage for channel 2 samples from ADC 1
data_3ch_test = []  # Storage for channel 3 samples from ADC 1
data_4ch_test = []  # Storage for channel 4 samples from ADC 1
data_5ch_test = []  # Storage for channel 5 samples from ADC 1
data_6ch_test = []  # Storage for channel 6 samples from ADC 1
data_7ch_test = []  # Storage for channel 7 samples from ADC 1
data_8ch_test = []  # Storage for channel 8 samples from ADC 1

data_9ch_test = []  # Storage for channel 1 samples from ADC 2
data_10ch_test = []  # Storage for channel 2 samples from ADC 2
data_11ch_test = []  # Storage for channel 3 samples from ADC 2
data_12ch_test = []  # Storage for channel 4 samples from ADC 2
data_13ch_test = []  # Storage for channel 5 samples from ADC 2
data_14ch_test = []  # Storage for channel 6 samples from ADC 2
data_15ch_test = []  # Storage for channel 7 samples from ADC 2
data_16ch_test = []  # Storage for channel 8 samples from ADC 2

axis_x=0  # Horizontal axis starting position for plotting
y_minus_graph=100  # Lower Y-axis padding value
y_plus_graph=100  # Upper Y-axis padding value
x_minux_graph=5000  # Leftward X-axis range extent for plotting
x_plus_graph=250  # Rightward X-axis range extent for plotting
sample_len = 250  # Number of samples collected before each update

fig, axis = plt.subplots(4, 4, figsize=(5, 5))  # Create a 4x4 grid of subplots for the 16 channels
plt.subplots_adjust(hspace=1)  # Increase vertical spacing between subplots
ch_name = 0  # Index used to cycle through channel titles
ch_name_title = [1,5,2,6,3,7,4,8]  # Titles assigned to each subplot pair
axi = [(i, j) for i in range(4) for j in range(2)]  # Generate row/column index pairs
for ax_row, ax_col in axi:  # Iterate through each subplot position
    axis[ax_row, ax_col].set_xlabel('Time')  # Label the X-axis with time
    axis[ax_row, ax_col].set_ylabel('Amplitude')  # Label the Y-axis with amplitude
    axis[ax_row, ax_col].set_title('Data after pass filter Ch-' + str(ch_name_title[ch_name]))  # Set subplot title
    ch_name = ch_name + 1  # Advance to the next channel title
    

test_DRDY = 5  # Initialize DRDY guard variable
test_DRDY_2 = 5  # Secondary DRDY guard variable (unused but retained)
#1.2 Band-pass filter  # Comment indicating the start of filter parameter section
data_before = []  # Placeholder list (unused but retained)
data_after =  []  # Placeholder list (unused but retained)
just_one_time = 0  # Flag used elsewhere (unused but retained)
data_lenght_for_Filter = 2     # how much we read data for filter, all lenght  [_____] + [_____] + [_____]
read_data_lenght_one_time = 1   # for one time how much read  [_____]
sample_len = 250  # Ensure the sample length constant remains defined
sample_lens = 250  # Duplicate of sample length used in plotting
fps = 250  # Frames (samples) per second for the data acquisition
highcut = 1  # High-pass cut-off frequency in Hz
lowcut = 10  # Low-pass cut-off frequency in Hz
data_before_1 = data_before_2 = data_before_3 = data_before_4 = data_before_5 = data_before_6 = data_before_7 = data_before_8 = [0]*250  # Initial buffers for the first 8 channels
data_before_9 = data_before_10 = data_before_11 = data_before_12 = data_before_13 = data_before_14 = data_before_15 = data_before_16 = [0]*250  # Initial buffers for the second 8 channels

print (data_lenght_for_Filter*read_data_lenght_one_time-read_data_lenght_one_time)  # Display derived filter window length

def butter_lowpass(cutoff, fs, order=5):  # Create coefficients for a low-pass Butterworth filter
    nyq = 0.5 * fs  # Compute the Nyquist frequency
    normal_cutoff = cutoff / nyq  # Normalize cutoff frequency relative to Nyquist
    b, a = signal.butter(order, normal_cutoff, btype='low', analog=False)  # Generate filter coefficients
    return b, a  # Return numerator and denominator coefficients
def butter_lowpass_filter(data, cutoff, fs, order=5):  # Apply a low-pass filter to the provided data
    b, a = butter_lowpass(cutoff, fs, order=order)  # Obtain filter coefficients
    y = signal.lfilter(b, a, data)  # Filter the data with a causal filter
    return y  # Return the filtered output
def butter_highpass(cutoff, fs, order=3):  # Create coefficients for a high-pass Butterworth filter
    nyq = 0.5 * fs  # Compute the Nyquist frequency
    normal_cutoff = cutoff / nyq  # Normalize cutoff frequency
    b, a = signal.butter(order, normal_cutoff, btype='high', analog=False)  # Generate high-pass coefficients
    return b, a  # Return numerator and denominator coefficients
def butter_highpass_filter(data, cutoff, fs, order=5):  # Apply a high-pass filter to the data
    b, a = butter_highpass(cutoff, fs, order=order)  # Obtain filter coefficients
    y = signal.filtfilt(b, a, data)  # Apply zero-phase filtering by running forward and backward
    return y  # Return the filtered output

last_valid_value = 5  # Initialize the last valid status word value for corruption detection
counter = 0  # Counter for the number of corrupted frames encountered

def _to_signed_24bit(msb: int, middle: int, lsb: int) -> int:  # Convert three bytes to a signed 24-bit integer
    # Combine the bytes into a 24-bit integer  # Comment retained from original implementation
    combined = (msb << 16) | (middle << 8) | lsb  # Merge the three bytes into one integer

    # Check the sign bit (bit 7 of the MSB)  # Comment retained
    if (msb & 0x80) != 0:  # If the sign bit is set, the value is negative
        # Convert to negative 24-bit signed integer  # Comment retained
        combined -= 1 << 24  # Sign-extend by subtracting 2^24

    return combined  # Return the signed integer result

def the_input_is_valid(input_list):  # Determine whether the incoming sample frame is valid
  #  print('oks')  # Legacy debug print left commented
    global last_valid_value, counter  # Use the global tracking variables
    msb = input_list[24]  # Most significant byte of the status word
    middle = input_list[25]  # Middle byte of the status word
    lsb = input_list[26]  # Least significant byte of the status word

    current_value = _to_signed_24bit(msb, middle, lsb)  # Convert status bytes to a signed integer
    
    if last_valid_value is None:  # If no previous value exists
        _last_valid_value = current_value  # Assign the current value to the placeholder
        #print('here')  # Commented debug statement
        return False  # Consider the first frame invalid to establish baseline

    difference = abs(current_value - last_valid_value)  # Compute change from previous frame
    if difference > 5000:  # Threshold for detecting corrupted data
        print(f'Corrupted data detected _counter: {counter}')  # Report the corruption count
        counter += 1  # Increment the counter whenever corruption is found
        return False  # Reject corrupted data

    else:  # Otherwise treat the frame as valid
        _last_valid_value = current_value  # Update the placeholder with the new value
        
        return True  # Accept the data

while 1:  # Infinite acquisition loop
    
    
    #print ("1", button_state)  # Debug snippet retained
    #print("2", button_state_2)  # Debug snippet retained

        #print ("ok3")  # Debug snippet retained
        button_state = line_1.get_value()  # Read the current state of the button line
        #print (button_state)  # Debug snippet retained
        if button_state == 1:  # Detect button press to arm DRDY edge
            test_DRDY = 10  # Set guard variable when button is pressed
        if test_DRDY == 10 and button_state == 0:  # Detect the release edge to trigger a read
            test_DRDY = 0  # Reset the guard variable after release

            output=spi.readbytes(27)  # Read 27 bytes (status + 8 channels) from the first ADC
            
            cs_line.set_value(0)  # Assert chip-select to read the second ADC
            output_2=spi_2.readbytes(27)  # Read 27 bytes from the second ADC
            cs_line.set_value(1)  # Release chip-select after the read

            if the_input_is_valid(output_2):  # Validate the secondary ADC frame
                #print('test')  # Debug snippet retained
                #continue  # Commented to allow processing when valid

#            print (output[0],output[1],output[2])  # Debug print showing status bytes
                if output_2[0]==192 and output_2[1] == 0 and output_2[2] == 8:  # Guard on expected frame header
                    #print ("ok4")  # Debug snippet retained
                    for a in range (3,25,3):  # Iterate over channel bytes in groups of three
                        voltage_1=(output[a]<<8)| output[a+1]  # Merge MSB and middle byte
                        voltage_1=(voltage_1<<8)| output[a+2]  # Merge the LSB into the 24-bit value
                        convert_voktage=voltage_1|data_test  # Combine with data test pattern
                        if convert_voktage==data_check:  # Detect overflow condition
                            voltage_1_after_convert=(voltage_1-16777214)  # Correct overflowed reading
                        else:
                           voltage_1_after_convert=voltage_1  # Otherwise keep raw reading
                        channel_num =  (a/3)  # Compute the channel index from byte offset

                        result[int (channel_num)]=round(1000000*4.5*(voltage_1_after_convert/16777215),2)  # Scale to microvolts

                    data_1ch_test.append(result[1])  # Append channel 1 sample from ADC 1
                    data_2ch_test.append(result[2])  # Append channel 2 sample from ADC 1
                    data_3ch_test.append(result[3])  # Append channel 3 sample from ADC 1
                    data_4ch_test.append(result[4])  # Append channel 4 sample from ADC 1
                    data_5ch_test.append(result[5])  # Append channel 5 sample from ADC 1
                    data_6ch_test.append(result[6])  # Append channel 6 sample from ADC 1
                    data_7ch_test.append(result[7])  # Append channel 7 sample from ADC 1
                    data_8ch_test.append(result[8])  # Append channel 8 sample from ADC 1


                    for a in range (3,25,3):  # Iterate over channel bytes from the second ADC
                        voltage_1=(output_2[a]<<8)| output_2[a+1]  # Merge MSB and middle byte
                        voltage_1=(voltage_1<<8)| output_2[a+2]  # Merge LSB into 24-bit value
                        convert_voktage=voltage_1|data_test  # Combine with data test pattern
                        if convert_voktage==data_check:  # Detect overflow
                            voltage_1_after_convert=(voltage_1-16777214)  # Correct overflow value
                        else:
                           voltage_1_after_convert=voltage_1  # Use raw value otherwise
                        channel_num =  (a/3)  # Determine channel index

                        result_2[int (channel_num)]=round(1000000*4.5*(voltage_1_after_convert/16777215),2)  # Scale to microvolts

                    data_9ch_test.append(result_2[1])  # Append channel 1 sample from ADC 2
                    data_10ch_test.append(result_2[2])  # Append channel 2 sample from ADC 2
                    data_11ch_test.append(result_2[3])  # Append channel 3 sample from ADC 2
                    data_12ch_test.append(result_2[4])  # Append channel 4 sample from ADC 2
                    data_13ch_test.append(result_2[5])  # Append channel 5 sample from ADC 2
                    data_14ch_test.append(result_2[6])  # Append channel 6 sample from ADC 2
                    data_15ch_test.append(result_2[7])  # Append channel 7 sample from ADC 2
                    data_16ch_test.append(result_2[8])  # Append channel 8 sample from ADC 2


                    
                    
                    if len(data_9ch_test)==sample_len:  # Once enough new samples have been collected


                        data_after_1 = data_1ch_test         # Copy current batch for channel 1
                        dataset_1 =  data_before_1 + data_after_1  # Concatenate history with new samples
                        data_before_1 = dataset_1[250:]  # Update history by dropping consumed samples
                        data_for_graph_1 = dataset_1  # Alias for clarity in plotting

                        data_filt_numpy_high_1 = butter_highpass_filter(data_for_graph_1, highcut, fps)  # Apply high-pass filter
                        data_for_graph_1 = butter_lowpass_filter(data_filt_numpy_high_1, lowcut, fps)  # Apply low-pass filter

                        axis[0,0].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_1[250:], color = '#0a0b0c')   # Plot filtered channel 1 data
                        axis[0,0].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_1[50]-y_minus_graph, data_for_graph_1[150]+y_plus_graph])  # Adjust axes for channel 1

                        # 2  # Marker comment kept from original code
                        data_after_2 = data_2ch_test         # Copy batch for channel 2
                        dataset_2 =  data_before_2 + data_after_2  # Concatenate history for channel 2
                        data_before_2 = dataset_2[250:]  # Update history for channel 2
                        data_for_graph_2 = dataset_2  # Alias for plotting

                        data_filt_numpy_high_2 = butter_highpass_filter(data_for_graph_2, highcut, fps)  # Apply high-pass to channel 2
                        data_for_graph_2 = butter_lowpass_filter(data_filt_numpy_high_2, lowcut, fps)  # Apply low-pass to channel 2

                        axis[1,0].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_2[250:], color = '#0a0b0c')   # Plot channel 2 data
                        axis[1,0].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_2[50]-y_minus_graph, data_for_graph_2[150]+y_plus_graph])  # Set axes for channel 2

                        # 3  # Marker retained
                        data_after_3 = data_3ch_test         # Copy batch for channel 3
                        dataset_3 =  data_before_3 + data_after_3  # Concatenate history for channel 3
                        data_before_3 = dataset_3[250:]  # Update history for channel 3
                        data_for_graph_3 = dataset_3  # Alias for plotting channel 3

                        data_filt_numpy_high_3 = butter_highpass_filter(data_for_graph_3, highcut, fps)  # Apply high-pass to channel 3
                        data_for_graph_3 = butter_lowpass_filter(data_filt_numpy_high_3, lowcut, fps)  # Apply low-pass to channel 3

                        axis[2,0].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_3[250:], color = '#0a0b0c')   # Plot channel 3 data
                        axis[2,0].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_3[50]-y_minus_graph, data_for_graph_3[150]+y_plus_graph])  # Set axes for channel 3

                        # 4  # Marker retained
                        data_after_4 = data_4ch_test         # Copy batch for channel 4
                        dataset_4 =  data_before_4 + data_after_4  # Concatenate history for channel 4
                        data_before_4 = dataset_4[250:]  # Update history for channel 4
                        data_for_graph_4 = dataset_4  # Alias for plotting channel 4

                        data_filt_numpy_high_4 = butter_highpass_filter(data_for_graph_4, highcut, fps)  # Apply high-pass to channel 4
                        data_for_graph_4 = butter_lowpass_filter(data_filt_numpy_high_4, lowcut, fps)  # Apply low-pass to channel 4

                        axis[3,0].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_4[250:], color = '#0a0b0c')   # Plot channel 4 data
                        axis[3,0].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_4[50]-y_minus_graph, data_for_graph_4[150]+y_plus_graph])  # Set axes for channel 4

                        #5
                        data_after_5 = data_5ch_test         # Copy batch for channel 5
                        dataset_5 =  data_before_5 + data_after_5  # Concatenate history for channel 5
                        data_before_5 = dataset_5[250:]  # Update history for channel 5
                        data_for_graph_5 = dataset_5  # Alias for plotting channel 5

                        data_filt_numpy_high_5 = butter_highpass_filter(data_for_graph_5, highcut, fps)  # Apply high-pass to channel 5
                        data_for_graph_5 = butter_lowpass_filter(data_filt_numpy_high_5, lowcut, fps)  # Apply low-pass to channel 5

                        axis[0,1].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_5[250:], color = '#0a0b0c')   # Plot channel 5 data
                        axis[0,1].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_5[50]-y_minus_graph, data_for_graph_5[150]+y_plus_graph])  # Set axes for channel 5
                         
                        #6
                        data_after_6 = data_6ch_test         # Copy batch for channel 6
                        dataset_6 =  data_before_6 + data_after_6  # Concatenate history for channel 6
                        data_before_6 = dataset_6[250:]  # Update history for channel 6
                        data_for_graph_6 = dataset_6  # Alias for plotting channel 6

                        data_filt_numpy_high_6 = butter_highpass_filter(data_for_graph_6, highcut, fps)  # Apply high-pass to channel 6
                        data_for_graph_6 = butter_lowpass_filter(data_filt_numpy_high_6, lowcut, fps)  # Apply low-pass to channel 6

                        axis[1,1].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_6[250:], color = '#0a0b0c')   # Plot channel 6 data
                        axis[1,1].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_6[50]-y_minus_graph, data_for_graph_6[150]+y_plus_graph])  # Set axes for channel 6

                        #7
                        data_after_7 = data_7ch_test         # Copy batch for channel 7
                        dataset_7 =  data_before_7 + data_after_7  # Concatenate history for channel 7
                        data_before_7 = dataset_7[250:]  # Update history for channel 7
                        data_for_graph_7 = dataset_7  # Alias for plotting channel 7

                        data_filt_numpy_high_7 = butter_highpass_filter(data_for_graph_7, highcut, fps)  # Apply high-pass to channel 7
                        data_for_graph_7 = butter_lowpass_filter(data_filt_numpy_high_7, lowcut, fps)  # Apply low-pass to channel 7

                        axis[2,1].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_7[250:], color = '#0a0b0c')   # Plot channel 7 data
                        axis[2,1].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_7[50]-y_minus_graph, data_for_graph_1[150]+y_plus_graph])  # Set axes for channel 7

                        #8
                        data_after_8 = data_8ch_test         # Copy batch for channel 8
                        dataset_8 =  data_before_8 + data_after_8  # Concatenate history for channel 8
                        data_before_8 = dataset_8[250:]  # Update history for channel 8
                        data_for_graph_8 = dataset_8  # Alias for plotting channel 8

                        data_filt_numpy_high_8 = butter_highpass_filter(data_for_graph_8, highcut, fps)  # Apply high-pass to channel 8
                        data_for_graph_8 = butter_lowpass_filter(data_filt_numpy_high_8, lowcut, fps)  # Apply low-pass to channel 8

                        axis[3,1].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_8[250:], color = '#0a0b0c')   # Plot channel 8 data
                        axis[3,1].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_8[50]-y_minus_graph, data_for_graph_8[150]+y_plus_graph])  # Set axes for channel 8
                        
                        # 9
                        data_after_9 = data_9ch_test         # Copy batch for channel 9
                        dataset_9 =  data_before_9 + data_after_9  # Concatenate history for channel 9
                        data_before_9 = dataset_9[250:]  # Update history for channel 9
                        data_for_graph_9 = dataset_9  # Alias for plotting channel 9

                        data_filt_numpy_high_9 = butter_highpass_filter(data_for_graph_9, highcut, fps)  # Apply high-pass to channel 9
                        data_for_graph_9 = butter_lowpass_filter(data_filt_numpy_high_9, lowcut, fps)  # Apply low-pass to channel 9

                        axis[0,2].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_9[250:], color = '#0a0b0c')   # Plot channel 9 data
                        axis[0,2].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_9[50]-y_minus_graph, data_for_graph_9[150]+y_plus_graph])  # Set axes for channel 9

                        # 10
                        data_after_10 = data_10ch_test         # Copy batch for channel 10
                        dataset_10 =  data_before_10 + data_after_10  # Concatenate history for channel 10
                        data_before_10 = dataset_10[250:]  # Update history for channel 10
                        data_for_graph_10 = dataset_10  # Alias for plotting channel 10

                        data_filt_numpy_high_10 = butter_highpass_filter(data_for_graph_10, highcut, fps)  # Apply high-pass to channel 10
                        data_for_graph_10 = butter_lowpass_filter(data_filt_numpy_high_10, lowcut, fps)  # Apply low-pass to channel 10

                        axis[1,2].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_10[250:], color = '#0a0b0c')   # Plot channel 10 data
                        axis[1,2].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_10[50]-y_minus_graph, data_for_graph_10[150]+y_plus_graph])  # Set axes for channel 10

                        # 11
                        data_after_11 = data_11ch_test         # Copy batch for channel 11
                        dataset_11 =  data_before_11 + data_after_11  # Concatenate history for channel 11
                        data_before_11 = dataset_11[250:]  # Update history for channel 11
                        data_for_graph_11 = dataset_11  # Alias for plotting channel 11

                        data_filt_numpy_high_11 = butter_highpass_filter(data_for_graph_11, highcut, fps)  # Apply high-pass to channel 11
                        data_for_graph_11 = butter_lowpass_filter(data_filt_numpy_high_11, lowcut, fps)  # Apply low-pass to channel 11

                        axis[2,2].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_11[250:], color = '#0a0b0c')   # Plot channel 11 data
                        axis[2,2].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_11[50]-y_minus_graph, data_for_graph_11[150]+y_plus_graph])  # Set axes for channel 11

                        # 12
                        data_after_12 = data_12ch_test         # Copy batch for channel 12
                        dataset_12 =  data_before_12 + data_after_12  # Concatenate history for channel 12
                        data_before_12 = dataset_12[250:]  # Update history for channel 12
                        data_for_graph_12 = dataset_12  # Alias for plotting channel 12

                        data_filt_numpy_high_12 = butter_highpass_filter(data_for_graph_12, highcut, fps)  # Apply high-pass to channel 12
                        data_for_graph_12 = butter_lowpass_filter(data_filt_numpy_high_12, lowcut, fps)  # Apply low-pass to channel 12

                        axis[3,2].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_12[250:], color = '#0a0b0c')   # Plot channel 12 data
                        axis[3,2].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_12[50]-y_minus_graph, data_for_graph_12[150]+y_plus_graph])  # Set axes for channel 12

                        # 13
                        data_after_13 = data_13ch_test         # Copy batch for channel 13
                        dataset_13 =  data_before_13 + data_after_13  # Concatenate history for channel 13
                        data_before_13 = dataset_13[250:]  # Update history for channel 13
                        data_for_graph_13 = dataset_13  # Alias for plotting channel 13

                        data_filt_numpy_high_13 = butter_highpass_filter(data_for_graph_13, highcut, fps)  # Apply high-pass to channel 13
                        data_for_graph_13 = butter_lowpass_filter(data_filt_numpy_high_13, lowcut, fps)  # Apply low-pass to channel 13

                        axis[0,3].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_13[250:], color = '#0a0b0c')   # Plot channel 13 data
                        axis[0,3].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_13[50]-y_minus_graph, data_for_graph_13[150]+y_plus_graph])  # Set axes for channel 13
                         
                        # 14 
                        data_after_14 = data_14ch_test         # Copy batch for channel 14
                        dataset_14 =  data_before_14 + data_after_14  # Concatenate history for channel 14
                        data_before_14 = dataset_14[250:]  # Update history for channel 14
                        data_for_graph_14 = dataset_14  # Alias for plotting channel 14

                        data_filt_numpy_high_14 = butter_highpass_filter(data_for_graph_14, highcut, fps)  # Apply high-pass to channel 14
                        data_for_graph_14 = butter_lowpass_filter(data_filt_numpy_high_14, lowcut, fps)  # Apply low-pass to channel 14

                        axis[1,3].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_14[250:], color = '#0a0b0c')   # Plot channel 14 data
                        axis[1,3].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_14[50]-y_minus_graph, data_for_graph_14[150]+y_plus_graph])  # Set axes for channel 14

                        # 15
                        data_after_15 = data_15ch_test         # Copy batch for channel 15
                        dataset_15 =  data_before_15 + data_after_15  # Concatenate history for channel 15
                        data_before_15 = dataset_15[250:]  # Update history for channel 15
                        data_for_graph_15 = dataset_15  # Alias for plotting channel 15

                        data_filt_numpy_high_15 = butter_highpass_filter(data_for_graph_15, highcut, fps)  # Apply high-pass to channel 15
                        data_for_graph_15 = butter_lowpass_filter(data_filt_numpy_high_15, lowcut, fps)  # Apply low-pass to channel 15

                        axis[2,3].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_15[250:], color = '#0a0b0c')   # Plot channel 15 data
                        axis[2,3].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_15[50]-y_minus_graph, data_for_graph_15[150]+y_plus_graph])  # Set axes for channel 15

                        # 16
                        data_after_16 = data_16ch_test         # Copy batch for channel 16
                        dataset_16 =  data_before_16 + data_after_16  # Concatenate history for channel 16
                        data_before_16 = dataset_16[250:]  # Update history for channel 16
                        data_for_graph_16 = dataset_16  # Alias for plotting channel 16

                        data_filt_numpy_high_16 = butter_highpass_filter(data_for_graph_16, highcut, fps)  # Apply high-pass to channel 16
                        data_for_graph_16 = butter_lowpass_filter(data_filt_numpy_high_16, lowcut, fps)  # Apply low-pass to channel 16

                        axis[3,3].plot(range(axis_x,axis_x+sample_lens,1),data_for_graph_16[250:], color = '#0a0b0c')   # Plot channel 16 data
                        axis[3,3].axis([axis_x-x_minux_graph, axis_x+x_plus_graph, data_for_graph_16[50]-y_minus_graph, data_for_graph_16[150]+y_plus_graph])  # Set axes for channel 16


                        plt.pause(0.0000000000001)  # Brief pause to refresh the Matplotlib canvas
                        
                        axis_x=axis_x+sample_lens  # Advance the X-axis offset for the next window
                        data_1ch_test = []  # Clear stored samples for channel 1
                        data_2ch_test = []  # Clear stored samples for channel 2
                        data_3ch_test = []  # Clear stored samples for channel 3
                        data_4ch_test = []  # Clear stored samples for channel 4
                        data_5ch_test = []  # Clear stored samples for channel 5
                        data_6ch_test = []  # Clear stored samples for channel 6
                        data_7ch_test = []  # Clear stored samples for channel 7
                        data_8ch_test = []  # Clear stored samples for channel 8
                        data_9ch_test = []  # Clear stored samples for channel 9
                        data_10ch_test = []  # Clear stored samples for channel 10
                        data_11ch_test = []  # Clear stored samples for channel 11
                        data_12ch_test = []  # Clear stored samples for channel 12
                        data_13ch_test = []  # Clear stored samples for channel 13
                        data_14ch_test = []  # Clear stored samples for channel 14
                        data_15ch_test = []  # Clear stored samples for channel 15
                        data_16ch_test = []  # Clear stored samples for channel 16
                    
                    else:
                        pass  # If buffer not yet full, skip plotting

                        #data_16ch_test = []  # Commented legacy reset call
                        
   
