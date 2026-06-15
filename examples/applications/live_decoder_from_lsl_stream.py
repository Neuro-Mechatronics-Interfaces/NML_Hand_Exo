import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pylsl
from pylsl import StreamInlet  # , resolve_stream
import time
import pandas as pd
from scipy.signal import butter, lfilter, lfilter_zi, iirnotch, filtfilt
import serial

############### Change PARAMETERS here ############### 

sampling_rate = 2000  # Hz
PORT = 'COM6'

stream_name = 'SAGAB'            # Search for active stream names using names_of_active_lsl_streams.py and connect to NML wifi
buffer_length_sec = 5
buffer_size = int(buffer_length_sec * sampling_rate)
threshold = 10  # EMG envelope threshold. Change based on participant threshold
#threshold = 1.75  # EMG envelope threshold. Change based on participant threshold

# Filters
bandpass_low = 20
bandpass_high = 450
notch_freq = 60
notch_q = 30.0
envelope_cutoff = 0.5  # Hz
#envelope_cutoff = 1  # Hz
envelope_order = 4


############### Filtering Functions ############### 

def bandpass_filter(data, fs, low, high, order=4):
    nyq = 0.5 * fs
    b, a = butter(order, [low / nyq, high / nyq], btype='band')
    return filtfilt(b, a, data)

def notch_filter(data, fs, freq=60.0, q=30.0 ):
    nyq = 0.5 * fs
    b, a = iirnotch(freq / nyq, q)
    return filtfilt(b, a, data)

def design_lowpass(fs, cutoff=2.0, order=4):
    nyq = 0.5 * fs
    b, a = butter(order, cutoff / nyq, btype='low')
    return b, a

def send_command(exo, msg):
    """Send a command to the exoskeleton."""
    if exo:
        try:
            if not msg.endswith('\n'):
                msg = msg + '\n'
            exo.write(msg.encode('utf-8'))
            print(f"Command sent: {msg.strip()}")
            time.sleep(0.01)
        except Exception as e:
            print(f"Error sending command: {e}")
    else:
        print("Exoskeleton not connected, command not sent.")

###############  Data Recording ############### 

recorded_data = []
recorded_timestamps = []


###############  Connect to LSL Stream ############### 

print("Resolving LSL stream...")
#streams = resolve_stream('name', stream_name)
streams = pylsl.resolve_streams()

inlet = None
for i, stream in enumerate(streams):
    if stream.name() == stream_name:
        print(f"Found stream {i+1}: {stream.name()} (Type: {stream.type()})")
        inlet = StreamInlet(stream, max_buflen=60)
        break

info = inlet.info()
n_channels = info.channel_count()

print(f"Connected to stream: {stream_name}")
print(f"Detected {n_channels} channels.")

# Get channel labels
channel_labels = []
try:
    chns = info.desc().child("channels").child("channel")
    for _ in range(n_channels):
        label = chns.child_value("label")
        channel_labels.append(label if label else f"ch{_}")
        chns = chns.next_sibling()
except Exception:
    print("!!!!!!! No channel names found. Using default labels.")
    channel_labels = [f"ch{i}" for i in range(n_channels)]
 
print("Channel labels:", channel_labels)



############### Connect to Exo on COM Port ###############

# Change COM based on exo connection
# Change baudrate if using old dynamixel motor to 1000000

try:
    exo = serial.Serial(PORT, 57600)
    print(f"Connected to Exo on {PORT}")
except Exception as e:
    print(f"ERROR!!!!!!!!! Could not connect to {PORT}: {e}")
    exo = None

############# Reconfigure the exo just in case ###################
# Reboot motors
send_command(exo, "reboot:1")
time.sleep(1)
#send_command(exo, "disable_torque:1")
#print("torque disabled, please reset linkage")
#time.sleep(2)
#print("torque re-enabled")
send_command(exo, "calibrate_zero:1\n")
time.sleep(1)
send_command(exo, "enable_torque:1\n")
time.sleep(1)
#send_command(exo, "set_vel:1:200\n")
print("Exo configured")

last_command = None  # Track last motor command to avoid repeats


###############  Channel to plot and decode from ############### 

desired_labels = ['UNI 41', 'TRIGGERS']             # Extensor : UNI 01 to UNI 34, Flexor : UNI 35 to UNI 64
plot_indices = [channel_labels.index(label) for label in desired_labels]
decode_index = plot_indices[0]


############### Buffers ############### 

raw_buffer = np.zeros(buffer_size)
bandpass_buffer = np.zeros(buffer_size)
envelope_buffer = np.zeros(buffer_size)
trigger_buffer = np.zeros(buffer_size) if len(plot_indices) > 1 else None
x_axis = np.linspace(-buffer_length_sec, 0, buffer_size)



###############  Plot visualization setup ############### 

n_plots = 3 if trigger_buffer is not None else 2
fig, axs = plt.subplots(n_plots, 1, figsize=(12, 2.8 * n_plots), sharex=True)

raw_line, = axs[0].plot(x_axis, bandpass_buffer, color='gray', label='Bandpass-filtered EMG')
axs[0].set_ylabel("Bandpass")
axs[0].legend(loc='upper right')
axs[0].grid(True)
axs[0].set_ylim(-45, 45)

env_line, = axs[1].plot(x_axis, envelope_buffer, color='blue', label='EMG Envelope')
axs[1].axhline(threshold, color='black', linestyle='--', linewidth=1, label='Threshold')
axs[1].set_ylabel("Envelope")
axs[1].legend(loc='upper right')
axs[1].grid(True)
axs[1].set_ylim(0, 10.5)

if trigger_buffer is not None:
    trig_line, = axs[2].plot(x_axis, trigger_buffer, color='green', label='Trigger')
    axs[2].set_ylabel("TRIGGERS")
    axs[2].legend(loc='upper right')
    axs[2].grid(True)
    axs[2].set_ylim(-1, 260)

axs[-1].set_xlabel("Time (s)")
fig.suptitle(f"Real-Time EMG Decoding: {desired_labels[0]}")
plt.tight_layout()


###############  Envelope filter setup ############### 

b_lpf, a_lpf = design_lowpass(sampling_rate, cutoff=envelope_cutoff, order=envelope_order)
zi_lpf = lfilter_zi(b_lpf, a_lpf) * 0  # zero-initialized filter state


############### Update function for animation ############### 

def update(frame):
    global raw_buffer, bandpass_buffer, envelope_buffer, trigger_buffer, zi_lpf, last_command

    chunk, timestamps = inlet.pull_chunk(timeout=0.0, max_samples=512)
    if chunk:
        chunk = np.array(chunk)
        if chunk.shape[0] < 28:  # Ensure sufficient samples for filtfilt
            return []

        recorded_data.extend(chunk.tolist())
        recorded_timestamps.extend(timestamps)

        # EMG processing
        emg_raw = chunk[:, decode_index]
        emg_filtered = bandpass_filter(emg_raw, sampling_rate, bandpass_low, bandpass_high)
        emg_filtered = notch_filter(emg_filtered, sampling_rate, freq=notch_freq, q=notch_q)
        emg_rectified = np.abs(emg_filtered)
        emg_envelope, zi_lpf = lfilter(b_lpf, a_lpf, emg_rectified, zi=zi_lpf)

        #b, a = butter(2, envelope_cutoff, btype="low", fs=sampling_rate)
        #emg_envelope = filtfilt(b, a, emg_rectified)

        # Apply an 100 sample rms to the envelop
        #window_size = int(0.5 * sampling_rate)
        #emg_envelope = np.sqrt(np.convolve(emg_rectified**2, np.ones(window_size)/window_size, mode='same'))

        # Update plot buffers
        bandpass_buffer = np.roll(bandpass_buffer, -len(emg_filtered)) 
        envelope_buffer = np.roll(envelope_buffer, -len(emg_envelope))
        bandpass_buffer[-len(emg_filtered):] = emg_filtered
        envelope_buffer[-len(emg_envelope):] = emg_envelope

        if trigger_buffer is not None:
            trig_raw = chunk[:, plot_indices[1]]
            trigger_buffer = np.roll(trigger_buffer, -len(trig_raw))
            trigger_buffer[-len(trig_raw):] = trig_raw
            trig_line.set_ydata(trigger_buffer)

        # Update plots
        raw_line.set_ydata(bandpass_buffer)
        env_line.set_ydata(envelope_buffer)

        # Update color and send command
        current_env = emg_envelope[-1]
        if current_env > threshold:
            env_line.set_color('red')
            if exo and last_command != "go":
                print("Setting exo to go position")
                send_command(exo, "set_angle:WRIST:40")
                last_command = "go"
        else:
            env_line.set_color('blue')
            if exo and last_command != "rest":
                print("Setting exo to rest position")
                send_command(exo, "set_angle:WRIST:0")
                last_command = "rest"

    return [raw_line, env_line] + ([trig_line] if trigger_buffer is not None else [])


############### Start live plotting ############### 

ani = FuncAnimation(fig, update, interval=50)
plt.show()


############### Save recorded data to CSV ############### 

df = pd.DataFrame(recorded_data, columns=channel_labels)
df.insert(0, 'timestamp', recorded_timestamps)
df.to_csv('lsl_recorded_data_all_channels.csv', index=False)
print(" SAVED! Data saved to lsl_recorded_data_all_channels.csv")
