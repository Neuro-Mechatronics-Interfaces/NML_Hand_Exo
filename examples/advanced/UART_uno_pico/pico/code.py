import time
from usbserialreader import USBSerialReader

# Initialize your custom serial reader using UART (not USB CDC)
reader = USBSerialReader(use_UART=True, baudrate=9600, TERMINATOR='\n', verbose=True)

print("USBSerialReader ready.")

while True:
    # Send command
    command = "version\n"
    reader.send(command)
    print(f"Sent: {command.strip()}")

    # Wait for response
    start = time.monotonic()
    timeout = 1.0  # 1 second timeout
    while time.monotonic() - start < timeout:
        reader.update()
        if reader._out_data:
            for parsed_cmd in reader.out_data:
                print("Received:", parsed_cmd)
            break
        time.sleep(0.01)
    
    time.sleep(0.5)  # Small delay before next command
