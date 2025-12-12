from nml_hand_exo.interface import HandExo, FakeHandExo, SerialComm, LSLClient
from nml_hand_exo.control import StateTriggerRMS

if __name__ == "__main__":

    # Serial port config
    port = "COM6"
    baudrate = 115200

    comm = SerialComm(port=port, baudrate=baudrate)
    #exo = FakeHandExo(comm, verbose=True)
    exo = HandExo(comm, verbose=True)
    exo.connect()

    # Create an LSL client to stream EMG data
    lsl = LSLClient(stream_type="EMG")

    # Create the state trigger object
    trigger = StateTriggerRMS(
        exo=exo, client=lsl,
        threshold_percent=0.15,  # Let's set it to 15%
        verbose=True
    )

    # Assign state triggers for specific channels crossing the rms threshold
    trigger.set_trigger_state("open", channels=list(range(0, 64)))
    trigger.set_trigger_state("close", channels=list(range(64, 128)))

    # Record at the beginning to establish a baseline...
    trigger.calibrate_rest_state()
    # ...or load a previously recorded baseline
    #trigger.load_baseline("path_to_baseline_file")

    trigger.start()
