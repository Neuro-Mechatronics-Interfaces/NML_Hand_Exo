from nml_hand_exo.interface import FakeHandExo, SerialComm, LSLClient
from nml_hand_exo.control import EMGClassifierTrigger

if __name__ == "__main__":
    # Serial port config
    port = "COM6"
    baudrate = 115200

    comm = SerialComm(port=port, baudrate=baudrate)
    exo = FakeHandExo(comm, verbose=True)
    exo.connect()

    # Create an LSL client to stream EMG data
    lsl = LSLClient(stream_type="EMG")

    # Create the EMGClassifierTrigger object
    trigger = EMGClassifierTrigger(
        exo=exo, client=lsl,
        model_path="path_to_your_classifier.pkl",  # Path to your trained classifier
        verbose=True
    )
    trigger.start()