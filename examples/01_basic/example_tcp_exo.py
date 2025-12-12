from nml_hand_exo.interface import HandExo, TCPComm

# TCP usage (e.g., Pico W on Wi-Fi)
ip = "192.168.1.200"
port = 5001

comm = TCPComm(ip=ip, port=port)
exo = HandExo(comm, verbose=True)

try:
    exo.connect()
    print(exo.version())
except Exception as e:
    print(f"Error: {e}")
finally:
    exo.close()