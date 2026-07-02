import subprocess, socket, threading

SIM = r"hand_exo_sim.exe"
proc = subprocess.Popen([SIM], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                        stderr=subprocess.DEVNULL, bufsize=0)

server = socket.socket(); server.bind(('localhost', 9999)); server.listen(1)
print("Waiting for GUI on socket://localhost:9999 ...")
conn, _ = server.accept(); print("Connected.")

def rx(): 
    while True:
        d = conn.recv(256)
        if not d: break
        proc.stdin.write(d); proc.stdin.flush()

def tx():
    while True:
        b = proc.stdout.read(1)
        if not b: break
        conn.sendall(b)

threading.Thread(target=rx, daemon=True).start()
threading.Thread(target=tx, daemon=True).start()
proc.wait()
