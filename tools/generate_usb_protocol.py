"""Regenerate checked-in USB bindings (no board required).

Build dependencies: nanopb==0.4.9.1, grpcio-tools==1.84.0.
Run from any directory with those packages installed in the Python environment.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
from grpc_tools import protoc

ROOT = Path(__file__).resolve().parents[1]
schema = ROOT / 'protocol'
with tempfile.TemporaryDirectory() as tmp:
    descriptor = Path(tmp) / 'exo_usb.pb'
    result = protoc.main(['protoc', '-I' + str(schema),
                          '--python_out=' + str(ROOT / 'src/nml_hand_exo/interface'),
                          '--descriptor_set_out=' + str(descriptor), str(schema / 'exo_usb.proto')])
    if result:
        raise SystemExit(result)
    subprocess.run([sys.executable, '-m', 'nanopb.generator.nanopb_generator',
                    '-I' + str(schema), '-D' + str(ROOT / 'src/cpp/nml_hand_exo/protocol'),
                    str(descriptor)], check=True)
