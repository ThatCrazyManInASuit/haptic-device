from importlib import import_module
from ast import literal_eval

import paramiko
import pickle
import numpy as np
import io
import struct

USERNAME = "ik4335"
REMOTE_PYTHON = f"/home/{USERNAME}/venv/bin/python3"

class Atoms:
    def __init__(self, **kwargs):
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(
            hostname="fri.cm.utexas.edu",
            username=USERNAME
        )

        sftp = self.ssh.open_sftp()
        sftp.put("haptic-device/server.py", "/tmp/server.py")
        sftp.close()
        self.stdin, self.stdout, self.stderr = self.ssh.exec_command(f"{REMOTE_PYTHON} -u /tmp/server.py", get_pty=False)
        print("Waiting for server to be ready...")
        while True:
            line = self.stdout.readline()
            if "Ready to accept instructions" in line:
                print("Server is ready")
                break
        data = pickle.dumps(kwargs)
        self.stdin.write(b"A" + struct.pack("!I", len(data)))
        self.stdin.write(data)
        self.stdin.flush()

    def set_positions(self, positions):
        data = pickle.dumps(positions)
        self.stdin.write(b"S" + struct.pack("!I", len(data)))
        self.stdin.write(data)
        self.stdin.flush()

    def get_forces(self):
        self.stdin.write(b"G" + struct.pack("!I", 0))
        self.stdin.flush()
        self.cached_potential_energy = struct.unpack("d", self.stdout.read(8))[0]
        length = struct.unpack("!I", self.stdout.read(4))[0]
        return np.load(io.BytesIO(self.stdout.read(length)))
    
    def get_potential_energy(self):
        return self.cached_potential_energy

def _resolve_calculator(spec):

    if not spec or spec in {"lj", "lennard-jones"}:
        module_name = "ase.calculators.lj"
        class_name = "LennardJones"
        kwargs = {}

        calculator_class = getattr(import_module(module_name), class_name)
        return calculator_class(**kwargs)

    elif spec == "morse":
        module_name = "ase.calculators.morse"
        class_name = "MorsePotential"
        kwargs = {}

        calculator_class = getattr(import_module(module_name), class_name)
        return calculator_class(**kwargs)

    elif spec == "emt":
        module_name = "ase.calculators.emt"
        class_name = "EMT"
        kwargs = {}

        calculator_class = getattr(import_module(module_name), class_name)
        return calculator_class(**kwargs)

    elif spec.startswith("uma"):
        return None
        return FAIRChemCalculator()

    else:
        parts = spec.split(":", 2)

        if len(parts) < 2:
            raise ValueError(
                "Calculator spec must be empty, a known alias, or module:Class[:kwargs]"
            )

        module_name, class_name = parts[0], parts[1]

        kwargs = literal_eval(parts[2]) if len(parts) == 3 else {}

        if not isinstance(kwargs, dict):
            raise ValueError("Calculator kwargs must evaluate to a dict")

        calculator_class = getattr(import_module(module_name), class_name)

        return calculator_class(**kwargs)
