from fairchem.core import pretrained_mlip, FAIRChemCalculator
from ase import Atoms
from ase.calculators.lj import LennardJones
import json, io, sys, numpy as np
import pickle
import struct
import termios

#predictor = pretrained_mlip.get_predict_unit(
#    "uma-s-1p2",
#    device="cuda",
#    inference_settings="turbo"
#)

calculator = LennardJones()
# calculator = FAIRChemCalculator(predictor, "uma")
atoms = None
print("Ready to accept instructions", flush=True)

while True:
    with open("/home/ik4335/log.txt", "w") as file:
        file.write("about to read\n")
    action = sys.stdin.buffer.read(1).decode()
    length = struct.unpack("!I", sys.stdin.buffer.read(4))[0]
    data = sys.stdin.buffer.read(length)
    if action == "A":
        atoms = Atoms(**pickle.loads(data))
        atoms.calc = LennardJones()
        # atoms.calc = FAIRChemCalculator(predictor, "uma")
    elif action == "S":
        atoms.set_positions(pickle.loads(data))
    elif action == "G":
        buffer = io.BytesIO()
        np.save(buffer, atoms.get_forces())
        buffer.seek(0)
        sys.stdout.buffer.write(struct.pack("d", atoms.get_potential_energy()))
        sys.stdout.buffer.write(struct.pack("!I", len(buffer.getbuffer())) + buffer.read())
        sys.stdout.flush()