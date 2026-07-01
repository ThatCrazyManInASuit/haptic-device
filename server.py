from fairchem.core import pretrained_mlip, FAIRChemCalculator
from ase import Atoms
from ase.calculators.lj import LennardJones
import io, sys, numpy as np
import pickle
import struct

#predictor = pretrained_mlip.get_predict_unit(
#    "uma-s-1p2",
#    device="cuda",
#    inference_settings="turbo"
#)

calculator = LennardJones()
# calculator = FAIRChemCalculator(predictor, "uma")
print("Ready to accept instructions", flush=True)

length = struct.unpack("!I", sys.stdin.buffer.read(4))[0]
kwargs = pickle.loads(sys.stdin.buffer.read(length))
num_atoms = len(kwargs["numbers"])
atoms = Atoms(**kwargs)
atoms.calc = LennardJones()
# atoms.calc = FAIRChemCalculator(predictor, "uma")

while True:
    data = sys.stdin.buffer.read(np.dtype(np.float64).itemsize * num_atoms * 3)
    atoms.set_positions(np.frombuffer(data, dtype=np.float64).reshape((num_atoms, 3)))
    sys.stdout.buffer.write(atoms.get_forces().tobytes())
    sys.stdout.buffer.write(struct.pack("d", atoms.get_potential_energy()))
    sys.stdout.flush()