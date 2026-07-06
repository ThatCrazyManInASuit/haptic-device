import sys
import ase
from ase.io import read, write

poscar = sys.argv[1]
structure = read(poscar)
structure = structure.repeat((int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])))

write("result.poscar", structure, format='vasp', direct=True)