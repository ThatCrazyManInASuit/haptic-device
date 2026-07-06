# Python side of the ASE calculator.
# Resolves a short calculator spec string to an ASE-compatible calculator and
# computes interatomic forces and potential energies.
# The C++ ASE calculator (potentials.cpp) drives the simulation and delegates
# UMA construction here via uma_wrapper, while get_values is a self-contained
# Python path that builds an Atoms object and returns forces/energy directly.

from importlib import import_module
from ast import literal_eval

from ase import Atoms

from fairchem.core import pretrained_mlip, FAIRChemCalculator


# Module-level cache of UMA predictors. Building a predictor loads a large model
# into memory, so we keep one per (model, device) alive for the whole session
# and hand the same instance back on every subsequent call.
_uma_predictor_cache = {}


# Returns the cached UMA predictor for the given model and device, building it
# lazily on first use. "turbo" inference settings trade a little accuracy for
# the speed the haptic loop needs.
def _get_uma_predictor(model_name="uma-s-1p2", device="cuda"):
    key = (model_name, device)

    if key not in _uma_predictor_cache:
        _uma_predictor_cache[key] = pretrained_mlip.get_predict_unit(
            model_name,
            device=device,
            inference_settings="turbo"
        )

    return _uma_predictor_cache[key]


# Resolves a short spec string to a constructed ASE calculator. Accepts built-in
# aliases for common potentials (lj, morse, emt, uma) or a generic
# "module:Class[:kwargs]" format for anything else. Mirrors parseCalculatorSpec
# on the C++ side.
def _resolve_calculator(spec):

    # Lennard-Jones is the default when no spec is given. ASE's built-in pair
    # potential.
    if not spec or spec in {"lj", "lennard-jones"}:
        module_name = "ase.calculators.lj"
        class_name = "LennardJones"
        kwargs = {}

        calculator_class = getattr(import_module(module_name), class_name)
        return calculator_class(**kwargs)

    # Morse pair potential.
    elif spec == "morse":
        module_name = "ase.calculators.morse"
        class_name = "MorsePotential"
        kwargs = {}

        calculator_class = getattr(import_module(module_name), class_name)
        return calculator_class(**kwargs)

    # Effective Medium Theory, a fast empirical potential for metals.
    elif spec == "emt":
        module_name = "ase.calculators.emt"
        class_name = "EMT"
        kwargs = {}

        calculator_class = getattr(import_module(module_name), class_name)
        return calculator_class(**kwargs)

    # UMA is Meta's universal ML potential. The optional ":task" suffix selects
    # the prediction head (omol, omat, oc20, ...); omol is the default.
    elif spec.startswith("uma"):

        # Examples:
        # "uma"
        # "uma:omol"
        # "uma:omat"
        # "uma:oc20"

        parts = spec.split(":")

        task_name = "oc20"

        if len(parts) > 1:
            task_name = parts[1]

        predictor = _get_uma_predictor(
            model_name="uma-s-1p2",
            device="cuda"
        )

        return FAIRChemCalculator(
            predictor,
            task_name=task_name
        )

    # Generic format: "module:ClassName" or "module:ClassName:{...kwargs...}".
    else:
        parts = spec.split(":", 2)

        if len(parts) < 2:
            raise ValueError(
                "Calculator spec must be empty, a known alias, or module:Class[:kwargs]"
            )

        module_name, class_name = parts[0], parts[1]

        # The kwargs string is user-supplied calculator configuration. We use
        # literal_eval rather than eval for safety: it only accepts Python
        # literals, not arbitrary code.
        kwargs = literal_eval(parts[2]) if len(parts) == 3 else {}

        if not isinstance(kwargs, dict):
            raise ValueError("Calculator kwargs must evaluate to a dict")

        calculator_class = getattr(import_module(module_name), class_name)

        return calculator_class(**kwargs)


# Builds an ASE Atoms object from the given atomic numbers and positions,
# attaches the resolved calculator, and returns the forces and total potential
# energy. cell and pbc describe the simulation box and periodic boundary
# conditions; both may be omitted for an isolated cluster.
def get_values(numbers, positions, cell=None, pbc=None, calculator_spec=""):

    atoms = Atoms(
        numbers=numbers,
        positions=positions,
        cell=cell,
        pbc=pbc
    )

    # Some ML calculators (including the UMA omol task) require charge and spin
    # in the info dict.
    atoms.info["charge"] = 0
    atoms.info["spin"] = 1

    atoms.calc = _resolve_calculator(calculator_spec)

    return {
        "forces": atoms.get_forces().tolist(),
        "energy": atoms.get_potential_energy(),
    }


# Quick self-test: run a two-atom hydrogen calculation with the UMA potential
# and print the resulting forces and energy.
if __name__ == "__main__":

    sample = get_values(
        numbers=[1, 1],
        positions=[[0, 0, 0], [1.0, 0, 0]],
        calculator_spec="uma:omol"
    )

    print(sample)