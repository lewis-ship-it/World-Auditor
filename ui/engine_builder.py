from alignment_core.engine.safety_engine import SafetyEngine

from alignment_core.constraints.braking import BrakingConstraint
from alignment_core.constraints.friction import FrictionConstraint
from alignment_core.constraints.load import LoadConstraint
from alignment_core.constraints.stability import StabilityConstraint


def build_engine():

    return SafetyEngine([
        BrakingConstraint(),
        FrictionConstraint(),
        LoadConstraint(),
        StabilityConstraint()
    ])