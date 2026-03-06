import sys
import os

# 1. Force the root directory into the path globally
# This ensures sub-imports like 'ui' can find 'alignment_core'
current_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.abspath(os.path.join(current_dir, ".."))

if root_dir not in sys.path:
    sys.path.insert(0, root_dir)

# 2. NOW perform your imports
import streamlit as st
from ui.engine_builder import build_engine
from alignment_core.constraints.braking import BrakingConstraint