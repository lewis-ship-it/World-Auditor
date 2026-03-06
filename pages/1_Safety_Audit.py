import sys
import os

# Get the absolute path of the current file's parent's parent (the root)
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# Add it to the system path if it's not already there
if root_path not in sys.path:
    sys.path.insert(0, root_path)

# NOW your imports will work
from ui.engine_builder import build_engine
from alignment_core.constraints.braking import BrakingConstraint
st.title("Safety Audit")

velocity = st.slider("Velocity",0.0,15.0,4.0)
distance = st.slider("Obstacle Distance",1.0,50.0,10.0)
friction = st.slider("Friction",0.1,1.2,0.6)

mass = st.slider("Robot Mass",50.0,2000.0,500.0)
load = st.slider("Load Weight",0.0,1000.0,200.0)

com_height = st.slider("COM Height",0.1,2.0,0.5)
wheelbase = st.slider("Wheelbase",0.5,3.0,1.2)

slope = st.slider("Slope",-20.0,20.0,0.0)

if st.button("Run Audit"):

    world = build_world(
        velocity,mass,friction,slope,
        distance,load,com_height,wheelbase
    )

    engine = build_engine()

    results = engine.evaluate(world)

for r in results:
    if not r.violated:
        st.success(r.name)
    else:
        st.error(r.name)
    st.json(r.to_dict())