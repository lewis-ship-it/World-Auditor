import streamlit as st
import math
import plotly.graph_objects as go
import json
from datetime import datetime

# --- CORRECTED IMPORT PATHS BASED ON YOUR REPO ---
from alignment_core.engine.safety_engine import SafetyEngine
from alignment_core.engine.report import SafetyReport
from alignment_core.constraints.braking import BrakingConstraint
from alignment_core.constraints.friction import FrictionConstraint
from alignment_core.constraints.load import LoadConstraint
from alignment_core.constraints.stability import StabilityConstraint
from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.environment import EnvironmentState
from alignment_core.world_model import WorldState

# -------------------------
# CONFIG & STYLING
# -------------------------
st.set_page_config(page_title="SafeBot Physics Auditor", layout="wide")

st.title("🛡️ SafeBot: Physics Reality Auditor")
st.markdown("Friendly Deterministic Middleware for AI-Controlled Robotics")

# -------------------------
# DATA MAPS (User Friendly Presets)
# -------------------------
ROBOT_PROFILES = {
    "Warehouse Forklift": {
        "mass": 4000.0, "max_load": 2000.0, "com_height": 1.2, "wheelbase": 2.0,
    },
    "Delivery Rover": {
        "mass": 80.0, "max_load": 20.0, "com_height": 0.4, "wheelbase": 0.6,
    },
}

SURFACE_MAP = {
    "Dry Concrete (Optimal)": 0.8,
    "Wet Asphalt (Slippery)": 0.4,
    "Icy Loading Dock (Danger)": 0.15
}

BRAKE_MAP = {
    "Brand New / Responsive": 5.0,
    "Standard / Used": 2.5,
    "Worn / Failing": 1.0
}

# -------------------------
# SIDEBAR CONTROLS
# -------------------------
st.sidebar.header("🕹️ Scenario Settings")

# One-Click Stress Tests
if st.sidebar.button("🚨 Load Emergency Stop Stress Test"):
    v_init, d_init, s_init = 12.0, 5.0, "Wet Asphalt (Slippery)"
else:
    v_init, d_init, s_init = 5.0, 10.0, "Dry Concrete (Optimal)"

profile_name = st.sidebar.selectbox("Robot Profile", list(ROBOT_PROFILES.keys()))
profile = ROBOT_PROFILES[profile_name]

surface_key = st.sidebar.selectbox("Road Condition", list(SURFACE_MAP.keys()), index=list(SURFACE_MAP.keys()).index(s_init))
friction = SURFACE_MAP[surface_key]

brake_key = st.sidebar.select_slider("Brake Condition", options=list(BRAKE_MAP.keys()), value="Standard / Used")
deceleration = BRAKE_MAP[brake_key]

velocity = st.sidebar.slider("Current Speed (m/s)", 0.0, 15.0, v_init)
distance = st.sidebar.slider("Distance to Obstacle (m)", 0.5, 20.0, d_init)
load_weight = st.sidebar.slider("Load Weight (kg)", 0.0, 3000.0, 500.0)
slope = st.sidebar.slider("Slope Angle (degrees)", 0.0, 45.0, 5.0)

compare_mode = st.sidebar.checkbox("Compare with another Robot?")

# -------------------------
# CORE ENGINE EXECUTION
# -------------------------
def run_audit(p_data, v, d, decel, load, fric, slp):
    # Mapping to your AgentState class attributes
    agent = AgentState(
        velocity=v, 
        max_deceleration=decel, 
        mass=p_data["mass"], 
        load_weight=load, 
        max_load=p_data["max_load"], 
        center_of_mass_height=p_data["com_height"], 
        wheelbase=p_data["wheelbase"]
    )
    
    # Mapping to your EnvironmentState class
    env = EnvironmentState(distance_to_obstacle=d, friction=fric, slope=slp)
    
    # Creating WorldState as required by your engine
    world_state = WorldState(
        timestamp=datetime.now().timestamp(),
        delta_time=0.1,
        gravity=None, # Assuming Vector3 or None based on your primitives
        environment=env,
        agents=[agent],
        objects=[],
        uncertainty=None
    )
    
    engine = SafetyEngine()
    # Adding the constraints from your /constraints/ folder
    engine.register_constraint(BrakingConstraint())
    engine.register_constraint(FrictionConstraint())
    engine.register_constraint(LoadConstraint())
    engine.register_constraint(StabilityConstraint())
    
    results = engine.evaluate(world_state)
    return SafetyReport(results)

report = run_audit(profile, velocity, distance, deceleration, load_weight, friction, slope)

# -------------------------
# MAIN DASHBOARD UI
# -------------------------

if report.is_safe():
    st.success("### ✅ CLEAR TO PROCEED")
else:
    st.error("### ❌ VETO: PHYSICS VIOLATION")

# Star Rating
risk_score = report.risk_score()
st.write(f"**Safety Rating:** {'⭐' * max(1, 5 - int(risk_score / 20))}")

# Impact Zone Visualizer
required_stop = (velocity ** 2) / (2 * deceleration)
buffer = distance - required_stop



fig_buffer = go.Figure()
fig_buffer.add_trace(go.Bar(
    y=["Path"], x=[required_stop], name="Stop Distance",
    orientation='h', marker_color='#EF553B' if buffer < 0 else '#00CC96'
))
if buffer > 0:
    fig_buffer.add_trace(go.Bar(y=["Path"], x=[buffer], name="Margin", orientation='h', marker_color='#AAAAAA'))

fig_buffer.update_layout(barmode='stack', height=180, margin=dict(l=20, r=20, t=20, b=20))
st.plotly_chart(fig_buffer, use_container_width=True)

# Metrics
c1, c2, c3 = st.columns(3)
c1.metric("Required Stop", f"{required_stop:.1f}m")
c2.metric("Stability", "⚠️ TIPPING" if any(r.name == "Stability" and r.violated for r in report.results) else "✅ OK")
c3.metric("Load", f"{load_weight}kg")

# Auto-Suggestions
if not report.is_safe():
    with st.expander("🛠️ How to fix these violations", expanded=True):
        for r in report.results:
            if r.violated:
                if "Braking" in r.name:
                    safe_v = math.sqrt(2 * deceleration * distance)
                    st.info(f"👉 Slow down to **{safe_v:.1f} m/s** to stop safely.")
                if "Load" in r.name:
                    st.info(f"👉 Remove **{load_weight - profile['max_load']} kg**.")

# Side-by-Side Comparison
if compare_mode:
    st.divider()
    alt_name = st.selectbox("Compare with:", [k for k in ROBOT_PROFILES.keys() if k != profile_name])
    alt_report = run_audit(ROBOT_PROFILES[alt_name], velocity, distance, deceleration, load_weight, friction, slope)
    
    col_a, col_b = st.columns(2)
    col_a.metric(f"{profile_name} Risk", f"{report.risk_score()}%")
    col_b.metric(f"{alt_name} Risk", f"{alt_report.risk_score()}%")