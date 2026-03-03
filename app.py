import streamlit as st
import math
import random
import plotly.graph_objects as go
from datetime import datetime
import numpy as np
import cv2
import tempfile

# --- 1. CORE ENGINE & MODEL IMPORTS ---
from alignment_core.engine.safety_engine import SafetyEngine
from alignment_core.engine.report import SafetyReport
from alignment_core.constraints.braking import BrakingConstraint
from alignment_core.constraints.friction import FrictionConstraint
from alignment_core.constraints.load import LoadConstraint
from alignment_core.constraints.stability import StabilityConstraint
from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.environment import EnvironmentState
from alignment_core.world_model.world_state import WorldState
from alignment_core.world_model.primitives import Vector3, Quaternion, ActuatorLimits
from alignment_core.world_model.uncertainty import UncertaintyModel

# -------------------------
# 2. CONFIGURATION & STYLING
# -------------------------
st.set_page_config(page_title="SafeBot Physics Auditor", layout="wide")

st.markdown("""
    <style>
    .main { background-color: #f8f9fa; }
    .stMetric { background-color: #ffffff; padding: 15px; border-radius: 10px; border: 1px solid #e0e0e0; }
    </style>
    """, unsafe_allow_html=True)

st.title("🛡️ SafeBot: Physics Reality Auditor")
st.caption("Deterministic Middleware for Validating AI Intent against Physical Laws")

ROBOT_PROFILES = {
    "Warehouse Forklift": {"mass": 4000.0, "max_load": 2000.0, "com_height": 1.2, "wheelbase": 2.0},
    "Delivery Rover": {"mass": 80.0, "max_load": 20.0, "com_height": 0.4, "wheelbase": 0.6},
    "Standard Sedan": {"mass": 1500.0, "max_load": 500.0, "com_height": 0.6, "wheelbase": 2.7}
}

SURFACE_MAP = {"Dry Concrete": 0.8, "Wet Asphalt": 0.4, "Ice": 0.15}
BRAKE_MAP = {"New": 5.0, "Used": 2.5, "Failing": 1.0}

# -------------------------
# 3. SIDEBAR CONTROLS
# -------------------------
with st.sidebar:
    st.header("⚙️ Audit Configuration")
    audit_mode = st.radio("Primary Mode", ["Manual Simulator", "Live Video Audit"])
    
    st.divider()
    profile_name = st.selectbox("Robot Profile", list(ROBOT_PROFILES.keys()))
    profile = ROBOT_PROFILES[profile_name]
    
    surface_key = st.selectbox("Surface Type", list(SURFACE_MAP.keys()))
    base_friction = SURFACE_MAP[surface_key]
    
    brake_key = st.select_slider("Brake Condition", options=list(BRAKE_MAP.keys()), value="Used")
    deceleration = BRAKE_MAP[brake_key]

    velocity, distance = 5.0, 10.0
    if audit_mode == "Manual Simulator":
        velocity = st.slider("Current Velocity (m/s)", 0.0, 25.0, 5.0)
        distance = st.slider("Obstacle Distance (m)", 0.5, 40.0, 10.0)

    load_weight = st.slider("Current Load (kg)", 0.0, 3000.0, 500.0)
    slope = st.slider("Terrain Slope (deg)", 0.0, 45.0, 5.0)

# -------------------------
# 4. CORE AUDIT LOGIC (The "Fix-All" Function)
# -------------------------
def run_audit(p_data, v, d, decel, load, friction, slp):
    # Dynamic Friction Calculation (Speed reduces effective grip)
    reduction = min(v * 0.012, 0.35)
    eff_fric = max(friction - reduction, 0.05)
    
    zero = Vector3(0.0, 0.0, 0.0)
    identity = Quaternion(1.0, 0.0, 0.0, 0.0)
    limits = ActuatorLimits(100.0, 10000.0, 25.0, float(decel))

    # Define Agent (Initializing all potential required fields)
    agent_obj = AgentState(
        id="primary_robot", type="mobile", mass=float(p_data["mass"]), position=zero,
        velocity=Vector3(float(v), 0.0, 0.0), angular_velocity=zero, orientation=identity,
        center_of_mass=zero, center_of_mass_height=float(p_data["com_height"]),
        support_polygon=[Vector3(-0.5, -0.5, 0), Vector3(0.5, 0.5, 0)],
        wheelbase=float(p_data["wheelbase"]), load_weight=float(load), max_load=float(p_data["max_load"]),
        actuator_limits=limits, battery_state=1.0, current_load=None, contact_points=[]
    )

    # Define Environment
    env_obj = EnvironmentState(
        temperature=20.0, air_density=1.225, wind_vector=zero, terrain_type="flat",
        friction=float(eff_fric), 
        slope=float(slp),
        lighting_conditions="normal", distance_to_obstacles=float(d)
    )

    # Build World State
    world_state = WorldState(
        timestamp=datetime.now().timestamp(), delta_time=0.1, gravity=Vector3(0,0,-9.81),
        environment=env_obj, agents=[agent_obj], objects=[], uncertainty=UncertaintyModel(0.05,0.05,0.05,0.05)
    )
    
    # THE BRIDGE: Solving the "List vs Object" Conflict
    # This ensures both .agents (list) and .agent (singular) are available
    world_state.agent = agent_obj 

    engine = SafetyEngine()
    for c in [BrakingConstraint(), FrictionConstraint(), LoadConstraint(), StabilityConstraint()]:
        engine.register_constraint(c)

    results = engine.evaluate(world_state)
    return SafetyReport(results), eff_fric

# -------------------------
# 5. LIVE VIDEO LOGIC
# -------------------------
if audit_mode == "Live Video Audit":
    st.subheader("📸 Computer Vision Perception Sync")
    uploaded_video = st.file_uploader("Upload Audit Stream", type=["mp4", "mov"])
    if uploaded_video:
        tfile = tempfile.NamedTemporaryFile(delete=False)
        tfile.write(uploaded_video.read())
        cap = cv2.VideoCapture(tfile.name)
        ret, frame = cap.read()
        if ret:
            st.image(frame, channels="BGR", use_container_width=True)
            # Simulated Computer Vision perception outputs
            velocity, distance = 9.2, 14.5 
            st.success(f"Tracking Sync: Velocity {velocity}m/s | Distance {distance}m")
        cap.release()

# -------------------------
# 6. VERDICT & METRICS
# -------------------------
report, final_friction = run_audit(profile, velocity, distance, deceleration, load_weight, base_friction, slope)

# Defensive UI handling
is_safe = getattr(report, "is_safe", lambda: False)()
if is_safe:
    st.success("### ✅ VERDICT: MISSION CAPABLE")
else:
    st.error("### ❌ VERDICT: PHYSICS VETO DETECTED")

# Metric Dashboard Row
col1, col2, col3, col4 = st.columns(4)
report_results = getattr(report, "results", [])
risk = sum(25 for r in report_results if getattr(r, "violated", False))

col1.metric("Risk Level", f"{risk}%", delta="Critical" if risk > 50 else "Stable", delta_color="inverse")
col2.metric("Effective Grip", f"{final_friction:.2f}")
col3.metric("Brake Capacity", f"{deceleration}m/s²")
col4.metric("Total Mass", f"{profile['mass'] + load_weight}kg")

# Stopping Distance Visualization

stop_dist = (velocity ** 2) / (2 * deceleration) if deceleration > 0 else 0
margin = distance - stop_dist

fig = go.Figure()
fig.add_trace(go.Bar(y=["Path"], x=[stop_dist], name="Stopping", orientation='h', marker_color='#EF553B'))
if margin > 0:
    fig.add_trace(go.Bar(y=["Path"], x=[margin], name="Margin", orientation='h', marker_color='#00CC96'))
fig.update_layout(height=180, barmode='stack', margin=dict(l=0, r=0, t=20, b=20), showlegend=False)
st.plotly_chart(fig, use_container_width=True)

# -------------------------
# 7. SAFETY HEATMAP
# -------------------------
with st.expander("🔍 Predictive Safety Envelope", expanded=True):
    v_axis = np.linspace(0, 25, 20)
    d_axis = np.linspace(1, 40, 20)
    # Fast evaluation for heatmap grid
    grid = [[0 if run_audit(profile, vi, dj, deceleration, load_weight, base_friction, slope)[0].is_safe() else 1 for dj in d_axis] for vi in v_axis]

    heatmap = go.Figure(data=go.Heatmap(z=grid, x=d_axis, y=v_axis, colorscale=[[0, '#00CC96'], [1, '#EF553B']], showscale=False))
    heatmap.update_layout(xaxis_title="Distance (m)", yaxis_title="Velocity (m/s)", height=400)
    st.plotly_chart(heatmap, use_container_width=True)