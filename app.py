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
# 2. CONFIGURATION & PROFILES 
# -------------------------
st.set_page_config(page_title="SafeBot Physics Auditor", layout="wide")

# Aesthetic Header with Custom CSS
st.markdown("""
    <style>
    .main { background-color: #f5f7f9; }
    .stMetric { background-color: #ffffff; padding: 15px; border-radius: 10px; box-shadow: 0 2px 4px rgba(0,0,0,0.05); }
    </style>
    """, unsafe_allow_html=True)

st.title("🛡️ SafeBot: Physics Reality Auditor")
st.caption("Deterministic Safety Middleware for AI-Controlled Robotics Systems")

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
    
    surface_key = st.selectbox("Environmental Surface", list(SURFACE_MAP.keys()))
    base_friction = SURFACE_MAP[surface_key]
    
    brake_key = st.select_slider("Mechanical Brake Condition", options=list(BRAKE_MAP.keys()), value="Used")
    deceleration = BRAKE_MAP[brake_key]

    # Initialize defaults
    velocity, distance = 5.0, 10.0
    
    if audit_mode == "Manual Simulator":
        velocity = st.slider("Velocity (m/s)", 0.0, 20.0, 5.0)
        distance = st.slider("Obstacle Distance (m)", 0.5, 30.0, 10.0)

    load_weight = st.slider("Current Load (kg)", 0.0, 3000.0, 500.0)
    slope = st.slider("Terrain Slope (deg)", 0.0, 45.0, 5.0)

# -------------------------
# 4. CORE AUDIT LOGIC 
# -------------------------
# Inside run_audit in app.py
def run_audit(p_data, v, d, decel, load, friction, slp):
    # ... previous logic ...
    zero = Vector3(0.0, 0.0, 0.0)
    
    effective_friction = max(friction - min(v * 0.01, 0.3), 0.05)

    env = EnvironmentState(
        temperature=20.0,
        air_density=1.225,
        wind_vector=zero,
        terrain_type="flat",
        friction=float(effective_friction), # Ensure this matches the class
        slope=float(slp),
        lighting_conditions="normal",
        distance_to_obstacles=float(d)
    )
    # ... rest of the function ...

    world_state = WorldState(
        timestamp=datetime.now().timestamp(), delta_time=0.1, gravity=Vector3(0,0,-9.81),
        environment=env, agents=[agent], objects=[], uncertainty=UncertaintyModel(0.05,0.05,0.05,0.05)
    )
    
    # Bridge for constraints 
    world_state.agent = agent 

    engine = SafetyEngine()
    for c in [BrakingConstraint(), FrictionConstraint(), LoadConstraint(), StabilityConstraint()]:
        engine.register_constraint(c)

    return SafetyReport(engine.evaluate(world_state)), effective_friction

# -------------------------
# 5. VIDEO AUDIT 
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
            # Simulated telemetry from vision translator logic 
            velocity, distance = 8.5, 12.0 
            st.info(f"📊 Tracking Sync: Velocity {velocity}m/s | Distance {distance}m")
        cap.release()

# -------------------------
# 6. RESULTS & VISUALIZATION [cite: 3, 8]
# -------------------------
report, eff_friction = run_audit(profile, velocity, distance, deceleration, load_weight, base_friction, slope)

# Main Verdict Header
if report.is_safe():
    st.success("### ✅ VERDICT: PHYSICS CLEAR TO PROCEED")
else:
    st.error("### ❌ VERDICT: PHYSICS VETO - SAFETY VIOLATION")

# Metric Dashboard Row
m1, m2, m3, m4 = st.columns(4)
risk_score = sum(25 for r in report.results if r.violated)
m1.metric("Risk Score", f"{risk_score}/100", delta=f"{risk_score}%", delta_color="inverse")
m2.metric("Effective Friction", f"{eff_friction:.2f}")
m3.metric("Brake Capacity", f"{deceleration} m/s²")
m4.metric("Load Status", f"{load_weight} kg", delta=f"{profile['max_load']} max")

# Stopping Distance Visualization 
required_stop = (velocity ** 2) / (2 * deceleration) if deceleration > 0 else 0
buffer = distance - required_stop

fig_buffer = go.Figure()
fig_buffer.add_trace(go.Bar(
    y=["Path"], x=[required_stop], name="Stopping Distance",
    orientation='h', marker_color='#EF553B'
))
if buffer > 0:
    fig_buffer.add_trace(go.Bar(
        y=["Path"], x=[buffer], name="Safety Margin",
        orientation='h', marker_color='#00CC96'
    ))
fig_buffer.update_layout(height=200, margin=dict(l=0, r=0, t=30, b=0), barmode='stack')
st.plotly_chart(fig_buffer, use_container_width=True)

# -------------------------
# 7. SAFETY HEATMAP 
# -------------------------
with st.expander("📊 Advanced Safety Region Analysis", expanded=True):
    st.write("Predicted safety zones across varying speeds and distances.")
    v_range = np.linspace(0, 20, 25)
    d_range = np.linspace(1, 30, 25)
    Z = [[0 if run_audit(profile, v_i, d_j, deceleration, load_weight, base_friction, slope)[0].is_safe() else 1 for d_j in d_range] for v_i in v_range]

    heatmap = go.Figure(data=go.Heatmap(
        z=Z, x=d_range, y=v_range, 
        colorscale=[[0, '#00CC96'], [1, '#EF553B']], 
        showscale=False
    ))
    heatmap.update_layout(xaxis_title="Distance (m)", yaxis_title="Speed (m/s)", height=450)
    st.plotly_chart(heatmap, use_container_width=True)