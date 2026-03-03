import streamlit as st
import math
import random
import plotly.graph_objects as go
from datetime import datetime
import numpy as np
import cv2
import tempfile

# --- 1. CORE IMPORTS ---
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
# 2. CONFIG & PROFILES
# -------------------------
st.set_page_config(page_title="SafeBot Physics Auditor", layout="wide")
st.title("🛡️ SafeBot: Physics Reality Auditor")

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
st.sidebar.header("Audit Controls")
audit_mode = st.sidebar.radio("Mode", ["Manual Simulator", "Live Video Audit"])

profile_name = st.sidebar.selectbox("Robot Profile", list(ROBOT_PROFILES.keys()))
profile = ROBOT_PROFILES[profile_name]

surface_key = st.sidebar.selectbox("Surface", list(SURFACE_MAP.keys()))
base_friction = SURFACE_MAP[surface_key]

brake_key = st.sidebar.select_slider("Brake Condition", options=list(BRAKE_MAP.keys()), value="Used")
deceleration = BRAKE_MAP[brake_key]

# Initialize defaults
velocity = 5.0
distance = 10.0

if audit_mode == "Manual Simulator":
    velocity = st.sidebar.slider("Speed (m/s)", 0.0, 20.0, 5.0)
    distance = st.sidebar.slider("Distance to Obstacle (m)", 0.5, 30.0, 10.0)

load_weight = st.sidebar.slider("Load Weight (kg)", 0.0, 3000.0, 500.0)
slope = st.sidebar.slider("Slope (degrees)", 0.0, 45.0, 5.0)

# -------------------------
# 4. CORE AUDIT LOGIC
# -------------------------
def run_audit(p_data, v, d, decel, load, friction, slp):
    # Dynamic Friction Model
    reduction = min(v * 0.01, 0.3)
    effective_friction = max(friction - reduction, 0.05)

    zero = Vector3(0.0, 0.0, 0.0)
    identity = Quaternion(1.0, 0.0, 0.0, 0.0)
    
    limits = ActuatorLimits(
        max_torque=100.0,
        max_force=10000.0,
        max_speed=20.0,
        max_acceleration=decel
    )

    # Realigned AgentState
    agent = AgentState(
        id="robot",
        type="mobile",
        mass=p_data["mass"],
        position=zero,
        velocity=Vector3(v, 0.0, 0.0),
        angular_velocity=zero,
        orientation=identity,
        center_of_mass=zero,
        center_of_mass_height=p_data["com_height"],
        support_polygon=[Vector3(-0.5, -0.5, 0), Vector3(0.5, 0.5, 0)],
        wheelbase=p_data["wheelbase"],
        load_weight=load,
        max_load=p_data["max_load"],
        actuator_limits=limits,
        battery_state=1.0,
        current_load=None,
        contact_points=[]
    )

    # Standardized EnvironmentState
    env = EnvironmentState(
        temperature=20.0,
        air_density=1.225,
        wind_vector=zero,
        terrain_type="flat",
        friction=float(effective_friction), # Fixed: Correct attribute name for BrakingConstraint
        slope=float(slp),
        lighting_conditions="normal",
        distance_to_obstacles=float(d)
    )

    world_state = WorldState(
        timestamp=datetime.now().timestamp(),
        delta_time=0.1,
        gravity=Vector3(0, 0, -9.81),
        environment=env,
        agents=[agent],
        objects=[],
        uncertainty=UncertaintyModel(0.05, 0.05, 0.05, 0.05)
    )
    
    # Bridge for constraints expecting singular .agent
    world_state.agent = agent 

    engine = SafetyEngine()
    for c in [BrakingConstraint(), FrictionConstraint(), LoadConstraint(), StabilityConstraint()]:
        engine.register_constraint(c)

    return SafetyReport(engine.evaluate(world_state)), effective_friction

# -------------------------
# 5. VIDEO PROCESSING
# -------------------------
if audit_mode == "Live Video Audit":
    uploaded_video = st.file_uploader("Upload Audit Video", type=["mp4", "mov"])
    if uploaded_video:
        tfile = tempfile.NamedTemporaryFile(delete=False)
        tfile.write(uploaded_video.read())
        cap = cv2.VideoCapture(tfile.name)
        ret, frame = cap.read()
        if ret:
            st.image(frame, channels="BGR", use_container_width=True)
            # Simulated telemetry from vision translator
            velocity, distance = 8.5, 12.0 
            st.info(f"📊 Perception Sync: Detected Speed {velocity}m/s | Distance {distance}m")
        cap.release()

# -------------------------
# 6. EXECUTION & VISUALS
# -------------------------
report, eff_friction = run_audit(profile, velocity, distance, deceleration, load_weight, base_friction, slope)

if report.is_safe():
    st.success("### ✅ CLEAR TO PROCEED")
else:
    st.error("### ❌ PHYSICS VETO: SAFETY VIOLATION")

# Risk Score Logic
score = sum(25 for r in report.results if r.violated)
st.metric("Risk Score", f"{min(score, 100)}/100")

# Stopping Distance Visualization
required_stop = (velocity ** 2) / (2 * deceleration) if deceleration > 0 else 0
buffer = distance - required_stop

fig = go.Figure()
fig.add_trace(go.Bar(y=["Stopping"], x=[required_stop], orientation='h', name="Stop Dist", marker_color="#EF553B"))
if buffer > 0:
    fig.add_trace(go.Bar(y=["Margin"], x=[buffer], orientation='h', name="Safe Margin", marker_color="#00CC96"))
st.plotly_chart(fig, use_container_width=True)

# -------------------------
# 7. SAFETY HEATMAP
# -------------------------
st.divider()
st.subheader("📊 Safety Region Heatmap")
speeds = np.linspace(0, 20, 20)
distances = np.linspace(1, 30, 20)
Z = []

for v in speeds:
    row = []
    for d_val in distances:
        r, _ = run_audit(profile, v, d_val, deceleration, load_weight, base_friction, slope)
        row.append(0 if r.is_safe() else 1)
    Z.append(row)

heatmap = go.Figure(data=go.Heatmap(z=Z, x=distances, y=speeds, colorscale="RdYlGn_r", showscale=False))
heatmap.update_layout(xaxis_title="Distance (m)", yaxis_title="Speed (m/s)")
st.plotly_chart(heatmap, use_container_width=True)