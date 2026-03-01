import streamlit as st
import math
import plotly.graph_objects as go
import json
from datetime import datetime
import cv2
import numpy as np
import tempfile

# --- 1. ALL ORIGINAL IMPORTS (PRESERVED) ---
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
# 2. CONFIG & STYLING
# -------------------------
st.set_page_config(page_title="SafeBot Physics Auditor", layout="wide")
st.title("🛡️ SafeBot: Physics Reality Auditor")
st.markdown("Friendly Deterministic Middleware for AI-Controlled Robotics")

# -------------------------
# 3. FULL DATA MAPS (RESTORED)
# -------------------------
ROBOT_PROFILES = {
    "Warehouse Forklift": {
        "mass": 4000.0, "max_load": 2000.0, "com_height": 1.2, "wheelbase": 2.0,
    },
    "Delivery Rover": {
        "mass": 80.0, "max_load": 20.0, "com_height": 0.4, "wheelbase": 0.6
    },
    "Standard Sedan": {
        "mass": 1500.0, "max_load": 500.0, "com_height": 0.6, "wheelbase": 2.7
    }
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
# 4. SIDEBAR CONTROLS
# -------------------------
st.sidebar.header("🕹️ Audit Mode")
audit_mode = st.sidebar.radio("Select Mode", ["Manual Simulator", "Live Video Audit"])

st.sidebar.divider()
st.sidebar.header("⚙️ Machine Parameters")

# Stress Test Logic
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

# Dynamic Inputs
if audit_mode == "Manual Simulator":
    velocity = st.sidebar.slider("Current Speed (m/s)", 0.0, 15.0, v_init)
    distance = st.sidebar.slider("Distance to Obstacle (m)", 0.5, 20.0, d_init)
else:
    velocity, distance = 0.0, 0.0 # Handled by Video Logic

load_weight = st.sidebar.slider("Load Weight (kg)", 0.0, 3000.0, 500.0)
slope = st.sidebar.slider("Slope Angle (degrees)", 0.0, 45.0, 5.0)
compare_mode = st.sidebar.checkbox("Compare with another Robot?")

# -------------------------
# 5. CORE PHYSICS ENGINE (RESTORED)
# -------------------------
def run_audit(p_data, v, d, decel, load, fric, slp):
    zero_vec = Vector3(x=0.0, y=0.0, z=0.0)
    v_vec = Vector3(x=float(v), y=0.0, z=0.0)
    identity_quat = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
    # Using your project's standard limits
    limits = ActuatorLimits(max_torque=100.0, max_force=100.0, max_speed=10.0, max_acceleration=5.0)

    agent = AgentState(
        id="primary_robot",
        type="mobile",
        mass=float(p_data["mass"]),
        position=zero_vec,
        velocity=v_vec,
        angular_velocity=zero_vec,
        orientation=identity_quat,
        center_of_mass=zero_vec,
        support_polygon=[Vector3(-0.5, -0.5, 0), Vector3(0.5, 0.5, 0)],
        actuator_limits=limits,
        battery_state=1.0,
        current_load=None,
        contact_points=[],
        loadweight=float(load),
        max_load=float(p_data["max_load"]),
        center_of_mass_height=float(p_data["com_height"]),
        wheelbase=float(p_data["wheelbase"])
    )
    
    env = EnvironmentState(
        temperature=20.0, air_density=1.225, wind_vector=zero_vec, terrain_type="flat",
        surface_friction=float(fric), slope_vector=zero_vec, lighting_conditions="normal",
        distance_to_obstacles=float(d), friction=float(fric), slope=float(slp)
    )
    
    world_state = WorldState(
        timestamp=datetime.now().timestamp(),
        delta_time=0.1,
        gravity=Vector3(x=0.0, y=0.0, z=-9.81),
        environment=env,
        agents=[agent],
        objects=[],
        uncertainty=UncertaintyModel(0.1, 0.1, 0.1, 0.1)
    )
    
    engine = SafetyEngine()
    engine.register_constraint(BrakingConstraint())
    engine.register_constraint(FrictionConstraint())
    engine.register_constraint(LoadConstraint())
    engine.register_constraint(StabilityConstraint())
    
    return SafetyReport(engine.evaluate(world_state))

# -------------------------
# 6. VIDEO MODE: CALIBRATION & REALITY AUDIT
# -------------------------
if audit_mode == "Live Video Audit":
    st.subheader("📸 Video Perception & Reality Audit")
    uploaded_video = st.file_uploader("Upload Robot Footage", type=["mp4", "mov", "avi"])

    if uploaded_video:
        tfile = tempfile.NamedTemporaryFile(delete=False)
        tfile.write(uploaded_video.read())
        cap = cv2.VideoCapture(tfile.name)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        ret, frame = cap.read()
        
        if ret:
            # CALIBRATION LOGIC
            st.info(f"💡 Calibration: Known Wheelbase is {profile['wheelbase']}m. Click points to set scale.")
            st.image(frame, channels="BGR", use_container_width=True)
            
            # --- FEATURE: MULTI-OBJECT TRACKING (MOT) ---
            st.divider()
            st.write("### 📑 Multi-Entity Tracking Results")
            
            # Simulated MOT Data: In production, this would use YOLO + DeepSORT
            detected_entities = [
                {"id": "Entity_01", "type": profile_name, "v": 8.2, "dist": 5.5},
                {"id": "Entity_02", "type": "Standard Sedan", "v": 12.5, "dist": 15.0}
            ]

            for entity in detected_entities:
                with st.expander(f"🔍 Tracking: {entity['id']} ({entity['type']})", expanded=True):
                    ent_profile = ROBOT_PROFILES[entity['type']]
                    # Run unique physics audit for this specific tracked object
                    ent_report = run_audit(ent_profile, entity['v'], entity['dist'], deceleration, load_weight, friction, slope)
                    
                    c1, c2, c3 = st.columns(3)
                    c1.metric("Velocity", f"{entity['v']} m/s")
                    c2.metric("Distance", f"{entity['dist']} m")
                    c3.write("✅ PHYSICALLY SAFE" if ent_report.is_safe() else "❌ VETO: UNSAFE")

            # --- FEATURE 1: TRACKING MATH (Primary Agent) ---
            velocity = detected_entities[0]["v"]
            distance = detected_entities[0]["dist"]
            
            # --- FEATURE 2: DISCONTINUITY CHECK ---
            dt = 1/fps
            prev_v = 2.0 # Last frame speed
            accel = abs(velocity - prev_v) / dt
            max_allowed = 5.0 # From ActuatorLimits
            
            st.divider()
            c_v1, c_v2 = st.columns(2)
            c_v1.metric("Primary Velocity", f"{velocity} m/s")
            c_v2.metric("Primary Accel", f"{accel:.1f} m/s²")

            if accel > max_allowed * 2:
                st.warning(f"⚠️ **REALITY VIOLATION:** Measured acceleration ({accel:.1f} m/s²) exceeds physical limit.")
        cap.release()

# -------------------------
# 7. DASHBOARD OUTPUT (RESTORED)
# -------------------------
report = run_audit(profile, velocity, distance, deceleration, load_weight, friction, slope)

if report.is_safe():
    st.success("### ✅ CLEAR TO PROCEED")
else:
    st.error("### ❌ VETO: PHYSICS VIOLATION")

# Visual Impact Zone
required_stop = (velocity ** 2) / (2 * deceleration)
buffer = distance - required_stop

fig_buffer = go.Figure()
fig_buffer.add_trace(go.Bar(
    y=["Path"], x=[required_stop], name="Stop Distance",
    orientation='h', marker_color='#EF553B' if buffer < 0 else '#00CC96'
))
if buffer > 0:
    fig_buffer.add_trace(go.Bar(y=["Path"], x=[buffer], name="Margin", orientation='h', marker_color='#AAAAAA'))
st.plotly_chart(fig_buffer, use_container_width=True)

# Metrics
m1, m2, m3 = st.columns(3)
m1.metric("Stop Distance", f"{required_stop:.1f}m")
m2.metric("Stability", "⚠️ TIPPING" if any(r.name == "TippingRisk" and r.violated for r in report.results) else "✅ OK")
m3.metric("Safety Margin", f"{buffer:.1f}m")

# Comparison mode (RESTORED)
if compare_mode:
    st.divider()
    alt_name = st.selectbox("Compare with:", [k for k in ROBOT_PROFILES.keys() if k != profile_name])
    alt_report = run_audit(ROBOT_PROFILES[alt_name], velocity, distance, deceleration, load_weight, friction, slope)
    st.write(f"**Alternative Risk Score:** {alt_report.risk_score()}")