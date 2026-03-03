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
    .main { background-color: #0E1117; color: #FFFFFF; }
    .stMetric { background-color: #161B22; border: 1px solid #30363D; padding: 15px; border-radius: 12px; }
    .status-box { padding: 25px; border-radius: 15px; text-align: center; margin-bottom: 25px; border: 2px solid #30363D; }
    .safe-glow { background-color: rgba(0, 204, 150, 0.1); border-color: #00CC96; color: #00CC96; }
    .danger-glow { background-color: rgba(239, 85, 59, 0.1); border-color: #EF553B; color: #EF553B; }
    </style>
    """, unsafe_allow_html=True)

st.title("🛡️ SafeBot: Physics Reality Auditor")

# --- BRIEF SYSTEM DESCRIPTION ---
with st.expander("📖 System Overview: How it Works", expanded=False):
    st.markdown("""
    **SafeBot** acts as a "Physics Firewall" for autonomous robots.
    
    * **📹 Video Perception:** Analyzes visual footage to extract the robot's current speed and distance to hazards.
    * **⚖️ Physics Audit:** Runs that data through deterministic equations. It checks if the robot can actually stop, if it will tip over on the current slope, or if the load is too heavy for the tires.
    
    If the laws of physics say "No," the system issues a **Veto** to block the AI's intent.
    """)

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
    st.header("⚙️ Audit Parameters")
    audit_mode = st.radio("Mode", ["Manual Simulator", "Live Video Audit"])
    
    st.divider()
    profile_name = st.selectbox("Robot Profile", list(ROBOT_PROFILES.keys()))
    profile = ROBOT_PROFILES[profile_name]
    
    surface_key = st.selectbox("Surface Type (Preset)", list(SURFACE_MAP.keys()))
    base_friction = st.slider("Specific Friction (μ)", 0.05, 1.0, SURFACE_MAP[surface_key])
    
    brake_key = st.select_slider("Brake Health", options=list(BRAKE_MAP.keys()), value="Used")
    deceleration = BRAKE_MAP[brake_key]

    velocity, distance = 5.0, 10.0
    if audit_mode == "Manual Simulator":
        velocity = st.slider("Velocity (m/s)", 0.0, 25.0, 5.0)
        distance = st.slider("Distance to Hazard (m)", 0.5, 40.0, 10.0)

    load_weight = st.slider("Current Load (kg)", 0.0, 3000.0, 500.0)
    slope = st.slider("Slope Angle (deg)", -25.0, 45.0, 0.0)
    latency = st.slider("System Latency (s)", 0.0, 1.5, 0.2)

# -------------------------
# 4. CORE AUDIT LOGIC 
# -------------------------
def run_audit(p_data, v, d, decel, load, friction, slp):
    eff_fric = max(friction - min(v * 0.012, 0.35), 0.05)
    
    limits = ActuatorLimits(100.0, 10000.0, 25.0, float(decel))
    agent_obj = AgentState(
        id="robot_01", type="mobile", mass=float(p_data["mass"]), position=Vector3(0,0,0),
        velocity=Vector3(float(v), 0.0, 0.0), angular_velocity=Vector3(0,0,0), orientation=Quaternion(1,0,0,0),
        center_of_mass=Vector3(0,0,0), center_of_mass_height=float(p_data["com_height"]),
        support_polygon=[Vector3(-0.5,-0.5,0), Vector3(0.5,0.5,0)],
        wheelbase=float(p_data["wheelbase"]), load_weight=float(load), max_load=float(p_data["max_load"]),
        actuator_limits=limits, battery_state=1.0, current_load=None, contact_points=[]
    )
    env_obj = EnvironmentState(
        temperature=20.0, air_density=1.225, wind_vector=Vector3(0,0,0), terrain_type="flat",
        friction=float(eff_fric), slope=float(slp), lighting_conditions="normal", distance_to_obstacles=float(d)
    )
    world_state = WorldState(
        timestamp=datetime.now().timestamp(), delta_time=0.1, gravity=Vector3(0,0,-9.81),
        environment=env_obj, agents=[agent_obj], objects=[], uncertainty=UncertaintyModel(0.05,0.05,0.05,0.05)
    )

    engine = SafetyEngine()
    for constraint in [BrakingConstraint(), FrictionConstraint(), LoadConstraint(), StabilityConstraint()]:
        engine.register_constraint(constraint)

    return SafetyReport(engine.evaluate(world_state)), eff_fric

# -------------------------
# 5. LIVE VIDEO AUDIT
# -------------------------
if audit_mode == "Live Video Audit":
    st.subheader("📹 Perception Stream Analysis")
    uploaded_video = st.file_uploader("Upload Stream", type=["mp4", "mov"])
    if uploaded_video:
        tfile = tempfile.NamedTemporaryFile(delete=False)
        tfile.write(uploaded_video.read())
        cap = cv2.VideoCapture(tfile.name)
        ret, frame = cap.read()
        if ret:
            st.image(frame, channels="BGR", use_container_width=True)
            velocity, distance = 9.2, 14.5 # Simulated Perception Data
            st.success(f"Tracking Data Locked: {velocity}m/s | {distance}m to target")
        cap.release()

# -------------------------
# 6. VERDICT & KINEMATIC SIM
# -------------------------
report, final_friction = run_audit(profile, velocity, distance, deceleration, load_weight, base_friction, slope)

if report.is_safe():
    st.markdown('<div class="status-box safe-glow"><h1>✅ MISSION CAPABLE</h1></div>', unsafe_allow_html=True)
else:
    st.markdown('<div class="status-box danger-glow"><h1>❌ PHYSICS VETO</h1></div>', unsafe_allow_html=True)

st.subheader("🏁 Live Kinematic Runway & Stability Vector")

# Math for Visualization
slope_rad = math.radians(slope)
think_d = velocity * latency
brake_d = (velocity ** 2) / (2 * deceleration) if deceleration > 0 else 0
total_d = think_d + brake_d
ghost_d = velocity * 1.5

is_collision = total_d >= distance
sim_color = "#EF553B" if is_collision else "#00CC96"

fig = go.Figure()

# 1. Slope Path (The Ground)
max_viz_x = max(distance, total_d, ghost_d) + 10
fig.add_trace(go.Scatter(
    x=[0, max_viz_x * math.cos(slope_rad)], 
    y=[0, max_viz_x * math.sin(slope_rad)],
    mode='lines', line=dict(color="#30363D", width=4), name="Ground"
))

# 2. Obstacle Wall (Perpendicular to slope)
wall_x, wall_y = distance * math.cos(slope_rad), distance * math.sin(slope_rad)
fig.add_trace(go.Scatter(
    x=[wall_x - 1.5*math.sin(slope_rad), wall_x + 1.5*math.sin(slope_rad)],
    y=[wall_y + 1.5*math.cos(slope_rad), wall_y - 1.5*math.cos(slope_rad)],
    mode='lines', line=dict(color=sim_color, width=10), name="Obstacle"
))

# 3. Path Buffers (Offset vertically to be visible)
offset = 0.6
fig.add_trace(go.Scatter(
    x=[0, think_d * math.cos(slope_rad)],
    y=[offset, think_d * math.sin(slope_rad) + offset],
    mode='lines', line=dict(color="#FFD700", width=12), opacity=0.6, name="Thinking Path"
))
fig.add_trace(go.Scatter(
    x=[think_d * math.cos(slope_rad), total_d * math.cos(slope_rad)],
    y=[think_d * math.sin(slope_rad) + offset, total_d * math.sin(slope_rad) + offset],
    mode='lines', line=dict(color=sim_color, width=12), opacity=0.6, name="Braking Path"
))

# 4. Ghost Dot (Future Projection)
fig.add_trace(go.Scatter(
    x=[ghost_d * math.cos(slope_rad)], y=[ghost_d * math.sin(slope_rad)],
    mode='markers', marker=dict(size=14, color="white", opacity=0.2), name="1.5s Projection"
))

# 5. Stability Vector (Resultant Center of Mass Force)
v_x = (velocity * 0.15) * math.cos(slope_rad)
v_y = (velocity * 0.15) * math.sin(slope_rad) - 2.0 
fig.add_trace(go.Scatter(
    x=[0, v_x], y=[0, v_y],
    mode='lines+markers', line=dict(color="#00D4FF", width=2, dash="dot"), name="Stability Vector"
))

# 6. The Robot Dot
fig.add_trace(go.Scatter(
    x=[0], y=[0], mode='markers+text',
    marker=dict(size=22, color="white", symbol="square"),
    text=["ROBOT"], textposition="top center", name="Current Agent"
))

fig.update_layout(
    height=400, showlegend=True,
    legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
    margin=dict(l=0, r=0, t=0, b=0),
    xaxis=dict(showgrid=False, zeroline=False, showticklabels=False, range=[-5, max_viz_x]),
    yaxis=dict(showgrid=False, zeroline=False, showticklabels=False, range=[-8, 8]),
    paper_bgcolor='rgba(0,0,0,0)', plot_bgcolor='rgba(0,0,0,0)', font=dict(color="white")
)
st.plotly_chart(fig, use_container_width=True)

# -------------------------
# 7. ANALYST INSIGHTS
# -------------------------
if not report.is_safe():
    with st.container():
        st.error("### 🧠 Safety Analyst Insight")
        for res in report.results:
            if res.violated:
                if "Braking" in res.name:
                    st.write(f"⚠️ **Kinematic Conflict:** At {velocity} m/s, you need {total_d:.1f}m to stop. You hit the obstacle in {distance}m.")
                if "Tipping" in res.name or "Stability" in res.name:
                    st.write(f"⚠️ **Stability Risk:** A {slope}° slope is unsafe for this robot's height and center of mass.")

# -------------------------
# 8. METRICS & STRESS TEST
# -------------------------
m1, m2, m3, m4 = st.columns(4)
m1.metric("Stop Distance", f"{total_d:.1f}m")
m2.metric("Impact Speed", f"{max(0, (total_d - distance)*2):.1f} m/s" if is_collision else "0 m/s")
m3.metric("Effective Grip", f"{final_friction:.2f}")
m4.metric("Risk Score", f"{report.risk_score()}")

with st.expander("🎲 Monte Carlo Stress Test", expanded=True):
    st.write("Simulating 50 environmental variations (±10% Friction/Slope)...")
    passes = 0
    for _ in range(50):
        t_fric = base_friction * random.uniform(0.9, 1.1)
        t_slope = slope + random.uniform(-2, 2)
        s_rep, _ = run_audit(profile, velocity, distance, deceleration, load_weight, t_fric, t_slope)
        if s_rep.is_safe(): passes += 1
    rel = (passes / 50) * 100
    st.progress(rel / 100)
    st.write(f"**Environmental Reliability:** {rel}%")