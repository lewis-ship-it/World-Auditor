import streamlit as st
import math
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
with st.expander("📖 System Overview", expanded=False):
    st.markdown("""
    **SafeBot** is a deterministic safety layer for autonomous systems.
    * **📹 Video Perception:** Extracts speed and hazard distance from footage. [cite: 96, 108]
    * **⚖️ Physics Audit:** Validates intent against stopping distance, tipping, and load limits. [cite: 112, 124, 143]
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
    
    surface_key = st.selectbox("Surface Type", list(SURFACE_MAP.keys()))
    base_friction = st.slider("Surface Friction (μ)", 0.05, 1.0, SURFACE_MAP[surface_key])
    
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
    # Standard effective friction calculation
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
# 5. VERDICT & PHYSICS RUNWAY
# -------------------------
report, final_friction = run_audit(profile, velocity, distance, deceleration, load_weight, base_friction, slope)

# VERDICT HEADER
if report.is_safe():
    st.markdown('<div class="status-box safe-glow"><h1>✅ MISSION CAPABLE</h1></div>', unsafe_allow_html=True)
else:
    st.markdown('<div class="status-box danger-glow"><h1>❌ PHYSICS VETO</h1></div>', unsafe_allow_html=True)

# THE INTERACTIVE RUNWAY SIMULATION
st.subheader("🏁 Kinematic Runway & Stability Vector")

# Calculations for Simulation
thinking_dist = velocity * latency
stopping_dist_mech = (velocity ** 2) / (2 * deceleration) if deceleration > 0 else 0
total_stop_dist = thinking_dist + stopping_dist_mech
ghost_dist = velocity * 1.5 # 1.5s projection
slope_rad = math.radians(slope)

# Determine collision
is_collision = total_stop_dist >= distance
sim_color = "#EF553B" if is_collision else "#00CC96"

fig_sim = go.Figure()

# 1. Slope-Adaptive Path
path_length = max(distance + 5, total_stop_dist + 5)
x_path = [0, path_length * math.cos(slope_rad)]
y_path = [0, path_length * math.sin(slope_rad)]
fig_sim.add_trace(go.Scatter(x=x_path, y=y_path, mode='lines', line=dict(color="#30363D", width=4)))

# 2. The Wall (Obstacle)
wall_x = distance * math.cos(slope_rad)
wall_y = distance * math.sin(slope_rad)
# Perpendicular wall line
fig_sim.add_shape(type="line", x0=wall_x - math.sin(slope_rad), y0=wall_y + math.cos(slope_rad), 
                  x1=wall_x + math.sin(slope_rad), y1=wall_y - math.cos(slope_rad), 
                  line=dict(color=sim_color, width=8))

# 3. Stopping Buffers (Thinking vs Braking)
think_x = thinking_dist * math.cos(slope_rad)
think_y = thinking_dist * math.sin(slope_rad)
fig_sim.add_trace(go.Scatter(x=[0, think_x], y=[0.2, think_y + 0.2], mode='lines', 
                             line=dict(color="#FFD700", width=15), opacity=0.4, name="Thinking Time"))
fig_sim.add_trace(go.Scatter(x=[think_x, total_stop_dist * math.cos(slope_rad)], 
                             y=[think_y + 0.2, (total_stop_dist * math.sin(slope_rad)) + 0.2], 
                             mode='lines', line=dict(color=sim_color, width=15), opacity=0.4, name="Mechanical Braking"))

# 4. The Ghost Dot (Projected Future)
ghost_x = ghost_dist * math.cos(slope_rad)
ghost_y = ghost_dist * math.sin(slope_rad)
fig_sim.add_trace(go.Scatter(x=[ghost_x], y=[ghost_y], mode='markers', 
                             marker=dict(size=15, color="white", opacity=0.3, symbol="circle"), name="Ghost Projection"))

# 5. Stability Vector (Center of Mass)
# Resultant vector of gravity and inertia
inertia_force = velocity * 0.5 # Simplified visual scale
resultant_x = 0 + (inertia_force * math.cos(slope_rad))
resultant_y = 1 - (9.81 * 0.1) # Gravity pull
fig_sim.add_trace(go.Scatter(x=[0, resultant_x], y=[0, resultant_y], mode='lines+markers', 
                             line=dict(color="#00D4FF", width=3, dash="dot"), name="Stability Vector"))

# 6. The Robot (Dot)
fig_sim.add_trace(go.Scatter(x=[0], y=[0], mode='markers+text', 
                             marker=dict(size=25, color="white", symbol="square"),
                             text=["ROBOT"], textposition="bottom center"))

fig_sim.update_layout(height=350, margin=dict(l=10, r=10, t=10, b=10), showlegend=True,
                      xaxis=dict(showgrid=False, zeroline=False), yaxis=dict(showgrid=False, zeroline=False),
                      paper_bgcolor='rgba(0,0,0,0)', plot_bgcolor='rgba(0,0,0,0)', font=dict(color="white"))

st.plotly_chart(fig_sim, use_container_width=True)

# Metrics Dashboard
col1, col2, col3, col4 = st.columns(4)
col1.metric("Stopping Distance", f"{total_stop_dist:.2f}m", delta=f"{distance - total_stop_dist:.1f}m Margin")
col2.metric("Grip (μ)", f"{final_friction:.2f}")
col3.metric("Brake Force", f"{deceleration}m/s²")
col4.metric("Risk Score", f"{report.risk_score()}", delta="High Risk" if not report.is_safe() else "Normal", delta_color="inverse")

# -------------------------
# 6. ANALYST INSIGHTS & LOGS
# -------------------------
if not report.is_safe():
    with st.container():
        st.error("### 🧠 Safety Analyst Insight")
        for res in report.results:
            if res.violated:
                if "Braking" in res.name:
                    st.write(f"⚠️ **Kinematic Conflict:** At {velocity}m/s, your 'Thinking Distance' alone is {thinking_dist:.1f}m. You will hit the obstacle before the brakes even lock.") [cite: 115, 116]
                if "Tipping" in res.name or "Stability" in res.name:
                    st.write(f"⚠️ **Instability:** The {slope}° slope exceeds the tipping threshold for a {profile_name}.") [cite: 124, 143]

# -------------------------
# 7. MONTE CARLO STRESS TEST (NEW)
# -------------------------
with st.expander("🎲 Monte Carlo Environmental Stress Test", expanded=False):
    st.write("Running 50 simulations with ±10% variation in friction and slope...")
    passes = 0
    for _ in range(50):
        test_fric = base_friction * random.uniform(0.9, 1.1)
        test_slope = slope + random.uniform(-2, 2)
        sim_report, _ = run_audit(profile, velocity, distance, deceleration, load_weight, test_fric, test_slope)
        if sim_report.is_safe(): passes += 1
    
    reliability = (passes / 50) * 100
    st.progress(reliability / 100)
    st.write(f"**Environmental Reliability:** {reliability}%")
    if reliability < 100:
        st.warning("Mission is vulnerable to small environmental changes (e.g. wet patches or uneven ground).")