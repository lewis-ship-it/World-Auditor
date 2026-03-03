import streamlit as st
import math
import random
import plotly.graph_objects as go
from datetime import datetime
import numpy as np
import cv2
import tempfile
import time

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
    .stExpander { border: 1px solid #30363D !important; background-color: #0E1117 !important; }
    </style>
    """, unsafe_allow_html=True)

st.title("🛡️ SafeBot: Physics Reality Auditor")

# -------------------------
# 3. SIDEBAR & PARAMETERS
# -------------------------
ROBOT_PROFILES = {
    "Warehouse Forklift": {"mass": 4000.0, "max_load": 2000.0, "com_height": 1.2, "wheelbase": 2.0},
    "Delivery Rover": {"mass": 80.0, "max_load": 20.0, "com_height": 0.4, "wheelbase": 0.6},
    "Standard Sedan": {"mass": 1500.0, "max_load": 500.0, "com_height": 0.6, "wheelbase": 2.7}
}

SURFACE_MAP = {"Dry Concrete": 0.8, "Wet Asphalt": 0.4, "Ice": 0.15}
BRAKE_MAP = {"New": 5.0, "Used": 2.5, "Failing": 1.0}

with st.sidebar:
    st.header("⚙️ Audit Parameters")
    audit_mode = st.radio("Mode", ["Manual Simulator", "Live Video Audit"])
    
    st.divider()
    profile_name = st.selectbox("Robot Profile", list(ROBOT_PROFILES.keys()))
    profile = ROBOT_PROFILES[profile_name]
    
    surface_key = st.selectbox("Surface Type", list(SURFACE_MAP.keys()))
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
# 4. CORE ENGINE LOGIC
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
        friction=float(eff_fric), slope=float(slp), distance_to_obstacles=float(d),
        temperature=20.0, air_density=1.225, wind_vector=Vector3(0,0,0), terrain_type="flat", lighting_conditions="normal"
    )
    world_state = WorldState(
        timestamp=datetime.now().timestamp(), delta_time=0.1, gravity=Vector3(0,0,-9.81),
        environment=env_obj, agents=[agent_obj], objects=[], uncertainty=UncertaintyModel(0.05,0.05,0.05,0.05)
    )
    engine = SafetyEngine()
    for c in [BrakingConstraint(), FrictionConstraint(), LoadConstraint(), StabilityConstraint()]:
        engine.register_constraint(c)
    return SafetyReport(engine.evaluate(world_state)), eff_fric

# -------------------------
# 5. LIVE VIDEO MODULE (RESTORED)
# -------------------------
if audit_mode == "Live Video Audit":
    st.subheader("📹 Perception Analysis")
    uploaded_video = st.file_uploader("Upload Stream", type=["mp4", "mov"])
    if uploaded_video:
        tfile = tempfile.NamedTemporaryFile(delete=False)
        tfile.write(uploaded_video.read())
        cap = cv2.VideoCapture(tfile.name)
        ret, frame = cap.read()
        if ret:
            st.image(frame, channels="BGR", use_container_width=True)
            velocity, distance = 8.4, 12.2 # Mock extraction
            st.success(f"Tracking: {velocity}m/s | {distance}m to wall")
        cap.release()

# -------------------------
# 6. REAL-TIME SIMULATION
# -------------------------
report, final_friction = run_audit(profile, velocity, distance, deceleration, load_weight, base_friction, slope)

if report.is_safe():
    st.markdown('<div class="status-box safe-glow"><h1>✅ MISSION CAPABLE</h1></div>', unsafe_allow_html=True)
else:
    st.markdown('<div class="status-box danger-glow"><h1>❌ PHYSICS VETO</h1></div>', unsafe_allow_html=True)

st.subheader("🏁 Real-Time Physics Runway")
sim_container = st.empty()
run_sim = st.button("▶️ Start Real-Time Mission")

def draw_world(current_pos):
    s_rad = math.radians(slope)
    think_d = velocity * latency
    stop_d_mech = (velocity**2 / (2 * deceleration)) if deceleration > 0 else 0
    total_stop = think_d + stop_d_mech
    
    fig = go.Figure()
    m_x = max(distance, total_stop) + 10
    
    # Ground & Wall
    fig.add_trace(go.Scatter(x=[0, m_x*math.cos(s_rad)], y=[0, m_x*math.sin(s_rad)], mode='lines', line=dict(color="#30363D", width=4)))
    wx, wy = distance*math.cos(s_rad), distance*math.sin(s_rad)
    fig.add_trace(go.Scatter(x=[wx-1.2*math.sin(s_rad), wx+1.2*math.sin(s_rad)], y=[wy+1.2*math.cos(s_rad), wy-1.2*math.cos(s_rad)], 
                             mode='lines', line=dict(color="#EF553B" if total_stop >= distance else "#00CC96", width=8)))
    
    # Static Shadows (Ghost)
    off = 0.8
    fig.add_trace(go.Scatter(x=[0, think_d*math.cos(s_rad)], y=[off, think_d*math.sin(s_rad)+off], mode='lines', line=dict(color="#FFD700", width=8), opacity=0.3))
    
    # Robot
    rx, ry = current_pos*math.cos(s_rad), current_pos*math.sin(s_rad)
    vx, vy = rx + (velocity*0.12)*math.cos(s_rad), ry + (velocity*0.12)*math.sin(s_rad) - 1.8
    fig.add_trace(go.Scatter(x=[rx, vx], y=[ry, vy], mode='lines', line=dict(color="#00D4FF", dash="dot")))
    fig.add_trace(go.Scatter(x=[rx], y=[ry], mode='markers', marker=dict(size=25, color="white", symbol="square")))
    
    fig.update_layout(height=400, showlegend=False, margin=dict(l=0,r=0,t=0,b=0),
                      xaxis=dict(range=[-5, m_x], showgrid=False, zeroline=False), yaxis=dict(range=[-10, 10], showgrid=False))
    return fig

if run_sim:
    start_t = time.time()
    t_stop_total = velocity * latency + (velocity**2 / (2 * deceleration))
    while True:
        elapsed = time.time() - start_t
        curr_p = velocity * elapsed
        sim_container.plotly_chart(draw_world(curr_p), use_container_width=True)
        if curr_p >= distance or curr_p >= t_stop_total: break
        time.sleep(0.01)
else:
    sim_container.plotly_chart(draw_world(0), use_container_width=True)

# -------------------------
# 7. ANALYSIS & HEATMAP (RESTORED)
# -------------------------
st.divider()
col1, col2, col3 = st.columns([1, 1, 2])

with col1:
    st.subheader("🎲 Monte Carlo Stress Test")
    st.caption("Testing 100 variations of environment ±10%.")
    passes = sum(1 for _ in range(100) if run_audit(profile, velocity, distance, deceleration, load_weight, base_friction * random.uniform(0.9, 1.1), slope + random.uniform(-2, 2))[0].is_safe())
    st.metric("Reliability Score", f"{passes}%")
    st.progress(passes/100)

with col2:
    st.subheader("📊 Physics Metrics")
    st.metric("Risk Score", f"{report.risk_score()}")
    if not report.is_safe():
        for res in report.results:
            if res.violated: st.warning(f"⚠️ {res.name}")

with col3:
    st.subheader("🔍 Safety Envelope")
    v_ax, d_ax = np.linspace(0, 25, 15), np.linspace(1, 40, 15)
    grid = [[1 if run_audit(profile, vi, dj, deceleration, load_weight, base_friction, slope)[0].is_safe() else 0 for dj in d_ax] for vi in v_ax]
    st.plotly_chart(go.Figure(data=go.Heatmap(z=grid, x=d_ax, y=v_ax, colorscale=[[0,'#EF553B'],[1,'#00CC96']], showscale=False)).update_layout(height=300, margin=dict(l=0,r=0,t=0,b=0), xaxis_title="Distance", yaxis_title="Velocity"), use_container_width=True)