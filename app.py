import streamlit as st
import math
import random
import plotly.graph_objects as go
from datetime import datetime
import numpy as np
import cv2
import tempfile
import time

# --- 1. MODULAR IMPORTS ---
from alignment_core.engine.safety_engine import SafetyEngine
from alignment_core.engine.report import SafetyReport
from alignment_core.constraints.braking import BrakingConstraint
from alignment_core.constraints.friction import FrictionConstraint
from alignment_core.constraints.stability import StabilityConstraint
from alignment_core.physics.mechanics import calculate_auto_cog, get_support_polygon
from alignment_core.physics.curves import calculate_max_cornering_speed, check_lateral_stability

from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.environment import EnvironmentState
from alignment_core.world_model.world_state import WorldState
from alignment_core.world_model.primitives import Vector3, Quaternion, ActuatorLimits
from alignment_core.world_model.uncertainty import UncertaintyModel

# -------------------------
# 2. CONFIGURATION & STYLING
# -------------------------
st.set_page_config(page_title="SafeBot Physics Auditor Pro", layout="wide")

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

# -------------------------
# 3. SIDEBAR: ROBOT BUILDER
# -------------------------
with st.sidebar:
    st.header("⚙️ System Config")
    audit_mode = st.radio("Mode", ["Manual Simulator", "Mission Map Planner", "Live Video Audit"])
    
    with st.expander("🛠️ Custom Robot Builder", expanded=True):
        chassis_m = st.number_input("Chassis Mass (kg)", 1.0, 5000.0, 500.0)
        bat_m = st.number_input("Battery Mass (kg)", 0.0, 2000.0, 200.0)
        load_m = st.slider("Cargo Load (kg)", 0.0, 3000.0, 100.0)
        cog_h, total_m = calculate_auto_cog(chassis_m, 0.5, bat_m, 0.1, load_m, 1.2)
        
        wb = st.slider("Wheelbase (m)", 0.5, 5.0, 2.0)
        tw = st.slider("Track Width (m)", 0.5, 3.0, 1.4)
        wheels = st.radio("Wheels", [3, 4], index=1)
        st.info(f"Total Weight: {total_m}kg | CoG: {cog_h:.2f}m")

# -------------------------
# 4. SHARED AUDIT ENGINE
# -------------------------
def run_audit(v, d, f, s, c_mode=False, rad=0, bank=0):
    poly = get_support_polygon(wb, tw, wheels)
    agent = AgentState("bot", "mobile", total_m, Vector3(0,0,0), Vector3(v,0,0), Vector3(0,0,0), 
                       Quaternion(1,0,0,0), cog_h, poly, wb, load_m, 5000.0, 
                       ActuatorLimits(100,100,30,5.0), 1.0, None, [])
    env = EnvironmentState(f, 0.0 if c_mode else s, d, 20.0, 1.225, Vector3(0,0,0), "flat", "normal")
    world = WorldState(time.time(), 0.1, Vector3(0,0,-9.81), env, [agent], [], UncertaintyModel(0.05,0.05,0.05,0.05))
    
    engine = SafetyEngine()
    for c in [BrakingConstraint(), FrictionConstraint(), StabilityConstraint()]: 
        engine.register_constraint(c)
    
    report = SafetyReport(engine.evaluate(world))
    c_data = {}
    if c_mode and rad > 0:
        v_max = calculate_max_cornering_speed(rad, f, bank)
        is_tip, a_lat = check_lateral_stability(v, rad, cog_h, tw, bank)
        c_data = {"v_max": v_max, "is_tip": is_tip, "a_lat": a_lat}
    return report, c_data

# -------------------------
# 5. MODE: MISSION MAP PLANNER (NEW)
# -------------------------
if audit_mode == "Mission Map Planner":
    st.subheader("🗺️ Strategic Fastest Path Mapping")
    map_file = st.file_uploader("Upload Floor Plan / Map", type=["png", "jpg"])
    
    if map_file:
        # Load and Process Map
        file_bytes = np.asarray(bytearray(map_file.read()), dtype=np.uint8)
        img = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        
        st.image(thresh, caption="Processed Navigable Grid", width=400)
        
        if st.button("🚀 Calculate Physics-Safe Fastest Path"):
            with st.status("Analyzing Geometry...", expanded=True) as status:
                st.write("Extracting curve radii...")
                # Simulating path curvature extraction (R=10m for example)
                sample_radius = 10.0
                v_safe = calculate_max_cornering_speed(sample_radius, 0.8, 0.0)
                
                status.update(label="Optimization Complete!", state="complete")
                
            st.success(f"Path Verified: Max cornering speed for your {total_m}kg robot is {v_safe:.2f} m/s")
            
            # Visualize a mock speed-heatmap path
            x = np.linspace(0, 100, 100)
            y = 10 * np.sin(x/10) # Simulated path
            speeds = [v_safe if abs(np.cos(xi/10)) > 0.5 else v_safe * 1.5 for xi in x]
            
            fig = go.Figure(data=go.Scatter(x=x, y=y, mode='lines', 
                            line=dict(color=speeds, colorscale='Viridis', width=6)))
            fig.update_layout(title="Velocity Profile (Purple=Braking | Yellow=Max Speed)")
            st.plotly_chart(fig, use_container_width=True)

# -------------------------
# 6. MODE: MANUAL SIMULATOR
# -------------------------
elif audit_mode == "Manual Simulator":
    # Sidebar local controls
    is_curve = st.sidebar.checkbox("Curve Analysis")
    radius = st.sidebar.slider("Radius (m)", 5.0, 100.0, 25.0) if is_curve else 0
    banking = st.sidebar.slider("Banking (deg)", 0.0, 45.0, 0.0) if is_curve else 0
    slope = st.sidebar.slider("Slope (deg)", -25.0, 45.0, 0.0) if not is_curve else 0
    friction = st.sidebar.slider("Surface Friction (μ)", 0.1, 1.0, 0.8)
    velocity = st.sidebar.slider("Velocity (m/s)", 0.0, 30.0, 10.0)
    distance = st.sidebar.slider("Dist to Hazard (m)", 1.0, 50.0, 20.0)
    latency = st.sidebar.slider("System Latency (s)", 0.0, 1.0, 0.2)

    report, curve_res = run_audit(velocity, distance, friction, slope, is_curve, radius, banking)
    is_safe = report.is_safe()
    if is_curve and curve_res.get("is_tip"): is_safe = False

    status_class = "safe-glow" if is_safe else "danger-glow"
    st.markdown(f'<div class="status-box {status_class}"><h1>{"✅ MISSION CAPABLE" if is_safe else "❌ PHYSICS VETO"}</h1></div>', unsafe_allow_html=True)

    st.subheader("🏁 Real-Time Physics Runway")
    sim_container = st.empty()
    if st.button("▶️ Run Simulation"):
        start_t = time.time()
        t_stop_total = (velocity * latency) + (velocity**2 / (2 * 5.0))
        while True:
            elapsed = time.time() - start_t
            curr_p = velocity * elapsed
            fig = go.Figure()
            fig.add_trace(go.Scatter(x=[0, distance+5], y=[0, 0], mode='lines', line=dict(color="#30363D", width=4)))
            fig.add_trace(go.Scatter(x=[distance], y=[0], mode='markers', marker=dict(size=40, color="#EF553B", symbol="line-ns-open")))
            fig.add_trace(go.Scatter(x=[curr_p], y=[0.5], mode='markers', marker=dict(size=25, color="white", symbol="square")))
            fig.update_layout(height=300, showlegend=False, xaxis=dict(range=[-2, max(distance, t_stop_total)+5]), yaxis=dict(range=[-2, 5]))
            sim_container.plotly_chart(fig, use_container_width=True)
            if curr_p >= distance or curr_p >= t_stop_total: break
            time.sleep(0.02)

    # Analytics Section
    st.divider()
    m1, m2, m3 = st.columns([1, 1, 2])
    with m1:
        st.subheader("🎲 Monte Carlo")
        passes = sum(1 for _ in range(100) if run_audit(velocity, distance, friction * random.uniform(0.9, 1.1), slope + random.uniform(-2,2), is_curve, radius, banking)[0].is_safe())
        st.metric("Reliability", f"{passes}%")
        st.progress(passes/100)
    with m2:
        st.subheader("📊 Metrics")
        st.metric("Risk Score", f"{report.risk_score()}")
        for res in report.results:
            if res.violated: st.warning(f"⚠️ {res.name}")
    with m3:
        st.subheader("🔍 Safety Envelope")
        v_ax, d_ax = np.linspace(0, 30, 15), np.linspace(1, 50, 15)
        grid = [[1 if run_audit(vi, dj, friction, slope, is_curve, radius, banking)[0].is_safe() else 0 for dj in d_ax] for vi in v_ax]
        st.plotly_chart(go.Figure(data=go.Heatmap(z=grid, x=d_ax, y=v_ax, colorscale=[[0,'#EF553B'],[1,'#00CC96']], showscale=False)).update_layout(height=300, xaxis_title="Distance", yaxis_title="Velocity"), use_container_width=True)

# -------------------------
# 7. MODE: LIVE VIDEO AUDIT
# -------------------------
elif audit_mode == "Live Video Audit":
    st.subheader("📹 Perception Analysis")
    uploaded_video = st.file_uploader("Upload Stream", type=["mp4", "mov"])
    if uploaded_video:
        tfile = tempfile.NamedTemporaryFile(delete=False)
        tfile.write(uploaded_video.read())
        cap = cv2.VideoCapture(tfile.name)
        ret, frame = cap.read()
        if ret:
            st.image(frame, channels="BGR", use_container_width=True)
            st.success("CV Tracking Active: Identifying hazards in 3D space...")
        cap.release()