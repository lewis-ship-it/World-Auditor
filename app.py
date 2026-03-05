import streamlit as st
import math
import random
import plotly.graph_objects as go
from datetime import datetime
import numpy as np
import cv2
import tempfile
import time
from PIL import Image

# --- 1. MODULAR IMPORTS [cite: 7] ---
# Direct imports to avoid circular initialization issues in __init__.py
from alignment_core.engine.safety_engine import SafetyEngine
from alignment_core.engine.report import SafetyReport
from alignment_core.constraints.braking import BrakingConstraint
from alignment_core.constraints.friction import FrictionConstraint
from alignment_core.constraints.load import LoadConstraint
from alignment_core.constraints.stability import StabilityConstraint
from alignment_core.constraints.base import ConstraintResult

# Physics & Mechanics Utilities [cite: 4, 7]
from alignment_core.physics.mechanics import calculate_auto_cog, get_support_polygon
from alignment_core.physics.curves import calculate_max_cornering_speed, check_lateral_stability

# World Model Data Structures [cite: 7, 8]
from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.environment import EnvironmentState
from alignment_core.world_model.world_state import WorldState
from alignment_core.world_model.primitives import Vector3, Quaternion, ActuatorLimits
from alignment_core.world_model.uncertainty import UncertaintyModel

# -------------------------
# 2. CONFIGURATION & STYLING [cite: 8-13]
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
# 3. SIDEBAR: ROBOT BUILDER [cite: 13-15]
# -------------------------
with st.sidebar:
    st.header("⚙️ System Config")
    audit_mode = st.radio("Mode", ["Manual Simulator", "Mission Map Planner", "Live Video Audit", "Real-Time Safety Shield"])
    
    with st.expander("🛠️ Custom Robot Builder", expanded=True):
        chassis_m = st.number_input("Chassis Mass (kg)", 1.0, 5000.0, 500.0)
        bat_m = st.number_input("Battery Mass (kg)", 0.0, 2000.0, 200.0)
        load_m = st.slider("Cargo Load (kg)", 0.0, 3000.0, 100.0)
        
        # Auto-calculate CoG using mechanics.py [cite: 14, 87]
        cog_h, total_m = calculate_auto_cog(chassis_m, 0.5, bat_m, 0.1, load_m, 1.2)
        
        wb = st.slider("Wheelbase (m)", 0.5, 5.0, 2.0)
        tw = st.slider("Track Width (m)", 0.5, 3.0, 1.4)
        wheels = st.radio("Wheels", [3, 4], index=1)
        st.info(f"Total Weight: {total_m}kg | CoG: {cog_h:.2f}m")

# -------------------------
# 4. SHARED AUDIT ENGINE [cite: 15-20]
# -------------------------
def run_audit(v, d, f, s, wb_in, tw_in, wh_in, total_m_in, cog_h_in, load_m_in, c_mode=False, rad=0, bank=0):
    # Support polygon for stability boundary [cite: 15, 88]
    poly = get_support_polygon(wb_in, tw_in, wh_in)
    
    # Initialize AgentState with required Vector3/Quaternion primitives [cite: 16, 105]
    agent = AgentState(
        id="robot_01",
        type="mobile",
        mass=float(total_m_in),
        position=Vector3(0.0, 0.0, 0.0), 
        velocity=Vector3(float(v), 0.0, 0.0), 
        angular_velocity=Vector3(0.0, 0.0, 0.0),
        orientation=Quaternion(1.0, 0.0, 0.0, 0.0), 
        center_of_mass=Vector3(0.0, 0.0, 0.0),
        center_of_mass_height=float(cog_h_in),
        support_polygon=poly,
        wheelbase=float(wb_in),
        load_weight=float(load_m_in),
        max_load=5000.0,
        actuator_limits=ActuatorLimits(100.0, 100.0, 30.0, 5.0), 
        battery_state=1.0,
        current_load=None,
        contact_points=[]
    )
    
    # Setup Environment [cite: 17, 104]
    env = EnvironmentState(
        friction=float(f),
        slope=0.0 if c_mode else float(s),
        distance_to_obstacles=float(d)
    )

    # Compile WorldState [cite: 18, 106]
    world = WorldState(
        timestamp=time.time(),
        delta_time=0.1,
        gravity=Vector3(0.0, 0.0, -9.81),
        environment=env,
        agents=[agent], 
        objects=[],
        uncertainty=UncertaintyModel(0.05, 0.05, 0.05, 0.01)
    )
    
    # Initialize SafetyEngine [cite: 18, 19, 78]
    engine = SafetyEngine([
        BrakingConstraint(),
        FrictionConstraint(),
        StabilityConstraint(),
        LoadConstraint()
    ])
    
    # FIX: Flattening logic to handle list-safe constraints and dict returns [cite: 45, 46, 52, 79]
    raw_results = engine.evaluate(world)
    flat_results = []
    for res in raw_results:
        if isinstance(res, list):
            flat_results.extend(res)
        elif isinstance(res, dict):
            # Wrap dictionary from braking.py into ConstraintResult [cite: 43, 45]
            flat_results.append(ConstraintResult(
                name=res.get("constraint", "Braking"),
                violated=not res.get("safe", True),
                details=res
            ))
        else:
            flat_results.append(res)
            
    report = SafetyReport(flat_results)
    
    curve_data = {}
    if c_mode and rad > 0:
        # Cornering physics [cite: 19, 20, 85, 86]
        v_max = calculate_max_cornering_speed(rad, f, bank)
        is_tip, a_lat = check_lateral_stability(v, rad, cog_h_in, tw_in, bank)
        curve_data = {"v_max": v_max, "is_tip": is_tip, "a_lat": a_lat}
        
    return report, curve_data

# -------------------------
# 5. MODE LOGIC [cite: 20-31]
# -------------------------
if audit_mode == "Manual Simulator":
    is_curve = st.sidebar.checkbox("Curve Analysis")
    radius = st.sidebar.slider("Radius (m)", 5.0, 100.0, 25.0) if is_curve else 0
    banking = st.sidebar.slider("Banking (deg)", 0.0, 45.0, 0.0) if is_curve else 0
    slope = st.sidebar.slider("Slope (deg)", -25.0, 45.0, 0.0) if not is_curve else 0
    friction = st.sidebar.slider("Surface Friction (μ)", 0.1, 1.2, 0.8)
    velocity = st.sidebar.slider("Velocity (m/s)", 0.0, 30.0, 10.0)
    distance = st.sidebar.slider("Dist to Hazard (m)", 1.0, 100.0, 20.0)
    latency = st.sidebar.slider("System Latency (s)", 0.0, 1.0, 0.2)

    report, curve_res = run_audit(velocity, distance, friction, slope, wb, tw, wheels, total_m, cog_h, load_m, is_curve, radius, banking)
    
    # Safety determination [cite: 21, 77]
    is_safe = report.is_safe()
    if is_curve and curve_res.get("is_tip"): 
        is_safe = False

    status_class = "safe-glow" if is_safe else "danger-glow"
    st.markdown(f'<div class="status-box {status_class}"><h1>{"✅ MISSION CAPABLE" if is_safe else "❌ PHYSICS VETO"}</h1></div>', unsafe_allow_html=True)

    # Reality Twin Visualization [cite: 22, 23]
    st.subheader("🌐 Real-Time Reality Twin")
    fig_3d = go.Figure()
    fig_3d.add_trace(go.Surface(z=np.zeros((10, 10)), x=np.linspace(-5, 5, 10), y=np.linspace(0, distance + 10, 10), colorscale='Greys', showscale=False, opacity=0.2))
    fig_3d.add_trace(go.Scatter3d(x=[-tw/2, tw/2, tw/2, -tw/2, -tw/2], y=[0, 0, wb, wb, 0], z=[0, 0, 0, 0, 0], mode='lines', line=dict(color='cyan', width=6)))
    fig_3d.add_trace(go.Scatter3d(x=[0], y=[wb/2], z=[cog_h], mode='markers', marker=dict(size=8, color='red'), name="CoG"))
    fig_3d.update_layout(scene=dict(aspectmode='data'), height=400, margin=dict(l=0,r=0,b=0,t=0))
    st.plotly_chart(fig_3d, use_container_width=True)

    # Detailed Metrics [cite: 26, 27]
    st.divider()
    m1, m2 = st.columns([1, 2])
    with m1:
        st.subheader("📊 Physics Metrics")
        st.metric("Risk Score", report.risk_score())
        for res in report.results:
            if res.violated:
                st.error(f"⚠️ {res.name}")
            else:
                st.caption(f"✅ {res.name}")
    with m2:
        st.subheader("🔍 Safety Envelope")
        v_ax, d_ax = np.linspace(0.1, 30, 15), np.linspace(1, 100, 15)
        grid = [[1 if run_audit(vi, dj, friction, slope, wb, tw, wheels, total_m, cog_h, load_m, is_curve, radius, banking)[0].is_safe() else 0 for dj in d_ax] for vi in v_ax]
        st.plotly_chart(go.Figure(data=go.Heatmap(z=grid, x=d_ax, y=v_ax, colorscale=[[0,'#EF553B'],[1,'#00CC96']], showscale=False)).update_layout(height=350, xaxis_title="Distance (m)", yaxis_title="Velocity (m/s)"), use_container_width=True)

elif audit_mode == "Real-Time Safety Shield":
    st.subheader("🛡️ AI Command Interceptor [cite: 31]")
    cmd_v = st.number_input("Commanded Velocity", 0.0, 30.0, 5.0)
    cmd_d = st.number_input("Detected Distance", 1.0, 100.0, 15.0)
    if st.button("Execute Shield Audit"):
        rep, _ = run_audit(cmd_v, cmd_d, 0.8, 0.0, wb, tw, wheels, total_m, cog_h, load_m)
        if rep.is_safe(): 
            st.success("✅ COMMAND APPROVED")
        else: 
            st.error("❌ COMMAND REJECTED")