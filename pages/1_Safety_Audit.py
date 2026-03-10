import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import sys
import os
from typing import List

# ---------------------------------------------------------
# 1. DYNAMIC PATH INJECTION
# ---------------------------------------------------------
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)

if project_root not in sys.path:
    sys.path.insert(0, project_root)

# ---------------------------------------------------------
# 2. CORE STATE & ENGINE IMPORTS (Replaced Local Classes)
# ---------------------------------------------------------
from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.environment import EnvironmentState
from alignment_core.world_model.world_state import WorldState

from alignment_core.engine.safety_engine import SafetyEngine
from alignment_core.constraints.braking import BrakingConstraint
from alignment_core.constraints.load import LoadConstraint
from alignment_core.constraints.friction import FrictionConstraint
from alignment_core.constraints.stability import StabilityConstraint

# ---------------------------------------------------------
# 4. UI CONFIG & BENTO CSS
# ---------------------------------------------------------
st.set_page_config(page_title="Safety Audit", layout="wide")

st.markdown("""
    <style>
    .main { background-color: #0E1117; }
    .bento-container {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
        gap: 15px;
        margin-bottom: 25px;
    }
    .bento-card {
        background-color: #161B22;
        border: 1px solid #30363D;
        padding: 20px;
        border-radius: 12px;
        text-align: center;
        transition: all 0.3s ease;
    }
    .status-pass { border-top: 5px solid #00CC96; box-shadow: 0 4px 15px rgba(0, 204, 150, 0.1); }
    .status-fail { border-top: 5px solid #FF4B4B; box-shadow: 0 4px 15px rgba(255, 75, 75, 0.1); }
    .metric-label { color: #8B949E; font-size: 14px; text-transform: uppercase; letter-spacing: 1px; }
    .metric-value { font-size: 24px; font-weight: bold; color: #58A6FF; margin-top: 8px; }
    </style>
    """, unsafe_allow_html=True)

st.title("🛡️ Robot Physics Safety Audit")
st.markdown("Evaluating planned maneuvers against deterministic physics constraints.")

st.divider()

# ---------------------------------------------------------
# 5. SIDEBAR INPUTS
# ---------------------------------------------------------
st.sidebar.header("🤖 Robot Profile")
robot_name = st.sidebar.text_input("Robot Name", "WarehouseBot")
mass = st.sidebar.number_input("Robot Mass (kg)", 1.0, 10000.0, 1200.0)
wheelbase = st.sidebar.number_input("Wheelbase (m)", 0.1, 5.0, 1.2)
com_height = st.sidebar.number_input("Center of Mass Height (m)", 0.05, 3.0, 0.6)
max_load = st.sidebar.number_input("Maximum Load (kg)", 0.0, 10000.0, 1500.0)
max_speed = st.sidebar.number_input("Maximum Speed (m/s)", 0.1, 20.0, 4.0)

st.sidebar.header("🌍 Environment")
ai_surface = st.sidebar.toggle("Enable AI Surface Intelligence")
if ai_surface:
    surface_type = st.sidebar.selectbox("AI Vision Perception", ["dry_concrete", "wet_concrete", "ice", "loose_gravel"])
    friction = 0.7 
else:
    friction = st.sidebar.slider("Manual Surface Friction (μ)", 0.1, 1.5, 0.7)
    surface_type = "default"

slope = st.sidebar.slider("Slope Angle (degrees)", -30.0, 30.0, 0.0)
distance = st.sidebar.number_input("Distance to Obstacle (m)", 0.1, 100.0, 5.0)

st.sidebar.header("🕹️ Planned Action")
velocity = st.sidebar.number_input("Current Speed (m/s)", 0.0, max_speed, 2.0)
deceleration = st.sidebar.number_input("Max Braking Power (m/s²)", 0.1, 20.0, 4.0)
load_weight = st.sidebar.number_input("Payload Weight (kg)", 0.0, max_load, 500.0)

run_audit = st.sidebar.button("🚀 EXECUTE PHYSICS AUDIT")

# ---------------------------------------------------------
# 6. LOGIC FUNCTIONS
# ---------------------------------------------------------
def build_world_state():
    agent = AgentState(
        id="robot",
        type="mobile",
        mass=mass,
        velocity=velocity,
        max_speed=max_speed,
        wheelbase=wheelbase,
        center_of_mass_height=com_height,
        load_weight=load_weight,
        max_load=max_load,
    )
    # FIX: Using explicit keyword arguments to match environment.py
    environment = EnvironmentState(
        gravity=9.81,
        surface_friction=friction,
        slope=slope,
        distance_to_obstacles=distance
    )
    return WorldState(agent=agent, environment=environment)

def compute_safety_score(results):
    if not results: return 100
    failures = sum(1 for r in results if not r.passed)
    return max(0, int(100 - (failures / len(results)) * 100))

def explain_results(results):
    messages = []
    for r in results:
        if r.passed: continue
        name = r.name.lower()
        if "braking" in name:
            messages.append("CRITICAL: Stopping distance exceeds available space.")
        elif "load" in name:
            messages.append("CRITICAL: Load weight exceeds structural safety limits.")
        elif "friction" in name:
            messages.append("WARNING: Low traction detected for current velocity.")
        elif "stability" in name:
            messages.append("WARNING: High tipping risk on current slope.")
        else:
            messages.append(f"VIOLATION: {r.message}")
    return messages

# ---------------------------------------------------------
# 7. MAIN DISPLAY
# ---------------------------------------------------------
if run_audit:
    world_state = build_world_state()
    
    constraints = [
        BrakingConstraint(),
        LoadConstraint(),
        FrictionConstraint(),
        StabilityConstraint(),
    ]
    engine = SafetyEngine(constraints)
    report = engine.evaluate(world_state)
    results = report.results
    score = compute_safety_score(results)

    st.markdown('<div class="bento-container">', unsafe_allow_html=True)
    for r in results:
        card_class = "status-pass" if r.passed else "status-fail"
        status_text = "SAFE" if r.passed else "DANGER"
        st.markdown(f"""
            <div class="bento-card {card_class}">
                <div class="metric-label">{r.name}</div>
                <div class="metric-value">{status_text}</div>
            </div>
        """, unsafe_allow_html=True)
    st.markdown('</div>', unsafe_allow_html=True)

    col_gauge, col_table = st.columns([1, 2])
    
    with col_gauge:
        st.subheader("Safety Integrity Score")
        fig = go.Figure(go.Indicator(
            mode="gauge+number",
            value=score,
            gauge={
                'axis': {'range': [0, 100]},
                'bar': {'color': "#58A6FF"},
                'steps': [
                    {'range': [0, 50], 'color': "#FF4B4B"},
                    {'range': [50, 80], 'color': "#FFAA00"},
                    {'range': [80, 100], 'color': "#00CC96"}
                ]
            }
        ))
        st.plotly_chart(fig, use_container_width=True)

    with col_table:
        st.subheader("Physics Data Log")
        log_data = [{"Constraint": r.name, "Status": "PASS" if r.passed else "FAIL", "Details": r.message} for r in results]
        st.table(pd.DataFrame(log_data))

    st.divider()
    st.subheader("📋 Auditor's Summary")
    warnings = explain_results(results)
    if not warnings:
        st.success("The planned action is within safe operating envelopes.")
    else:
        for w in warnings:
            st.warning(w)