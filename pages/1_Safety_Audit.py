import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import sys
import os

# --- MODULAR PATH FIX ---
# Ensures sub-pages can find the alignment_core and ui folders
current_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.abspath(os.path.join(current_dir, ".."))
if root_path not in sys.path:
    sys.path.insert(0, root_path)

from alignment_core.engine.safety_engine import SafetyEngine
from alignment_core.constraints.braking import BrakingConstraint
from alignment_core.constraints.load import LoadConstraint
from alignment_core.constraints.friction import FrictionConstraint
from alignment_core.constraints.stability import StabilityConstraint

from alignment_core.state.agent import AgentState
from alignment_core.state.environment import EnvironmentState
from alignment_core.state.world_state import WorldState

# --- PAGE CONFIG ---
st.set_page_config(page_title="Safety Audit", layout="wide")

# --- CUSTOM CSS FOR BENTO BOX AESTHETIC ---
st.markdown("""
    <style>
    .main { background-color: #0E1117; }
    .bento-container {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
        gap: 15px;
        margin-bottom: 25px;
    }
    .bento-card {
        background-color: #161B22;
        border: 1px solid #30363D;
        padding: 20px;
        border-radius: 12px;
        text-align: center;
        transition: transform 0.2s;
    }
    .bento-card:hover { transform: translateY(-5px); }
    .status-pass { border-top: 4px solid #00CC96; }
    .status-fail { border-top: 4px solid #FF4B4B; }
    .metric-value { font-size: 20px; font-weight: bold; color: #58A6FF; margin-top: 10px; }
    </style>
    """, unsafe_allow_html=True)

st.title("🛡️ Robot Physics Safety Audit")

st.markdown("""
This tool evaluates whether a robot's planned action violates **real-world physics constraints**.
It analyzes braking feasibility, traction, load safety, and tipping risk.
""")

st.divider()

# -----------------------------
# SIDEBAR INPUTS (Full Integrity Maintained)
# -----------------------------
st.sidebar.header("Robot Profile")
robot_name = st.sidebar.text_input("Robot Name", "WarehouseBot")
mass = st.sidebar.number_input("Robot Mass (kg)", 1.0, 10000.0, 1200.0)
wheelbase = st.sidebar.number_input("Wheelbase (m)", 0.1, 5.0, 1.2)
com_height = st.sidebar.number_input("Center of Mass Height (m)", 0.05, 3.0, 0.6)
max_load = st.sidebar.number_input("Maximum Load (kg)", 0.0, 10000.0, 1500.0)
max_speed = st.sidebar.number_input("Maximum Speed (m/s)", 0.1, 20.0, 4.0)

st.sidebar.header("Environment")
# Added AI Toggle for Intelligence Module
ai_surface = st.sidebar.toggle("Use Gemini AI Surface Detection")
if ai_surface:
    surface_type = st.sidebar.selectbox("Detected Surface", ["dry_concrete", "wet_concrete", "ice", "loose_gravel"])
    friction = 0.7 # Will be mapped in the FrictionConstraint
else:
    friction = st.sidebar.slider("Surface Friction", 0.1, 1.5, 0.7)
    surface_type = "default"

slope = st.sidebar.slider("Slope Angle (degrees)", -30, 30, 0)
distance = st.sidebar.number_input("Distance to Obstacle (m)", 0.1, 100.0, 5.0)

st.sidebar.header("Robot Action")
velocity = st.sidebar.number_input("Current Speed (m/s)", 0.0, max_speed, 2.0)
deceleration = st.sidebar.number_input("Max Deceleration (m/s²)", 0.1, 20.0, 4.0)
load_weight = st.sidebar.number_input("Load Weight (kg)", 0.0, max_load, 500.0)

run = st.sidebar.button("Run Safety Audit")

st.divider()

# -----------------------------
# CORE LOGIC (Full Original Logic)
# -----------------------------
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
    environment = EnvironmentState(
        friction=friction,
        slope=slope,
        obstacle_distance=distance,
        temperature=20,
    )
    # Inject AI Surface Type if active
    environment.surface_type = surface_type
    
    return WorldState(agent=agent, environment=environment)

def run_audit():
    world_state = build_world_state()
    constraints = [
        BrakingConstraint(),
        LoadConstraint(),
        FrictionConstraint(),
        StabilityConstraint(),
    ]
    engine = SafetyEngine(constraints)
    return engine.evaluate(world_state)

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
            messages.append("The robot cannot stop before reaching the obstacle.")
        elif "load" in name:
            messages.append("The robot is carrying more load than its safe limit.")
        elif "friction" in name:
            messages.append("Surface traction is too low for the current speed.")
        elif "stability" in name:
            messages.append("The robot may tip over due to high center of mass or slope.")
        else:
            messages.append(r.message)
    return messages

# -----------------------------
# BENTO DISPLAY
# -----------------------------
if run:
    results = run_audit()
    score = compute_safety_score(results)

    # NEW: Bento Grid Summary Cards
    st.markdown('<div class="bento-container">', unsafe_allow_html=True)
    for r in results:
        card_class = "status-pass" if r.passed else "status-fail"
        status_label = "SAFE" if r.passed else "CRITICAL"
        st.markdown(f"""
            <div class="bento-card {card_class}">
                <p style="color: #8B949E; margin-bottom: 0px; font-size: 14px;">{r.name.upper()}</p>
                <div class="metric-value">{status_label}</div>
            </div>
        """, unsafe_allow_html=True)
    st.markdown('</div>', unsafe_allow_html=True)

    col1, col2 = st.columns([1, 2])
    with col1:
        st.subheader("Safety Score")
        fig = go.Figure(go.Indicator(
            mode="gauge+number",
            value=score,
            gauge={
                "axis": {"range": [0, 100]},
                "bar": {"color": "#58A6FF"},
                "steps": [
                    {"range": [0, 40], "color": "#FF4B4B"},
                    {"range": [40, 70], "color": "#FFAA00"},
                    {"range": [70, 100], "color": "#00CC96"},
                ],
            },
        ))
        st.plotly_chart(fig, use_container_width=True)

    with col2:
        st.subheader("Physics Telemetry")
        table = [{"Constraint": r.name, "Passed": r.passed, "Message": r.message} for r in results]
        st.dataframe(pd.DataFrame(table), use_container_width=True)

    st.divider()
    st.subheader("Human Explanation")
    messages = explain_results(results)
    if not messages:
        st.success("The robot action appears physically safe.")
    else:
        for m in messages:
            st.warning(m)

    if score < 50:
        st.error("High risk scenario detected.")
    elif score < 80:
        st.warning("Moderate safety risk.")
    else:
        st.success("Robot action is likely safe.")