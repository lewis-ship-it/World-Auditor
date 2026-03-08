import sys
import os
import streamlit as st
import pandas as pd
import plotly.graph_objects as go

# 1. Path Fix for Modular Navigation
current_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.abspath(os.path.join(current_dir, ".."))
if root_path not in sys.path:
    sys.path.insert(0, root_path)

from ui.engine_builder import build_engine
from alignment_core.engine.world_factory import build_world
from alignment_core.physics.braking_model import BrakingConstraint
from alignment_core.constraints.load import LoadConstraint
from alignment_core.constraints.friction import FrictionConstraint
from alignment_core.constraints.stability import StabilityConstraint

# --- PAGE CONFIG ---
st.set_page_config(page_title="SafeBot Audit", layout="wide")

# --- CUSTOM CSS FOR BENTO BOX AESTHETIC ---
st.markdown("""
    <style>
    .main { background-color: #0E1117; }
    .bento-container {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
        gap: 15px;
        margin-bottom: 25px;
    }
    .bento-card {
        background-color: #161B22;
        border: 1px solid #30363D;
        padding: 20px;
        border-radius: 12px;
        text-align: center;
    }
    .status-pass { border-top: 5px solid #00CC96; }
    .status-fail { border-top: 5px solid #FF4B4B; }
    .metric-value { font-size: 24px; font-weight: bold; color: #58A6FF; }
    </style>
    """, unsafe_allow_html=True)

st.title("🛡️ Robot Physics Safety Audit")
st.caption("Deterministic Safety Alignment Engine v0.1.0")

# --- SIDEBAR INPUTS ---
with st.sidebar:
    st.header("Mechanical Profile")
    mass = st.number_input("Total Mass (kg)", 1.0, 5000.0, 1200.0)
    wb = st.number_input("Wheelbase (m)", 0.5, 5.0, 1.2)
    cog_h = st.number_input("CoG Height (m)", 0.1, 3.0, 0.6)
    
    st.header("Environment")
    # AI Simulation Toggle
    surface_mode = st.selectbox("Surface Detection", ["Manual", "AI Predicted"])
    if surface_mode == "AI Predicted":
        surface_type = st.selectbox("AI Detected Surface", ["dry_concrete", "wet_concrete", "ice"])
        friction = 0.7 # Placeholder, overridden by AI map
    else:
        friction = st.slider("Manual Friction (mu)", 0.1, 1.2, 0.7)
        surface_type = "default"
        
    slope = st.slider("Slope (°)", -20, 20, 0)
    dist = st.number_input("Obstacle Dist (m)", 0.1, 50.0, 5.0)
    
    st.header("Current Action")
    v = st.number_input("Velocity (m/s)", 0.0, 15.0, 3.0)
    run_audit = st.button("🚀 EXECUTE AUDIT")

# --- AUDIT LOGIC ---
if run_audit:
    # Build state using the unified factory
    world = build_world(
        velocity=v, mass=mass, friction=friction, 
        distance=dist, slope=slope, cog_h=cog_h, wheelbase=wb
    )
    # Inject AI Surface if selected
    world.environment.surface_type = surface_type
    
    engine = build_engine([
        BrakingConstraint(), FrictionConstraint(), 
        StabilityConstraint(), LoadConstraint()
    ])
    
    report = engine.evaluate(world)
    
    # --- BENTO BOX RESULTS ---
    st.markdown('<div class="bento-container">', unsafe_allow_html=True)
    
    for res in report.results:
        status_class = "status-fail" if res.violated else "status-pass"
        status_text = "DANGER" if res.violated else "SAFE"
        
        st.markdown(f"""
            <div class="bento-card {status_class}">
                <p style="color: #8B949E; margin-bottom: 5px;">{res.name.upper()}</p>
                <div class="metric-value">{status_text}</div>
                <p style="font-size: 12px; margin-top: 10px;">{res.message}</p>
            </div>
        """, unsafe_allow_html=True)
    
    st.markdown('</div>', unsafe_allow_html=True)

    # --- DETAILED ANALYSIS ---
    col_gauge, col_table = st.columns([1, 2])
    
    with col_gauge:
        score = report.safety_score
        fig = go.Figure(go.Indicator(
            mode="gauge+number", value=score,
            title={'text': "Total Safety Score"},
            gauge={'axis': {'range': [0, 100]}, 'bar': {'color': "#58A6FF"}}
        ))
        st.plotly_chart(fig, use_container_width=True)
        
    with col_table:
        st.subheader("Physics Telemetry")
        df = pd.DataFrame([
            {"Constraint": r.name, "Violation": r.violated, "Severity": r.severity}
            for r in report.results
        ])
        st.table(df)