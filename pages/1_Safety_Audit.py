import sys
import os
import streamlit as st
import pandas as pd
import plotly.graph_objects as go

# Modular Path Fix
current_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.abspath(os.path.join(current_dir, ".."))
if root_path not in sys.path:
    sys.path.insert(0, root_path)

from ui.engine_builder import build_engine
from alignment_core.engine.world_factory import build_world
from alignment_core.constraints.braking import BrakingConstraint
from alignment_core.constraints.load import LoadConstraint
from alignment_core.constraints.friction import FrictionConstraint
from alignment_core.constraints.stability import StabilityConstraint

st.set_page_config(page_title="SafeBot Audit", layout="wide")

# Custom CSS for Bento Grid and Status Glow
st.markdown("""
    <style>
    .bento-container { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; }
    .bento-card { background-color: #161B22; border: 1px solid #30363D; padding: 20px; border-radius: 12px; text-align: center; }
    .pass { border-top: 4px solid #00CC96; }
    .fail { border-top: 4px solid #FF4B4B; }
    .metric { font-size: 22px; font-weight: bold; color: #58A6FF; }
    </style>
    """, unsafe_allow_html=True)

st.title("🛡️ Robot Physics Safety Audit")

# --- SIDEBAR: ROBOT & ENV ---
with st.sidebar:
    st.header("Mechanical Profile")
    mass = st.number_input("Mass (kg)", 1.0, 5000.0, 1200.0)
    load = st.number_input("Load (kg)", 0.0, 2000.0, 500.0)
    wheelbase = st.number_input("Wheelbase (m)", 0.1, 5.0, 1.2)
    com_h = st.number_input("CoG Height (m)", 0.05, 3.0, 0.6)
    
    st.header("Environment & Action")
    surface_mode = st.toggle("Enable AI Surface Detection")
    if surface_mode:
        surface_type = st.selectbox("AI Perception", ["dry_concrete", "wet_concrete", "ice", "loose_gravel"])
        friction = 0.7 # Placeholder
    else:
        friction = st.slider("Manual Friction", 0.1, 1.2, 0.7)
        surface_type = "default"
        
    velocity = st.number_input("Current Speed (m/s)", 0.0, 20.0, 2.0)
    distance = st.number_input("Distance to Obstacle (m)", 0.1, 100.0, 5.0)
    run_audit = st.button("🚀 EXECUTE AUDIT")

# --- AUDIT EXECUTION ---
if run_audit:
    # Build the digital twin world
    world = build_world(
        velocity=velocity, mass=mass, load_weight=load,
        friction=friction, distance=distance, cog_h=com_h, wheelbase=wheelbase
    )
    world.environment.surface_type = surface_type
    
    # Run the engine
    engine = build_engine([
        BrakingConstraint(), FrictionConstraint(), 
        StabilityConstraint(), LoadConstraint()
    ])
    report = engine.evaluate(world)
    
    # 1. Bento Grid Summary
    st.markdown('<div class="bento-container">', unsafe_allow_html=True)
    for res in report.results:
        card_class = "pass" if res.passed else "fail"
        label = "SAFE" if res.passed else "CRITICAL"
        st.markdown(f"""
            <div class="bento-card {card_class}">
                <p style="color: #8B949E; font-size: 12px;">{res.name.upper()}</p>
                <div class="metric">{label}</div>
            </div>
        """, unsafe_allow_html=True)
    st.markdown('</div>', unsafe_allow_html=True)

    # 2. Visual Safety Score
    col1, col2 = st.columns([1, 2])
    with col1:
        score = max(0, int(100 - (sum(1 for r in report.results if not r.passed) / len(report.results) * 100)))
        fig = go.Figure(go.Indicator(
            mode="gauge+number", value=score,
            gauge={'axis': {'range': [0, 100]}, 'bar': {'color': "#58A6FF"}}
        ))
        st.plotly_chart(fig, use_container_width=True)

    # 3. Human Explanations
    with col2:
        st.subheader("Human-Readable Audit")
        for res in report.results:
            if not res.passed:
                st.warning(f"**{res.name}**: {res.message}")
            else:
                st.success(f"**{res.name}**: Constraint satisfied.")