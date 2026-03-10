import streamlit as st
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import sys
import os

# ---------------------------------------------------------
# 1. PATH & CORE IMPORTS
# ---------------------------------------------------------
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from alignment_core.physics.braking_model import BrakingModel
from alignment_core.physics.energy_model import EnergyModel
from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.environment import EnvironmentState

# ---------------------------------------------------------
# 2. UI CONFIG
# ---------------------------------------------------------
st.set_page_config(layout="wide", page_title="World-Auditor | Path Lab")
st.title("🏁 Ultimate Path & Speed Auditor")

# ---------------------------------------------------------
# 3. SIDEBAR: VEHICLE & WORLD BUILDER (Merged Inputs)
# ---------------------------------------------------------
with st.sidebar:
    st.header("🤖 Vehicle Config")
    mass = st.number_input("Mass (kg)", value=1200.0)
    max_speed = st.number_input("Max Speed (m/s)", value=20.0)
    
    st.header("🌍 World Elements")
    track_length = st.number_input("Track Length (m)", value=500)
    base_friction = st.slider("Base Surface Grip (μ)", 0.1, 1.2, 0.8)
    
    st.header("📐 Terrain Profile")
    complexity = st.select_slider("Terrain Complexity", options=["Flat", "Hilly", "Mountainous"])
    
    st.header("🚧 Obstacles & Gaps")
    obs_pos = st.number_input("Obstacle Position (m)", value=450)
    gap_start = st.number_input("Gap Start (m)", value=200)
    gap_end = st.number_input("Gap End (m)", value=210)

# ---------------------------------------------------------
# 4. PATH GENERATION LOGIC (The "Map")
# ---------------------------------------------------------
ds = 1.0  # 1-meter increments
dist_steps = np.arange(0, track_length, ds)

# Generate Elevation (Hills/Valleys)
if complexity == "Hilly":
    elevation = 5 * np.sin(dist_steps * 0.05)
elif complexity == "Mountainous":
    elevation = 15 * np.sin(dist_steps * 0.02) + 5 * np.cos(dist_steps * 0.1)
else:
    elevation = np.zeros_like(dist_steps)

# Calculate Slopes (Rise/Run)
slopes = np.degrees(np.arctan(np.gradient(elevation, ds)))

# Generate Friction Map (Handle Gaps)
friction_map = np.full_like(dist_steps, base_friction)
friction_map[(dist_steps >= gap_start) & (dist_steps <= gap_end)] = 0.05 # Near zero grip in gaps

# ---------------------------------------------------------
# 5. SIMULATION ENGINE
# ---------------------------------------------------------
v_plan = np.full_like(dist_steps, max_speed)
energy_usage = []
braking_model = BrakingModel(base_friction)

# Simple forward-pass simulation
v_actual = [max_speed]
for i in range(len(dist_steps)-1):
    current_slope = slopes[i]
    current_mu = friction_map[i]
    
    # Physics adjustment: Gravity helps/hurts based on slope
    # a_max = (mu * g * cos(theta)) - (g * sin(theta))
    g = 9.81
    theta = np.radians(current_slope)
    max_decel = (current_mu * g * np.cos(theta)) - (g * np.sin(theta))
    
    # If the robot needs to stop for the obstacle at obs_pos
    dist_to_obs = obs_pos - dist_steps[i]
    safe_v = np.sqrt(max(0, 2 * max_decel * dist_to_obs)) if dist_to_obs > 0 else 0
    
    v_actual.append(min(max_speed, safe_v))

# ---------------------------------------------------------
# 6. VISUALIZATION
# ---------------------------------------------------------
col1, col2 = st.columns([2, 1])

with col1:
    st.subheader("🗺️ Path Physics Profile")
    fig = go.Figure()
    
    # Elevation Trace
    fig.add_trace(go.Scatter(x=dist_steps, y=elevation, fill='tozeroy', name="Terrain (Hills)", line=dict(color='gray')))
    
    # Velocity Trace
    fig.add_trace(go.Scatter(x=dist_steps, y=v_actual, name="Safe Velocity Profile", line=dict(color='cyan', width=3)))
    
    # Obstacle Marker
    fig.add_vline(x=obs_pos, line_dash="dash", line_color="red", annotation_text="Target Stop")
    
    fig.update_layout(template="plotly_dark", xaxis_title="Distance (m)", yaxis_title="Velocity / Elevation")
    st.plotly_chart(fig, use_container_width=True)

with col2:
    st.subheader("📊 Audit Results")
    stop_dist_required = (max_speed**2) / (2 * base_friction * 9.81)
    
    st.metric("Required Braking (Flat)", f"{stop_dist_required:.2f}m")
    
    avg_slope = np.mean(slopes)
    st.metric("Avg Path Slope", f"{avg_slope:.1f}°")
    
    if obs_pos < stop_dist_required:
        st.error(f"❌ UNAVOIDABLE COLLISION: Insufficient distance for current mass/friction.")
    elif any(friction_map == 0.05):
        st.warning("⚠️ TRACTION GAP DETECTED: Velocity reduced for safety through zero-grip zone.")
    else:
        st.success("✅ PATH VERIFIED: Maneuver is safe for current terrain.")

st.subheader("📉 Advanced Data Export")
df_log = pd.DataFrame({
    "Distance (m)": dist_steps,
    "Slope (°)": slopes,
    "Elevation (m)": elevation,
    "Friction (μ)": friction_map,
    "Safe Speed (m/s)": v_actual
})
st.dataframe(df_log.iloc[::10], use_container_width=True) # Show every 10th meter