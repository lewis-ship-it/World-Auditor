import streamlit as st
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from streamlit_drawable_canvas import st_canvas
import sys
import os

# --- PATH SETUP ---
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# ---------------------------------------------------------
# 1. SHARED PHYSICS CONFIGURATIONS
# ---------------------------------------------------------
SURFACE_PHYSICS = {
    "Concrete (Grey)": {"color": "#808080", "mu": 0.9},
    "Tarmac (Black)": {"color": "#000000", "mu": 1.0},
    "Grass (Green)": {"color": "#228B22", "mu": 0.3},
    "Ice (Blue)": {"color": "#ADD8E6", "mu": 0.05},
    "Mud (Brown)": {"color": "#8B4513", "mu": 0.2},
    "Sand (Light Brown)": {"color": "#D2B48C", "mu": 0.4}
}

TIRE_PROFILES = {
    "Slick": {"base_mod": 1.1, "wet_penalty": 0.6},
    "All-Terrain": {"base_mod": 0.9, "wet_penalty": 0.1},
    "Studded Ice": {"base_mod": 0.8, "wet_penalty": 0.1}
}

# ---------------------------------------------------------
# 2. CORE ENGINES
# ---------------------------------------------------------

def get_safe_velocity(dist, elevation, friction_map, width_map, mass, max_speed, robot_width):
    slopes = np.arctan(np.gradient(elevation, dist))
    v_safe = np.zeros_like(dist)
    g = 9.81
    v_safe[-1] = 0
    for i in range(len(dist) - 2, -1, -1):
        if width_map[i] < robot_width:
            v_safe[i] = 0
            continue
        theta = slopes[i]
        mu = friction_map[i]
        ds = dist[i+1] - dist[i]
        max_decel = (mu * g * np.cos(theta)) - (g * np.sin(theta))
        v_safe[i] = np.sqrt(max(0, v_safe[i+1]**2 + 2 * max_decel * ds))
    return np.minimum(v_safe, max_speed)

def check_chassis_geometry(w_base, clearance, elevation, dist):
    slopes = np.degrees(np.arctan(np.gradient(elevation, dist)))
    limit = np.degrees(np.arctan((2 * clearance) / w_base))
    crest_severity = np.abs(np.diff(slopes))
    max_sev = np.max(crest_severity) if len(crest_severity) > 0 else 0
    return max_sev < limit, max_sev, limit

def get_chaos_bounds(dist, elevation, friction_map, width_map, r_mass, r_v_max, r_width):
    all_ghosts = []
    for _ in range(10):
        v_mass = r_mass * np.random.uniform(0.9, 1.2)
        v_mu = friction_map * np.random.uniform(0.7, 1.0)
        v_ghost = get_safe_velocity(dist, elevation, v_mu, width_map, v_mass, r_v_max, r_width)
        all_ghosts.append(v_ghost)
    all_ghosts = np.array(all_ghosts)
    return np.min(all_ghosts, axis=0), np.max(all_ghosts, axis=0)

# ---------------------------------------------------------
# 3. SIDEBAR & UI SETTINGS
# ---------------------------------------------------------
st.set_page_config(layout="wide", page_title="World Auditor | Lab")
st.title("🛡️ World-Auditor: Terrain & Chassis Lab")

with st.sidebar:
    st.header("🤖 Robot Config")
    r_mass = st.number_input("Mass (kg)", 500, 5000, 1200)
    r_v_max = st.slider("Max Speed (m/s)", 5, 40, 20)
    r_width = st.slider("Robot Width (m)", 0.5, 3.0, 1.8)
    r_wheelbase = st.slider("Wheelbase (m)", 0.5, 4.0, 2.5)
    r_clearance = st.slider("Ground Clearance (m)", 0.05, 0.5, 0.20)
    
    st.divider()
    st.header("🏁 Race Conditions")
    track_condition = st.radio("Weather", ["Dry", "Wet"], horizontal=True)
    global_width = st.number_input("Standard Track Width (m)", 1.0, 10.0, 5.0)
    tire_type = st.selectbox("Tire Compound", list(TIRE_PROFILES.keys()))
    
    st.divider()
    chaos_mode = st.toggle("Enable Chaos Mode", value=True)
    use_painter = st.toggle("Use Painter Data", value=False)

# ---------------------------------------------------------
# 4. PATH PAINTER & LEGEND
# ---------------------------------------------------------
st.write("### 🎨 Surface Material Legend")
cols = st.columns(len(SURFACE_PHYSICS))
for i, (name, data) in enumerate(SURFACE_PHYSICS.items()):
    cols[i].markdown(f"<div style='background-color:{data['color']}; height:10px; border-radius:5px;'></div>", unsafe_allow_html=True)
    cols[i].caption(name)

brush_color = st.color_picker("Pick Surface Material (Color)", "#000000")

canvas_result = st_canvas(
    stroke_width=4, stroke_color=brush_color, background_color="#0E1117",
    height=250, drawing_mode="freedraw", key="canvas"
)

# ---------------------------------------------------------
# 5. DYNAMIC WORLD MODELING
# ---------------------------------------------------------
# ---------------------------------------------------------
# 5. DYNAMIC WORLD MODELING (Solid Color Mapping)
# ---------------------------------------------------------
track_len = 400
dist = np.linspace(0, track_len, 400)

# A. Tire & Weather Logic
t_cfg = TIRE_PROFILES[tire_type]
weather_mod = (1.0 - t_cfg["wet_penalty"]) if track_condition == "Wet" else 1.0

# B. Initialize Base Maps
width_map = np.full_like(dist, global_width)
base_mu = SURFACE_PHYSICS["Tarmac (Black)"]["mu"] * t_cfg["base_mod"] * weather_mod
friction_map = np.full_like(dist, base_mu)
elevation = 5 * np.sin(dist * 0.04) # Default fallback

# C. Process Painter Overrides (Discrete Zones)
if use_painter and canvas_result.json_data is not None:
    objects = canvas_result.json_data.get("objects")
    if objects:
        for idx, obj in enumerate(objects):
            stroke_color = obj.get("stroke", "#000000").upper()
            path_points = obj.get("path", [])
            
            if len(path_points) > 1:
                # 1. Elevation comes from the FIRST stroke only
                if idx == 0:
                    raw_y = np.array([p[2] for p in path_points])
                    elevation = np.interp(dist, np.linspace(0, track_len, len(raw_y)), (250 - raw_y) / 10)
                
                # 2. Friction Zones (Discrete Color Detection)
                # Map horizontal stroke range to distance array
                x_coords = np.array([p[1] for p in path_points])
                m_start = np.min(x_coords) * (track_len / 600) # Mapping 600px canvas to 400m
                m_end = np.max(x_coords) * (track_len / 600)
                
                # Check which specific surface was selected
                for name, data in SURFACE_PHYSICS.items():
                    if data["color"].upper() == stroke_color:
                        # Apply solid friction to this specific distance mask
                        mask = (dist >= m_start) & (dist <= m_end)
                        friction_map[mask] = data["mu"] * t_cfg["base_mod"] * weather_mod

# ---------------------------------------------------------
# 6. CALCULATIONS & VISUALIZATION
# ---------------------------------------------------------
safe_v_base = get_safe_velocity(dist, elevation, friction_map, width_map, r_mass, r_v_max, r_width)
is_clear, peak_sev, crit_limit = check_chassis_geometry(r_wheelbase, r_clearance, elevation, dist)
# --- ROBOT ANIMATION CONTROL ---
st.divider()
st.subheader("🏃 Real-Time Simulation")
# --- AUTO-PLAY ENGINE ---
if "run_sim" not in st.session_state:
    st.session_state.run_sim = False

def toggle_sim():
    st.session_state.run_sim = not st.session_state.run_sim

st.button("▶️ Play / ⏸️ Pause Animation", on_click=toggle_sim)

# If Play is active, increment the slider automatically
if st.session_state.run_sim:
    if sim_pos < track_len:
        st.session_state.sim_pos = sim_pos + 5 # Adjust speed here
        st.rerun()
    else:
        st.session_state.run_sim = False
        st.session_state.sim_pos = 0
sim_pos = st.slider("Manual Drive / Playback", 0, int(track_len), 0, help="Slide to move the robot along the track")

# Find the index in our arrays that matches the slider position
sim_idx = np.argmin(np.abs(dist - sim_pos))
robot_x = dist[sim_idx]
robot_y = elevation[sim_idx]
robot_v = safe_v_base[sim_idx]

fig = go.Figure()

# Background Chaos Heatmap
if chaos_mode:
    v_min, v_max = get_chaos_bounds(dist, elevation, friction_map, width_map, r_mass, r_v_max, r_width)
    fig.add_trace(go.Scatter(x=dist, y=v_max, line=dict(width=0), showlegend=False, hoverinfo='skip'))
    fig.add_trace(go.Scatter(x=dist, y=v_min, fill='tonexty', fillcolor='rgba(255, 75, 75, 0.2)', line=dict(width=0), name="Chaos Envelope"))

# Terrain & Safety
fig.add_trace(go.Scatter(x=dist, y=elevation, fill='tozeroy', name="Terrain", line=dict(color='gray', width=1)))
fig.add_trace(go.Scatter(x=dist, y=safe_v_base, name="Safe Velocity", line=dict(color='#00CC96', width=4)))
fig.add_trace(go.Scatter(x=dist, y=friction_map * 10, name="Grip (μ x 10)", line=dict(color='orange', dash='dot')))

fig.update_layout(template="plotly_dark", height=600, xaxis_title="Distance (m)", yaxis_title="m/s | Elevation")
st.plotly_chart(fig, use_container_width=True)

# --- THE ROBOT (Tiny Point) ---
fig.add_trace(go.Scatter(
    x=[robot_x], 
    y=[robot_y + 0.5], # Offset slightly above ground
    mode="markers+text",
    marker=dict(symbol="car", size=15, color="yellow", line=dict(width=2, color="white")),
    text=[f" {robot_v:.1f} m/s"],
    textposition="top center",
    name="Robot Digital Twin"
))

# ---------------------------------------------------------
# 7. RESULTS
# ---------------------------------------------------------
c1, c2, c3 = st.columns(3)
with c1: st.metric("Chassis Status", "✅ CLEAR" if is_clear else "❌ CRITICAL")
with c2: st.metric("Weather Impact", track_condition, delta=f"{weather_mod:.1f}x Grip")
with c3: st.metric("Avg Safe Speed", f"{np.mean(safe_v_base):.1f} m/s")

# --- 8. SURFACE TELEMETRY ---
st.write("### 📊 Journey Composition")
surface_counts = {}
for i in range(len(dist)):
    # Match current friction to a surface name
    for name, data in SURFACE_PHYSICS.items():
        # Check if current friction matches the surface's mu (adjusted for weather/tires)
        target_mu = data["mu"] * t_cfg["base_mod"] * weather_mod
        if np.isclose(friction_map[i], target_mu, atol=0.01):
            surface_counts[name] = surface_counts.get(name, 0) + 1
            break

# Display as mini-dashboard
t_cols = st.columns(len(surface_counts))
for i, (surface, count) in enumerate(surface_counts.items()):
    distance_m = count * (track_len / len(dist))
    t_cols[i].caption(surface)
    t_cols[i].write(f"**{distance_m:.0f}m**")