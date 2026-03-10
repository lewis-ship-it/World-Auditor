import streamlit as st
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import sys
import os

# --- PATH SETUP ---
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# ---------------------------------------------------------
# 1. CORE PHYSICS & CLEARANCE ENGINES
# ---------------------------------------------------------

def get_safe_velocity(dist, elevation, friction_map, width_map, mass, max_speed, robot_width):
    """Backwards-pass integration for safety limits + Width Clearance."""
    slopes = np.arctan(np.gradient(elevation, dist))
    v_safe = np.zeros_like(dist)
    g = 9.81
    
    # Target stop at the end
    v_safe[-1] = 0
    
    for i in range(len(dist) - 2, -1, -1):
        # 1. Width Check: If the gap is narrower than the robot, velocity must be 0 (Collision)
        if width_map[i] < robot_width:
            v_safe[i] = 0
            continue

        theta = slopes[i]
        mu = friction_map[i]
        ds = dist[i+1] - dist[i]
        
        # 2. Traction Physics: Deceleration vs Gravity
        max_decel = (mu * g * np.cos(theta)) - (g * np.sin(theta))
        
        # 3. Kinematic Integration
        v_safe[i] = np.sqrt(max(0, v_safe[i+1]**2 + 2 * max_decel * ds))
        
    return np.minimum(v_safe, max_speed)
def run_chaos_sim(dist, elevation, friction_map, width_map, r_mass, r_v_max, r_width):
    """
    Runs 10 randomized ghosts to stress-test the safety envelope.
    """
    ghost_results = []
    for _ in range(10):
        # 1. Vary mass by +/- 15% (e.g., changing payload)
        v_mass = r_mass * np.random.uniform(0.85, 1.15)
        
        # 2. Vary friction by +/- 20% (e.g., patchy ice or rain)
        v_mu = friction_map * np.random.uniform(0.8, 1.0)
        
        # 3. Calculate safety profile for this specific ghost
        # We reuse the same get_safe_velocity function we built earlier
        v_ghost = get_safe_velocity(dist, elevation, v_mu, width_map, v_mass, r_v_max, r_width)
        ghost_results.append(v_ghost)
        
    return ghost_results
def get_chaos_bounds(dist, elevation, friction_map, width_map, r_mass, r_v_max, r_width):
    """Runs 10 ghosts and returns the upper and lower bounds for the heatmap."""
    all_ghosts = []
    for _ in range(10):
        # Vary mass and friction to simulate real-world uncertainty
        v_mass = r_mass * np.random.uniform(0.9, 1.2)
        v_mu = friction_map * np.random.uniform(0.7, 1.0)
        
        v_ghost = get_safe_velocity(dist, elevation, v_mu, width_map, v_mass, r_v_max, r_width)
        all_ghosts.append(v_ghost)
    
    # Calculate the 'Envelope' (the spread of all safety plans)
    all_ghosts = np.array(all_ghosts)
    v_min = np.min(all_ghosts, axis=0)
    v_max = np.max(all_ghosts, axis=0)
    return v_min, v_max

def check_chassis_geometry(w_base, clearance, elevation, dist):
    """Calculates Breakover Angle violations."""
    slopes = np.degrees(np.arctan(np.gradient(elevation, dist)))
    limit = np.degrees(np.arctan((2 * clearance) / w_base))
    
    # Severity = change in slope over distance (hill crests)
    crest_severity = np.abs(np.diff(slopes))
    max_sev = np.max(crest_severity) if len(crest_severity) > 0 else 0
    
    return max_sev < limit, max_sev, limit

# ---------------------------------------------------------
# 2. UI CONFIG & LAYOUT
# ---------------------------------------------------------
st.set_page_config(layout="wide", page_title="World Auditor | Lab")
st.title("🛡️ World-Auditor: Terrain & Chassis Lab")

# --- SIDEBAR: DIGITAL TWIN SETTINGS ---
with st.sidebar:
    st.header("🤖 Robot Digital Twin")
    r_mass = st.number_input("Mass (kg)", 500, 5000, 1200)
    r_width = st.slider("Robot Width (m)", 0.5, 3.0, 1.8)
    r_wheelbase = st.slider("Wheelbase (m)", 0.5, 4.0, 2.5)
    r_clearance = st.slider("Ground Clearance (m)", 0.05, 0.5, 0.20)
    
    st.divider()
    st.header("🏔️ Terrain Synthesis")
    hill_amp = st.slider("Hill Amplitude (m)", 0, 15, 5)
    
    st.divider()
    st.header("🎲 Stress Test")
    chaos_mode = st.toggle("Enable Chaos Mode (10 Ghosts)", value=False)
    rain_reduction = st.slider("Rain/Slick Factor (%)", 0, 80, 0)
    

# ---------------------------------------------------------
# 3. WORLD MODELING
# ---------------------------------------------------------
track_len = 400
dist = np.linspace(0, track_len, 400)
elevation = hill_amp * np.sin(dist * 0.04) + (dist * 0.01) # Hills + Slight grade

# Friction & Width Maps
friction_map = np.full_like(dist, 0.8 * (1 - rain_reduction/100))
width_map = np.full_like(dist, 5.0) # Standard 5m wide path

# Inject Obstacles/Gaps
# 1. A Traction Gap (Gravel/Ice)
friction_map[150:180] = 0.1 
# 2. A Narrow Gap (The "Width Bottleneck")
width_map[280:310] = 1.5 # Narrower than the 1.8m default robot!

# ---------------------------------------------------------
# 4. EXECUTE AUDITS
# ---------------------------------------------------------
# Baseline
safe_v_base = get_safe_velocity(dist, elevation, friction_map, width_map, r_mass, 25.0, r_width)

# Chassis
is_clear, peak_sev, crit_limit = check_chassis_geometry(r_wheelbase, r_clearance, elevation, dist)

# ---------------------------------------------------------
# 5. VISUALIZATION
# ---------------------------------------------------------
fig = go.Figure()

# Plot 1: The Terrain (Gray)
fig.add_trace(go.Scatter(x=dist, y=elevation, fill='tozeroy', name="Terrain Elevation", line=dict(color='gray', width=0)))

# Plot 2: Safety Envelope (Green)
fig.add_trace(go.Scatter(x=dist, y=safe_v_base, name="Safety Envelope (m/s)", line=dict(color='#00CC96', width=4)))

# Plot 3: Chaos Mode (Red Ghosts)
if chaos_mode:
    ghosts = run_chaos_sim(dist, elevation, friction_map, width_map, r_mass, r_v_max, r_width)
    for i, g_v in enumerate(ghosts):
        fig.add_trace(go.Scatter(
            x=dist, 
            y=g_v, 
            line=dict(color='rgba(255, 75, 75, 0.15)', width=1), # Faint red lines
            showlegend=False,
            hoverinfo='skip'
        ))

# Annotations for Gaps
fig.add_vrect(x0=150, x1=180, fillcolor="blue", opacity=0.1, annotation_text="LOW FRICTION")
fig.add_vrect(x0=280, x1=310, fillcolor="orange", opacity=0.1, annotation_text="NARROW GAP")

fig.update_layout(template="plotly_dark", height=500, xaxis_title="Distance (m)", yaxis_title="Velocity / Elevation")
st.plotly_chart(fig, use_container_width=True)

# ---------------------------------------------------------
# 6. RESULTS & INSIGHTS
# ---------------------------------------------------------
c1, c2, c3 = st.columns(3)

with c1:
    st.metric("Chassis Status", "✅ CLEAR" if is_clear else "❌ CRITICAL")
    if not is_clear:
        st.error(f"Robot will bottom out! Hill crest at {peak_sev:.1f}° exceeds your {crit_limit:.1f}° breakover limit.")

with c2:
    collision_at = dist[np.where(safe_v_base == 0)[0]]
    if len(collision_at) > 0:
        st.metric("Width Clearance", "❌ COLLISION", delta="Width Violation")
        st.error(f"Robot (Width: {r_width}m) cannot fit through the {width_map[280]}m gap at {collision_at[0]:.0f}m.")
    else:
        st.metric("Width Clearance", "✅ PASSED")

with c3:
    avg_speed = np.mean(safe_v_base)
    st.metric("Avg Safe Speed", f"{avg_speed:.1f} m/s")

st.divider()
with st.expander("📝 View Engineering Log"):
    log_df = pd.DataFrame({"Dist": dist, "Elev": elevation, "Safe_V": safe_v_base, "Width": width_map})
    st.dataframe(log_df.iloc[::10]) # Show every 10th row for brevity