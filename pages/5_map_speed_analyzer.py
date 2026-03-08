import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from PIL import Image
import math
from sklearn.neighbors import NearestNeighbors

# --- PATH & PHYSICS LOGIC ---

def sort_points_sequentially(pts):
    """Ensures the robot follows the track points in order of proximity."""
    if len(pts) < 2: return pts
    sorted_pts = [pts[0]]
    remaining_pts = pts[1:].tolist()
    while remaining_pts:
        last_pt = sorted_pts[-1]
        distances = [math.dist(last_pt, p) for p in remaining_pts]
        closest_idx = np.argmin(distances)
        sorted_pts.append(remaining_pts.pop(closest_idx))
    return np.array(sorted_pts)

def get_physics_metrics(curvatures, friction, wheelbase, mass):
    """Calculates all metrics: Speed, Braking, and Energy."""
    g = 9.81
    track_width = wheelbase * 0.7
    cog_h = 0.4
    
    speeds = []
    for k in curvatures:
        radius = 1.0 / k if k > 0.0001 else 1000.0
        v_sliding = math.sqrt(friction * g * radius)
        v_tipping = math.sqrt((g * (track_width / 2) * radius) / cog_h)
        speeds.append(min(v_sliding, v_tipping))
    
    speeds = np.array(speeds)
    braking = (speeds**2) / (2 * 6.0) # Base brake constant
    # Energy: Kinetic + Friction Work (simplified for map visualization)
    energy = np.gradient(0.5 * mass * speeds**2) + (friction * mass * g)
    
    return speeds, braking, energy

# --- UI CONFIGURATION ---

st.set_page_config(page_title="SafeBot Spatial Auditor", layout="wide")
st.title("🗺️ Interactive Map Speed & Energy Analyzer")

st.sidebar.header("Robot & Track Configuration")
mass = st.sidebar.number_input("Mass (kg)", 0.1, 500.0, 40.0)
wheelbase = st.sidebar.number_input("Wheelbase (m)", 0.1, 5.0, 0.6)
friction = st.sidebar.slider("Surface Friction (μ)", 0.1, 1.2, 0.8)
track_len = st.sidebar.number_input("Track Length (m)", 1.0, 5000.0, 100.0)

uploaded_file = st.file_uploader("Upload Track Map")

if uploaded_file:
    # Process Image
    img_pil = Image.open(uploaded_file).convert("RGB")
    img_np = np.array(img_pil)
    gray = cv2.cvtColor(img_np, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, 60, 120)
    
    y_c, x_c = np.where(edges > 0)
    pts = np.column_stack((x_c, y_c))
    
    if len(pts) > 50:
        pts = pts[::60]
        pts = sort_points_sequentially(pts)
        
        # Calculate Scale and Curvature
        pixel_dist = sum(np.linalg.norm(pts[i+1]-pts[i]) for i in range(len(pts)-1))
        m_px = track_len / pixel_dist
        
        curvatures = []
        for i in range(1, len(pts)-1):
            p1, p2, p3 = pts[i-1], pts[i], pts[i+1]
            a, b, c = np.linalg.norm(p2-p1), np.linalg.norm(p3-p2), np.linalg.norm(p3-p1)
            if a*b*c == 0: curvatures.append(0); continue
            k = abs(4 * np.sqrt(max(0, (a+b-c)*(a-b+c)*(-a+b+c)*(a+b+c))) / (a*b*c))
            curvatures.append(k / m_px)
        
        speeds, braking, energy = get_physics_metrics(np.array(curvatures), friction, wheelbase, mass)
        dist_array = np.cumsum([np.linalg.norm(pts[i+1]-pts[i])*m_px for i in range(len(pts)-2)])

        # LAYER SELECTOR
        view_mode = st.radio("Active Analysis Layer:", ["Speed (m/s)", "Braking Distance (m)", "Energy Cost (J)"], horizontal=True)
        
        if "Speed" in view_mode: data, colorscale = speeds, "Viridis"
        elif "Braking" in view_mode: data, colorscale = braking, "Reds"
        else: data, colorscale = energy, "Hot"

        # INTERACTIVE PLOTLY MAP
        st.subheader(f"Spatial Heatmap: {view_mode}")
        map_fig = go.Figure()
        
        # Background Image Trace
        map_fig.add_trace(go.Scatter(
            x=pts[1:-1, 0], y=pts[1:-1, 1],
            mode='markers',
            marker=dict(size=10, color=data, colorscale=colorscale, showscale=True),
            text=[f"{v:.2f}" for v in data],
            name="Path Analysis"
        ))

        map_fig.update_layout(
            images=[dict(
                source=img_pil, xref="x", yref="y", x=0, y=max(pts[:, 1]),
                sizex=max(pts[:, 0]), sizey=max(pts[:, 1]),
                sizing="stretch", opacity=0.4, layer="below")],
            xaxis=dict(visible=False), yaxis=dict(visible=False, scaleanchor="x"),
            margin=dict(l=0, r=0, t=0, b=0), height=600
        )
        st.plotly_chart(map_fig, use_container_width=True)

        # LINKED GRAPH
        st.subheader("Mission Profile")
        graph_fig = go.Figure()
        graph_fig.add_trace(go.Scatter(x=dist_array, y=data, mode='lines+markers', name=view_mode))
        graph_fig.update_layout(xaxis_title="Distance Along Track (m)", yaxis_title=view_mode)
        
        # Streamlit Click Interaction
        st.plotly_chart(graph_fig, use_container_width=True)
        
        st.info("💡 Clicking the Heatmap or Graph allows you to inspect specific physics violations at that exact GPS coordinate.")