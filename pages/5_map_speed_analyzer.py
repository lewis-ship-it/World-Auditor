import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from PIL import Image
import math
from sklearn.neighbors import NearestNeighbors

# --- PATH PROCESSING HELPERS ---

def sort_points_sequentially(pts):
    """Uses Nearest Neighbors to walk the track spine in order."""
    if len(pts) < 2: return pts
    sorted_pts = [pts[0]]
    remaining_pts = pts[1:].tolist()
    
    while remaining_pts:
        last_pt = sorted_pts[-1]
        nn = NearestNeighbors(n_neighbors=1).fit(remaining_pts)
        dist, idx = nn.kneighbors([last_pt])
        sorted_pts.append(remaining_pts.pop(idx[0][0]))
        
    return np.array(sorted_pts)

def get_physics_metrics(curvatures, friction, wheelbase, mass):
    """Calculates Speed (Traction & Tipping), Braking, and Energy."""
    g = 9.81
    track_width = wheelbase * 0.7
    cog_h = 0.45 # Estimated Center of Gravity height
    
    speeds = []
    for k in curvatures:
        radius = 1.0 / k if k > 0.0001 else 1000.0
        # Limit 1: When will it slide?
        v_sliding = math.sqrt(friction * g * radius)
        # Limit 2: When will it tip over?
        v_tipping = math.sqrt((g * (track_width / 2) * radius) / cog_h)
        
        speeds.append(min(v_sliding, v_tipping))
    
    speeds = np.array(speeds)
    # Simple energy model: Kinetic energy + Rolling Resistance
    energy = np.abs(np.gradient(0.5 * mass * speeds**2)) + (0.05 * mass * g)
    return speeds, energy

# --- UI CONFIGURATION ---

st.set_page_config(page_title="SafeBot Map Auditor", layout="wide")
st.title("🗺️ Map Speed & Stability Analyzer")

st.sidebar.header("Robot & Mission Settings")
mass = st.sidebar.number_input("Robot Mass (kg)", 0.1, 500.0, 40.0)
wheelbase = st.sidebar.number_input("Wheelbase (m)", 0.1, 5.0, 0.6)
friction = st.sidebar.slider("Surface Friction (μ)", 0.1, 1.2, 0.8)
track_len = st.sidebar.number_input("Total Track Length (m)", 1.0, 10000.0, 100.0)

uploaded_file = st.file_uploader("Upload Track Map (High Contrast Recommended)")

if uploaded_file:
    # 1. Image Processing & Skeletonization
    img_pil = Image.open(uploaded_file).convert("RGB")
    img_np = np.array(img_pil)
    gray = cv2.cvtColor(img_np, cv2.COLOR_RGB2GRAY)
    
    # Threshold to create a clean binary mask of the track
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    # SKELETONIZATION: This prevents the 'scattered' look by finding the track spine
    skeleton = cv2.ximgproc.thinning(binary)
    
    y_c, x_c = np.where(skeleton > 0)
    pts = np.column_stack((x_c, y_c))
    
    if len(pts) > 30:
        # 2. Downsample and Sort
        pts = pts[::12] # Adjust density based on track complexity
        pts = sort_points_sequentially(pts)
        
        # 3. Scaling & Physics
        pixel_dist = sum(np.linalg.norm(pts[i+1]-pts[i]) for i in range(len(pts)-1))
        m_px = track_len / pixel_dist
        
        curvatures = []
        for i in range(1, len(pts)-1):
            p1, p2, p3 = pts[i-1], pts[i], pts[i+1]
            a, b, c = np.linalg.norm(p2-p1), np.linalg.norm(p3-p2), np.linalg.norm(p3-p1)
            if a*b*c == 0: curvatures.append(0); continue
            k = abs(4 * np.sqrt(max(0, (a+b-c)*(a-b+c)*(-a+b+c)*(a+b+c))) / (a*b*c))
            curvatures.append(k / m_px)
        
        speeds, energy = get_physics_metrics(np.array(curvatures), friction, wheelbase, mass)
        dist_axis = np.cumsum([np.linalg.norm(pts[i+1]-pts[i])*m_px for i in range(len(pts)-2)])

        # 4. Interactive Display Switcher
        view_mode = st.radio("Map Layer:", ["Speed (m/s)", "Energy (J)"], horizontal=True)
        data, colorscale = (speeds, "Viridis") if "Speed" in view_mode else (energy, "YlOrRd")

        # 5. SPATIAL HEATMAP
        map_fig = go.Figure()
        map_fig.add_trace(go.Scatter(
            x=pts[1:-1, 0], y=pts[1:-1, 1],
            mode='lines+markers',
            line=dict(color='rgba(255,255,255,0.2)', width=1),
            marker=dict(size=8, color=data, colorscale=colorscale, showscale=True),
            text=[f"Dist: {d:.1f}m | Val: {v:.2f}" for d, v in zip(dist_axis, data)],
            hoverinfo='text'
        ))

        map_fig.update_layout(
            images=[dict(
                source=img_pil, xref="x", yref="y", x=0, y=max(pts[:, 1]),
                sizex=max(pts[:, 0]), sizey=max(pts[:, 1]),
                sizing="stretch", opacity=0.4, layer="below")],
            xaxis=dict(visible=False), yaxis=dict(visible=False, scaleanchor="x"),
            margin=dict(l=10, r=10, t=10, b=10), height=600, template="plotly_dark"
        )
        st.plotly_chart(map_fig, use_container_width=True)

        # 6. PERFORMANCE GRAPH
        graph_fig = go.Figure()
        graph_fig.add_trace(go.Scatter(x=dist_axis, y=data, mode='lines', line=dict(color='cyan')))
        graph_fig.update_layout(
            title=f"Mission Profile: {view_mode}",
            xaxis_title="Distance (m)", yaxis_title=view_mode,
            template="plotly_dark"
        )
        st.plotly_chart(graph_fig, use_container_width=True)
        
    else:
        st.warning("⚠️ Track not found. Try an image with a clearer, darker track on a light background.")