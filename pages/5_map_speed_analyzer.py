import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from PIL import Image
import math
from sklearn.neighbors import NearestNeighbors

# --- THE HYBRID EXTRACTOR ---

def extract_hybrid_path(image_np, thresh_val):
    # 1. Pre-process
    gray = cv2.cvtColor(image_np, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # 2. Binary Threshold (The "Perfect View" you saw)
    _, binary = cv2.threshold(blurred, thresh_val, 255, cv2.THRESH_BINARY_INV)
    
    # 3. Noise Removal: Only keep the largest solid object (The Track)
    # This deletes all the "trash" dots/text
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if num_labels <= 1: return None
    largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    binary_cleaned = np.zeros_like(binary)
    binary_cleaned[labels == largest_label] = 255

    # 4. Skeletonization: Find the center-line of that one largest shape
    skeleton = cv2.ximgproc.thinning(binary_cleaned)
    
    # 5. Extract Points from the spine
    y_c, x_c = np.where(skeleton > 0)
    pts = np.column_stack((x_c, y_c))
    
    if len(pts) < 10: return None

    # 6. Final Sequence Sort
    # Since it's a 1-pixel spine, the proximity sort will now be perfect
    return sort_points_sequentially(pts[::10]) # Downsample by 10 for smoothness

def sort_points_sequentially(pts):
    if len(pts) < 2: return pts
    sorted_pts = [pts[0]]
    remaining_pts = pts[1:].tolist()
    while remaining_pts:
        last_pt = sorted_pts[-1]
        nn = NearestNeighbors(n_neighbors=1).fit(remaining_pts)
        dist, idx = nn.kneighbors([last_pt])
        sorted_pts.append(remaining_pts.pop(idx[0][0]))
    return np.array(sorted_pts)

# --- FULL APP LOGIC ---

st.set_page_config(page_title="SafeBot Hybrid Auditor", layout="wide")
st.title("🗺️ Hybrid Map Speed Analyzer")

st.sidebar.header("Processing Settings")
thresh_val = st.sidebar.slider("Map Sensitivity", 0, 255, 127)
track_len = st.sidebar.number_input("Track Length (m)", 1.0, 5000.0, 100.0)

uploaded_file = st.file_uploader("Upload Track Map")

if uploaded_file:
    img_pil = Image.open(uploaded_file).convert("RGB")
    img_np = np.array(img_pil)
    
    # Run the Hybrid Extraction
    pts = extract_hybrid_path(img_np, thresh_val)
    
    if pts is not None:
        xs, ys = pts[:, 0], pts[:, 1]
        
        # Calculate Curvature & Physics (Integrated)
        pixel_dist = sum(np.linalg.norm(pts[i+1]-pts[i]) for i in range(len(pts)-1))
        m_px = track_len / pixel_dist
        
        curvatures = []
        for i in range(1, len(pts)-1):
            p1, p2, p3 = pts[i-1], pts[i], pts[i+1]
            a, b, c = np.linalg.norm(p2-p1), np.linalg.norm(p3-p2), np.linalg.norm(p3-p1)
            if a*b*c == 0: curvatures.append(0); continue
            k = abs(4 * np.sqrt(max(0, (a+b-c)*(a-b+c)*(-a+b+c)*(a+b+c))) / (a*b*c))
            curvatures.append(k / m_px)
        
        # Simple Speed Model
        speeds = [math.sqrt(0.8 * 9.81 * (1.0/k if k > 0.001 else 1000.0)) for k in curvatures]
        
        # Map Visualization
        map_fig = go.Figure()
        map_fig.add_trace(go.Scatter(
            x=xs, y=ys, mode='lines+markers',
            marker=dict(size=6, color=speeds, colorscale="Viridis", showscale=True),
            line=dict(color="rgba(255,255,255,0.3)", width=2)
        ))
        
        map_fig.update_layout(
            images=[dict(source=img_pil, xref="x", yref="y", x=0, y=max(ys),
                         sizex=max(xs), sizey=max(ys),
                         sizing="stretch", opacity=0.4, layer="below")],
            xaxis=dict(visible=False), yaxis=dict(visible=False, scaleanchor="x"),
            template="plotly_dark", height=600
        )
        st.plotly_chart(map_fig, use_container_width=True)
        st.success("✅ Path extracted using Hybrid Center-Line detection.")
    else:
        st.error("Could not isolate the track center-line. Try adjusting Map Sensitivity.")