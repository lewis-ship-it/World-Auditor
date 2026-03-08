import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from PIL import Image
import math

# --- THE STABLE TRACER ---

def trace_center_line(binary_img):
    """
    Takes the 'Perfect AI View' and extracts the middle spine.
    """
    # 1. Clean up any tiny holes in your 'perfect' view
    kernel = np.ones((5,5), np.uint8)
    binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)
    
    # 2. Skeletonize: This is the magic step. 
    # It collapses the thick track into a 1-pixel wide line.
    skeleton = cv2.ximgproc.thinning(binary_img)
    
    # 3. Find all points on that 1-pixel spine
    y_idx, x_idx = np.where(skeleton > 0)
    if len(x_idx) < 10: return None
    
    # 4. Walk the path (Sequential Ordering)
    # We find a starting point and 'walk' to the next closest neighbor
    pts = np.column_stack((x_idx, y_idx))
    path = [pts[0]]
    pts = pts[1:].tolist()
    
    while pts and len(path) < 1500:
        last_p = path[-1]
        # Find the index of the closest point
        dists = [math.dist(last_p, p) for p in pts]
        closest_idx = np.argmin(dists)
        
        # If the jump is too big (>40px), we stop to avoid 'nonsense' lines
        if dists[closest_idx] > 40:
            break
            
        path.append(pts.pop(closest_idx))
    
    return np.array(path)

# --- APP UI ---

st.set_page_config(page_title="AI Vision Tracer", layout="wide")
st.title("🗺️ AI-View Path Tracer")

# SIDEBAR
thresh_val = st.sidebar.slider("AI Sensitivity", 0, 255, 127)
downsample = st.sidebar.slider("Path Smoothness", 1, 20, 10)

uploaded_file = st.file_uploader("Upload Track Map")

if uploaded_file:
    img_pil = Image.open(uploaded_file).convert("RGB")
    img_np = np.array(img_pil)
    
    # 1. GENERATE THE 'PERFECT' BINARY VIEW
    gray = cv2.cvtColor(img_np, cv2.COLOR_RGB2GRAY)
    _, binary = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    col1, col2 = st.columns(2)
    with col1:
        st.subheader("1. AI Vision View")
        st.image(binary, caption="The computer's clean view of the track", use_container_width=True)
    
    # 2. TRACE THE CENTER OF THAT VIEW
    # We pass the binary image you said looks perfect
    path_pts = trace_center_line(binary)
    
    if path_pts is not None:
        # Downsample for smoother curves and faster graphs
        final_pts = path_pts[::downsample]
        xs, ys = final_pts[:, 0], final_pts[:, 1]
        
        with col2:
            st.subheader("2. Extracted Path")
            fig = go.Figure()
            # Draw the original image as background
            fig.add_trace(go.Scatter(x=xs, y=ys, mode='lines+markers', 
                                   line=dict(color='cyan', width=3),
                                   marker=dict(size=4, color='white')))
            
            fig.update_layout(
                images=[dict(source=img_pil, xref="x", yref="y", x=0, y=max(ys),
                             sizex=max(xs), sizey=max(ys),
                             sizing="stretch", opacity=0.5, layer="below")],
                xaxis=dict(visible=False), yaxis=dict(visible=False, scaleanchor="x"),
                template="plotly_dark", margin=dict(l=0,r=0,t=0,b=0)
            )
            st.plotly_chart(fig, use_container_width=True)
        
        st.success(f"✅ Successfully traced the center of the track ({len(final_pts)} nodes).")
    else:
        st.error("AI could not find a continuous path. Adjust Sensitivity.")