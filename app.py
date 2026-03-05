import streamlit as st

# --- 1. SETTINGS & GLASS-MORPHISM STYLE ---
st.set_page_config(page_title="SafeBot Auditor", page_icon="🛡️", layout="wide")

st.markdown("""
    <style>
    .main { background-color: #0E1117; color: #FFFFFF; }
    /* Modern Card Style */
    .module-card {
        background-color: #161B22;
        border: 1px solid #30363D;
        padding: 20px;
        border-radius: 15px;
        margin-bottom: 20px;
        height: 250px;
    }
    h2 { color: #58A6FF !important; }
    .stButton>button {
        width: 100%;
        border-radius: 8px;
        background-color: #21262D;
        color: #58A6FF;
        border: 1px solid #30363D;
        font-weight: bold;
    }
    .stButton>button:hover {
        border-color: #58A6FF;
        background-color: #1c2128;
    }
    </style>
    """, unsafe_allow_html=True)

# --- 2. WELCOME HEADER ---
st.title("🛡️ SafeBot: Physics Reality Auditor")
st.markdown("### Select a module to begin the safety verification process.")
st.divider()

# --- 3. THE INTERACTIVE DASHBOARD ---
col1, col2 = st.columns(2)

with col1:
    st.markdown('<div class="module-card"><h2>⚖️ Safety Audit</h2><p>Run deterministic kernels to check Braking, Stability, and Load limits against real-time telemetry.</p></div>', unsafe_allow_html=True)
    # THIS IS THE LINK:
    if st.button("Launch Audit Engine"):
        st.switch_page("pages/1_Safety_Audit.py")

    st.markdown('<div class="module-card"><h2>🏎️ Physics Simulator</h2><p>Interactive stress-testing. Adjust environmental friction and velocity to visualize the safety envelope.</p></div>', unsafe_allow_html=True)
    if st.button("Open Stress-Test Simulator"):
        st.switch_page("pages/2_Simulator.py")

with col2:
    st.markdown('<div class="module-card"><h2>📹 Video Intelligence</h2><p>AI Perception Layer. Upload footage to estimate motion vectors and detect physics violations in the wild.</p></div>', unsafe_allow_html=True)
    if st.button("Start Video Analysis"):
        st.switch_page("pages/3_Video_Audit.py")

    st.markdown('<div class="module-card"><h2>🗺️ Mission Planner</h2><p>Strategic route optimization. Calculate safe cornering speeds and tipping risks for complex pathing.</p></div>', unsafe_allow_html=True)
    if st.button("View Map Optimizer"):
        st.switch_page("pages/4_Map_path_Planner.py")

# --- 4. SIDEBAR ---
with st.sidebar:
    st.header("System Status")
    st.success("Deterministic Kernel: LOADED")
    st.caption("All modules are currently isolated for modular safety.")