import streamlit as st
import math
import plotly.graph_objects as go
from datetime import datetime
import cv2
import tempfile

from alignment_core.engine.safety_engine import SafetyEngine
from alignment_core.engine.report import SafetyReport
from alignment_core.constraints.braking import BrakingConstraint
from alignment_core.constraints.friction import FrictionConstraint
from alignment_core.constraints.load import LoadConstraint
from alignment_core.constraints.stability import StabilityConstraint
from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.environment import EnvironmentState
from alignment_core.world_model.world_state import WorldState
from alignment_core.world_model.primitives import Vector3, Quaternion, ActuatorLimits
from alignment_core.world_model.uncertainty import UncertaintyModel


st.set_page_config(page_title="SafeBot Physics Auditor", layout="wide")
st.title("🛡️ SafeBot: Physics Reality Auditor")


# -------------------------
# PROFILES
# -------------------------

ROBOT_PROFILES = {
    "Warehouse Forklift": {"mass": 4000.0, "max_load": 2000.0, "com_height": 1.2, "wheelbase": 2.0},
    "Delivery Rover": {"mass": 80.0, "max_load": 20.0, "com_height": 0.4, "wheelbase": 0.6},
    "Standard Sedan": {"mass": 1500.0, "max_load": 500.0, "com_height": 0.6, "wheelbase": 2.7}
}

SURFACE_MAP = {
    "Dry Concrete": 0.8,
    "Wet Asphalt": 0.4,
    "Ice": 0.15
}

BRAKE_MAP = {
    "New": 5.0,
    "Used": 2.5,
    "Failing": 1.0
}


# -------------------------
# SIDEBAR
# -------------------------

st.sidebar.header("Audit Mode")
audit_mode = st.sidebar.radio("Mode", ["Manual Simulator", "Live Video Audit"])

profile_name = st.sidebar.selectbox("Robot Profile", list(ROBOT_PROFILES.keys()))
profile = ROBOT_PROFILES[profile_name]

surface_key = st.sidebar.selectbox("Surface", list(SURFACE_MAP.keys()))
friction = SURFACE_MAP[surface_key]

brake_key = st.sidebar.select_slider("Brake Condition", options=list(BRAKE_MAP.keys()), value="Used")
deceleration = BRAKE_MAP[brake_key]

velocity = 0.0
distance = 0.0

if audit_mode == "Manual Simulator":
    velocity = st.sidebar.slider("Speed (m/s)", 0.0, 15.0, 5.0)
    distance = st.sidebar.slider("Distance to Obstacle (m)", 0.5, 20.0, 10.0)

load_weight = st.sidebar.slider("Load Weight (kg)", 0.0, 3000.0, 500.0)
slope = st.sidebar.slider("Slope (degrees)", 0.0, 45.0, 5.0)
compare_mode = st.sidebar.checkbox("Compare Robot")


# -------------------------
# CORE AUDIT FUNCTION
# -------------------------

def run_audit(p_data, v, d, decel, load, fric, slp):

    zero_vec = Vector3(0.0, 0.0, 0.0)
    velocity_vec = Vector3(v, 0.0, 0.0)
    identity_quat = Quaternion(1.0, 0.0, 0.0, 0.0)

    limits = ActuatorLimits(max_torque=100.0, max_force=100.0, max_speed=20.0, max_acceleration=5.0)

    agent = AgentState(
        id="primary",
        type="mobile",
        mass=p_data["mass"],
        position=zero_vec,
        velocity=velocity_vec,
        angular_velocity=zero_vec,
        orientation=identity_quat,
        center_of_mass=zero_vec,
        support_polygon=[
            Vector3(-0.5, -0.5, 0),
            Vector3(0.5, -0.5, 0),
            Vector3(0.5, 0.5, 0),
            Vector3(-0.5, 0.5, 0),
        ],
        actuator_limits=limits,
        battery_state=1.0,
        current_load=None,
        contact_points=[],
        load_weight=load,
        max_load=p_data["max_load"],
        center_of_mass_height=p_data["com_height"],
        wheelbase=p_data["wheelbase"]
    )

    env = EnvironmentState(
        temperature=20.0,
        air_density=1.225,
        wind_vector=zero_vec,
        terrain_type="flat",
        friction=fric,
        slope=slp,
        distance_to_obstacle=d
    )

    world_state = WorldState(
        timestamp=datetime.now().timestamp(),
        delta_time=0.1,
        gravity=Vector3(0.0, 0.0, -9.81),
        environment=env,
        agents=[agent],
        objects=[],
        uncertainty=UncertaintyModel(0.1, 0.1, 0.1, 0.1)
    )

    engine = SafetyEngine()
    engine.register_constraint(BrakingConstraint())
    engine.register_constraint(FrictionConstraint())
    engine.register_constraint(LoadConstraint())
    engine.register_constraint(StabilityConstraint())

    return SafetyReport(engine.evaluate(world_state))


# -------------------------
# VIDEO MODE
# -------------------------

if audit_mode == "Live Video Audit":

    st.subheader("Video Audit")
    uploaded_video = st.file_uploader("Upload Video", type=["mp4", "mov", "avi"])

    if uploaded_video:
        tfile = tempfile.NamedTemporaryFile(delete=False)
        tfile.write(uploaded_video.read())
        cap = cv2.VideoCapture(tfile.name)

        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        ret, frame = cap.read()

        if ret:
            st.image(frame, channels="BGR", use_container_width=True)

            # Simulated detection
            velocity = 8.0
            distance = 6.0

            dt = 1 / fps

            if "prev_v" not in st.session_state:
                st.session_state.prev_v = velocity

            prev_v = st.session_state.prev_v
            accel = abs(velocity - prev_v) / dt
            st.session_state.prev_v = velocity

            st.metric("Measured Acceleration", f"{accel:.2f} m/s²")

            if accel > 10:
                st.warning("Reality violation detected")

        cap.release()


# -------------------------
# RUN AUDIT
# -------------------------

report = run_audit(profile, velocity, distance, deceleration, load_weight, friction, slope)

if report.is_safe():
    st.success("✅ CLEAR TO PROCEED")
else:
    st.error("❌ PHYSICS VETO")

required_stop = (velocity ** 2) / (2 * deceleration) if deceleration > 0 else 0
buffer = distance - required_stop

fig = go.Figure()
fig.add_trace(go.Bar(
    y=["Path"],
    x=[required_stop],
    orientation='h'
))

if buffer > 0:
    fig.add_trace(go.Bar(
        y=["Path"],
        x=[buffer],
        orientation='h'
    ))

st.plotly_chart(fig, use_container_width=True)

st.metric("Stopping Distance", f"{required_stop:.2f} m")
st.metric("Safety Margin", f"{buffer:.2f} m")


if compare_mode:
    alt_name = st.selectbox("Compare With", [k for k in ROBOT_PROFILES if k != profile_name])
    alt_report = run_audit(ROBOT_PROFILES[alt_name], velocity, distance, deceleration, load_weight, friction, slope)
    alt_risk = sum(50 for r in alt_report.results if r.violated)
    st.write(f"Alternative Risk Score: {alt_risk}")