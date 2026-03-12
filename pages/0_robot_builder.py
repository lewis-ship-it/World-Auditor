import streamlit as st

st.set_page_config(page_title="Robot Builder", layout="wide")

st.title("🔧 Robot Builder")

st.write("Configure the robot used in the World Auditor simulator.")

# ---------------------------------------------------------
# INITIALIZE ROBOT STATE
# ---------------------------------------------------------

if "robot_config" not in st.session_state:

    st.session_state.robot_config = {
        "mass": 800,
        "wheelbase": 1.2,
        "track_width": 0.8,
        "wheel_radius": 0.25,
        "motor_torque": 120,
        "max_rpm": 3000,
        "battery_capacity": 500
    }

config = st.session_state.robot_config

# ---------------------------------------------------------
# CHASSIS
# ---------------------------------------------------------

st.header("Chassis")

config["mass"] = st.slider("Mass (kg)",100,2000,config["mass"])
config["wheelbase"] = st.slider("Wheelbase (m)",0.5,3.0,config["wheelbase"])
config["track_width"] = st.slider("Track Width (m)",0.3,2.0,config["track_width"])

# ---------------------------------------------------------
# WHEELS
# ---------------------------------------------------------

st.header("Wheels")

config["wheel_radius"] = st.slider("Wheel Radius (m)",0.05,0.5,config["wheel_radius"])

# ---------------------------------------------------------
# ACTUATORS
# ---------------------------------------------------------

st.header("Motor")

config["motor_torque"] = st.slider("Motor Torque (Nm)",10,500,config["motor_torque"])
config["max_rpm"] = st.slider("Max RPM",500,8000,config["max_rpm"])

# ---------------------------------------------------------
# POWER
# ---------------------------------------------------------

st.header("Battery")

config["battery_capacity"] = st.slider("Battery Capacity (Wh)",100,2000,config["battery_capacity"])

# ---------------------------------------------------------
# SAVE
# ---------------------------------------------------------

st.session_state.robot_config = config

st.success("Robot configuration saved.")

st.write("### Current Robot Config")
st.json(config)