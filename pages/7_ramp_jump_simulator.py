import streamlit as st
import numpy as np
import matplotlib.pyplot as plt

g = 9.81

st.title("Ramp Gap Jump Simulator")

st.sidebar.header("Robot Profile")

mass = st.sidebar.number_input("Robot Mass (kg)", 1.0, 5000.0, 50.0)
max_speed = st.sidebar.number_input("Max Speed (m/s)", 0.1, 20.0, 5.0)
wheelbase = st.sidebar.number_input("Wheelbase (m)", 0.1, 5.0, 0.5)
com_height = st.sidebar.number_input("Center of Mass Height (m)", 0.01, 2.0, 0.3)

st.sidebar.header("Ramp")

angle = st.sidebar.slider("Ramp Angle (degrees)", 0, 60, 20)
ramp_length = st.sidebar.number_input("Ramp Length (m)", 0.5, 20.0, 3.0)

st.sidebar.header("Gap")

gap_distance = st.sidebar.number_input("Gap Distance (m)", 0.1, 50.0, 2.0)
landing_height = st.sidebar.number_input("Landing Height Difference (m)", -5.0, 5.0, 0.0)

st.sidebar.header("Surface")

friction = st.sidebar.slider("Surface Friction", 0.1, 1.5, 0.7)

run = st.button("Run Simulation")

if run:

    theta = np.radians(angle)

    v = max_speed

    vx = v * np.cos(theta)
    vy = v * np.sin(theta)

    discriminant = vy**2 + 2*g*landing_height

    if discriminant < 0:
        st.error("Robot cannot reach landing height")
    else:

        t = (vy + np.sqrt(discriminant)) / g

        horizontal_distance = vx * t

        st.subheader("Results")

        st.write(f"Maximum Jump Distance: {horizontal_distance:.2f} m")

        if horizontal_distance >= gap_distance:
            st.success("Robot CAN clear the gap")
        else:
            st.error("Robot CANNOT clear the gap")

        t_vals = np.linspace(0, t, 100)

        x = vx * t_vals
        y = vy * t_vals - 0.5 * g * t_vals**2

        fig, ax = plt.subplots()

        ax.plot(x, y)

        ax.axvline(gap_distance, linestyle="--")

        ax.set_xlabel("Distance (m)")
        ax.set_ylabel("Height (m)")

        st.pyplot(fig)