import streamlit as st
import math

st.title("🧠 Robot Physics Brain")

velocity = st.slider("Velocity (m/s)",0.1,40.0,5.0)
radius = st.slider("Turn Radius (m)",0.1,100.0,10.0)
friction = st.slider("Surface Friction",0.1,1.5,0.8)

g = 9.81

lat_acc = velocity**2 / radius
max_grip = friction * g

st.subheader("Physics Analysis")

if lat_acc > max_grip:

    st.error("Robot will slip.")

    st.write(
        f"Lateral acceleration {lat_acc:.2f} exceeds grip {max_grip:.2f}"
    )

else:

    st.success("Robot maintains traction.")

    st.write(
        f"Lateral acceleration {lat_acc:.2f} is within grip limit."
    )