import streamlit as st
import numpy as np
import plotly.graph_objects as go
import heapq
import pandas as pd
import time

st.set_page_config(layout="wide", page_title="Mini ROS Navigation Simulator")

st.title("🤖 Mini ROS Navigation Simulator")

# ---------------------------------------------------
# LOAD ROBOT
# ---------------------------------------------------

# --- UPDATE THIS AT THE TOP OF 2_World_Auditor_Lab.py ---

if "robot_config" not in st.session_state:
    st.error("⚠️ Robot not found! Please save your robot in 'Robot Builder' first.")
    st.stop()

# Link to the shared configuration
robot_cfg = st.session_state.robot_config

# Extract the specific values you want to use
mass = robot_cfg.get("mass", 800)
torque = robot_cfg.get("motor_torque", 120)
max_rpm = robot_cfg.get("max_rpm", 3000)
battery_cap = robot_cfg.get("battery_capacity", 500)
wheel_r = robot_cfg.get("wheel_radius", 0.25)

# BRAINPOWER: Calculate acceleration based on Torque instead of a slider
# Force = Torque / Radius | Acceleration = Force / Mass
max_speed = robot_cfg.get("max_speed", 15.0)
accel = (torque / wheel_r) / mass

# ---------------------------------------------------
# CONSTANTS
# ---------------------------------------------------

GRID = 80
dt = 0.1
g = 9.81

# ---------------------------------------------------
# MAP SERVER
# ---------------------------------------------------

def generate_world():
    elevation = np.zeros((GRID, GRID))
    friction = np.ones((GRID, GRID))
    obstacles = np.zeros((GRID, GRID))

    # 🏔️ THE CLIMB: Steep hill at the start to test Torque/Mass
    for x in range(0, 20):
        elevation[x, :] = (x / 5) ** 2 

    # 🧊 THE ICE TRAP: Low friction zone in the middle
    friction[30:50, 30:50] = 0.05 

    # 🚧 THE MAZE: Static obstacles to test A* and turning radius
    obstacles[25:55, 20] = 1
    obstacles[25:55, 40] = 1

    return elevation, friction, obstacles

elevation,friction,obstacles = generate_world()

start = (5,5)
goal = (70,70)

# ---------------------------------------------------
# GLOBAL PLANNER (A*)
# ---------------------------------------------------

def astar(grid,start,goal):

    def h(a,b):
        return abs(a[0]-b[0])+abs(a[1]-b[1])

    open=[]
    heapq.heappush(open,(0,start))

    came={}
    cost={start:0}

    while open:

        _,cur=heapq.heappop(open)

        if cur==goal:

            path=[cur]

            while cur in came:
                cur=came[cur]
                path.append(cur)

            return path[::-1]

        x,y=cur

        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:

            nx,ny=x+dx,y+dy

            if nx<0 or ny<0 or nx>=GRID or ny>=GRID:
                continue

            if grid[nx,ny]==1:
                continue

            new=cost[cur]+1

            if (nx,ny) not in cost or new<cost[(nx,ny)]:

                cost[(nx,ny)]=new
                came[(nx,ny)]=cur
                heapq.heappush(open,(new+h((nx,ny),goal),(nx,ny)))

    return []

path = astar(obstacles,start,goal)

# ---------------------------------------------------
# ROBOT STATE
# ---------------------------------------------------

x,y = start
theta = 0
speed = 0

positions=[]
speeds=[]
times=[]
headings=[]

distance=0
time_elapsed=0

# ---------------------------------------------------
# UI LAYOUT
# ---------------------------------------------------

map_col,telemetry_col = st.columns([3,1])
placeholder = map_col.empty()

run = st.button("Start Navigation")
# --- UPDATE YOUR START BUTTON ---
if st.button("▶️ Start Navigation"):
    st.session_state.running = True
    st.session_state.robot_state = {"pos": st.session_state.start, "speed": 0.0, "i": 0}
    
    # FIX: Initialize the battery energy using the builder's capacity
    st.session_state.remaining_energy = robot_cfg.get("battery_capacity", 500)

# ---------------------------------------------------
# LOCAL PLANNER + CONTROLLER
# ---------------------------------------------------

if run and path:

    for waypoint in path:

        wx,wy = waypoint

        dx = wx-x
        dy = wy-y

        desired_heading = np.arctan2(dy,dx)

        heading_error = desired_heading-theta

        omega = heading_error

        v = min(speed+accel*dt,max_speed)

        # Calculate Slope Angle
        slope_grad = np.gradient(elevation)
        # Using the robot's current integer position for lookup
        curr_slope = slope_grad[0][int(x), int(y)] 

        # Power consumption logic: P = (Gravity + Friction) * Velocity
        gravity_force = mass * g * np.sin(np.arctan(curr_slope))
        friction_force = mass * g * friction[int(x), int(y)]
        power_draw = (gravity_force + friction_force) * v

        # Update Battery in Session State
        # (Make sure to initialize st.session_state.remaining_energy in the Builder)
        st.session_state.remaining_energy -= (power_draw * dt) / 3600 # Wh

        # differential wheel speeds
        w_r = (2*v + omega*track_width)/(2*wheel_radius)
        w_l = (2*v - omega*track_width)/(2*wheel_radius)

        # update pose
        theta += omega*dt

        x += v*np.cos(theta)*dt
        y += v*np.sin(theta)*dt

        speed=v

        rx,ry=int(x),int(y)

        mu=friction[rx,ry]

        brake = v**2/(2*mu*g)

        slip = v > np.sqrt(mu*g*5)

        move = v*dt
        distance+=move
        time_elapsed+=dt

        positions.append((x,y))
        speeds.append(v)
        times.append(time_elapsed)
        headings.append(theta)

        # ------------------------------------
        # VISUALIZATION
        # ------------------------------------

        fig=go.Figure()

        fig.add_trace(go.Surface(
            z=elevation,
            colorscale="Viridis",
            showscale=False
        ))

        xs=[p[0] for p in path]
        ys=[p[1] for p in path]
        zs=[elevation[int(x),int(y)]+0.3 for x,y in zip(xs,ys)]

        fig.add_trace(go.Scatter3d(
            x=xs,y=ys,z=zs,
            mode="lines",
            line=dict(color="yellow",width=6),
            name="Global Path"
        ))

        fig.add_trace(go.Scatter3d(
            x=[x],
            y=[y],
            z=[elevation[int(x),int(y)]+1],
            mode="markers",
            marker=dict(size=8,color="red"),
            name="Robot"
        ))

        fig.update_layout(
            template="plotly_dark",
            height=650,
            scene=dict(
                xaxis_title="X",
                yaxis_title="Y",
                zaxis_title="Elevation"
            )
        )

        placeholder.plotly_chart(fig,use_container_width=True)

        time.sleep(0.03)

# ---------------------------------------------------
# TELEMETRY
# ---------------------------------------------------

if "robot_state" in st.session_state:
    st.divider()
    t_col1, t_col2, t_col3 = st.columns(3)
    
    # Get current live data from the simulation
    s = st.session_state.robot_state
    v = s.get("speed", 0.0)
    
    with t_col1:
        st.metric("Live Speed", f"{v:.2f} m/s", delta=f"{accel:.2f} m/s²")
    with t_col2:
        # Calculate power draw: (Force * Velocity) / Efficiency
        pwr = (v * mass) / 1000 
        st.metric("Power Draw", f"{pwr:.1f} kW")
    with t_col3:
        st.metric("Battery Capacity", f"{battery_cap} Wh")

    st.subheader("⚙️ Robot Performance Specs")
    
    # This table pulls Torque, RPM, and Battery from your Builder page
    performance_specs = pd.DataFrame({
        "Component": ["Motor Torque", "Max RPM", "Battery Capacity", "Chassis Mass"],
        "Value": [
            f"{torque} Nm", 
            f"{max_rpm} RPM", 
            f"{battery_cap} Wh", 
            f"{mass} kg"
        ]
    })
    
    st.table(performance_specs)