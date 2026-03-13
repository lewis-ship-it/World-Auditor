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

if "robot_config" not in st.session_state:
    st.error("⚠️ Robot not found! Please save your robot in 'Robot Builder' first.")
    st.stop()

robot_cfg = st.session_state.robot_config

# Extract specific builder values
mass = robot_cfg.get("mass", 800)
torque = robot_cfg.get("motor_torque", 120)
max_rpm = robot_cfg.get("max_rpm", 3000)
battery_cap = robot_cfg.get("battery_capacity", 500)
wheel_r = robot_cfg.get("wheel_radius", 0.25)
max_speed = robot_cfg.get("max_speed", 15.0)
track_width = robot_cfg.get("track_width", 1.5) # Added to fix NameError

# Calculate acceleration based on Torque
# Force = Torque / Radius | Acceleration = Force / Mass
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

    for x in range(0, 20):
        elevation[x, :] = (x / 5) ** 2 

    friction[30:50, 30:50] = 0.05 

    obstacles[25:55, 20] = 1
    obstacles[25:55, 40] = 1

    return elevation, friction, obstacles

elevation, friction, obstacles = generate_world()

start = (5,5)
goal = (70,70)

# ---------------------------------------------------
# GLOBAL PLANNER (A*)
# ---------------------------------------------------

def astar(grid,start,goal):
    def h(a,b):
        return abs(a[0]-b[0])+abs(a[1]-b[1])
    open_list=[]
    heapq.heappush(open_list,(0,start))
    came={}
    cost={start:0}
    while open_list:
        _,cur=heapq.heappop(open_list)
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
            new_cost=cost[cur]+1
            if (nx,ny) not in cost or new_cost<cost[(nx,ny)]:
                cost[(nx,ny)]=new_cost
                came[(nx,ny)]=cur
                heapq.heappush(open_list,(new_cost+h((nx,ny),goal),(nx,ny)))
    return []

path = astar(obstacles,start,goal)

# ---------------------------------------------------
# ROBOT STATE INITIALIZATION
# ---------------------------------------------------

if "robot_state" not in st.session_state:
    st.session_state.robot_state = {"pos": start, "speed": 0.0, "i": 0}
if "remaining_energy" not in st.session_state:
    st.session_state.remaining_energy = battery_cap
if "run" not in st.session_state:
    st.session_state.run = False

# ---------------------------------------------------
# UI LAYOUT
# ---------------------------------------------------

map_col, telemetry_col = st.columns([3,1])
placeholder = map_col.empty()

if st.button("▶️ Start Navigation"):
    st.session_state.run = True
    st.session_state.robot_state = {"pos": start, "speed": 0.0, "i": 0}
    st.session_state.remaining_energy = battery_cap

# ---------------------------------------------------
# LOCAL PLANNER + CONTROLLER
# ---------------------------------------------------

if st.session_state.run and path:
    x, y = st.session_state.robot_state["pos"]
    theta = 0
    speed = st.session_state.robot_state["speed"]
    distance = 0
    time_elapsed = 0

    for waypoint in path:
        wx, wy = waypoint
        dx = wx - x
        dy = wy - y
        desired_heading = np.arctan2(dy, dx)
        heading_error = desired_heading - theta
        omega = heading_error
        
        v = min(speed + accel * dt, max_speed)

        # Physics/Power Calculations
        slope_grad = np.gradient(elevation)
        curr_slope = slope_grad[0][int(x), int(y)] 
        gravity_force = mass * g * np.sin(np.arctan(curr_slope))
        friction_force = mass * g * friction[int(x), int(y)]
        power_draw = (gravity_force + friction_force) * v

        # Battery Update
        st.session_state.remaining_energy -= (power_draw * dt) / 3600

        # Differential drive update (Using wheel_r from builder)
        w_r = (2*v + omega*track_width)/(2*wheel_r)
        w_l = (2*v - omega*track_width)/(2*wheel_r)

        theta += omega * dt
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        speed = v
        time_elapsed += dt

        # Update Session State for Telemetry
        st.session_state.robot_state = {"pos": (x, y), "speed": v, "i": 0}

        # ------------------------------------
        # VISUALIZATION
        # ------------------------------------
        fig = go.Figure()
        fig.add_trace(go.Surface(z=elevation, colorscale="Viridis", showscale=False))
        
        xpath, ypath = zip(*path)
        zpath = [elevation[int(px), int(py)] + 0.3 for px, py in path]
        
        fig.add_trace(go.Scatter3d(x=xpath, y=ypath, z=zpath, mode="lines", 
                                   line=dict(color="yellow", width=6), name="Path"))
        fig.add_trace(go.Scatter3d(x=[x], y=[y], z=[elevation[int(x), int(y)] + 1],
                                   mode="markers", marker=dict(size=8, color="red"), name="Robot"))
        
        fig.update_layout(template="plotly_dark", height=650, margin=dict(l=0, r=0, b=0, t=0))
        placeholder.plotly_chart(fig, use_container_width=True)
        time.sleep(0.01)

# ---------------------------------------------------
# TELEMETRY
# ---------------------------------------------------

if "robot_state" in st.session_state:
    st.divider()
    t_col1, t_col2, t_col3 = st.columns(3)
    
    s = st.session_state.robot_state
    v = s.get("speed", 0.0)
    
    with t_col1:
        st.metric("Live Speed", f"{v:.2f} m/s", delta=f"{accel:.2f} m/s²")
    with t_col2:
        st.metric("Battery Remaining", f"{st.session_state.remaining_energy:.1f} Wh")
    with t_col3:
        st.metric("Motor Capacity", f"{max_rpm} RPM")

    st.subheader("⚙️ Robot Performance Specs")
    performance_specs = pd.DataFrame({
        "Component": ["Motor Torque", "Max RPM", "Battery Capacity", "Chassis Mass"],
        "Value": [f"{torque} Nm", f"{max_rpm} RPM", f"{battery_cap} Wh", f"{mass} kg"]
    })
    st.table(performance_specs)