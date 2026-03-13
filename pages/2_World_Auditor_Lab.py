import streamlit as st
import numpy as np
import plotly.graph_objects as go
import heapq
import pandas as pd
import time

st.set_page_config(layout="wide", page_title="World Auditor RViz Simulator")

st.title("🤖 World Auditor Navigation Simulator")

# ---------------------------------------------------------
# SAFE ROBOT LOADING
# ---------------------------------------------------------

if "robot" not in st.session_state:

    st.warning("No robot found — loading default robot")

    st.session_state.robot = {
        "mass":500,
        "accel":3,
        "max_speed":10,
        "track_width":0.6,
        "wheel_radius":0.25
    }

robot = st.session_state.robot

mass = robot["mass"]
accel = robot["accel"]
max_speed = robot["max_speed"]
track_width = robot["track_width"]
wheel_radius = robot["wheel_radius"]

# ---------------------------------------------------------
# CONSTANTS
# ---------------------------------------------------------

GRID = 100
dt = 0.1
g = 9.81

# ---------------------------------------------------------
# MAP SERVER
# ---------------------------------------------------------
def generate_world():
    # Standardizing names to avoid NameError
    elevation = np.zeros((GRID, GRID))
    friction = np.ones((GRID, GRID))
    obstacle = np.zeros((GRID, GRID))

    # --- PRESET 1: THE TORQUE CLIMB ---
    # Elevation rises to test motor torque vs mass
    for x in range(0, 25):
        elevation[x, :] = (x / 4) 

    # --- PRESET 2: THE TIRE TEST ZONES ---
    # Mud Zone (Medium Grip)
    friction[35:50, :] = 0.4 
    # Ice Zone (Low Grip)
    friction[60:75, :] = 0.1 

    # --- PRESET 3: NAVIGATION WALL ---
    obstacle[85, 0:40] = 1
    obstacle[85, 60:100] = 1

    return elevation, friction, obstacle

# Ensure these match the return statement exactly
elevation, friction, obstacle = generate_world()

# ---------------------------------------------------------
# COSTMAP (ROS style)
# ---------------------------------------------------------

costmap = obstacle.copy()

inflation_radius = 3

for x in range(GRID):
    for y in range(GRID):

        if obstacle[x,y] == 1:

            for i in range(-inflation_radius, inflation_radius):
                for j in range(-inflation_radius, inflation_radius):

                    nx = x+i
                    ny = y+j

                    if 0 <= nx < GRID and 0 <= ny < GRID:
                        costmap[nx,ny] = max(costmap[nx,ny],0.5)

# ---------------------------------------------------------
# START GOAL
# ---------------------------------------------------------

start = (5,5)
goal = (90,90)

# ---------------------------------------------------------
# GLOBAL PLANNER
# ---------------------------------------------------------

def astar(grid,start,goal):

    def h(a,b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

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

            if grid[nx,ny] >= 0.5:
                continue

            new=cost[cur]+1

            if (nx,ny) not in cost or new<cost[(nx,ny)]:

                cost[(nx,ny)] = new
                came[(nx,ny)] = cur

                heapq.heappush(open,(new+h((nx,ny),goal),(nx,ny)))

    return []

path = astar(costmap,start,goal)

# ---------------------------------------------------------
# UI
# ---------------------------------------------------------

map_col, telemetry_col = st.columns([3,1])

run = st.button("Start Navigation")

placeholder = map_col.empty()

# ---------------------------------------------------------
# ROBOT STATE
# ---------------------------------------------------------

x,y = start
theta = 0
speed = 0

positions=[]
speeds=[]
times=[]
frictions=[]
headings=[]

distance=0
time_elapsed=0

# ---------------------------------------------------------
# SIMULATION
# ---------------------------------------------------------

if run and path:

    for wp in path:

        wx,wy = wp

        dx = wx-x
        dy = wy-y

        target_heading = np.arctan2(dy,dx)

        heading_error = target_heading - theta

        omega = heading_error

        v = min(speed + accel*dt, max_speed)

        theta += omega*dt

        x += v*np.cos(theta)*dt
        y += v*np.sin(theta)*dt

        speed = v

        rx,ry=int(x),int(y)

        mu = friction[rx,ry]

        brake = v**2/(2*mu*g)

        slip = v > np.sqrt(mu*g*5)
        # Calculate how much torque is actually being used vs what is available
        # motor_force = torque / wheel_radius
        max_power = (torque * max_rpm) / 9.5488
        effort = (power_draw / (torque * max_rpm / 9.5488)) * 100
        efforts.append(min(effort, 100))  # Ensure it doesn't exceed 100%

        distance += v*dt
        time_elapsed += dt

        positions.append((x,y))
        speeds.append(v)
        times.append(time_elapsed)
        frictions.append(mu)
        headings.append(theta)

        # ------------------------------------
        # RVIZ STYLE MAP
        # ------------------------------------

        fig = go.Figure()

        fig.add_trace(go.Heatmap(
            z=costmap,
            colorscale="inferno",
            showscale=False
        ))

        xs=[p[0] for p in path]
        ys=[p[1] for p in path]

        fig.add_trace(go.Scatter(
            x=xs,
            y=ys,
            mode="lines",
            line=dict(color="cyan",width=3),
            name="Global Path"
        ))

        fig.add_trace(go.Scatter(
            x=[x],
            y=[y],
            mode="markers",
            marker=dict(size=12,color="red"),
            name="Robot"
        ))

        # robot heading arrow

        fig.add_annotation(
            x=x,
            y=y,
            ax=x+np.cos(theta)*3,
            ay=y+np.sin(theta)*3,
            showarrow=True,
            arrowwidth=3
        )

        fig.update_layout(
            template="plotly_dark",
            height=700,
            xaxis_title="Map X",
            yaxis_title="Map Y"
        )

        placeholder.plotly_chart(fig,use_container_width=True)

        time.sleep(0.03)


# ---------------------------------------------------------
# TELEMETRY
# ---------------------------------------------------------

if positions:
    # Get specs for calculation
    robot_data = st.session_state.get("robot_config", {})
    tq = robot_data.get("motor_torque", 120)
    
    telemetry = pd.DataFrame({
        "Metric":["Total Time", "Distance", "Max Speed", "Motor Torque", "Battery Capacity"],
        "Value":[f"{time_elapsed:.2f} s", f"{distance:.2f} m", f"{max(speeds):.2f} m/s", f"{tq} Nm", f"{battery_cap} Wh"]
    })

    telemetry_col.subheader("Telemetry")
    telemetry_col.table(telemetry)

    # --- SPEED GRAPH ---
    speed_fig = go.Figure()
    speed_fig.add_trace(go.Scatter(x=times, y=speeds, mode="lines", 
                                   line=dict(color="#00e5ff", width=3, shape='spline'),
                                   fill='tozeroy', fillcolor='rgba(0, 229, 255, 0.1)'))
    speed_fig.update_layout(template="plotly_dark", title="Velocity Profile (m/s)", height=300)
    st.plotly_chart(speed_fig, use_container_width=True)

    # --- NEW: CONTROL EFFORT GRAPH ---
    effort_fig = go.Figure()
    effort_fig.add_trace(go.Scatter(x=times, y=efforts, mode="lines", 
                                    line=dict(color="#ffea00", width=3),
                                    fill='tozeroy', fillcolor='rgba(255, 234, 0, 0.1)'))
    effort_fig.update_layout(template="plotly_dark", title="Control Effort (% Motor Load)", 
                             yaxis=dict(range=[0, 110]), height=300)
    st.plotly_chart(effort_fig, use_container_width=True)

    # --- HEADING GRAPH ---
    heading_fig = go.Figure()
    heading_fig.add_trace(go.Scatter(x=times, y=headings, mode="lines", line=dict(color="#ff007f", width=3)))
    heading_fig.update_layout(template="plotly_dark", title="Heading (Theta)", height=300)
    st.plotly_chart(heading_fig, use_container_width=True)