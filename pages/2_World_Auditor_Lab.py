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

    elevation = np.zeros((GRID,GRID))
    friction = np.ones((GRID,GRID))
    obstacle = np.zeros((GRID,GRID))

    for x in range(GRID):
        for y in range(GRID):
            elevation[x,y] = 2*np.sin(x/12)+np.cos(y/10)

    obstacle[40:80,50] = 1
    obstacle[20:60,30] = 1

    friction[10:40,60:90] = 0.3

    return elevation, friction, obstacle

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

# ---------------------------------------------------------
# TELEMETRY
# ---------------------------------------------------------

if positions:
    # Safety Check: Pull from session_state or use a placeholder if missing
    # This prevents the NameError you just saw
    robot_data = st.session_state.get("robot_config", {})
    tq = robot_data.get("motor_torque", "N/A")
    rpm = robot_data.get("max_rpm", "N/A")
    bat = robot_data.get("battery_capacity", "N/A")

    telemetry = pd.DataFrame({
        "Metric":[
            "Total Time",
            "Distance Travelled",
            "Max Speed achieved",
            "Robot Mass",
            "Motor Torque",
            "Max RPM",
            "Battery Capacity"
        ],
        "Value":[
            f"{time_elapsed:.2f} s",
            f"{distance:.2f} m",
            f"{max(speeds):.2f} m/s",
            f"{mass} kg",
            f"{tq} Nm",
            f"{rpm} RPM",
            f"{bat} Wh"
        ]
    })

    telemetry_col.subheader("Telemetry")
    telemetry_col.table(telemetry)

    # Speed Chart
    speed_fig = go.Figure()
    speed_fig.add_trace(go.Scatter(
        x=times,
        y=speeds,
        mode="lines",
        line=dict(color="#00CC96")
    ))
    speed_fig.update_layout(
        template="plotly_dark",
        title="Speed vs Time",
        xaxis_title="Time (s)",
        yaxis_title="Velocity (m/s)"
    )
    st.plotly_chart(speed_fig, use_container_width=True)

    # Heading Chart
    heading_fig = go.Figure()
    heading_fig.add_trace(go.Scatter(
        x=times,
        y=headings,
        mode="lines",
        line=dict(color="#AB63FA")
    ))
    heading_fig.update_layout(
        template="plotly_dark",
        title="Heading vs Time",
        xaxis_title="Time (s)",
        yaxis_title="Heading (rad)"
    )
    st.plotly_chart(heading_fig, use_container_width=True)