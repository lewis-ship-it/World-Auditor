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

robot_cfg = st.session_state.get("robot")

if robot_cfg is None:
    st.error("Build a robot first in Robot Builder.")
    st.stop()

mass = robot_cfg["mass"]
max_speed = robot_cfg["max_speed"]
accel = robot_cfg["accel"]
track_width = robot_cfg["track_width"]

wheel_radius = robot_cfg.get("wheel_radius",0.25)

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

    elevation = np.zeros((GRID,GRID))
    friction = np.ones((GRID,GRID))
    obstacles = np.zeros((GRID,GRID))

    for x in range(GRID):
        for y in range(GRID):
            elevation[x,y] = 2*np.sin(x/10)+np.cos(y/8)

    obstacles[30:70,40] = 1
    obstacles[50,10:60] = 1

    friction[20:40,20:50] = 0.3

    return elevation,friction,obstacles

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

if positions:

    telemetry = pd.DataFrame({

        "Metric":[
            "Total Time",
            "Distance Travelled",
            "Max Speed",
            "Robot Mass",
            "Track Width"
        ],

        "Value":[
            round(time_elapsed,2),
            round(distance,2),
            round(max(speeds),2),
            mass,
            track_width
        ]

    })

    telemetry_col.subheader("Robot Telemetry")
    telemetry_col.table(telemetry)

    speed_fig=go.Figure()

    speed_fig.add_trace(go.Scatter(
        x=times,
        y=speeds,
        mode="lines"
    ))

    speed_fig.update_layout(
        template="plotly_dark",
        title="Speed vs Time",
        xaxis_title="Time",
        yaxis_title="Speed"
    )

    st.plotly_chart(speed_fig,use_container_width=True)

    heading_fig=go.Figure()

    heading_fig.add_trace(go.Scatter(
        x=times,
        y=headings,
        mode="lines"
    ))

    heading_fig.update_layout(
        template="plotly_dark",
        title="Heading vs Time",
        xaxis_title="Time",
        yaxis_title="Heading"
    )

    st.plotly_chart(heading_fig,use_container_width=True)