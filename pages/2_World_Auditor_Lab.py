import streamlit as st
import numpy as np
import plotly.graph_objects as go
import heapq
import pandas as pd
import time

st.set_page_config(layout="wide", page_title="World Auditor Lab")

st.title("🌍 World Auditor Robotics Simulation")

# -------------------------------------------------
# LOAD ROBOT
# -------------------------------------------------

robot_cfg = st.session_state.get("robot", None)

if robot_cfg is None:
    st.error("No robot configured. Please build one in Robot Builder.")
    st.stop()

mass = robot_cfg["mass"]
accel = robot_cfg["accel"]
max_speed = robot_cfg["max_speed"]
track_width = robot_cfg["track_width"]

# -------------------------------------------------
# CONSTANTS
# -------------------------------------------------

GRID = 100
dt = 0.1
g = 9.81

# -------------------------------------------------
# MAP GENERATORS
# -------------------------------------------------

def hill_map():

    elev = np.zeros((GRID,GRID))
    friction = np.ones((GRID,GRID))
    obs = np.zeros((GRID,GRID))

    for x in range(GRID):
        for y in range(GRID):
            elev[x,y] = 2*np.sin(x/10)+1.5*np.cos(y/15)

    return elev,friction,obs


def obstacle_field():

    elev = np.zeros((GRID,GRID))
    friction = np.ones((GRID,GRID))
    obs = np.zeros((GRID,GRID))

    obs[20:80,50] = 1
    obs[60,10:70] = 1
    obs[40:70,30] = 1

    return elev,friction,obs


def ice_world():

    elev = np.zeros((GRID,GRID))
    friction = np.ones((GRID,GRID))*0.2
    obs = np.zeros((GRID,GRID))

    return elev,friction,obs


def mixed_world():

    elev = np.zeros((GRID,GRID))
    friction = np.ones((GRID,GRID))
    obs = np.zeros((GRID,GRID))

    for x in range(GRID):
        for y in range(GRID):
            elev[x,y] = np.sin(x/8)*2

    friction[40:80,10:40] = 0.3
    obs[50:70,50] = 1

    return elev,friction,obs


# -------------------------------------------------
# SIDEBAR
# -------------------------------------------------

with st.sidebar:

    st.header("Simulation")

    world = st.selectbox(
        "Terrain",
        ["Hill Terrain","Obstacle Field","Ice World","Mixed Terrain"]
    )

    run = st.button("Run Robot")

start = (5,5)
goal = (GRID-5, GRID-5)

# -------------------------------------------------
# LOAD MAP
# -------------------------------------------------

if world=="Hill Terrain":
    elevation,friction,obstacles = hill_map()

elif world=="Obstacle Field":
    elevation,friction,obstacles = obstacle_field()

elif world=="Ice World":
    elevation,friction,obstacles = ice_world()

else:
    elevation,friction,obstacles = mixed_world()

# -------------------------------------------------
# A* PATH PLANNER
# -------------------------------------------------

def astar(grid,start,goal):

    def h(a,b):
        return abs(a[0]-b[0])+abs(a[1]-b[1])

    open=[]
    heapq.heappush(open,(0,start))

    came={}
    cost={start:0}

    while open:

        _,cur = heapq.heappop(open)

        if cur==goal:

            path=[cur]

            while cur in came:
                cur=came[cur]
                path.append(cur)

            return path[::-1]

        x,y = cur

        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:

            nx,ny = x+dx,y+dy

            if nx<0 or ny<0 or nx>=GRID or ny>=GRID:
                continue

            if grid[nx,ny]==1:
                continue

            new = cost[cur]+1

            if (nx,ny) not in cost or new<cost[(nx,ny)]:

                cost[(nx,ny)] = new
                came[(nx,ny)] = cur

                heapq.heappush(
                    open,
                    (new+h((nx,ny),goal),(nx,ny))
                )

    return []

path = astar(obstacles,start,goal)

# -------------------------------------------------
# LAYOUT
# -------------------------------------------------

map_col, dashboard = st.columns([3,1])

placeholder = map_col.empty()

# -------------------------------------------------
# ROBOT SIM
# -------------------------------------------------

robot = np.array(start,dtype=float)
speed = 0

positions=[]
speeds=[]
times=[]
frictions=[]
slopes=[]

distance=0
time_elapsed=0

if run and path:

    for p in path:

        target=np.array(p)

        direction=target-robot
        dist=np.linalg.norm(direction)

        if dist>0:
            direction/=dist

        speed=min(speed+accel*dt,max_speed)

        move=speed*dt
        robot+=direction*move

        rx,ry=int(robot[0]),int(robot[1])

        mu=friction[rx,ry]

        slope=elevation[rx,ry]

        brake = speed**2/(2*mu*g)

        slip = speed > np.sqrt(mu*g*5)

        distance+=move
        time_elapsed+=dt

        positions.append(robot.copy())
        speeds.append(speed)
        times.append(time_elapsed)
        frictions.append(mu)
        slopes.append(slope)

        # ----------------------------------
        # 3D MAP
        # ----------------------------------

        fig=go.Figure()

        fig.add_trace(go.Surface(
            z=elevation,
            colorscale="Viridis",
            showscale=False
        ))

        xs=[p[0] for p in path]
        ys=[p[1] for p in path]
        zs=[elevation[int(x),int(y)]+0.4 for x,y in zip(xs,ys)]

        fig.add_trace(go.Scatter3d(
            x=xs,y=ys,z=zs,
            mode="lines",
            line=dict(color="yellow",width=6),
            name="Path"
        ))

        fig.add_trace(go.Scatter3d(
            x=[robot[0]],
            y=[robot[1]],
            z=[elevation[rx,ry]+1],
            mode="markers",
            marker=dict(size=8,color="red"),
            name="Robot"
        ))

        fig.update_layout(
            template="plotly_dark",
            height=700,
            scene=dict(
                xaxis_title="X Position",
                yaxis_title="Y Position",
                zaxis_title="Elevation"
            )
        )

        placeholder.plotly_chart(fig,use_container_width=True)

        time.sleep(0.03)

# -------------------------------------------------
# TELEMETRY DASHBOARD
# -------------------------------------------------

if positions:

    telemetry = pd.DataFrame({

        "Metric":[
            "Total Time",
            "Distance",
            "Max Speed",
            "Robot Mass",
            "Track Width"
        ],

        "Value":[
            round(time_elapsed,2),
            round(distance,2),
            max(speeds),
            mass,
            track_width
        ]
    })

    dashboard.subheader("Telemetry")
    dashboard.table(telemetry)

    # SPEED GRAPH
    speed_fig=go.Figure()

    speed_fig.add_trace(go.Scatter(
        x=times,
        y=speeds,
        mode="lines",
        name="Speed"
    ))

    speed_fig.update_layout(
        template="plotly_dark",
        title="Speed vs Time",
        xaxis_title="Time",
        yaxis_title="Speed"
    )

    st.plotly_chart(speed_fig,use_container_width=True)

    # FRICTION GRAPH
    friction_fig=go.Figure()

    friction_fig.add_trace(go.Scatter(
        x=times,
        y=frictions,
        mode="lines",
        name="Friction"
    ))

    friction_fig.update_layout(
        template="plotly_dark",
        title="Surface Friction"
    )

    st.plotly_chart(friction_fig,use_container_width=True)

    # SLOPE GRAPH
    slope_fig=go.Figure()

    slope_fig.add_trace(go.Scatter(
        x=times,
        y=slopes,
        mode="lines",
        name="Slope"
    ))

    slope_fig.update_layout(
        template="plotly_dark",
        title="Elevation Profile"
    )

    st.plotly_chart(slope_fig,use_container_width=True)