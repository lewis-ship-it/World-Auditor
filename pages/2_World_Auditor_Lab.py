import streamlit as st
import numpy as np
import plotly.graph_objects as go
import heapq
import time
import pandas as pd

st.set_page_config(layout="wide", page_title="World Auditor Lab")

st.title("🤖 World Auditor Navigation Simulator")

# -------------------------------------------------
# CONSTANTS
# -------------------------------------------------

GRID = 80
WORLD_SIZE = 20
CELL = WORLD_SIZE / GRID

g = 9.81
dt = 0.05

# -------------------------------------------------
# LOAD ROBOT CONFIG
# -------------------------------------------------

if "robot_cfg" not in st.session_state:

    st.error("⚠ Build a robot first in the Robot Builder page.")
    st.stop()

robot = st.session_state.robot_cfg

mass = robot["mass"]
wheelbase = robot["wheelbase"]
traction = robot["traction"]
motor_force = robot["motor_force"]

# -------------------------------------------------
# GENERATE TEST TERRAIN
# -------------------------------------------------

np.random.seed(0)

x = np.linspace(-3,3,GRID)
y = np.linspace(-3,3,GRID)

X,Y = np.meshgrid(x,y)

elevation = np.sin(X)*np.cos(Y)*2
elevation += np.exp(-(X**2 + Y**2))*4

# friction map

friction = np.ones((GRID,GRID))*0.9

friction[20:40,10:30] = 0.3
friction[50:70,50:70] = 0.1

# obstacles

obstacles = np.zeros((GRID,GRID))

obstacles[30:35,:] = 1
obstacles[:,40:42] = 1

# -------------------------------------------------
# SIDEBAR CONTROLS
# -------------------------------------------------

with st.sidebar:

    st.header("Simulation")

    start_x = st.slider("Start X",0,GRID-1,5)
    start_y = st.slider("Start Y",0,GRID-1,5)

    goal_x = st.slider("Goal X",0,GRID-1,70)
    goal_y = st.slider("Goal Y",0,GRID-1,70)

    run = st.button("Run Navigation")

# -------------------------------------------------
# PATH PLANNER
# -------------------------------------------------

def astar(grid,start,goal):

    h=lambda a,b:abs(a[0]-b[0])+abs(a[1]-b[1])

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

                heapq.heappush(
                    open,
                    (new+h((nx,ny),goal),(nx,ny))
                )

    return []

# -------------------------------------------------
# COMPUTE PATH
# -------------------------------------------------

start=(start_x,start_y)
goal=(goal_x,goal_y)

path = astar(obstacles,start,goal)

# -------------------------------------------------
# ROBOT SIMULATION
# -------------------------------------------------

robot_pos=None
speed=0
time_elapsed=0

telemetry=[]

if run and path:

    robot_pos=np.array(path[0],dtype=float)

    for target in path[1:]:

        target=np.array(target)

        while True:

            direction=target-robot_pos
            dist=np.linalg.norm(direction)

            if dist < 0.1:
                break

            direction/=dist

            mu = friction[int(robot_pos[0]),int(robot_pos[1])]

            max_traction_speed = np.sqrt(mu*g*wheelbase)

            accel = motor_force/mass

            speed += accel*dt
            speed = min(speed,max_traction_speed)

            robot_pos += direction*speed*dt

            curvature = 1/max(dist,0.01)

            brake = speed**2/(2*mu*g)

            slip = speed > max_traction_speed

            telemetry.append([
                time_elapsed,
                robot_pos[0],
                robot_pos[1],
                speed,
                accel,
                mu,
                curvature,
                brake,
                slip
            ])

            time_elapsed+=dt

# -------------------------------------------------
# TELEMETRY TABLE
# -------------------------------------------------

telemetry_df = pd.DataFrame(
    telemetry,
    columns=[
        "time",
        "x",
        "y",
        "speed",
        "accel",
        "friction",
        "curvature",
        "brake_distance",
        "slip"
    ]
)

# -------------------------------------------------
# 3D VISUALIZATION
# -------------------------------------------------

fig = go.Figure()

fig.add_trace(go.Surface(
    z=elevation,
    colorscale="Viridis",
    showscale=False
))

if path:

    xs=[p[0] for p in path]
    ys=[p[1] for p in path]
    zs=[elevation[x][y]+0.5 for x,y in path]

    fig.add_trace(go.Scatter3d(
        x=xs,
        y=ys,
        z=zs,
        mode="lines",
        line=dict(color="yellow",width=6),
        name="Path"
    ))

if telemetry:

    fig.add_trace(go.Scatter3d(
        x=telemetry_df["x"],
        y=telemetry_df["y"],
        z=elevation[
            telemetry_df["x"].astype(int),
            telemetry_df["y"].astype(int)
        ]+1,
        mode="markers",
        marker=dict(size=4,color="red"),
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

st.plotly_chart(fig,use_container_width=True)

# -------------------------------------------------
# TELEMETRY UI
# -------------------------------------------------

st.subheader("Robot Telemetry")

if not telemetry_df.empty:

    col1,col2,col3,col4 = st.columns(4)

    col1.metric("Final Speed",round(telemetry_df.speed.iloc[-1],2))
    col2.metric("Total Time",round(telemetry_df.time.iloc[-1],2))
    col3.metric("Max Brake Distance",round(telemetry_df.brake_distance.max(),2))
    col4.metric("Slip Events",telemetry_df.slip.sum())

    st.dataframe(telemetry_df)