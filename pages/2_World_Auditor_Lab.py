import streamlit as st
import numpy as np
import plotly.graph_objects as go
from streamlit_drawable_canvas import st_canvas
import heapq
import time
import pandas as pd

st.set_page_config(layout="wide", page_title="World Auditor Lab")

st.title("🌍 World Auditor Robot Navigation Lab")

# ---------------------------------------------------------
# LOAD ROBOT CONFIG
# ---------------------------------------------------------

if "robot_config" not in st.session_state:

    st.warning("⚠️ Please configure a robot in Robot Builder first")
    st.stop()

robot_cfg = st.session_state.get("robot_config", {})

mass = robot_cfg.get("mass", 800)
max_speed = robot_cfg.get("max_speed", 15)
accel = robot_cfg.get("accel", 4)

# ---------------------------------------------------------
# CONSTANTS
# ---------------------------------------------------------

GRID = 60
WORLD_SIZE = 10
CELL = WORLD_SIZE / GRID
g = 9.81
dt = 0.1

BACKGROUND="#202830"

SURFACES = {

"Tarmac":{"color":"#2f2f2f","mu":1.0},
"Concrete":{"color":"#909090","mu":0.9},
"Grass":{"color":"#2ecc71","mu":0.35},
"Mud":{"color":"#8e5a2b","mu":0.2},
"Ice":{"color":"#6fd3ff","mu":0.05}

}

# ---------------------------------------------------------
# SIDEBAR
# ---------------------------------------------------------

with st.sidebar:

    st.header("Painter")

    mode = st.selectbox(
        "Paint Mode",
        ["Elevation","Friction","Obstacle","Start","Goal"]
    )

    surface = st.selectbox("Surface",list(SURFACES.keys()))

brush_color = SURFACES[surface]["color"]

# ---------------------------------------------------------
# MAP STATE
# ---------------------------------------------------------

def init(name,val):
    if name not in st.session_state:
        st.session_state[name]=val

init("elevation",np.zeros((GRID,GRID)))
init("friction",np.ones((GRID,GRID)))
init("obstacle",np.zeros((GRID,GRID)))
init("start",None)
init("goal",None)

# ---------------------------------------------------------
# CANVAS
# ---------------------------------------------------------

canvas = st_canvas(
stroke_width=8,
stroke_color=brush_color,
background_color=BACKGROUND,
height=360,
width=360,
drawing_mode="freedraw",
key="canvas"
)

# ---------------------------------------------------------
# PROCESS DRAW
# ---------------------------------------------------------

if canvas.json_data:

    for obj in canvas.json_data["objects"]:

        for p in obj["path"]:

            if len(p)<3:
                continue

            x=int(p[1]/360*GRID)
            y=int(p[2]/360*GRID)

            x=np.clip(x,0,GRID-1)
            y=np.clip(y,0,GRID-1)

            if mode=="Start":
                st.session_state.start=(x,y)

            elif mode=="Goal":
                st.session_state.goal=(x,y)

            elif mode=="Obstacle":
                st.session_state.obstacle[x,y]=1

            elif mode=="Friction":
                st.session_state.friction[x,y]=SURFACES[surface]["mu"]

            elif mode=="Elevation":
                st.session_state.elevation[x,y]+=0.2

# ---------------------------------------------------------
# PATH PLANNER
# ---------------------------------------------------------

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
                heapq.heappush(open,(new+h((nx,ny),goal),(nx,ny)))

    return []

# ---------------------------------------------------------
# BUILD PATH
# ---------------------------------------------------------

path=[]

if st.session_state.start and st.session_state.goal:

    path=astar(
        st.session_state.obstacle,
        st.session_state.start,
        st.session_state.goal
    )

# ---------------------------------------------------------
# ROBOT SIMULATION
# ---------------------------------------------------------

if "robot_state" not in st.session_state:

    st.session_state.robot_state={
        "i":0,
        "pos":None,
        "speed":0
    }

if st.button("🚀 Run Audit") and path:

    st.session_state.robot_state["i"]=0
    st.session_state.robot_state["pos"]=np.array(path[0],dtype=float)
    st.session_state.robot_state["speed"]=0
    st.session_state.running=True

if "running" not in st.session_state:
    st.session_state.running=False

robot=None

if st.session_state.running and path:

    s=st.session_state.robot_state

    target=np.array(path[s["i"]])

    direction=target-s["pos"]

    dist=np.linalg.norm(direction)

    if dist>0:
        direction/=dist

    s["speed"]=min(s["speed"]+accel*dt,max_speed)

    s["pos"]+=direction*s["speed"]*dt

    if dist<0.3:
        s["i"]+=1

    if s["i"]>=len(path):
        st.session_state.running=False

    robot=s["pos"]

    time.sleep(0.03)
    st.rerun()

# ---------------------------------------------------------
# 3D WORLD
# ---------------------------------------------------------

z=st.session_state.elevation

fig=go.Figure()

fig.add_trace(go.Surface(
z=z,
colorscale="Viridis",
showscale=False,
name="Elevation"
))

if path:

    xs=[p[0] for p in path]
    ys=[p[1] for p in path]
    zs=[z[int(x),int(y)]+0.3 for x,y in zip(xs,ys)]

    fig.add_trace(go.Scatter3d(
    x=xs,
    y=ys,
    z=zs,
    mode="lines",
    line=dict(color="yellow",width=6),
    name="Path"
    ))

if robot is not None:

    fig.add_trace(go.Scatter3d(
    x=[robot[0]],
    y=[robot[1]],
    z=[z[int(robot[0]),int(robot[1])]+1],
    mode="markers",
    marker=dict(size=10,color="red"),
    name="Robot"
    ))

fig.update_layout(

scene=dict(
xaxis_title="X Position (East-West)",
yaxis_title="Y Position (North-South)",
zaxis_title="Elevation (meters)"
),

template="plotly_dark",
height=700,
showlegend=True
)

st.plotly_chart(fig,use_container_width=True)

# ---------------------------------------------------------
# TELEMETRY
# ---------------------------------------------------------

if robot is not None:

    rx,ry=int(robot[0]),int(robot[1])

    mu=st.session_state.friction[rx,ry]

    v=st.session_state.robot_state["speed"]

    brake=v**2/(2*mu*g)

    slip=v>np.sqrt(mu*g*5)

    telemetry=pd.DataFrame({

    "Metric":[
    "Robot X",
    "Robot Y",
    "Speed",
    "Surface μ",
    "Brake Distance",
    "Slip Risk"
    ],

    "Value":[
    round(robot[0],2),
    round(robot[1],2),
    round(v,2),
    round(mu,2),
    round(brake,2),
    "YES" if slip else "NO"
    ]

    })

    st.write("### Robot Telemetry")

    st.table(telemetry)