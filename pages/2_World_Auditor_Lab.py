import streamlit as st
import numpy as np
import plotly.graph_objects as go
from streamlit_drawable_canvas import st_canvas
import heapq

st.set_page_config(layout="wide", page_title="World Auditor Lab")

st.title("World Auditor Robot Navigation Lab")

# ---------------------------------------------------------
# CONSTANTS
# ---------------------------------------------------------

GRID = 60
CELL = 6
g = 9.81

SURFACES = {
    "Tarmac": {"color":"#000000","mu":1.0},
    "Concrete":{"color":"#808080","mu":0.9},
    "Grass":{"color":"#228B22","mu":0.35},
    "Mud":{"color":"#8B4513","mu":0.2},
    "Ice":{"color":"#ADD8E6","mu":0.05}
}

# ---------------------------------------------------------
# SIDEBAR
# ---------------------------------------------------------

with st.sidebar:

    st.header("Robot")

    mass = st.slider("Mass",200,2000,800)
    max_speed = st.slider("Max Speed",5,30,15)

    st.header("Painter Mode")

    mode = st.selectbox(
        "Mode",
        ["Elevation","Friction","Obstacle","Start","Goal"]
    )

    surface = st.selectbox(
        "Surface",
        list(SURFACES.keys())
    )

brush_color = SURFACES[surface]["color"]

# ---------------------------------------------------------
# MAP STORAGE
# ---------------------------------------------------------

if "elevation" not in st.session_state:
    st.session_state.elevation = np.zeros((GRID,GRID))

if "friction" not in st.session_state:
    st.session_state.friction = np.ones((GRID,GRID))

if "obstacle" not in st.session_state:
    st.session_state.obstacle = np.zeros((GRID,GRID))

if "start" not in st.session_state:
    st.session_state.start = None

if "goal" not in st.session_state:
    st.session_state.goal = None

# ---------------------------------------------------------
# CANVAS
# ---------------------------------------------------------

canvas = st_canvas(
    stroke_width=6,
    stroke_color=brush_color,
    background_color="#111",
    height=360,
    width=360,
    drawing_mode="freedraw",
    key="map"
)

# ---------------------------------------------------------
# PROCESS DRAWINGS
# ---------------------------------------------------------

if canvas.json_data:

    objects = canvas.json_data["objects"]

    for obj in objects:

        xs = [p[1] for p in obj["path"]]
        ys = [p[2] for p in obj["path"]]

        gx = int(np.mean(xs)/360*GRID)
        gy = int(np.mean(ys)/360*GRID)

        gx = np.clip(gx,0,GRID-1)
        gy = np.clip(gy,0,GRID-1)

        if mode=="Start":
            st.session_state.start=(gx,gy)

        elif mode=="Goal":
            st.session_state.goal=(gx,gy)

        elif mode=="Obstacle":
            st.session_state.obstacle[gx,gy]=1

        elif mode=="Friction":
            mu=SURFACES[surface]["mu"]
            st.session_state.friction[gx,gy]=mu

        elif mode=="Elevation":
            st.session_state.elevation[gx,gy]+=0.3

# ---------------------------------------------------------
# A* PATH PLANNER
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

            nx=x+dx
            ny=y+dy

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
# BUILD GRID
# ---------------------------------------------------------

grid = st.session_state.obstacle.copy()

path=[]

if st.session_state.start and st.session_state.goal:

    path=astar(grid,st.session_state.start,st.session_state.goal)

# ---------------------------------------------------------
# SIMULATION
# ---------------------------------------------------------

if "robot_i" not in st.session_state:
    st.session_state.robot_i=0

if st.button("Run Robot"):
    st.session_state.robot_i=0
    st.session_state.running=True

if "running" not in st.session_state:
    st.session_state.running=False

if st.session_state.running and path:

    st.session_state.robot_i+=1

    if st.session_state.robot_i>=len(path):
        st.session_state.running=False
    else:
        st.rerun()

robot=None

if path:

    robot=path[min(st.session_state.robot_i,len(path)-1)]

# ---------------------------------------------------------
# PHYSICS
# ---------------------------------------------------------

speed=max_speed

if robot:

    mu=st.session_state.friction[robot]

    brake=(speed**2)/(2*mu*g)

    slip=speed>np.sqrt(mu*g*5)

# ---------------------------------------------------------
# VISUALIZATION
# ---------------------------------------------------------

z=st.session_state.elevation

fig=go.Figure()

fig.add_trace(go.Surface(
    z=z,
    colorscale="Viridis",
    showscale=False
))

if path:

    xs=[p[0] for p in path]
    ys=[p[1] for p in path]

    fig.add_trace(go.Scatter3d(
        x=xs,
        y=ys,
        z=z[xs,ys]+0.5,
        mode="lines",
        line=dict(color="yellow",width=6),
        name="Path"
    ))

if robot:

    fig.add_trace(go.Scatter3d(
        x=[robot[0]],
        y=[robot[1]],
        z=[z[robot]+1],
        mode="markers",
        marker=dict(size=6,color="red"),
        name="Robot"
    ))

fig.update_layout(
    template="plotly_dark",
    height=650
)

st.plotly_chart(fig,use_container_width=True)

# ---------------------------------------------------------
# TELEMETRY
# ---------------------------------------------------------

if robot:

    c1,c2,c3=st.columns(3)

    c1.metric("Speed",f"{speed:.1f} m/s")
    c2.metric("Brake Distance",f"{brake:.1f} m")
    c3.metric("Slip Risk","YES" if slip else "NO")

# ---------------------------------------------------------
# HELP
# ---------------------------------------------------------

st.info(
"""
How to use:

1. Select **Start** mode and click map
2. Select **Goal** mode and click map
3. Draw **Obstacles**
4. Paint **Friction surfaces**
5. Press **Run Robot**

Robot will compute a path and drive the map.
"""
)