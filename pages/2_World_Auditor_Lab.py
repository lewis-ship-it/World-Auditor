import streamlit as st
import numpy as np
import plotly.graph_objects as go
from streamlit_drawable_canvas import st_canvas
import heapq

st.set_page_config(layout="wide", page_title="World Auditor Lab")

# ---------------------------------------------------------
# PHYSICS CONSTANTS
# ---------------------------------------------------------

g = 9.81

SURFACE_PHYSICS = {
    "Tarmac": {"color": "#000000", "mu": 1.0},
    "Concrete": {"color": "#808080", "mu": 0.9},
    "Grass": {"color": "#228B22", "mu": 0.35},
    "Mud": {"color": "#8B4513", "mu": 0.2},
    "Ice": {"color": "#ADD8E6", "mu": 0.05}
}

# ---------------------------------------------------------
# ROBOT PHYSICS
# ---------------------------------------------------------

def braking_distance(v, mu):
    return v**2 / (2 * mu * g)

def max_turn_speed(radius, mu):
    return np.sqrt(mu * g * radius)

def slip_detect(v, mu):
    limit = np.sqrt(mu * g * 10)
    return v > limit

# ---------------------------------------------------------
# AI PATH PLANNER (A*)
# ---------------------------------------------------------

def astar(grid,start,goal):

    h=lambda a,b: abs(a[0]-b[0])+abs(a[1]-b[1])

    open_set=[]
    heapq.heappush(open_set,(0,start))

    came={}
    gscore={start:0}

    while open_set:

        _,current=heapq.heappop(open_set)

        if current==goal:
            path=[current]
            while current in came:
                current=came[current]
                path.append(current)
            return path[::-1]

        x,y=current

        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:

            nx,ny=x+dx,y+dy

            if nx<0 or ny<0 or nx>=grid.shape[0] or ny>=grid.shape[1]:
                continue

            if grid[nx,ny]==1:
                continue

            tentative=gscore[current]+1

            if (nx,ny) not in gscore or tentative<gscore[(nx,ny)]:

                came[(nx,ny)]=current
                gscore[(nx,ny)]=tentative
                f=tentative+h((nx,ny),goal)
                heapq.heappush(open_set,(f,(nx,ny)))

    return []

# ---------------------------------------------------------
# SIDEBAR CONFIG
# ---------------------------------------------------------

with st.sidebar:

    st.header("Robot")

    mass=st.slider("Mass",200,2000,800)
    max_speed=st.slider("Max speed",5,40,20)
    width=st.slider("Robot width",0.5,2.5,1.2)
    wheelbase=st.slider("Wheelbase",0.5,3.5,1.6)
    clearance=st.slider("Ground clearance",0.05,0.5,0.2)

    st.header("Environment")

    terrain_seed=st.slider("Terrain seed",0,100,42)
    generate=st.button("Generate Terrain")

# ---------------------------------------------------------
# TERRAIN GENERATION
# ---------------------------------------------------------

np.random.seed(terrain_seed)

track_len=400
dist=np.linspace(0,track_len,400)

base=np.sin(dist*0.03)*3
noise=np.random.normal(0,0.3,len(dist))

elevation=base+noise

friction=np.ones_like(dist)

# ---------------------------------------------------------
# PAINTER
# ---------------------------------------------------------

st.subheader("Terrain Painter")

mode=st.selectbox("Painter Mode",
["Elevation","Friction","Obstacles"])

brush_surface=st.selectbox(
"Surface",
list(SURFACE_PHYSICS.keys())
)

brush_color=SURFACE_PHYSICS[brush_surface]["color"]

canvas=st_canvas(
stroke_width=5,
stroke_color=brush_color,
height=250,
background_color="#0E1117",
drawing_mode="freedraw",
key="canvas"
)

# ---------------------------------------------------------
# CANVAS PROCESSING
# ---------------------------------------------------------

obstacles=[]

if canvas.json_data:

    for obj in canvas.json_data["objects"]:

        color=obj["stroke"]

        xs=[p[1] for p in obj["path"]]
        ys=[p[2] for p in obj["path"]]

        m_start=min(xs)/600*track_len
        m_end=max(xs)/600*track_len

        mask=(dist>=m_start)&(dist<=m_end)

        if mode=="Elevation":
            elev=(250-np.array(ys))/20
            elevation=np.interp(dist,
                                np.linspace(0,track_len,len(elev)),
                                elev)

        elif mode=="Friction":

            for name,data in SURFACE_PHYSICS.items():
                if data["color"]==color:
                    friction[mask]=data["mu"]

        elif mode=="Obstacles":

            obstacles.append((m_start,0))

# ---------------------------------------------------------
# SAFE SPEED PROFILE
# ---------------------------------------------------------

safe_v=np.zeros_like(dist)

for i in range(len(dist)-2,-1,-1):

    mu=friction[i]

    ds=dist[i+1]-dist[i]

    a=mu*g

    safe_v[i]=min(max_speed,
        np.sqrt(safe_v[i+1]**2+2*a*ds))

# ---------------------------------------------------------
# SIMULATION ENGINE
# ---------------------------------------------------------

if "sim_pos" not in st.session_state:
    st.session_state.sim_pos=0

play=st.button("Play Simulation")

if play:
    st.session_state.run=True

if "run" not in st.session_state:
    st.session_state.run=False

if st.session_state.run:

    st.session_state.sim_pos+=3

    if st.session_state.sim_pos>=track_len:
        st.session_state.run=False
        st.session_state.sim_pos=0

    st.rerun()

pos=st.session_state.sim_pos

idx=np.argmin(abs(dist-pos))

robot_x=dist[idx]
robot_y=elevation[idx]
robot_v=safe_v[idx]

# ---------------------------------------------------------
# BRAKE DISTANCE
# ---------------------------------------------------------

mu_now=friction[idx]

brake_d=braking_distance(robot_v,mu_now)

# ---------------------------------------------------------
# SLIP DETECTION
# ---------------------------------------------------------

slip=slip_detect(robot_v,mu_now)

# ---------------------------------------------------------
# 2D VISUALIZATION
# ---------------------------------------------------------

fig=go.Figure()

fig.add_trace(go.Scatter(
x=dist,
y=elevation,
fill="tozeroy",
name="Terrain"
))

fig.add_trace(go.Scatter(
x=dist,
y=safe_v,
name="Safe Speed"
))

# robot body

fig.add_trace(go.Scatter(
x=[robot_x],
y=[robot_y],
mode="markers",
marker=dict(size=14,color="red"),
name="Robot"
))

# brake distance line

fig.add_shape(
type="line",
x0=robot_x,
x1=robot_x+brake_d,
y0=robot_y,
y1=robot_y,
line=dict(color="yellow",dash="dash")
)

fig.update_layout(
template="plotly_dark",
height=500
)

st.plotly_chart(fig,use_container_width=True)

# ---------------------------------------------------------
# 3D TERRAIN
# ---------------------------------------------------------

st.subheader("3D Terrain")

X=np.tile(dist,(20,1))
Y=np.tile(np.linspace(-5,5,20),(len(dist),1)).T
Z=np.tile(elevation,(20,1))

fig3d=go.Figure(
data=[go.Surface(
x=X,
y=Y,
z=Z,
colorscale="Viridis"
)]
)

fig3d.update_layout(
template="plotly_dark",
height=600
)

st.plotly_chart(fig3d,use_container_width=True)

# ---------------------------------------------------------
# TELEMETRY
# ---------------------------------------------------------

c1,c2,c3,c4=st.columns(4)

c1.metric("Speed",f"{robot_v:.1f} m/s")
c2.metric("Brake distance",f"{brake_d:.1f} m")
c3.metric("Friction μ",f"{mu_now:.2f}")
c4.metric("Slip Risk","YES" if slip else "NO")

# ---------------------------------------------------------
# OBSTACLE VISUALIZATION
# ---------------------------------------------------------

if obstacles:

    st.warning(f"{len(obstacles)} obstacles detected on track")