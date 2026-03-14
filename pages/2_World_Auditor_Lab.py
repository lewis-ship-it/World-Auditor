import streamlit as st
import numpy as np
import plotly.graph_objects as go
import pandas as pd
import heapq
import time

st.set_page_config(
    page_title="World Auditor Simulator",
    layout="wide",
    initial_sidebar_state="expanded"
)

st.title("🤖 World Auditor Robot Simulator")

# -------------------------------------------------
# SESSION STATE
# -------------------------------------------------

if "robot_cfg" not in st.session_state:
    st.session_state.robot_cfg = None

if "telemetry" not in st.session_state:
    st.session_state.telemetry = []

# -------------------------------------------------
# SIDEBAR
# -------------------------------------------------

with st.sidebar:

    st.header("Robot Configuration")

    mass = st.slider("Mass (kg)",10,200,50)
    wheelbase = st.slider("Wheelbase (m)",0.2,2.0,0.5)
    traction = st.slider("Traction Coefficient",0.1,1.5,0.8)
    motor_force = st.slider("Motor Force (N)",50,1000,200)

    if st.button("Save Robot"):

        st.session_state.robot_cfg = {
            "mass":mass,
            "wheelbase":wheelbase,
            "traction":traction,
            "motor_force":motor_force
        }

        st.success("Robot saved!")

# -------------------------------------------------
# TABS
# -------------------------------------------------

tab1,tab2,tab3,tab4 = st.tabs([
    "Robot Builder",
    "Navigation Simulator",
    "Telemetry",
    "Terrain Analysis"
])

# -------------------------------------------------
# ROBOT BUILDER TAB
# -------------------------------------------------

with tab1:

    st.header("Robot Builder")

    if st.session_state.robot_cfg:

        st.success("Robot Loaded")

        st.json(st.session_state.robot_cfg)

    else:

        st.info("Configure robot in sidebar")

# -------------------------------------------------
# TERRAIN
# -------------------------------------------------

GRID=80
g=9.81
dt=0.05

x=np.linspace(-3,3,GRID)
y=np.linspace(-3,3,GRID)
X,Y=np.meshgrid(x,y)

elevation=np.sin(X)*np.cos(Y)*2
elevation+=np.exp(-(X**2+Y**2))*4

friction=np.ones((GRID,GRID))*0.9
friction[20:40,10:30]=0.3
friction[50:70,50:70]=0.1

obstacles=np.zeros((GRID,GRID))
obstacles[30:35,:]=1
obstacles[:,40:42]=1

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
# NAVIGATION TAB
# -------------------------------------------------

with tab2:

    st.header("Navigation Simulator")

    if not st.session_state.robot_cfg:

        st.warning("Build a robot first.")
        st.stop()

    robot=st.session_state.robot_cfg

    start_x=st.slider("Start X",0,GRID-1,5)
    start_y=st.slider("Start Y",0,GRID-1,5)

    goal_x=st.slider("Goal X",0,GRID-1,70)
    goal_y=st.slider("Goal Y",0,GRID-1,70)

    run=st.button("Run Simulation")

    start=(start_x,start_y)
    goal=(goal_x,goal_y)

    path=astar(obstacles,start,goal)

    telemetry=[]

    if run and path:

        robot_pos=np.array(path[0],dtype=float)

        speed=0
        time_elapsed=0

        for target in path[1:]:

            target=np.array(target)

            while True:

                direction=target-robot_pos
                dist=np.linalg.norm(direction)

                if dist<0.1:
                    break

                direction/=dist

                mu=friction[int(robot_pos[0]),int(robot_pos[1])]

                max_speed=np.sqrt(mu*g*robot["wheelbase"])

                accel=robot["motor_force"]/robot["mass"]

                speed+=accel*dt
                speed=min(speed,max_speed)

                robot_pos+=direction*speed*dt

                curvature=1/max(dist,0.01)

                brake=speed**2/(2*mu*g)

                slip=speed>max_speed

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

    df=pd.DataFrame(
        telemetry,
        columns=[
            "time","x","y","speed",
            "accel","friction",
            "curvature",
            "brake_distance",
            "slip"
        ]
    )

    st.session_state.telemetry=df

    # 3D Map

    fig=go.Figure()

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
            line=dict(color="yellow",width=6)
        ))

    if not df.empty:

        fig.add_trace(go.Scatter3d(
            x=df["x"],
            y=df["y"],
            z=elevation[
                df["x"].astype(int),
                df["y"].astype(int)
            ]+1,
            mode="markers",
            marker=dict(size=4,color="red")
        ))

    fig.update_layout(
        template="plotly_dark",
        height=700
    )

    st.plotly_chart(fig,use_container_width=True)

# -------------------------------------------------
# TELEMETRY TAB
# -------------------------------------------------

with tab3:

    st.header("Telemetry")

    df=st.session_state.telemetry

    if df is None or df.empty:

        st.info("Run simulation to generate telemetry")

    else:

        col1,col2,col3,col4=st.columns(4)

        col1.metric("Total Time",round(df.time.iloc[-1],2))
        col2.metric("Max Speed",round(df.speed.max(),2))
        col3.metric("Max Brake Distance",round(df.brake_distance.max(),2))
        col4.metric("Slip Events",df.slip.sum())

        st.dataframe(df)

# -------------------------------------------------
# TERRAIN TAB
# -------------------------------------------------

with tab4:

    st.header("Terrain Maps")

    st.subheader("Elevation")

    st.plotly_chart(
        go.Figure(go.Surface(z=elevation)),
        use_container_width=True
    )

    st.subheader("Friction")

    st.imshow(friction)