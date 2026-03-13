import streamlit as st
import numpy as np
import plotly.graph_objects as go
import heapq
import pandas as pd
import time

st.set_page_config(layout="wide", page_title="World Auditor Lab")

st.title("🌍 World Auditor Robot Simulation")

# -------------------------------------------------------
# LOAD ROBOT FROM BUILDER PAGE
# -------------------------------------------------------

robot_cfg = st.session_state.get("robot", None)

if robot_cfg is None:
    st.error("⚠️ No robot configured. Please build one in Robot Builder.")
    st.stop()

mass = robot_cfg["mass"]
accel = robot_cfg["accel"]
max_speed = robot_cfg["max_speed"]
track_width = robot_cfg["track_width"]

# -------------------------------------------------------
# CONSTANTS
# -------------------------------------------------------

GRID = 80
dt = 0.1
g = 9.81

# -------------------------------------------------------
# PREMADE MAPS
# -------------------------------------------------------

def flat_world():

    elevation = np.zeros((GRID,GRID))
    friction = np.ones((GRID,GRID))
    obstacles = np.zeros((GRID,GRID))

    return elevation, friction, obstacles


def hill_world():

    elevation = np.zeros((GRID,GRID))
    friction = np.ones((GRID,GRID))
    obstacles = np.zeros((GRID,GRID))

    for x in range(GRID):
        for y in range(GRID):
            elevation[x,y] = np.sin(x/8)*2

    return elevation, friction, obstacles


def obstacle_maze():

    elevation = np.zeros((GRID,GRID))
    friction = np.ones((GRID,GRID))
    obstacles = np.zeros((GRID,GRID))

    obstacles[20:60,40] = 1
    obstacles[40,10:60] = 1
    obstacles[10:50,25] = 1

    return elevation, friction, obstacles


def slippery_ice():

    elevation = np.zeros((GRID,GRID))
    friction = np.ones((GRID,GRID))*0.2
    obstacles = np.zeros((GRID,GRID))

    return elevation, friction, obstacles


# -------------------------------------------------------
# SIDEBAR
# -------------------------------------------------------

with st.sidebar:

    st.header("Simulation Map")

    map_choice = st.selectbox(
        "Select Test Map",
        ["Flat Test Track","Hill Climb","Obstacle Maze","Ice Field"]
    )

    start = (2,2)
    goal = (GRID-5, GRID-5)

    run = st.button("Run Simulation")

# -------------------------------------------------------
# MAP SELECTION
# -------------------------------------------------------

if map_choice == "Flat Test Track":
    elevation, friction, obstacles = flat_world()

elif map_choice == "Hill Climb":
    elevation, friction, obstacles = hill_world()

elif map_choice == "Obstacle Maze":
    elevation, friction, obstacles = obstacle_maze()

else:
    elevation, friction, obstacles = slippery_ice()

# -------------------------------------------------------
# A* PATH PLANNER
# -------------------------------------------------------

def astar(grid, start, goal):

    def h(a,b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    open_set = []
    heapq.heappush(open_set,(0,start))

    came = {}
    cost = {start:0}

    while open_set:

        _,cur = heapq.heappop(open_set)

        if cur == goal:

            path=[cur]

            while cur in came:
                cur = came[cur]
                path.append(cur)

            return path[::-1]

        x,y = cur

        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:

            nx,ny = x+dx,y+dy

            if nx<0 or ny<0 or nx>=GRID or ny>=GRID:
                continue

            if grid[nx,ny] == 1:
                continue

            new_cost = cost[cur] + 1

            if (nx,ny) not in cost or new_cost < cost[(nx,ny)]:

                cost[(nx,ny)] = new_cost
                priority = new_cost + h((nx,ny),goal)

                heapq.heappush(open_set,(priority,(nx,ny)))
                came[(nx,ny)] = cur

    return []

path = astar(obstacles,start,goal)

# -------------------------------------------------------
# ROBOT SIMULATION
# -------------------------------------------------------

robot_pos = np.array(start,dtype=float)
speed = 0

positions = []
speeds = []
times = []

total_time = 0
distance = 0

# -------------------------------------------------------
# LAYOUT
# -------------------------------------------------------

map_col, data_col = st.columns([3,1])

placeholder = map_col.empty()

# -------------------------------------------------------
# RUN SIMULATION
# -------------------------------------------------------

if run and path:

    for i in range(len(path)):

        target = np.array(path[i])

        direction = target - robot_pos
        dist = np.linalg.norm(direction)

        if dist > 0:
            direction /= dist

        speed = min(speed + accel*dt, max_speed)

        move = speed*dt

        robot_pos += direction * move

        rx,ry = int(robot_pos[0]), int(robot_pos[1])
        mu = friction[rx,ry]

        brake = speed**2 / (2*mu*g)

        slip = speed > np.sqrt(mu*g*5)

        distance += move
        total_time += dt

        positions.append(robot_pos.copy())
        speeds.append(speed)
        times.append(total_time)

        # ------------------------------------------------
        # PLOT TERRAIN
        # ------------------------------------------------

        fig = go.Figure()

        fig.add_trace(go.Surface(
            z=elevation,
            colorscale="Viridis",
            showscale=False
        ))

        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        zs = [elevation[int(x),int(y)]+0.5 for x,y in zip(xs,ys)]

        fig.add_trace(go.Scatter3d(
            x=xs,
            y=ys,
            z=zs,
            mode="lines",
            line=dict(color="yellow",width=6),
            name="Planned Path"
        ))

        fig.add_trace(go.Scatter3d(
            x=[robot_pos[0]],
            y=[robot_pos[1]],
            z=[elevation[rx,ry]+1],
            mode="markers",
            marker=dict(size=8,color="red"),
            name="Robot"
        ))

        fig.update_layout(
            template="plotly_dark",
            height=650,
            scene=dict(
                xaxis_title="X Position",
                yaxis_title="Y Position",
                zaxis_title="Elevation"
            )
        )

        placeholder.plotly_chart(fig,use_container_width=True)

        time.sleep(0.03)

# -------------------------------------------------------
# TELEMETRY
# -------------------------------------------------------

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
            round(total_time,2),
            round(distance,2),
            max(speeds),
            mass,
            track_width
        ]
    })

    data_col.subheader("Telemetry")
    data_col.table(telemetry)

    # --------------------------------------------
    # SPEED GRAPH
    # --------------------------------------------

    speed_fig = go.Figure()

    speed_fig.add_trace(go.Scatter(
        x=times,
        y=speeds,
        mode="lines",
        name="Speed"
    ))

    speed_fig.update_layout(
        template="plotly_dark",
        title="Robot Speed Over Time",
        xaxis_title="Time (s)",
        yaxis_title="Speed (m/s)"
    )

    st.plotly_chart(speed_fig,use_container_width=True)