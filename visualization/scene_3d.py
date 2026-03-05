import plotly.graph_objects as go

def create_robot_scene(obstacle_distance, stopping_distance):

    fig = go.Figure()

    # Robot
    fig.add_trace(go.Scatter3d(
        x=[0],
        y=[0],
        z=[0],
        mode="markers",
        marker=dict(size=10),
        name="Robot"
    ))

    # Obstacle
    fig.add_trace(go.Scatter3d(
        x=[obstacle_distance],
        y=[0],
        z=[0],
        mode="markers",
        marker=dict(size=10),
        name="Obstacle"
    ))

    # Stopping distance line
    fig.add_trace(go.Scatter3d(
        x=[0, stopping_distance],
        y=[0,0],
        z=[0,0],
        mode="lines",
        name="Stopping Path"
    ))

    fig.update_layout(
        title="Robot Braking Simulation",
        scene=dict(
            xaxis_title="Distance",
            yaxis_title="Width",
            zaxis_title="Height"
        )
    )

    return fig