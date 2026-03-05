import plotly.graph_objects as go


def stopping_graph(stop_distance, margin):

    fig = go.Figure()

    fig.add_trace(

        go.Bar(
            y=["Path"],
            x=[stop_distance],
            orientation="h",
            name="Stopping Distance"
        )

    )

    if margin > 0:

        fig.add_trace(

            go.Bar(
                y=["Path"],
                x=[margin],
                orientation="h",
                name="Safety Margin"
            )

        )

    return fig