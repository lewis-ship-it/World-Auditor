def generate_explanation(report, parsed_data):
    if report.is_safe():
        return (
            "The proposed action appears physically feasible. "
            "Based on the provided parameters, the robot has sufficient stopping distance "
            "and no immediate instability risks were detected."
        )

    explanations = []

    for result in report.results:
        if result.violated:

            if result.name == "BrakingFeasibility":
                explanations.append(
                    "The robot cannot stop in time given its current speed and distance to the obstacle."
                )

            if result.name == "LoadConstraint":
                explanations.append(
                    "The load may exceed stability limits and increase tipping risk."
                )

            if result.name == "FrictionConstraint":
                explanations.append(
                    "Surface friction appears insufficient for safe maneuvering."
                )

    if not explanations:
        explanations.append("Potential physics risks detected based on the scenario.")

    return " ".join(explanations)