import re


def extract_physics_parameters(text):
    velocity_match = re.search(r"(\d+)\s*m/s", text)
    distance_match = re.search(r"(\d+)\s*m", text)
    weight_match = re.search(r"(\d+)\s*kg", text)

    data = {}

    if velocity_match:
        data["velocity"] = float(velocity_match.group(1))

    if distance_match:
        data["distance"] = float(distance_match.group(1))

    if weight_match:
        data["weight"] = float(weight_match.group(1))

    data["mentions_slope"] = "slope" in text.lower()
    data["mentions_corner"] = "corner" in text.lower()
    data["mentions_fast"] = "fast" in text.lower()

    return data