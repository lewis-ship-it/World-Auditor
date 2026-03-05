def tipping_risk(speed, com_height, wheelbase):

    lateral_acc = speed * 0.2

    risk = (lateral_acc * com_height) / wheelbase

    if risk > 1:
        return "HIGH"

    if risk > 0.6:
        return "MEDIUM"

    return "LOW"