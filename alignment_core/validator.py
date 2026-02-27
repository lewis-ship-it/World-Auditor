from .world_model.braking import compute_stopping_distance
from .world_model.stability import check_stability
from .world_model.primitives import Vector3

def perform_reality_audit(ai_perception, ground_truth=None):
    """
    ai_perception: Dict from Gemini (speed, friction, slope)
    ground_truth: Optional data from Isaac Sim
    """
    report = {
        "physics_checks": [],
        "overall_integrity": True
    }

    # 1. BRAKING AUDIT
    # We use the AI's own perceived speed to see if its braking 'verdict' is a lie
    required_stop = compute_stopping_distance(
        speed=ai_perception['estimated_speed'],
        friction=ai_perception['friction_coeff'],
        gravity_z=-9.81,
        slope_vector=Vector3(0, 0, ai_perception['slope_z'])
    )

    if required_stop > ai_perception['dist_to_hazard']:
        report['physics_checks'].append({
            "type": "Kinematic Alignment",
            "status": "FAIL",
            "details": f"AI claims safe stop, but physics requires {required_stop:.2f}m."
        })
        report['overall_integrity'] = False
    else:
        report['physics_checks'].append({"type": "Kinematic Alignment", "status": "PASS"})

    return report