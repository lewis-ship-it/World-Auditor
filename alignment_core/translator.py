from alignment_core.world_model.world_state import WorldState
from alignment_core.world_model.environment import EnvironmentState
from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.primitives import Vector3, Quaternion, ActuatorLimits
from alignment_core.world_model.uncertainty import UncertaintyModel

def translate_perception_to_world(ai_json):
    """
    Standardizes Gemini's vision output into your 
    WorldState architecture for physical auditing.
    """
    # 1. Create the Environment from AI guesses
    env = EnvironmentState(
        temperature=20.0,
        air_density=1.225,
        wind_vector=Vector3(0, 0, 0),
        terrain_type="detected",
        surface_friction=ai_json.get('friction_coeff', 0.5),
        slope_vector=Vector3(0, 0, ai_json.get('slope_z', 0)),
        lighting_conditions="normal"
    )

    # 2. Create the Agent
    agent = AgentState(
        id="audited_robot",
        type="mobile",
        mass=20.0, # Assumed standard mass
        position=Vector3(0,0,0),
        velocity=Vector3(ai_json.get('estimated_speed', 0), 0, 0),
        angular_velocity=Vector3(0, 0, 0),
        orientation=Quaternion(1, 0, 0, 0), 
        center_of_mass=Vector3(0, 0, 0.5),
        support_polygon=[Vector3(-0.5,-0.5,0), Vector3(0.5,-0.5,0), 
                         Vector3(0.5,0.5,0), Vector3(-0.5,0.5,0)],
        actuator_limits=ActuatorLimits(100, 100, 5, 2),
        battery_state=1.0,
        current_load=None,
        contact_points=[]
    )

    return WorldState(
        timestamp=0.0,
        delta_time=0.01,
        gravity=Vector3(0, 0, -9.81),
        environment=env,
        agents=[agent],
        objects=[],
        uncertainty=UncertaintyModel(0.1, 0.1, 0.05, 0.01)
    )