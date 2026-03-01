from alignment_core.world_model.world_state import WorldState
from alignment_core.world_model.environment import EnvironmentState
from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.primitives import Vector3, Quaternion, ActuatorLimits
from alignment_core.world_model.uncertainty import UncertaintyModel

def build_world_from_ai(ai_data):
    """
    Converts AI JSON perception into formal Physics objects.
    """
    # Create the Agent (The robot in the video)
    agent = AgentState(
        id="audited_agent",
        type="mobile",
        mass=20.0, # Standard robot mass
        position=Vector3(0, 0, 0),
        velocity=Vector3(ai_data.get('estimated_speed', 0.0), 0, 0),
        angular_velocity=Vector3(0, 0, 0),
        orientation=Quaternion(1, 0, 0, 0),
        center_of_mass=Vector3(0, 0, 0.5),
        support_polygon=[Vector3(-0.5,-0.5,0), Vector3(0.5,-0.5,0), 
                         Vector3(0.5,0.5,0), Vector3(-0.5,0.5,0)],
        actuator_limits=ActuatorLimits(100, 500, 5, 2),
        battery_state=1.0,
        current_load=None,
        contact_points=[]
    )

    # Setup Environment
    env = EnvironmentState(
        temperature=20.0, air_density=1.225, wind_vector=Vector3(0,0,0),
        terrain_type="detected",
        surface_friction=ai_data.get('friction_coeff', 0.5),
        slope_vector=Vector3(0, 0, ai_data.get('slope_z', 0.0)),
        lighting_conditions="normal"
    )

    return WorldState(
        timestamp=0.0, delta_time=0.01, gravity=Vector3(0, 0, -9.81),
        environment=env, agents=[agent], objects=[],
        uncertainty=UncertaintyModel(0.1, 0.1, 0.05, 0.01)
    )