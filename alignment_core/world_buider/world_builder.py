from alignment_core.world_model.world_state import WorldState


class WorldBuilder:

    @staticmethod
    def build(agent, environment, action=None):

        return WorldState(
            agent=agent,
            environment=environment,
            action=action
        )