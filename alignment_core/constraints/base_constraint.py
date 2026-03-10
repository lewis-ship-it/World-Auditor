class BaseConstraint:

    name = "BaseConstraint"

    def evaluate(self, world_state):
        raise NotImplementedError