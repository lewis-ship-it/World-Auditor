class ConstraintRegistry:
    def __init__(self):
        self._constraints = []

    def register(self, constraint):
        self._constraints.append(constraint)

    def get_all(self):
        return self._constraints

    def clear(self):
        self._constraints = []