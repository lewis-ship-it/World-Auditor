# alignment_core/engine/safety_engine.py
def evaluate(self, world_state):
    all_results = []
    for constraint in self.constraints:
        r = constraint.evaluate(world_state)
        if isinstance(r, list):
            all_results.extend(r) # Flatten the list
        else:
            all_results.append(r)
    return all_results