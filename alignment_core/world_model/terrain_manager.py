class TerrainManager:
    def __init__(self, default_surface="dry_asphalt"):
        self.default_surface = default_surface

    def get_surface(self, position=None):
        return self.default_surface