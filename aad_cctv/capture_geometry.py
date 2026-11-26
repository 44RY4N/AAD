"""Keep simulator reference geometry independent of the CCTV capture."""
from experiments.geometry_recording import GeometryRecorder


class IndependentGeometryRecorder(GeometryRecorder):
    def __init__(self, env, output, vehicles, settings, dt, camera=None):
        super().__init__(env, output, vehicles, settings, dt, camera=None)
