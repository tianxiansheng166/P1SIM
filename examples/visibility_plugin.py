"""External reference sensor plugin: periodic blackout and seeded frame drop."""
from p1sim_dev.builtins import Sensor, fields
from p1sim_dev.config import number


class PeriodicVisibility(Sensor):
    version = '1.0.0'

    def configure(self, config):
        fields(config, ('period_seconds', 'occlusion_seconds', 'drop_probability', 'calibration_scale'))
        self.period = number(config.get('period_seconds', 2), 'period_seconds', .001)
        self.occlusion = number(config.get('occlusion_seconds', .1), 'occlusion_seconds', 0)
        self.drop = number(config.get('drop_probability', 0), 'drop_probability', 0)
        self.scale = number(config.get('calibration_scale', 1), 'calibration_scale', .001)
        if self.drop > 1 or self.occlusion > self.period:
            raise ValueError('Invalid visibility fraction')

    def process(self, source_seconds, image):
        occluded = source_seconds % self.period < self.occlusion
        if occluded:
            image.data = bytes(len(image.data))
        return dict(image=image, dropped=self.random.random() < self.drop, occluded=occluded, calibration_scale=self.scale)
