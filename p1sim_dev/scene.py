"""Scene geometry used only for developer visibility diagnostics."""
import math
import xml.etree.ElementTree as ET


class Scene:
    def __init__(self, path):
        world = ET.parse(path).getroot().find('world')
        camera = world.find("model[@name='camera']")
        self.camera_position = tuple(map(float, camera.findtext('pose').split()[:3]))
        self.fov = float(camera.findtext('link/sensor/camera/horizontal_fov'))
        self.targets = []
        for model in world.findall('model'):
            if model.get('name').startswith('target'):
                pose = tuple(map(float, model.findtext('pose').split()))
                size = tuple(map(float, model.findtext("link/visual[@name='plate']/geometry/box/size").split()))
                self.targets.append((model.get('name'), pose, size))

    def project(self, width, height, yaw, pitch):
        focal = width / (2 * math.tan(self.fov / 2))
        cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
        def pixel(point):
            x, y, z = (a-b for a, b in zip(point, self.camera_position))
            forward, left = cy*x+sy*y, -sy*x+cy*y
            depth, up = cp*forward-sp*z, sp*forward+cp*z
            return None if depth <= 0 else [width/2-focal*left/depth, height/2-focal*up/depth]
        result = []
        for name, pose, size in self.targets:
            x, y, z, roll, tilt, heading = pose
            # Authored profiles have upright, untilted plates; fail explicitly
            # rather than reporting false visibility for an unsupported pose.
            if roll != 0 or tilt != 0:
                raise ValueError('Visibility diagnostics require upright targets')
            points = [(x-math.sin(heading)*dy, y+math.cos(heading)*dy, z+dz)
                      for dy in (-size[1]/2, size[1]/2) for dz in (-size[2]/2, size[2]/2)]
            corners = [pixel(point) for point in points]
            result.append(dict(name=name, center_px=pixel((x, y, z)), corners_px=corners,
                fully_in_fov=all(p is not None and 0 <= p[0] < width and 0 <= p[1] < height for p in corners)))
        return result
