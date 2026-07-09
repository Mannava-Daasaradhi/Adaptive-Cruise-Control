"""Gazebo (gz-sim Harmonic) scene assets for the CACC platoon.

The cars are **kinematic** — they carry an inertial (so the physics system
accepts them) but ``<gravity>false</gravity>`` and no collisions, so nothing
falls or contacts. Their pose is written every tick by :mod:`gz_bridge_node`
via the world's ``set_pose`` service, from the *authoritative* CACC state
published by the vehicle nodes. Gazebo therefore renders the validated
longitudinal dynamics without ever altering them — the offline core, the ROS
nodes and this 3-D view all run the one control law (single source of truth).

Everything is emitted as a self-contained SDF string (no ``model://`` refs),
so no ``GZ_SIM_RESOURCE_PATH`` wiring is needed to spawn a car.
"""

from __future__ import annotations

#: name of the world these assets are spawned into (matches worlds/highway.sdf)
WORLD = "highway"

#: SDF spec version emitted for spawned models / the standalone car model
SDF_VERSION = "1.10"

#: car body geometry [m] — matches VehicleParams.length used by the sim
VEH_LEN = 4.0
VEH_WIDTH = 1.9
WHEEL_R = 0.34
LANE_HALF = 5.0  # [m] road half-width (visual only)

# viridis-ish follower gradient (dark green -> teal -> blue -> violet); the
# leader is white. Kept close to the plotstyle / viz_node palette so the
# browser sim, rviz and Gazebo all read as the same platoon.
_FOLLOWER_RGB = [
    (0.13, 0.57, 0.55),
    (0.18, 0.72, 0.51),
    (0.22, 0.60, 0.82),
    (0.35, 0.42, 0.88),
    (0.55, 0.35, 0.82),
    (0.72, 0.32, 0.68),
    (0.85, 0.36, 0.52),
    (0.90, 0.49, 0.30),
    (0.93, 0.62, 0.22),
]


def car_rgb(index: int) -> tuple[float, float, float]:
    """Body colour for vehicle ``index`` (0 = leader = white)."""
    if index <= 0:
        return (0.93, 0.93, 0.93)
    return _FOLLOWER_RGB[(index - 1) % len(_FOLLOWER_RGB)]


def car_sdf(name: str, rgb: tuple[float, float, float]) -> str:
    """Self-contained SDF ``<model>`` for one kinematic car.

    A box chassis + tapered cabin + four wheel cylinders + two emissive
    head-lights. Visual-only (no ``<collision>``) and ``gravity`` off, so the
    body holds whatever pose ``set_pose`` last wrote.
    """
    r, g, b = rgb
    L, W, H = VEH_LEN, VEH_WIDTH, 0.75
    body_z = WHEEL_R + 0.5 * H          # chassis centre height
    cabin_z = WHEEL_R + H + 0.28        # cabin centre height
    wx = 0.5 * L - 0.85                 # wheel x offset from centre
    wy = 0.5 * W - 0.03                 # wheel y offset
    glass = (0.10, 0.12, 0.16)

    def _mat(cr, cg, cb, spec=0.4, emis=0.0):
        return (f"<material>"
                f"<ambient>{cr} {cg} {cb} 1</ambient>"
                f"<diffuse>{cr} {cg} {cb} 1</diffuse>"
                f"<specular>{spec} {spec} {spec} 1</specular>"
                f"<emissive>{emis*cr} {emis*cg} {emis*cb} 1</emissive>"
                f"</material>")

    def _wheel(tag, x, y):
        return (f'<visual name="wheel_{tag}">'
                f'<pose>{x} {y} {WHEEL_R} 1.5708 0 0</pose>'
                f'<geometry><cylinder><radius>{WHEEL_R}</radius>'
                f'<length>0.22</length></cylinder></geometry>'
                f'{_mat(0.05, 0.05, 0.06, spec=0.2)}</visual>')

    def _light(tag, y):
        return (f'<visual name="head_{tag}">'
                f'<pose>{0.5*L-0.05} {y} {body_z-0.1} 0 1.5708 0</pose>'
                f'<geometry><cylinder><radius>0.09</radius>'
                f'<length>0.06</length></cylinder></geometry>'
                f'{_mat(1.0, 0.95, 0.7, spec=0.9, emis=0.8)}</visual>')

    return (
        f'<model name="{name}">'
        f'<static>false</static><self_collide>false</self_collide>'
        f'<pose>0 0 0 0 0 0</pose>'
        f'<link name="body">'
        f'<gravity>false</gravity>'
        f'<inertial><mass>1200</mass><inertia>'
        f'<ixx>500</ixx><iyy>900</iyy><izz>1100</izz>'
        f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
        # chassis
        f'<visual name="chassis"><pose>0 0 {body_z} 0 0 0</pose>'
        f'<geometry><box><size>{L} {W} {H}</size></box></geometry>'
        f'{_mat(r, g, b)}</visual>'
        # cabin / greenhouse (shorter, set back)
        f'<visual name="cabin"><pose>-0.15 0 {cabin_z} 0 0 0</pose>'
        f'<geometry><box><size>{0.55*L} {W-0.28} 0.55</size></box></geometry>'
        f'{_mat(*glass, spec=0.6)}</visual>'
        + _wheel("fl", wx, wy) + _wheel("fr", wx, -wy)
        + _wheel("rl", -wx, wy) + _wheel("rr", -wx, -wy)
        + _light("l", wy - 0.25) + _light("r", -(wy - 0.25))
        + f'</link></model>'
    )


def car_sdf_doc(name: str, rgb: tuple[float, float, float]) -> str:
    """Full SDF *document* for one car — what ``ros_gz_sim create -string``
    and the ``/world/.../create`` service require (a bare ``<model>`` is
    rejected as "not an SDFormat string")."""
    return f'<sdf version="{SDF_VERSION}">{car_sdf(name, rgb)}</sdf>'
