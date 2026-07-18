"""AgentWrapper runtime patches (no modification of the vendored carla_garage code).

``leaderboard_autopilot``'s ``agent_wrapper_local.py`` hardcodes the LiDAR beam
parameters (``channels=64``, ``range=85``, ``upper_fov=10``, ``lower_fov=-30``,
dropoff/attenuation rates) and only reads ``rotation_frequency`` /
``points_per_second`` from the agent's sensor spec (and only when ``DATAGEN=1``).
To collect data with a rig that matches a real vehicle's LiDAR, this module
wraps ``AgentWrapper._preprocess_sensor_spec`` at runtime so that any of the
keys in :data:`OVERRIDABLE_LIDAR_KEYS` present in a ``sensor.lidar.ray_cast``
spec override the wrapper's attributes. Specs without these keys keep the
upstream hardcoded values bit-for-bit, so existing agents are unaffected.

Usage pattern (inside ``leaderboard_evaluator_local_ext.py``)::

    from agent_wrapper_patches import apply_agent_wrapper_patches

    # Before delegating to the original main():
    apply_agent_wrapper_patches()

The patch targets the class, so ``ROSAgentWrapper`` (a subclass) is covered
too. It only relies on ``_preprocess_sensor_spec`` returning
``(type_, id_, sensor_transform, attributes)``; it keeps working across
upstream updates as long as that shape is unchanged.
"""

# LiDAR spec keys that may override the wrapper's hardcoded blueprint
# attributes. GeneralizedDataAgent.tick() merges carla_fps/rotation_frequency
# partial sweeps per frame, so rotation_frequency must divide carla_fps (20).
OVERRIDABLE_LIDAR_KEYS = (
    'range',
    'channels',
    'upper_fov',
    'lower_fov',
    'rotation_frequency',
    'points_per_second',
    'atmosphere_attenuation_rate',
    'dropoff_general_rate',
    'dropoff_intensity_limit',
    'dropoff_zero_intensity',
)


def apply_agent_wrapper_patches():
    """Patch ``AgentWrapper._preprocess_sensor_spec`` to honor per-spec LiDAR keys.

    Must be called before ``setup_sensors`` runs (i.e. before the scenario
    starts); calling it before the leaderboard ``main()`` is sufficient.
    The function is idempotent: calling it multiple times is safe.
    """
    from leaderboard.autoagents.agent_wrapper_local import AgentWrapper  # noqa: PLC0415

    if getattr(AgentWrapper._preprocess_sensor_spec, '_ext_patched', False):
        return  # already patched

    _orig_preprocess = AgentWrapper._preprocess_sensor_spec

    def _patched_preprocess(self, sensor_spec):
        type_, id_, sensor_transform, attributes = _orig_preprocess(self, sensor_spec)
        if type_ == 'sensor.lidar.ray_cast':
            for key in OVERRIDABLE_LIDAR_KEYS:
                if key in sensor_spec:
                    attributes[key] = str(sensor_spec[key])
        return type_, id_, sensor_transform, attributes

    _patched_preprocess._ext_patched = True
    AgentWrapper._preprocess_sensor_spec = _patched_preprocess
