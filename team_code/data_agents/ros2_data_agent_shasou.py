"""
Base class for ROS2 data agents that pair each route with one shasou-recorder
drive (= one nuScenes log). The recorder is a separate ROS2 node that opens and
closes the bag on service calls; this agent is the client:

    setup()      -> create clients for the fixed service names
                    /shasou/recorder/start_recording (shasou_msgs/srv/StartRecording)
                    /shasou/recorder/stop_recording  (shasou_msgs/srv/StopRecording)
    run_step()   -> after the first parent run_step (which runs _init and thus
                    shuffle_weather), call StartRecording once per route
    destroy()    -> call StopRecording and wait for the response (bag closed,
                    manifest written) before the node is destroyed

Because the recorder only records while a drive is open, the silent gap between
routes never enters a bag, and the route <-> drive mapping is exact.

This class only adds the recorder coupling; it defines no sensor rig
(subclasses provide _sensors(), e.g. a future ROS2DataAgentShasouNuScenes).
The StartRecording payload (location / route_id / scenario / weather) is filled
from the CARLA world state; weather is snapped to the nearest Bench2Drive preset
(see _classify_weather and _BENCH2DRIVE_WEATHER_PRESETS at the end of the file).

Requires the shasou_msgs package importable from the agent's interpreter
(a built shasou-msgs colcon workspace sourced on top of ROS2 Humble). When it
is missing, RECORDER_REQUIRED below decides between aborting and driving on
without recording.
"""
import rclpy

from generalized_ros2_data_agent import GeneralizedROS2DataAgent


def get_entry_point():
    # This module only provides the abstract base class; point the leaderboard at a
    # subclass agent file instead.
    raise NotImplementedError('ROS2DataAgentShasou is abstract; use a subclass agent file.')


class ROS2DataAgentShasou(GeneralizedROS2DataAgent):
    """
    Publish-only agent coupled to shasou-recorder: one StartRecording /
    StopRecording pair per route. Subclasses still define _sensors() and
    TOPIC_NAMESPACE like any GeneralizedROS2DataAgent child.
    """

    # Service names are fixed by the shasou-msgs README convention.
    START_RECORDING_SERVICE = '/shasou/recorder/start_recording'
    STOP_RECORDING_SERVICE = '/shasou/recorder/stop_recording'
    # True: fail setup() when the recorder (or shasou_msgs) is unavailable.
    # Recording is this class's whole purpose, and driving on silently would
    # waste a full unrecorded route; for recorder-less test drives either set
    # this to False or use the shasou-independent ROS2DataAgentNuScenes.
    RECORDER_REQUIRED = True
    # How long setup() waits for the recorder services to appear.
    RECORDER_WAIT_TIMEOUT_SEC = 10.0
    # How long a service call waits for its response. StopRecording flushes the
    # bag and writes the manifest before responding, so this is generous.
    RECORDER_RESPONSE_TIMEOUT_SEC = 30.0

    def setup(self, path_to_conf_file, route_index=None, traffic_manager=None):
        super().setup(path_to_conf_file, route_index, traffic_manager=traffic_manager)

        # One StartRecording per route: setup() runs again for every route (and
        # retry), recreating _ros_node and with it the clients below.
        self._recording_started = False
        self._recorder_enabled = False
        self._start_client = None
        self._stop_client = None

        # Lazy import like the message types in the base setup(); a missing
        # package is the same situation as a missing service.
        try:
            from shasou_msgs.srv import StartRecording, StopRecording
        except ImportError as error:
            self._recorder_unavailable(f'shasou_msgs is not importable ({error})')
            return
        self._srv_start = StartRecording
        self._srv_stop = StopRecording

        self._start_client = self._ros_node.create_client(StartRecording, self.START_RECORDING_SERVICE)
        self._stop_client = self._ros_node.create_client(StopRecording, self.STOP_RECORDING_SERVICE)
        # Wait here, not on the first run_step: with RECORDER_REQUIRED the run
        # must abort before the route is driven, or a whole unrecorded route
        # is wasted. Both services come from the one recorder node, so waiting
        # on the start service covers the stop service too.
        if not self._start_client.wait_for_service(timeout_sec=self.RECORDER_WAIT_TIMEOUT_SEC):
            self._recorder_unavailable(f'service {self.START_RECORDING_SERVICE} did not appear '
                                       f'within {self.RECORDER_WAIT_TIMEOUT_SEC}s')
            return
        self._recorder_enabled = True

    def _recorder_unavailable(self, reason):
        # Single branch point for every "cannot record" condition.
        if self.RECORDER_REQUIRED:
            raise RuntimeError(f'[ROS2DataAgentShasou] shasou-recorder unavailable: {reason}. '
                               'Start the recorder, or set RECORDER_REQUIRED = False to drive '
                               'without recording.')
        print(f'[ROS2DataAgentShasou] shasou-recorder unavailable: {reason}. '
              'Driving without recording (RECORDER_REQUIRED = False).')
        self._recorder_enabled = False

    # ── Recording lifecycle ──────────────────────────────────────────────────

    def run_step(self, input_data, timestamp, sensors=None, plant=False):
        control = super().run_step(input_data, timestamp, sensors=sensors, plant=plant)
        # StartRecording only after the first parent run_step: the weather sent
        # in the request is fixed by shuffle_weather() inside _init(), which the
        # parent triggers on its first call, and self._world is only live from
        # then on. The first tick's publishes therefore predate the bag; that is
        # accepted.
        if self._recorder_enabled and not self._recording_started:
            self._start_recording()
        return control

    def _start_recording(self):
        request = self._srv_start.Request()
        # Only what the recorder cannot know itself (platform / vehicle / calib
        # come from its own config). Every field is best-effort: a missing CARLA
        # attribute degrades to '' rather than aborting, which the recorder
        # contract accepts.
        request.source = 'carla'
        request.location = self._map_location()
        request.route_id = str(self.route_index) if self.route_index is not None else ''
        request.scenario = getattr(self, 'scenario_name', '') or ''
        request.weather = self._current_weather_label()

        response = self._call_service(self._start_client, request)
        if response is None or not response.success:
            detail = (f'no response within {self.RECORDER_RESPONSE_TIMEOUT_SEC}s'
                      if response is None else response.message)
            if self.RECORDER_REQUIRED:
                raise RuntimeError(f'[ROS2DataAgentShasou] StartRecording failed: {detail}')
            print(f'[ROS2DataAgentShasou] StartRecording failed: {detail}. '
                  'Driving without recording (RECORDER_REQUIRED = False).')
            self._recorder_enabled = False
            return
        self._recording_started = True
        print(f'[ROS2DataAgentShasou] recording started (drive_id={response.drive_id}, '
              f"location='{request.location}', route_id='{request.route_id}', "
              f"scenario='{request.scenario}', weather='{request.weather}')")

    def _map_location(self):
        # self._world.get_map().name is a path like "Carla/Maps/Town12"; the
        # recorder wants only the leaf ("Town12") for manifest.location /
        # nuScenes log.location. Best-effort: any failure -> ''.
        try:
            map_name = self._world.get_map().name
        except Exception as error:  # noqa: BLE001 — a missing map must not abort the run
            print(f'[ROS2DataAgentShasou] could not read the map name ({error}); location left empty.')
            return ''
        return map_name.rsplit('/', 1)[-1] if map_name else ''

    def _current_weather_label(self):
        # shuffle_weather() sets the weather once in _init() and it stays fixed
        # for the whole route (the Bench2Drive presets we match against use the
        # same values at route_percentage 0 and 100), so classifying once at
        # recording start labels the entire drive. A future setup that ramps the
        # weather via route_percentage would break this assumption.
        try:
            weather = self._world.get_weather()
        except Exception as error:  # noqa: BLE001 — missing weather must not abort the run
            print(f'[ROS2DataAgentShasou] could not read the weather ({error}); weather left empty.')
            return ''
        params = {name: getattr(weather, name, None) for name in _WEATHER_PARAM_RANGES}
        return self._classify_weather(params)

    def _call_service(self, client, request):
        """
        Call a recorder service and wait for the response; returns None on timeout.

        call_async + spin_until_future_complete is safe here: _ros_node is
        publish-only (no subscriptions or timers), so spinning it services
        nothing but this client's response, and the leaderboard loop never spins
        ROS itself. The synchronous client.call() would instead deadlock, as it
        expects another thread to spin the node.
        """
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._ros_node, future,
                                         timeout_sec=self.RECORDER_RESPONSE_TIMEOUT_SEC)
        if not future.done():
            future.cancel()
            return None
        return future.result()

    def destroy(self, results=None):
        # StopRecording must go out (and its response arrive) while _ros_node is
        # still alive; super().destroy() destroys the node. Skipped entirely
        # when StartRecording never succeeded. getattr: destroy() also runs when
        # setup() failed before the flag existed.
        if getattr(self, '_recording_started', False):
            try:
                request = self._srv_stop.Request()
                request.completed, request.reason = self._route_outcome(results)
                response = self._call_service(self._stop_client, request)
                if response is None:
                    print(f'[ROS2DataAgentShasou] StopRecording: no response within '
                          f'{self.RECORDER_RESPONSE_TIMEOUT_SEC}s; the drive may be left open.')
                elif not response.success:
                    print(f'[ROS2DataAgentShasou] StopRecording failed: {response.message}')
                else:
                    print(f'[ROS2DataAgentShasou] recording stopped (drive_id={response.drive_id}, '
                          f'completed={request.completed}, {response.message_count} messages, '
                          f'{response.duration_sec:.1f}s)')
            except Exception as error:  # noqa: BLE001 — never mask the evaluator teardown
                print(f'[ROS2DataAgentShasou] StopRecording failed: {error}')
            self._recording_started = False
        super().destroy(results)

    def _route_outcome(self, results):
        """
        Map the leaderboard result passed to destroy() to (completed, reason).

        The data-collection evaluator (leaderboard_evaluator_local.py) passes a
        RouteRecord whose .status is 'Perfect' / 'Completed' on a route that
        reached its target, and 'Failed - <reason>' otherwise. Some evaluator
        variants (and crashes before statistics are computed) instead call
        destroy() with no result; then completion cannot be judged and the drive
        is reported as a nominal completion (completed=True), which studio-side
        filters treat as "keep".
        """
        status = getattr(results, 'status', None)
        if status is None:
            return True, ''
        completed = status in ('Completed', 'Perfect')
        return completed, ('' if completed else status)

    # ── Weather classification ───────────────────────────────────────────────

    def _classify_weather(self, params):
        """
        Snap the current CARLA weather to the nearest Bench2Drive preset name.

        `params` is a dict of CARLA WeatherParameters attribute name -> value
        (cloudiness, precipitation, sun_altitude_angle, ...). Each parameter is
        normalized onto [0, 1] with _WEATHER_PARAM_RANGES so the differently
        scaled axes (0-100 vs the -90..90 sun altitude) contribute comparably,
        then a weighted squared-Euclidean distance to every preset in
        _BENCH2DRIVE_WEATHER_PRESETS is minimized. sun_azimuth_angle is not used
        (it is -1.0 for every preset but 26, so it carries no signal). A missing
        / None parameter is skipped for that axis. Returns the preset name (e.g.
        'ClearNoon'), which becomes manifest.weather and feeds nuScenes
        scene.description; '' only if there are no presets at all.

        The label is exact for CARLA's built-in presets that already match a
        Bench2Drive row and nearest-neighbour otherwise (shuffle_weather picks
        CARLA presets, whose values need not equal the Bench2Drive ones).
        """
        best_name = ''
        best_distance = None
        for preset in _BENCH2DRIVE_WEATHER_PRESETS:
            name = preset[1]
            preset_values = dict(zip(_WEATHER_PARAM_NAMES, preset[2:]))
            distance = 0.0
            for param_name, (low, high) in _WEATHER_PARAM_RANGES.items():
                value = params.get(param_name)
                if value is None:
                    continue
                span = high - low
                norm_value = (value - low) / span
                norm_preset = (preset_values[param_name] - low) / span
                weight = _WEATHER_PARAM_WEIGHTS.get(param_name, 1.0)
                distance += weight * (norm_value - norm_preset) ** 2
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_name = name
        return best_name


# ── Bench2Drive weather presets (task 2) ─────────────────────────────────────
# Kept together here so the definitions are easy to swap. Normalization range
# (min, max) per classification parameter, so the Euclidean distance runs on
# comparable [0, 1] scales.
_WEATHER_PARAM_RANGES = {
    'cloudiness': (0.0, 100.0),
    'precipitation': (0.0, 100.0),
    'precipitation_deposits': (0.0, 100.0),
    'wetness': (0.0, 100.0),
    'wind_intensity': (0.0, 100.0),
    'sun_altitude_angle': (-90.0, 90.0),
    'fog_density': (0.0, 100.0),
}
# Column order of the preset tuples below matches the range keys above.
_WEATHER_PARAM_NAMES = tuple(_WEATHER_PARAM_RANGES)
# Extra weight on sun_altitude_angle: time of day (noon / day / sunset / night)
# is the visually dominant axis and the one nuScenes scene.description keys on,
# so it should pull classification harder than the rest (each weight 1.0). Any
# positive weight keeps the self-consistency test valid (an exact preset match
# is distance 0 regardless of weights).
_WEATHER_PARAM_WEIGHTS = {'sun_altitude_angle': 2.0}

# Bench2Drive weather.xml presets: 23 rows (weather_id 4, 16, 17, 24 are absent).
# Each row is (weather_id, name, cloudiness, precipitation, precipitation_deposits,
# wetness, wind_intensity, sun_altitude_angle, fog_density). weather_id is kept
# for future id-based lookups. sun_azimuth_angle is omitted: it is -1.0 for every
# preset except id 26 and is excluded from the distance either way.
# Naming rule: precipitation 30=Soft / 50-60=Mid / 100=Hard; fog_density>=50
# =Foggy, 100=DenseFog; high wetness=Wet / VeryWet; sun_altitude 70-90=Noon /
# 45=Day / 0-15=Sunset / -90=Night.
_BENCH2DRIVE_WEATHER_PRESETS = (
    (0,  'ClearNoon',                5,   0,   0,   0,   10,  90,   2),
    (1,  'ClearSunset',              5,   0,   0,   0,   10,  15,   2),
    (2,  'SoftRainDay',             20,  30,  50,   0,   30,  45,   3),
    (3,  'MidRainDay',              60,  60,  60,   0,   60,  45,   3),
    (5,  'WetCloudyDay',            80,   0,  50,  20,   10,  45,   3),
    (6,  'ClearFoggySunset',         5,   0,  50,   0,   10,  15,  10),
    (7,  'ClearDay',                 5,   0,   0,   0,   10,  45,   2),
    (8,  'HardRainDay',            100, 100,  90,   0,  100,  45,   7),
    (9,  'MidRainFoggyDay',         70,  60,  60,   0,   60,  45,  50),
    (10, 'HardRainSunset',          40, 100,  90,   0,  100,   0,   7),
    (11, 'MidRainFoggyCloudyDay',  100,  60,  60,   0,   60,  45,  50),
    (12, 'FoggyDay',                 5,   0,   0,   0,   10,  45,  50),
    (13, 'FoggySunset',              5,   0,   0,   0,   10,  15,  50),
    (14, 'HardRainWetDay',         100, 100,  50,  50,  100,  45,  10),
    (15, 'MidRainVeryWetDay',      100,  50, 100,  80,   80,  45,  10),
    (18, 'CloudySunset',            40,   0,  50,   0,   10,  15,   2),
    (19, 'CloudyNight',             40,   0,  50,   0,   10, -90,   2),
    (20, 'SoftRainNight',           60,  30,  50,  60,   30, -90,   3),
    (21, 'HardRainNight',          100, 100,  90, 100,  100, -90,   3),
    (22, 'FoggyNight',               5,   0,   0,   0,   10, -90,  60),
    (23, 'MidRainFoggyNight',       80,  60,  60,  80,   60, -90,  60),
    (25, 'HardRainDenseFogNight',  100, 100,  90, 100,  100, -90, 100),
    (26, 'PartlyCloudyNoon',        50,   0,   0,   0,    0,  70,   0),
)


if __name__ == '__main__':
    # Weather self-consistency check (task 2 completion criterion): feeding each
    # preset's own parameters must return that preset's name. _classify_weather
    # ignores self, so it is called on the class with a dummy self. Run with the
    # collect_dataset PYTHONPATH so the base-class imports resolve, e.g.:
    #   python team_code/data_agents/ros2_data_agent_shasou.py
    failures = []
    for _preset in _BENCH2DRIVE_WEATHER_PRESETS:
        sample = dict(zip(_WEATHER_PARAM_NAMES, _preset[2:]))
        got = ROS2DataAgentShasou._classify_weather(None, sample)
        if got != _preset[1]:
            failures.append((_preset[1], got))
    if failures:
        raise SystemExit(f'weather self-test FAILED (expected -> got): {failures}')
    print(f'weather self-test passed: {len(_BENCH2DRIVE_WEATHER_PRESETS)} presets')
