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
The request payload (location / route_id / scenario / weather) is task 2;
this class sends placeholders.

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
        # StartRecording only after the first parent run_step: the weather (part
        # of the request from task 2 on) is fixed by shuffle_weather() inside
        # _init(), which the parent triggers on its first call. The first tick's
        # publishes therefore predate the bag; that is accepted.
        if self._recorder_enabled and not self._recording_started:
            self._start_recording()
        return control

    def _start_recording(self):
        request = self._srv_start.Request()
        request.source = 'carla'
        # TODO(task 2): fill location / route_id / scenario / weather (via
        # _classify_weather) from the CARLA world state.
        request.location = ''
        request.route_id = ''
        request.scenario = ''
        request.weather = ''
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
        print(f'[ROS2DataAgentShasou] recording started (drive_id={response.drive_id})')

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
                # TODO(task 2+): report the real route outcome; for now every
                # stop is a nominal completion.
                request.completed = True
                request.reason = ''
                response = self._call_service(self._stop_client, request)
                if response is None:
                    print(f'[ROS2DataAgentShasou] StopRecording: no response within '
                          f'{self.RECORDER_RESPONSE_TIMEOUT_SEC}s; the drive may be left open.')
                elif not response.success:
                    print(f'[ROS2DataAgentShasou] StopRecording failed: {response.message}')
                else:
                    print(f'[ROS2DataAgentShasou] recording stopped (drive_id={response.drive_id}, '
                          f'{response.message_count} messages, {response.duration_sec:.1f}s)')
            except Exception as error:  # noqa: BLE001 — never mask the evaluator teardown
                print(f'[ROS2DataAgentShasou] StopRecording failed: {error}')
            self._recording_started = False
        super().destroy(results)

    # ── Weather classification (task 2) ──────────────────────────────────────

    def _classify_weather(self, params):
        """
        Map CARLA WeatherParameters, given as a dict of attribute name -> value
        (cloudiness, precipitation, sun_altitude_angle, ...), to the weather
        label sent in StartRecording.weather.

        Stub: always returns 'dummy'. Task 2 replaces this with a
        nearest-neighbour classification against the 27 Bench2Drive weather
        presets.
        """
        return 'dummy'
