"""Bounded latest-message store. Callbacks only replace a reference under a lock."""
import threading
import time

from .adapters.vision import VisionAdapter


class DashboardState:
    def __init__(self, stale_after=2.0, field_length=28.0, field_width=15.0,
                 world_z_toward_blue=True):
        self.lock = threading.Lock()
        self.samples = {}
        self.adapters = [VisionAdapter()]
        self.stale_after = stale_after
        self.field = dict(length=field_length, width=field_width,
                          world_z_toward_blue=world_z_toward_blue)

    def update(self, source, message):
        with self.lock:
            self.samples[source] = (message, time.monotonic())

    def snapshot(self, channel='state'):
        with self.lock:
            samples = self.samples.copy()
        now = time.monotonic()
        # Serialization and conversion happen only on HTTP demand, never in ROS callbacks.
        if channel == 'performance':
            samples = {k: v for k, v in samples.items() if k == 'timing'}
        sources = {}
        for adapter in self.adapters:
            sources.update(adapter.snapshot(samples, now, self.stale_after))
        if channel == 'performance':
            return dict(performance=sources['timing'])
        modules = {}
        for name, source in [('Detection', 'detection'), ('Localization', 'measurement'),
                             ('Tracking', 'tracking'), ('Prior', 'prior'),
                             ('Fusion', 'fusion'), ('Decision', 'decision'), ('Camera', 'timing')]:
            s = sources[source]
            status = s['status']
            if name == 'Prior' and status == 'LIVE' and not s.get('model_enabled', True):
                status = 'DISABLED'
            modules[name] = dict(status=status, topic=s['topic'], age_s=s['age_s'])
        modules['Camera']['note'] = '由 PipelineTiming 推断输入活跃；不区分视频与相机硬件'
        for name in ('LiDAR', 'Radio', 'Referee'):
            modules[name] = dict(status='NOT IMPLEMENTED', topic=None, age_s=None)
        match = samples.get('match')
        # MatchState is durable configuration, not a periodically published heartbeat.
        own_team = match[0].own_team if match and match[0].own_team in (1, 2) else None
        return dict(schema_version=1, field=self.field, stale_after_s=self.stale_after,
                    match=dict(own_team=own_team, revision=str(match[0].revision) if match else None),
                    modules=modules, sources=sources)
