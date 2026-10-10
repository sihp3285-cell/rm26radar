"""Map the repository's actual message contract into JSON-compatible telemetry.

Never infer associations for WorldMeasurement (it has no track_id). The raw
measurement belonging to a track is explicitly provided by WorldTarget.
"""
import math


TOPICS = {
    'detection': ('/armor_detections', 'DetectionArray'),
    'timing': ('/pipeline_timing', 'PipelineTiming'),
    'measurement': ('/world_measurements', 'WorldMeasurementArray'),
    'tracking': ('/world_targets', 'WorldTargetArray'),
    'prior': ('/prior_predictions', 'PriorPredictionArray'),
    'fusion': ('/fused_targets', 'FusedTargetArray'),
    'decision': ('/map_tactics', 'MapTactics'),
}
METRICS = ('camera_fps', 'fps', 'dropped_fps', 'dropped_count', 'car_ms',
           'armor_ms', 'cls_ms', 'total_ms', 'end_to_end_ms', 'outpost_ms')
TRACKING = ('INVALID', 'ACTIVE', 'PREDICTED', 'LOST', 'DEAD')
SOURCES = ('INVALID', 'TRACKED', 'PREDICTED', 'PRIOR')
CLASSES = ('car', 'armor', '1 / Hero', '2 / Engineer', '3 / Infantry',
           '4 / Infantry', 'S / Sentry', 'Outpost')


def number(value):
    return value if math.isfinite(value) else None


def stamp(value):
    # String avoids JS precision loss for nanosecond stamps / calibration epochs.
    return str(value.sec * 1_000_000_000 + value.nanosec)


def position(x, z, valid=True):
    return [x, z] if valid and math.isfinite(x) and math.isfinite(z) else None


def label(names, value):
    return names[value] if 0 <= value < len(names) else str(value)


def identity(slot, track, team, role):
    return dict(key=f'{slot}:{track}:{team}:{role}', slot_idx=slot,
                track_id=track, team_id=team, class_id=role,
                class_name=label(CLASSES, role))


def measurements(message):
    return [dict(position=position(m.world_x, m.world_z, m.valid),
                 class_id=m.detection.idx, team_id=m.detection.armor_color,
                 confidence=number(m.detection.confidence), negative=m.is_negative)
            for m in message.measurements]


def tracking(message):
    result = []
    for slot, t in enumerate(message.targets):
        if t.track_id < 0 and not t.valid:
            continue
        result.append(dict(
            **identity(slot, t.track_id, t.team_id, t.class_id),
            valid=t.valid, observed=t.observed, is_dead=t.is_dead,
            tracking_state=label(TRACKING, t.tracking_state),
            position_source=t.position_source,
            position=position(t.world_x, t.world_z, t.valid and t.position_source != 0),
            measurement_position=position(t.measurement_x, t.measurement_z,
                                          t.measurement_covariance_valid),
            measurement_stamp_ns=stamp(t.last_observed_time),
            velocity=position(t.velocity_x, t.velocity_z, t.track_id >= 0),
            lost_duration=number(t.lost_duration_s),
            detection_confidence=number(t.detection_confidence),
            tracking_confidence=number(t.tracking_confidence)))
    return result


def priors(message):
    result = []
    for p in message.predictions:
        result.append(dict(
            **identity(p.slot_idx, p.track_id, p.team_id, p.role_class_id),
            valid=p.valid, position=position(p.prior_world_x, p.prior_world_z, p.valid),
            # This anchor is the last reliable Tracker output, NOT the raw measurement.
            last_position=position(p.last_world_x, p.last_world_z,
                                   int(stamp(p.last_observed_time)) > 0),
            last_observed_stamp_ns=stamp(p.last_observed_time),
            # Early rejections return before tracker_world is populated (default 0,0).
            tracker_position=position(p.tracker_world_x, p.tracker_world_z,
                                      p.horizon_seconds > 0),
            confidence=number(p.prior_confidence), lost_duration=number(p.lost_duration_s),
            rejection_code=p.rejection_code, rejection_reason=p.rejection_reason,
            entropy=number(p.normalized_entropy),
            reachable_mass=number(p.reachable_probability_mass), sample_count=p.sample_count,
            motion_gate=p.motion_gated, mesh_used=p.mesh_used,
            blind_zone_bias=p.blind_zone_biased,
            blind_zone_mass=number(p.blind_zone_probability_mass),
            stay_anchor_mass=number(p.stay_anchor_probability_mass),
            horizon_seconds=p.horizon_seconds,
            candidates=[dict(position=position(c.world_x, c.world_z), grid_index=c.grid_index,
                             prior_probability=number(c.prior_probability),
                             probability=number(c.fused_probability), reachable=c.reachable,
                             blocked=c.blocked, blind_zone=c.from_blind_zone,
                             stay_anchor=c.stay_anchor,
                             distance=number(c.distance_from_last_m)) for c in p.candidates]))
    return result


def fused(message):
    return [dict(**identity(t.slot_idx, t.track_id, t.team_id, t.class_id),
                 valid=t.valid, position=position(t.world_x, t.world_z, t.valid and t.source != 0),
                 source=label(SOURCES, t.source), confidence=number(t.confidence),
                 lost_duration=number(t.lost_duration_s), source_stamp_ns=stamp(t.source_stamp))
            for t in message.targets if t.track_id >= 0 or t.valid]


def performance(message):
    return {key: number(getattr(message, key)) for key in METRICS}


def decision(message):
    return {key: getattr(message, key) for key in (
        'engineer_on_island', 'opponent_attack', 'our_attack', 'opponent_near_fortress')}


class VisionAdapter:
    topics = TOPICS
    converters = dict(measurement=measurements, tracking=tracking, prior=priors,
                      fusion=fused, timing=performance, decision=decision,
                      detection=lambda m: {'count': len(m.detections)})

    def snapshot(self, samples, now, stale_after):
        sources = {}
        for name, (topic, _) in self.topics.items():
            sample = samples.get(name)
            age = now - sample[1] if sample else None
            message = sample[0] if sample else None
            sources[name] = dict(topic=topic, age_s=age,
                status='WAITING' if not sample else 'STALE' if age > stale_after else 'LIVE',
                stamp_ns=stamp(message.header.stamp) if message else None,
                calibration_version=str(message.calibration_version)
                if message and hasattr(message, 'calibration_version') else None,
                data=self.converters[name](message) if message else None)
        prior = samples.get('prior')
        if prior:
            sources['prior'].update(model_enabled=prior[0].model_enabled,
                                    model_status=prior[0].model_status)
        return sources
