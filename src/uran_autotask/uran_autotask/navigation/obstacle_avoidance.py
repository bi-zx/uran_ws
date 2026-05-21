import math
import statistics
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class GapChoice:
    theta_target: float
    gap_start: float
    gap_end: float
    gap_width_rad: float
    score: float
    front_clearance_m: float
    nearest_obstacle_m: float
    blocked_ratio: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            'theta_target_rad': self.theta_target,
            'theta_target_deg': math.degrees(self.theta_target),
            'gap_start_deg': math.degrees(self.gap_start),
            'gap_end_deg': math.degrees(self.gap_end),
            'gap_width_deg': math.degrees(self.gap_width_rad),
            'score': self.score,
            'front_clearance_m': self.front_clearance_m,
            'nearest_obstacle_m': self.nearest_obstacle_m,
            'blocked_ratio': self.blocked_ratio,
        }


@dataclass
class AvoidanceDecision:
    state: str
    theta_target: float
    front_clearance_m: float
    nearest_obstacle_m: float
    blocked_ratio: float
    choice: Optional[GapChoice] = None
    reason: str = ''

    def to_dict(self) -> Dict[str, Any]:
        return {
            'state': self.state,
            'theta_target_rad': self.theta_target,
            'theta_target_deg': math.degrees(self.theta_target),
            'front_clearance_m': self.front_clearance_m,
            'nearest_obstacle_m': self.nearest_obstacle_m,
            'blocked_ratio': self.blocked_ratio,
            'reason': self.reason,
            'choice': self.choice.to_dict() if self.choice is not None else None,
        }


class GapAvoidancePlanner:
    """Small Follow-the-Gap style planner for straight-drive obstacle avoidance."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        cfg = dict(config or {})
        self.usable_fov_rad = math.radians(float(cfg.get('usable_fov_deg', 180.0)))
        self.angle_offset_rad = math.radians(float(cfg.get('angle_offset_deg', 0.0)))
        self.obstacle_consider_range_m = float(cfg.get('obstacle_consider_range_m', 3.0))
        self.robot_radius_m = float(cfg.get('robot_radius_m', 0.35))
        self.safety_margin_m = float(cfg.get('safety_margin_m', 0.20))
        self.inflation_radius_m = float(
            cfg.get('inflation_radius_m', self.robot_radius_m + self.safety_margin_m)
        )
        self.min_gap_width_rad = math.radians(float(cfg.get('min_gap_width_deg', 28.0)))
        self.max_bubble_angle_rad = math.radians(float(cfg.get('max_bubble_angle_deg', 35.0)))
        self.front_sector_rad = math.radians(float(cfg.get('front_sector_deg', 18.0)))
        self.min_stop_distance_m = float(cfg.get('min_stop_distance_m', 0.65))
        self.clear_path_distance_m = float(cfg.get('clear_path_distance_m', 1.4))
        self.median_window = max(1, int(cfg.get('median_window', 5)))
        if self.median_window % 2 == 0:
            self.median_window += 1
        self.goal_weight = float(cfg.get('goal_weight', 2.0))
        self.clearance_weight = float(cfg.get('clearance_weight', 1.0))
        self.width_weight = float(cfg.get('width_weight', 0.25))
        self.center_bias_weight = float(cfg.get('center_bias_weight', 0.4))

    def plan(self, scan, *, theta_goal: float = 0.0) -> AvoidanceDecision:
        beams = self._preprocess(scan)
        if not beams:
            return AvoidanceDecision(
                state='blocked',
                theta_target=0.0,
                front_clearance_m=0.0,
                nearest_obstacle_m=0.0,
                blocked_ratio=1.0,
                reason='no usable laser beams',
            )

        front_ranges = [r for theta, r in beams if abs(theta) <= self.front_sector_rad]
        front_clearance = min(front_ranges) if front_ranges else min(r for _, r in beams)
        nearest_obstacle = min(r for _, r in beams)

        blocked = self._build_blocked_mask(beams)
        blocked_ratio = float(sum(1 for item in blocked if item)) / float(len(blocked))

        if front_clearance <= self.min_stop_distance_m:
            choice = self._choose_gap(beams, blocked, theta_goal=theta_goal)
            return AvoidanceDecision(
                state='stop',
                theta_target=choice.theta_target if choice is not None else 0.0,
                front_clearance_m=front_clearance,
                nearest_obstacle_m=nearest_obstacle,
                blocked_ratio=blocked_ratio,
                choice=choice,
                reason='front obstacle inside stop distance',
            )

        if self._has_clear_goal_corridor(beams, blocked, theta_goal):
            return AvoidanceDecision(
                state='clear',
                theta_target=_normalize_angle(theta_goal),
                front_clearance_m=front_clearance,
                nearest_obstacle_m=nearest_obstacle,
                blocked_ratio=blocked_ratio,
                reason='goal corridor is clear',
            )

        choice = self._choose_gap(beams, blocked, theta_goal=theta_goal)
        if choice is None:
            return AvoidanceDecision(
                state='blocked',
                theta_target=0.0,
                front_clearance_m=front_clearance,
                nearest_obstacle_m=nearest_obstacle,
                blocked_ratio=blocked_ratio,
                reason='no safe gap found',
            )

        return AvoidanceDecision(
            state='avoid',
            theta_target=choice.theta_target,
            front_clearance_m=front_clearance,
            nearest_obstacle_m=nearest_obstacle,
            blocked_ratio=blocked_ratio,
            choice=choice,
            reason='selected safest local gap',
        )

    def _preprocess(self, scan) -> List[Tuple[float, float]]:
        ranges = list(getattr(scan, 'ranges', []) or [])
        if not ranges:
            return []

        angle_min = float(getattr(scan, 'angle_min', 0.0))
        angle_increment = float(getattr(scan, 'angle_increment', 0.0))
        if not math.isfinite(angle_increment) or abs(angle_increment) < 1e-9:
            return []

        range_min = float(getattr(scan, 'range_min', 0.05) or 0.05)
        range_max = float(getattr(scan, 'range_max', 10.0) or 10.0)
        if not math.isfinite(range_max) or range_max <= range_min:
            range_max = max(range_min + 1.0, self.obstacle_consider_range_m)

        half_fov = max(0.0, self.usable_fov_rad / 2.0)
        normalized_ranges: List[float] = []
        for raw in ranges:
            try:
                value = float(raw)
            except Exception:
                value = range_max
            if not math.isfinite(value) or value <= 0.0:
                value = range_max
            value = _clamp(value, range_min, range_max)
            normalized_ranges.append(value)

        filtered_ranges = self._median_filter(normalized_ranges)
        beams: List[Tuple[float, float]] = []
        for index, value in enumerate(filtered_ranges):
            theta = angle_min + float(index) * angle_increment + self.angle_offset_rad
            theta = _normalize_angle(theta)
            if -half_fov <= theta <= half_fov:
                beams.append((theta, value))
        beams.sort(key=lambda item: item[0])
        return beams

    def _median_filter(self, values: Sequence[float]) -> List[float]:
        if self.median_window <= 1 or len(values) <= 2:
            return list(values)
        radius = self.median_window // 2
        result: List[float] = []
        for index in range(len(values)):
            start = max(0, index - radius)
            end = min(len(values), index + radius + 1)
            result.append(float(statistics.median(values[start:end])))
        return result

    def _build_blocked_mask(self, beams: Sequence[Tuple[float, float]]) -> List[bool]:
        blocked = [False] * len(beams)
        if not beams:
            return blocked

        angle_increment = self._estimate_angle_increment(beams)
        for index, (_, distance) in enumerate(beams):
            if distance > self.obstacle_consider_range_m:
                continue
            ratio = _clamp(self.inflation_radius_m / max(distance, 0.05), 0.0, 1.0)
            bubble = min(math.asin(ratio), self.max_bubble_angle_rad)
            k = max(0, int(math.ceil(bubble / max(angle_increment, 1e-6))))
            start = max(0, index - k)
            end = min(len(beams), index + k + 1)
            for beam_index in range(start, end):
                blocked[beam_index] = True
        return blocked

    def _estimate_angle_increment(self, beams: Sequence[Tuple[float, float]]) -> float:
        if len(beams) < 2:
            return math.radians(1.0)
        diffs = [
            abs(float(beams[index + 1][0]) - float(beams[index][0]))
            for index in range(len(beams) - 1)
        ]
        diffs = [item for item in diffs if item > 1e-9 and math.isfinite(item)]
        if not diffs:
            return math.radians(1.0)
        return float(statistics.median(diffs))

    def _has_clear_goal_corridor(
        self,
        beams: Sequence[Tuple[float, float]],
        blocked: Sequence[bool],
        theta_goal: float,
    ) -> bool:
        theta_goal = _normalize_angle(theta_goal)
        corridor_half_width = max(self.min_gap_width_rad / 2.0, self.front_sector_rad)
        matched = False
        for (theta, distance), is_blocked in zip(beams, blocked):
            if abs(_normalize_angle(theta - theta_goal)) > corridor_half_width:
                continue
            matched = True
            if is_blocked or distance < self.clear_path_distance_m:
                return False
        return matched

    def _choose_gap(
        self,
        beams: Sequence[Tuple[float, float]],
        blocked: Sequence[bool],
        *,
        theta_goal: float,
    ) -> Optional[GapChoice]:
        gaps = self._find_gaps(beams, blocked)
        if not gaps:
            return None

        theta_goal = _normalize_angle(theta_goal)
        best: Optional[GapChoice] = None
        front_ranges = [r for theta, r in beams if abs(theta) <= self.front_sector_rad]
        front_clearance = min(front_ranges) if front_ranges else min(r for _, r in beams)
        nearest_obstacle = min(r for _, r in beams)
        blocked_ratio = float(sum(1 for item in blocked if item)) / float(len(blocked))

        for start, end in gaps:
            start_theta = beams[start][0]
            end_theta = beams[end][0]
            width = end_theta - start_theta
            if width < self.min_gap_width_rad:
                continue
            target = _clamp(theta_goal, start_theta, end_theta)
            center = (start_theta + end_theta) / 2.0
            gap_clearance = min(beams[index][1] for index in range(start, end + 1))
            score = (
                -self.goal_weight * abs(_normalize_angle(target - theta_goal)) +
                self.clearance_weight * min(gap_clearance, self.obstacle_consider_range_m) +
                self.width_weight * width -
                self.center_bias_weight * abs(center)
            )
            choice = GapChoice(
                theta_target=target,
                gap_start=start_theta,
                gap_end=end_theta,
                gap_width_rad=width,
                score=score,
                front_clearance_m=front_clearance,
                nearest_obstacle_m=nearest_obstacle,
                blocked_ratio=blocked_ratio,
            )
            if best is None or choice.score > best.score:
                best = choice
        return best

    def _find_gaps(
        self,
        beams: Sequence[Tuple[float, float]],
        blocked: Sequence[bool],
    ) -> List[Tuple[int, int]]:
        gaps: List[Tuple[int, int]] = []
        start = None
        for index, is_blocked in enumerate(blocked):
            if not is_blocked and start is None:
                start = index
            elif is_blocked and start is not None:
                gaps.append((start, index - 1))
                start = None
        if start is not None:
            gaps.append((start, len(blocked) - 1))
        return gaps
