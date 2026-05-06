import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from geometry_msgs.msg import PoseStamped

from .task_models import GeoWaypoint


EARTH_RADIUS_M = 6378137.0


@dataclass
class GeoReferenceConfig:
    enabled: bool = False
    frame_id: str = 'map'
    reference_lat: float = 0.0
    reference_lon: float = 0.0
    map_origin_x: float = 0.0
    map_origin_y: float = 0.0
    map_yaw_deg: float = 0.0
    scale: float = 1.0

    @classmethod
    def from_dict(cls, payload: Optional[Dict[str, Any]]) -> 'GeoReferenceConfig':
        payload = payload or {}
        return cls(
            enabled=bool(payload.get('enabled', False)),
            frame_id=str(payload.get('frame_id', 'map')),
            reference_lat=float(payload.get('reference_lat', 0.0)),
            reference_lon=float(payload.get('reference_lon', 0.0)),
            map_origin_x=float(payload.get('map_origin_x', 0.0)),
            map_origin_y=float(payload.get('map_origin_y', 0.0)),
            map_yaw_deg=float(payload.get('map_yaw_deg', 0.0)),
            scale=float(payload.get('scale', 1.0)),
        )


class MapProjector:
    """Project WGS84 latitude/longitude into the Nav2 map frame."""

    def __init__(self, config: GeoReferenceConfig):
        self._config = config

    @property
    def config(self) -> GeoReferenceConfig:
        return self._config

    def is_ready(self) -> bool:
        return self._config.enabled

    def latlon_to_enu(self, lat: float, lon: float) -> Tuple[float, float]:
        ref_lat_rad = math.radians(self._config.reference_lat)
        dlat_rad = math.radians(lat - self._config.reference_lat)
        dlon_rad = math.radians(lon - self._config.reference_lon)
        east = dlon_rad * EARTH_RADIUS_M * math.cos(ref_lat_rad)
        north = dlat_rad * EARTH_RADIUS_M
        return east, north

    def latlon_to_map_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        if not self.is_ready():
            raise RuntimeError('geo reference is not enabled')

        east, north = self.latlon_to_enu(lat, lon)
        yaw_rad = math.radians(self._config.map_yaw_deg)

        # map_yaw_deg is defined as the CCW angle from ENU-East to map-x.
        local_x = east * math.cos(yaw_rad) + north * math.sin(yaw_rad)
        local_y = -east * math.sin(yaw_rad) + north * math.cos(yaw_rad)

        return (
            self._config.map_origin_x + local_x * self._config.scale,
            self._config.map_origin_y + local_y * self._config.scale,
        )

    def map_xy_to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        if not self.is_ready():
            raise RuntimeError('geo reference is not enabled')

        scale = self._config.scale if abs(self._config.scale) > 1e-9 else 1.0
        local_x = (float(x) - self._config.map_origin_x) / scale
        local_y = (float(y) - self._config.map_origin_y) / scale
        yaw_rad = math.radians(self._config.map_yaw_deg)

        east = local_x * math.cos(yaw_rad) - local_y * math.sin(yaw_rad)
        north = local_x * math.sin(yaw_rad) + local_y * math.cos(yaw_rad)

        ref_lat_rad = math.radians(self._config.reference_lat)
        lat = self._config.reference_lat + math.degrees(north / EARTH_RADIUS_M)
        lon = self._config.reference_lon + math.degrees(
            east / (EARTH_RADIUS_M * math.cos(ref_lat_rad))
        )
        return lat, lon

    def heading_deg_to_map_yaw(self, heading_deg: Optional[float], heading_mode: str) -> float:
        if heading_deg is None:
            return 0.0

        if heading_mode != 'true_north_deg':
            return math.radians(heading_deg)

        # Task heading is clockwise-positive from true north.
        enu_yaw = math.radians(90.0 - heading_deg)
        return enu_yaw - math.radians(self._config.map_yaw_deg)

    def waypoint_to_pose_stamped(
        self,
        waypoint: GeoWaypoint,
        *,
        stamp,
        heading_mode: str = 'true_north_deg',
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._config.frame_id
        map_x, map_y = self.latlon_to_map_xy(waypoint.lat, waypoint.lon)
        pose.pose.position.x = map_x
        pose.pose.position.y = map_y
        pose.pose.position.z = waypoint.alt

        yaw = self.heading_deg_to_map_yaw(waypoint.heading_deg, heading_mode)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def reference_summary(self) -> Dict[str, Any]:
        return {
            'enabled': self._config.enabled,
            'frame_id': self._config.frame_id,
            'reference_lat': self._config.reference_lat,
            'reference_lon': self._config.reference_lon,
            'map_origin_x': self._config.map_origin_x,
            'map_origin_y': self._config.map_origin_y,
            'map_yaw_deg': self._config.map_yaw_deg,
            'scale': self._config.scale,
        }
