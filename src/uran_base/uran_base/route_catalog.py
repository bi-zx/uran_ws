from typing import Dict

from .models import RouteDefinition


class RouteCatalog:

    def __init__(self, route_config: Dict[str, object]):
        routes = dict(route_config.get('routes', {}))
        services = dict(routes.get('services', {}))
        self._topics = {
            key: RouteDefinition(name=key, ros_name=str(value), kind='topic')
            for key, value in routes.items()
            if key != 'services'
        }
        self._services = {
            key: RouteDefinition(name=key, ros_name=str(value), kind='service')
            for key, value in services.items()
        }

    @classmethod
    def from_snapshot(cls, snapshot: Dict[str, object]) -> 'RouteCatalog':
        return cls(
            {
                'routes': {
                    **dict(snapshot.get('topics', {})),
                    'services': dict(snapshot.get('services', {})),
                }
            }
        )

    def topic(self, name: str) -> str:
        return self._topics[name].ros_name

    def service(self, name: str) -> str:
        return self._services[name].ros_name

    def snapshot(self) -> Dict[str, Dict[str, str]]:
        return {
            'topics': {name: route.ros_name for name, route in self._topics.items()},
            'services': {name: route.ros_name for name, route in self._services.items()},
        }
