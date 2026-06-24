from typing import Dict, Iterable, List, Optional

from .models import PackageDescriptor, ProtocolDescriptor


class PackageRegistry:
    def __init__(self):
        self._packages: Dict[str, PackageDescriptor] = {}

    def register(self, descriptor: PackageDescriptor) -> None:
        self._packages[descriptor.package_id] = descriptor

    def bulk_register(self, descriptors: Iterable[PackageDescriptor]) -> None:
        for descriptor in descriptors:
            self.register(descriptor)

    def get(self, package_id: str) -> Optional[PackageDescriptor]:
        return self._packages.get(package_id)

    def list(self, *, enabled_only: bool = False) -> List[PackageDescriptor]:
        packages = list(self._packages.values())
        if enabled_only:
            packages = [pkg for pkg in packages if pkg.enabled]
        return sorted(packages, key=lambda item: item.package_id)

    def as_dict(self) -> Dict[str, dict]:
        return {
            descriptor.package_id: {
                'category': descriptor.category,
                'version': descriptor.version,
                'enabled': descriptor.enabled,
                'provides': list(descriptor.provides),
                'metadata': dict(descriptor.metadata),
            }
            for descriptor in self.list()
        }


class ProtocolRegistry:
    def __init__(self):
        self._protocols: Dict[str, ProtocolDescriptor] = {}

    def register(self, descriptor: ProtocolDescriptor) -> None:
        self._protocols[descriptor.protocol_id] = descriptor

    def get(self, protocol_id: str) -> Optional[ProtocolDescriptor]:
        return self._protocols.get(protocol_id)

    def list(self) -> List[ProtocolDescriptor]:
        return sorted(self._protocols.values(), key=lambda item: item.priority)

    def active_protocols(self) -> List[ProtocolDescriptor]:
        return sorted(
            [item for item in self._protocols.values() if item.enabled],
            key=lambda item: item.priority,
        )

    def protocol_table(self) -> Dict[str, dict]:
        table: Dict[str, dict] = {}
        for descriptor in self.active_protocols():
            try:
                table[descriptor.protocol_id] = descriptor.adapter.health_report()
            except Exception:
                table[descriptor.protocol_id] = {
                    'available': False,
                    'latency_ms': -1,
                    'last_check_ts': 0,
                    'registered': False,
                }
        return table

    def publish(self, protocol_id: str, payload: dict, qos: int = 1) -> bool:
        descriptor = self.get(protocol_id)
        if descriptor is None or not descriptor.enabled:
            return False
        return bool(descriptor.adapter.publish(payload, qos=qos))

    def disconnect_all(self) -> None:
        for descriptor in self.list():
            try:
                descriptor.adapter.disconnect()
            except Exception:
                continue
