import time
from typing import Any, Dict, Optional


class CyberdogCameraCaptureAdapter:
    """Trigger a still capture on CyberDog2 camera services."""

    def __init__(self, node, config: Dict[str, Any]):
        self._node = node
        self._ready_timeout_s = float(config.get('ready_timeout_s', 5.0))
        self._prefer_take_photo = bool(config.get('prefer_take_photo_service', False))
        self._width = int(config.get('width', 1280))
        self._height = int(config.get('height', 960))
        self._fps = int(config.get('fps', 10))
        self._namespace = str(config.get('namespace', '')).strip().strip('/')

        self._take_photo_service_name = self._resolve_name(
            config.get('take_photo_service_name', 'take_photo')
        )
        self._camera_service_name = self._resolve_name(
            config.get('camera_service_name', 'camera_service')
        )

        self._available = False
        self._take_photo_client = None
        self._camera_client = None
        self._pending_future = None
        self._pending_deadline = 0.0
        self._pending_backend = ''
        self._pending_service_name = ''
        self._pending_decode = None

        try:
            from protocol.srv import CameraService, TakePhoto
        except ImportError as exc:
            raise RuntimeError(
                'protocol camera interfaces are unavailable. '
                'Source cyberdog_ws/install/setup.bash before using capture_image.'
            ) from exc

        self._CameraService = CameraService
        self._TakePhoto = TakePhoto
        self._take_photo_client = node.create_client(TakePhoto, self._take_photo_service_name)
        self._camera_client = node.create_client(CameraService, self._camera_service_name)
        self._available = True

    def _resolve_name(self, relative_name: str) -> str:
        relative_name = str(relative_name).strip()
        if not relative_name:
            return ''
        if relative_name.startswith('/'):
            return relative_name
        if not self._namespace:
            return '/' + relative_name.lstrip('/')
        return '/' + self._namespace + '/' + relative_name.lstrip('/')

    def capture(self) -> Dict[str, Any]:
        started = self.start_capture()
        if not started.get('accepted', False):
            return started['result']

        while self.has_pending_capture():
            result = self.poll_capture()
            if result is not None:
                return result
            time.sleep(0.05)

        return {
            'success': False,
            'backend': 'none',
            'service_name': '',
            'message': 'camera capture state lost before completion',
            'code': -1,
        }

    def has_pending_capture(self) -> bool:
        return self._pending_future is not None

    def cancel_pending(self):
        self._pending_future = None
        self._pending_deadline = 0.0
        self._pending_backend = ''
        self._pending_service_name = ''
        self._pending_decode = None

    def start_capture(self) -> Dict[str, Any]:
        if not self._available:
            return {
                'accepted': False,
                'result': {
                    'success': False,
                    'backend': 'none',
                    'service_name': '',
                    'message': 'camera adapter unavailable',
                    'code': -1,
                },
            }
        if self.has_pending_capture():
            return {
                'accepted': False,
                'result': {
                    'success': False,
                    'backend': self._pending_backend or 'unknown',
                    'service_name': self._pending_service_name,
                    'message': 'camera capture request already in progress',
                    'code': -1,
                },
            }

        if self._prefer_take_photo:
            started = self._start_take_photo_capture()
            if started is not None:
                return started

        started = self._start_camera_service_capture()
        if started is not None:
            return started

        if not self._prefer_take_photo:
            retry = self._start_take_photo_capture()
            if retry is not None:
                return retry

        return {
            'accepted': False,
            'result': {
                'success': False,
                'backend': 'none',
                'service_name': '',
                'message': 'no camera capture service is ready',
                'code': -1,
            },
        }

    def poll_capture(self) -> Optional[Dict[str, Any]]:
        if self._pending_future is None:
            return {
                'success': False,
                'backend': 'none',
                'service_name': '',
                'message': 'no pending camera capture request',
                'code': -1,
            }

        if time.time() >= self._pending_deadline:
            backend = self._pending_backend
            service_name = self._pending_service_name
            self.cancel_pending()
            return {
                'success': False,
                'backend': backend,
                'service_name': service_name,
                'message': 'camera capture service call timeout',
                'code': -1,
            }

        if not self._pending_future.done():
            return None

        future = self._pending_future
        backend = self._pending_backend
        service_name = self._pending_service_name
        decode = self._pending_decode
        self.cancel_pending()

        try:
            response = future.result()
        except Exception as exc:
            return {
                'success': False,
                'backend': backend,
                'service_name': service_name,
                'message': str(exc),
                'code': -1,
            }

        return decode(response, backend=backend, service_name=service_name)

    def _start_take_photo_capture(self) -> Optional[Dict[str, Any]]:
        client = self._take_photo_client
        if client is None:
            return None
        if not client.wait_for_service(timeout_sec=0.1):
            return None

        request = self._TakePhoto.Request()
        future = client.call_async(request)
        return self._start_future(
            future,
            backend='take_photo',
            service_name=self._take_photo_service_name,
            decode=self._decode_take_photo_response,
        )

    def _start_camera_service_capture(self) -> Optional[Dict[str, Any]]:
        client = self._camera_client
        if client is None:
            return None
        if not client.wait_for_service(timeout_sec=0.1):
            return None

        request = self._CameraService.Request()
        request.command = self._CameraService.Request.TAKE_PICTURE
        request.args = ''
        request.width = self._width
        request.height = self._height
        request.fps = self._fps
        future = client.call_async(request)
        return self._start_future(
            future,
            backend='camera_service',
            service_name=self._camera_service_name,
            decode=self._decode_camera_service_response,
        )

    def _start_future(self, future, *, backend: str, service_name: str, decode):
        self._pending_future = future
        self._pending_deadline = time.time() + self._ready_timeout_s
        self._pending_backend = backend
        self._pending_service_name = service_name
        self._pending_decode = decode
        return {
            'accepted': True,
            'backend': backend,
            'service_name': service_name,
        }

    def _decode_take_photo_response(self, response, *, backend: str, service_name: str) -> Dict[str, Any]:
        image = response.img
        return {
            'success': bool(response.result),
            'backend': backend,
            'service_name': service_name,
            'message': str(response.message),
            'code': int(response.code),
            'image_meta': {
                'width': int(image.width),
                'height': int(image.height),
                'encoding': str(image.encoding),
                'frame_id': str(image.header.frame_id),
                'stamp_sec': int(image.header.stamp.sec),
                'stamp_nanosec': int(image.header.stamp.nanosec),
            },
        }

    def _decode_camera_service_response(
        self,
        response,
        *,
        backend: str,
        service_name: str,
    ) -> Dict[str, Any]:
        success = int(response.result) == int(self._CameraService.Response.RESULT_SUCCESS)
        return {
            'success': success,
            'backend': backend,
            'service_name': service_name,
            'message': str(response.msg),
            'code': int(response.code),
            'result': int(response.result),
            'requested_width': self._width,
            'requested_height': self._height,
            'requested_fps': self._fps,
        }
