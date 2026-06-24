from .control_switch import handle_control_switch
from .frpc_control import handle_frpc_ctrl
from .media_control import handle_media_ctrl
from .move_command import handle_move_cmd
from .param_update import handle_param_update
from .state_query import handle_state_query
from .task_control import handle_task_ctrl

__all__ = [
    'handle_control_switch',
    'handle_frpc_ctrl',
    'handle_media_ctrl',
    'handle_move_cmd',
    'handle_param_update',
    'handle_state_query',
    'handle_task_ctrl',
]
