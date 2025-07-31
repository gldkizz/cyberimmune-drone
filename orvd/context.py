import logging
from dataclasses import dataclass, field

@dataclass
class Context:
    log_level: int = logging.INFO
    display_only: bool = False
    flight_info_response: bool = True
    auto_mission_approval: bool = True
    arm_queue: set = field(default_factory=set)
    revise_mission_queue: set = field(default_factory=set)
    loaded_keys: dict = field(default_factory=dict)
    permission_revoke_coords: dict = field(default_factory=dict)
    permission_revoke_enabled: bool = False
    permission_revoked_uavs: set = field(default_factory=set)
    connection_break_coords: dict = field(default_factory=dict)
    connection_break_enabled: bool = False
    connection_broken_uavs: set = field(default_factory=set)

context = Context()