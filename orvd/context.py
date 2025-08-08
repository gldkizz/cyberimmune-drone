import logging
import os
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
    change_forbidden_zones_enabled: bool = False
    change_forbidden_zones_A: dict = field(default_factory=dict)
    change_forbidden_zones_B: dict = field(default_factory=dict)
    change_forbidden_zones_C: dict = field(default_factory=dict)

    def __post_init__(self):
        def _get_coords(env_var):
            value = os.getenv(env_var)
            if value:
                try:
                    lat, lon = map(float, value.split(','))
                    return {'lat': lat, 'lon': lon}
                except (ValueError, TypeError):
                    logging.warning(f"Invalid format for {env_var}. Expected 'lat,lon'.")
            return {}

        if not self.permission_revoke_coords:
            self.permission_revoke_coords = _get_coords('PERMISSION_REVOKE_COORDS')
        
        if not self.connection_break_coords:
            self.connection_break_coords = _get_coords('CONNECTION_BREAK_COORDS')

        if not self.change_forbidden_zones_A:
            self.change_forbidden_zones_A = _get_coords('CHANGE_FORBIDDEN_ZONES_A')
        
        if not self.change_forbidden_zones_B:
            self.change_forbidden_zones_B = _get_coords('CHANGE_FORBIDDEN_ZONES_B')

        if not self.change_forbidden_zones_C:
            self.change_forbidden_zones_C = _get_coords('CHANGE_FORBIDDEN_ZONES_C')

context = Context()