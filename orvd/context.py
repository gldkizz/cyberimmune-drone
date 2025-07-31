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

context = Context()