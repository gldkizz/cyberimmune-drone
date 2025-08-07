import logging

ORVD_KEY_SIZE = 1024

ARMED = 0
DISARMED = 1

KILL_SWITCH_OFF = 1
KILL_SWITCH_ON = 0

MISSION_ACCEPTED = 0
MISSION_NOT_ACCEPTED = 1

NOT_FOUND = '$-1'
OK = '$OK'

LOGS_PATH = './logs'
FORBIDDEN_ZONES_PATH = './static/resources/forbidden_zones.json'
FORBIDDEN_ZONES_DELTA_PATH = './static/resources/forbidden_zones_delta.json'
TILES_PATH = './static/resources/tiles'

log_level_map = {
    'CRITICAL': logging.CRITICAL,
    'FATAL': logging.FATAL,
    'ERROR': logging.ERROR,
    'WARN': logging.WARNING,
    'WARNING': logging.WARNING,
    'INFO': logging.INFO,
    'DEBUG': logging.DEBUG,
    'NOTSET': logging.NOTSET,
}

class KeyGroup:
    KOS = 'kos'
    MS = 'ms'
    ORVD = 'orvd'

class MissionVerificationStatus:
    OK = 'Mission accepted.'
    NON_ZERO_DELAY_WAYPOINT = 'Error: The mission contains a waypoint with non-zero delay.'
    WRONG_DELAY = 'Error: Delay in the mission can contain only one parameter (delay in seconds).'
    UNKNOWN_COMMAND = 'Error: The mission contains an unknown command. Allowed commands: 16, 21, 22, 93, 183.'

class MQTTTopic:
    # receivable
    LOGS = 'api/logs/{id}'
    TELEMETRY = 'api/telemetry/{id}'
    FMISSION_MS = 'api/mission/{id}'
    NMISSION_REQUEST = 'api/nmission/request/{id}'
    ARM_REQUEST = 'api/arm/request/{id}'
    EVENTS = 'api/events/{id}'
    
    # sendable
    PING = 'ping/{id}'
    ARM_RESPONSE = 'api/arm/response/{id}'
    NMISSION_RESPONSE = 'api/nmission/response/{id}'
    FLIGHT_STATUS = 'api/flight_status/{id}'
    FORBIDDEN_ZONES = 'api/forbidden_zones'
    FMISSION_KOS = 'api/fmission_kos/{id}'
    AUTH = 'api/auth/{id}'
    CONNECTION_STATUS = 'api/connection_status'
    
class APIRoute:
    NMISSION = '/api/nmission'
    LOGS = '/api/logs'
    KEY_EXCHANGE = '/api/key'
    ARM = '/api/arm'
    AUTH = '/api/auth'
    FLY_ACCEPT = '/api/fly_accept'
    FLIGHT_INFO = '/api/flight_info'
    TELEMETRY = '/api/telemetry'
    KILL_SWITCH = '/api/kill_switch'
    FMISSION_KOS = '/api/fmission_kos'
    GET_ALL_FORBIDDEN_ZONES = '/api/get_all_forbidden_zones'
    GET_FORBIDDEN_ZONES_DELTA = '/api/get_forbidden_zones_delta'
    FORBIDDEN_ZONES_HASH = '/api/forbidden_zones_hash'

class AdminRoute:
    INDEX = '/admin'
    AUTH = '/admin/auth'
    AUTH_PAGE = '/admin/auth_page'
    ARM_DECISION = '/admin/arm_decision'
    MISSION_DECISION = '/admin/mission_decision'
    FORCE_DISARM = '/admin/force_disarm'
    FORCE_DISARM_ALL = '/admin/force_disarm_all'
    KILL_SWITCH = '/admin/kill_switch'
    GET_STATE = '/admin/get_state'
    GET_MISSION_STATE = '/admin/get_mission_state'
    GET_MISSION = '/admin/get_mission'
    GET_TELEMETRY = '/admin/get_telemetry'
    GET_WAITER_NUMBER = '/admin/get_waiter_number'
    GET_ID_LIST = '/admin/get_id_list'
    CHANGE_FLY_ACCEPT = '/admin/change_fly_accept'
    GET_FORBIDDEN_ZONES = '/admin/get_forbidden_zones'
    GET_FORBIDDEN_ZONE = '/admin/get_forbidden_zone'
    GET_FORBIDDEN_ZONES_NAMES = '/admin/get_forbidden_zones_names'
    SET_FORBIDDEN_ZONE = '/admin/set_forbidden_zone'
    DELETE_FORBIDDEN_ZONE = '/admin/delete_forbidden_zone'
    FORBIDDEN_ZONES = '/admin/forbidden_zones'
    EXPORT_FORBIDDEN_ZONES = '/admin/export_forbidden_zones'
    IMPORT_FORBIDDEN_ZONES = '/admin/import_forbidden_zones'
    GET_DELAY = '/admin/get_delay'
    SET_DELAY = '/admin/set_delay'
    REVISE_MISSION_DECISION = '/admin/revise_mission_decision'
    GET_DISPLAY_MODE = '/admin/get_display_mode'
    TOGGLE_DISPLAY_MODE = '/admin/toggle_display_mode'
    GET_FLIGHT_INFO_RESPONSE_MODE = '/admin/get_flight_info_response_mode'
    TOGGLE_FLIGHT_INFO_RESPONSE_MODE = '/admin/toggle_flight_info_response_mode'
    GET_ALL_DATA = '/admin/get_all_data'
    GET_AUTO_MISSION_APPROVAL_MODE = '/admin/get_auto_mission_approval_mode'
    TOGGLE_AUTO_MISSION_APPROVAL_MODE = '/admin/toggle_auto_mission_approval_mode'
    TOGGLE_AUTO_REVOKE_PERMISSION = '/admin/toggle_auto_revoke_permission'
    SET_REVOKE_COORDS = '/admin/set_revoke_coords'
    TOGGLE_AUTO_BREAK_CONNECTION = '/admin/toggle_auto_break_connection'
    SET_BREAK_COORDS = '/admin/set_break_coords'
    TOGGLE_CHANGE_FORBIDDEN_ZONES = '/admin/toggle_change_forbidden_zones'
    SET_CHANGE_FORBIDDEN_ZONES_COORDS = '/admin/set_change_forbidden_zones_coords'

class GeneralRoute:
    INDEX = '/'
    TILES_INDEX = '/tiles/index'
    LOGS_PAGE = '/logs'
    GET_LOGS = '/logs/get_logs'
    GET_EVENTS = '/logs/get_events'
    GET_TELEMETRY_CSV = '/logs/get_telemetry_csv'
    MISSION_SENDER = '/mission_sender'
    FMISSION_MS = '/mission_sender/fmission_ms'
    KEY_MS_EXCHANGE = '/mission_sender/key'