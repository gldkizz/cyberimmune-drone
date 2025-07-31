from context import context
from constants import (
    ORVD_KEY_SIZE, NOT_FOUND, LOGS_PATH, MissionVerificationStatus
)
from db.dao import (
    add_and_commit, add_changes, commit_changes, delete_entity, get_entity_by_key,
    get_entities_by_field, get_entities_by_field_with_order, save_public_key,
    get_key
)
from db.models import Mission, MissionStep, MissionSenderPublicKeys, Uav, UavTelemetry, Event
from utils import (
    generate_keys, read_mission, encode_mission, create_csv_from_telemetry
)


def key_ms_exchange_handler(id: str):
    """
    Обрабатывает обмен ключами с Mission Sender.

    Args:
        id (str): Идентификатор отправителя миссии.

    Returns:
        str: Строка с открытым ключом ORVD.
    """
    key_group = f'ms{id}'
    if f'ms{id}' not in context.loaded_keys:
        generate_keys(ORVD_KEY_SIZE, key_group)
    key = context.loaded_keys[key_group].publickey()
    n, e = str(key.n), str(key.e)
    key_entity = get_entity_by_key(MissionSenderPublicKeys, id)
    if key_entity is None:
        save_public_key(n, e, f'ms{id}')
    else:
        key_entity.n = n
        key_entity.e = e
        commit_changes()
    orvd_key_pk = get_key('orvd', private=True).publickey()
    orvd_n, orvd_e = orvd_key_pk.n, orvd_key_pk.e
    str_to_send = f'$Key: {hex(orvd_n)[2:]} {hex(orvd_e)[2:]}'
    return str_to_send


def fmission_ms_handler(id: str, mission_str: str, **kwargs):
    """
    Обрабатывает запрос на сохранение полетного задания от Mission Sender.

    Args:
        id (str): Идентификатор БПЛА.
        mission_str (str): Строка с полетным заданием.

    Returns:
        str: Статус верификации миссии.
    """
    mission_list, mission_verification_status = read_mission(mission_str)
    
    if mission_verification_status == MissionVerificationStatus.OK:
        uav_entity = get_entity_by_key(Uav, id)
        if not uav_entity and context.display_only:
            uav_entity = Uav(id=id, is_armed=False, state='В сети', kill_switch_state=False)
            add_and_commit(uav_entity)
            
        mission_entity = get_entity_by_key(Mission, id)
        if mission_entity:
            get_entities_by_field(MissionStep, MissionStep.mission_id, id).delete()
            delete_entity(mission_entity)
            commit_changes()
        
        mission_entity = Mission(uav_id=id, is_accepted=context.auto_mission_approval)
        add_changes(mission_entity)
        encoded_mission = encode_mission(mission_list)
        for idx, cmd in enumerate(encoded_mission):
            mission_step_entity = MissionStep(mission_id=id, step=idx, operation=cmd)
            add_changes(mission_step_entity)
        commit_changes()
        
    return mission_verification_status


def get_logs_handler(id: str):
    """
    Обрабатывает запрос на получение логов БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Строка с логами или NOT_FOUND.
    """
    uav_log = None
    try:
        with open(f'{LOGS_PATH}/{id}.txt') as f:
            uav_log = f.read()
        if uav_log:
            return uav_log
        else:
            return NOT_FOUND
    except Exception:
        return NOT_FOUND


def get_telemetry_csv_handler(id: str):
    """
    Обрабатывает запрос на получение всей телеметрии БПЛА в формате CSV.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: CSV-строка с телеметрическими данными или NOT_FOUND.
    """
    uav_telemetry_entities = get_entities_by_field_with_order(UavTelemetry, UavTelemetry.uav_id, id, UavTelemetry.record_time.asc())
    if not uav_telemetry_entities:
        return NOT_FOUND

    csv_data = create_csv_from_telemetry(uav_telemetry_entities)
    return csv_data


def get_events_handler(id: str):
    """
    Обрабатывает запрос на получение событий БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Строка с событиями в формате JSON или NOT_FOUND.
    """
    events = get_entities_by_field_with_order(Event, Event.uav_id, id, Event.timestamp.asc())
    if not events:
        return NOT_FOUND

    events_list = []
    for event in events:
        try:
            params = dict(p.split('=', 1) for p in event.log_message.split('&'))
            events_list.append({
                'type': params.get('type'),
                'event': params.get('event'),
                'timestamp': event.timestamp.isoformat()
            })
        except ValueError:
            continue
            
    import json
    return json.dumps(events_list)