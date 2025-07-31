import json
from flask import jsonify
from context import context
from extensions import task_scheduler_client as scheduler
from db.models import User, UavTelemetry, Mission, MissionStep, Uav
from constants import (
    ARMED, NOT_FOUND, OK, FORBIDDEN_ZONES_PATH,
    MISSION_ACCEPTED, MISSION_NOT_ACCEPTED
)
from db.dao import (
    commit_changes, get_entity_by_key, get_entities_by_field_with_order, flush
)
from utils import (
    get_sha256_hex, get_new_polygon_feature, compute_and_save_forbidden_zones_delta
)
from .mqtt_handlers import (
    mqtt_publish_flight_state, mqtt_publish_forbidden_zones, mqtt_publish_ping, mqtt_send_mission
)


def admin_auth_handler(login: str, password: str):
    """
    Обрабатывает запрос на аутентификацию администратора.

    Args:
        login (str): Логин администратора.
        password (str): Пароль администратора.

    Returns:
        str: Токен доступа или пустая строка в случае неудачи.
    """
    user_entity = get_entity_by_key(User, login)
    if not user_entity:
        return NOT_FOUND
    else:
        password_hash = get_sha256_hex(password)
        if password_hash == user_entity.password_hash:
            return user_entity.access_token
        else:
            return ''


def arm_decision_handler(id: str, decision: int):
    """
    Обрабатывает решение об арме БПЛА.

    Args:
        id (str): Идентификатор БПЛА.
        decision (int): Решение об арме (ARMED или DISARMED).

    Returns:
        str: Статус арма или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    elif id in context.arm_queue:
        uav_entity.is_armed = True if decision == ARMED else False
        commit_changes()
        context.arm_queue.remove(id)
        return f'$Arm: {decision}'
    else:
        return '$Arm: -1'


def force_disarm_handler(id: str):
    """
    Обрабатывает запрос на принудительный дизарм БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: OK в случае успешного дизарма или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        uav_entity.is_armed = False
        uav_entity.state = 'В сети'
        commit_changes()
        flush()
        mqtt_publish_flight_state(id)
        return OK


def force_disarm_all_handler():
    """
    Обрабатывает запрос на принудительный дизарм всех БПЛА.

    Returns:
        str: OK в случае успешного дизарма всех БПЛА.
    """
    uav_entities = Uav.query.all()
    for uav_entity in uav_entities:
        uav_entity.is_armed = False
        uav_entity.state = 'В сети'
    commit_changes()
    return OK


def get_state_handler(id: str):
    """
    Обрабатывает запрос на получение состояния БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Состояние БПЛА или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        return uav_entity.state


def get_mission_handler(id: str):
    """
    Обрабатывает запрос на получение полетного задания БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Строка с полетным заданием или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        mission = get_entity_by_key(Mission, id)
        if mission:
            mission_steps = get_entities_by_field_with_order(MissionStep, MissionStep.mission_id, id, order_by_field=MissionStep.step)
            if mission_steps and mission_steps.count() != 0:
                mission_steps = list(map(lambda e: e.operation, mission_steps))
                return "&".join(mission_steps)
    return NOT_FOUND


def get_telemetry_handler(id: str):
    """
    Обрабатывает запрос на получение телеметрии БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        json: JSON-объект с телеметрическими данными или NOT_FOUND.
    """
    uav_telemetry_entity = get_entities_by_field_with_order(UavTelemetry, UavTelemetry.uav_id, id, UavTelemetry.record_time.desc()).first()
    if not uav_telemetry_entity:
        return jsonify({'error': 'NOT_FOUND'})
    else:
        telemetry = {
            'lat': uav_telemetry_entity.lat,
            'lon': uav_telemetry_entity.lon,
            'alt': uav_telemetry_entity.alt,
            'azimuth': uav_telemetry_entity.azimuth,
            'dop': uav_telemetry_entity.dop,
            'sats': uav_telemetry_entity.sats,
            'speed': uav_telemetry_entity.speed
        }
        return jsonify(telemetry)


def get_waiter_number_handler():
    """
    Обрабатывает запрос на получение количества БПЛА, ожидающих решения об арме.

    Returns:
        str: Количество ожидающих БПЛА.
    """
    return str(len(context.arm_queue))


def mission_decision_handler(id: str, decision: int):
    """
    Обрабатывает решение о принятии или отклонении миссии.

    Args:
        id (str): Идентификатор БПЛА.
        decision (int): Решение (0 - принять, 1 - отклонить).

    Returns:
        str: OK в случае успешной обработки или NOT_FOUND.
    """
    mission_entity = get_entity_by_key(Mission, id)
    if not mission_entity:
        return NOT_FOUND
    else:
        if decision == 0:
            mission_entity.is_accepted = True
            commit_changes()
            flush()
            mqtt_send_mission(id)
        else:
            mission_entity.is_accepted = False
            commit_changes()
        return OK


def admin_kill_switch_handler(id: str):
    """
    Обрабатывает запрос на активацию аварийного выключателя БПЛА администратором.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: OK в случае успешной активации или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        uav_entity.is_armed = False
        uav_entity.kill_switch_state = True
        uav_entity.state = "Kill switch ON"
        commit_changes()
        flush()
        mqtt_publish_flight_state(id)
        return OK


def get_id_list_handler():
    """
    Обрабатывает запрос на получение списка идентификаторов всех БПЛА.

    Returns:
        str: Строка со списком идентификаторов БПЛА
    """
    uav_entities = Uav.query.order_by(Uav.created_date).all()
    uav_ids = list(map(lambda e: e.id, uav_entities))
    return str(uav_ids)


def get_mission_state_handler(id: str):
    """
    Обрабатывает запрос на получение состояния миссии БПЛА.
    
    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Состояние миссии (принята/не принята) или NOT_FOUND.
    """
    if id in context.revise_mission_queue:
        return '2'
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        mission = get_entity_by_key(Mission, id)
        if mission:
            if mission.is_accepted:
                return str(MISSION_ACCEPTED)
            else:
                return str(MISSION_NOT_ACCEPTED)
    return NOT_FOUND


def change_fly_accept_handler(id: str, decision: int):
    """
    Обрабатывает запрос на изменение статуса принятия полета БПЛА.

    Args:
        id (str): Идентификатор БПЛА.
        decision (int): Решение (0 - принять, 1 - отклонить).

    Returns:
        str: OK в случае успешного изменения или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        if decision == 0:
            uav_entity.is_armed = True 
            uav_entity.state = 'В полете'
        else:
            uav_entity.is_armed = False
            uav_entity.state = 'В сети'
        commit_changes()
        flush()
        mqtt_publish_flight_state(id)
        return OK
    return NOT_FOUND


def get_forbidden_zone_handler(name: str):
    """
    Обрабатывает запрос на получение координат запрещенной для полета зоны по ее имени.

    Args:
        name (str): Имя запрещенной зоны.

    Returns:
        json: JSON-массив с координатами зоны или NOT_FOUND.
    """
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        forbidden_zones = json.load(f)
        matching_zone = None
        for zone in forbidden_zones['features']:
            if zone['properties'].get('name') == name:
                matching_zone = zone
                break
        if matching_zone:
            return jsonify(matching_zone['geometry']['coordinates'][0])
        else:
            return NOT_FOUND
    return NOT_FOUND


def get_forbidden_zones_handler():
    """
    Обрабатывает запрос на получение всех запрещенных зон.

    Returns:
        dict: GeoJSON с запрещенными зонами
    """
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        forbidden_zones = json.load(f)
    return forbidden_zones


def get_forbidden_zones_names_handler():
    """ 
    Обрабатывает запрос на получение имен всех запрещенных зон.

    Returns:
        json: JSON-массив с именами запрещенных зон или NOT_FOUND.
    """
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        forbidden_zones = json.load(f)
        zones_names = []
        for zone in forbidden_zones['features']:
            zones_names.append(zone['properties'].get('name'))
        return jsonify(zones_names)
        
    return NOT_FOUND


def set_forbidden_zone_handler(name: str, geometry: list):
    """
    Обрабатывает запрос на установку или обновление запрещенной для полета зоны.

    Args:
        name (str): Имя запрещенной зоны.
        geometry (list): Массив координат зоны.

    Returns:
        str: OK в случае успешной установки или сообщение об ошибке.
    """
    if not isinstance(geometry, list) or not all(isinstance(coord, list) and len(coord) == 2 for coord in geometry):
        return 'Bad geometry'
    
    for idx in range(len(geometry)):
        geometry[idx][0] = round(geometry[idx][0], 7)
        geometry[idx][1] = round(geometry[idx][1], 7)
        
    forbidden_zones = None
    
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        old_zones = json.load(f)
    
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        forbidden_zones = json.load(f)
        existing_zone = False
        for zone in forbidden_zones['features']:
            if zone['properties'].get('name') == name:
                zone['geometry']['coordinates'][0] = geometry
                existing_zone = True
        
        if not existing_zone:
            new_feature = get_new_polygon_feature(name, geometry)
            forbidden_zones['features'].append(new_feature)
    
    if forbidden_zones is not None:
        with open(FORBIDDEN_ZONES_PATH, 'w', encoding='utf-8') as f:
            json.dump(forbidden_zones, f, ensure_ascii=False, indent=4)
            
        compute_and_save_forbidden_zones_delta(old_zones, forbidden_zones)
    
    mqtt_publish_forbidden_zones()
    return OK


def delete_forbidden_zone_handler(name: str):
    """
    Обрабатывает запрос на удаление запрещенной для полета зоны.

    Args:
        name (str): Имя запрещенной зоны для удаления.

    Returns:
        str: OK в случае успешного удаления или NOT_FOUND.
    """
    forbidden_zones = None
    
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        old_zones = json.load(f)
    
    with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
        forbidden_zones = json.load(f)
        for idx, zone in enumerate(forbidden_zones['features']):
            if zone['properties'].get('name') == name:
                forbidden_zones['features'].pop(idx)
                break
        
    if forbidden_zones is not None:
        with open(FORBIDDEN_ZONES_PATH, 'w', encoding='utf-8') as f:
            json.dump(forbidden_zones, f, ensure_ascii=False, indent=4)
            
        compute_and_save_forbidden_zones_delta(old_zones, forbidden_zones)
        mqtt_publish_forbidden_zones()
        return OK
    
    return NOT_FOUND


def get_delay_handler(id: str):
    """
    Обрабатывает запрос на получение времени до следующего сеанса связи для указанного БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Время до следующего сеанса связи или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        return str(uav_entity.delay)


def set_delay_handler(id: str, delay: int):
    """
    Обрабатывает запрос на установку времени до следующего сеанса связи для указанного БПЛА.

    Args:
        id (str): Идентификатор БПЛА.
        delay (int): Время до следующего сеанса связи.

    Returns:
        str: OK в случае успешной установки или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        uav_entity.delay = delay
        commit_changes()
        flush()
        scheduler.add_interval_task(
            task_name=f"ping_{id}",
            seconds=uav_entity.delay,
            func=mqtt_publish_ping,
            args=(id,)
        )
        return OK


def revise_mission_decision_handler(id: str, decision: int):
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    elif id in context.revise_mission_queue:
        mission_entity = get_entity_by_key(Mission, id)
        if decision == 0:
            uav_entity.is_armed = True
            uav_entity.state = 'В полете'
            mission_entity.is_accepted = True
            commit_changes()
            flush()
            mqtt_send_mission(id)
        else:
            uav_entity.is_armed = False
            uav_entity.state = 'В сети'
            mission_entity.is_accepted = False
            commit_changes()
            flush()
        mqtt_publish_flight_state(id)
        context.revise_mission_queue.remove(id)
        return f'$Arm: {decision}'
    else:
        return '$Arm: -1'

def get_display_mode_handler():
    return '0' if context.display_only else '1'

def toggle_display_mode_handler():
    context.display_only = not context.display_only
    return OK

def get_flight_info_response_mode_handler():
    return '0' if context.flight_info_response else '1'

def toggle_flight_info_response_mode_handler():
    context.flight_info_response = not context.flight_info_response
    return OK

def get_auto_mission_approval_handler():
    return '0' if context.auto_mission_approval else '1'

def toggle_auto_mission_approval_handler():
    context.auto_mission_approval = not context.auto_mission_approval
    return OK

def get_all_data_handler():
    all_data = {}

    uav_entities = Uav.query.order_by(Uav.created_date).all()
    all_data['ids'] = [uav.id for uav in uav_entities]

    all_data['waiters'] = str(len(context.arm_queue))

    all_data['uav_data'] = {}
    for uav in uav_entities:
        uav_data = {}
        
        uav_data['state'] = uav.state

        uav_telemetry_entity = get_entities_by_field_with_order(UavTelemetry, UavTelemetry.uav_id, uav.id, UavTelemetry.record_time.desc()).first()
        if uav_telemetry_entity:
            uav_data['telemetry'] = {
                'lat': uav_telemetry_entity.lat,
                'lon': uav_telemetry_entity.lon,
                'alt': uav_telemetry_entity.alt,
                'azimuth': uav_telemetry_entity.azimuth,
                'dop': uav_telemetry_entity.dop,
                'sats': uav_telemetry_entity.sats,
                'speed': uav_telemetry_entity.speed
            }
        else:
            uav_data['telemetry'] = None

        uav_data['mission_state'] = get_mission_state_handler(uav.id)

        uav_data['delay'] = str(uav.delay)

        all_data['uav_data'][uav.id] = uav_data
    return jsonify(all_data)

def toggle_auto_revoke_permission_handler(enabled: bool):
    context.permission_revoke_enabled = enabled
    return OK

def set_revoke_coords_handler(lat: str, lon: str):
    if lat and lon:
        try:
            context.permission_revoke_coords = {'lat': float(lat), 'lon': float(lon)}
        except ValueError:
            return "Bad coordinates"
    else:
        context.permission_revoke_coords = {}
    return OK

def get_auto_revoke_permission_state_handler():
    return jsonify({
        'enabled': context.permission_revoke_enabled,
        'coords': context.permission_revoke_coords
    })

def toggle_auto_break_connection_handler(enabled: bool):
    context.connection_break_enabled = enabled
    return OK

def set_break_coords_handler(lat: str, lon: str):
    if lat and lon:
        try:
            context.connection_break_coords = {'lat': float(lat), 'lon': float(lon)}
        except ValueError:
            return "Bad coordinates"
    else:
        context.connection_break_coords = {}
    return OK

def get_auto_break_connection_state_handler():
    return jsonify({
        'enabled': context.connection_break_enabled,
        'coords': context.connection_break_coords
    })