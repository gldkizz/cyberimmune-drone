import datetime
import json
import os
import time
from context import context
from extensions import task_scheduler_client as scheduler
from constants import (
    ARMED, DISARMED, NOT_FOUND, OK, LOGS_PATH,
    FORBIDDEN_ZONES_PATH, FORBIDDEN_ZONES_DELTA_PATH,
)
from db.dao import (
    add_and_commit, add_changes, commit_changes, delete_entity, get_entity_by_key,
    get_entities_by_field, get_entities_by_field_with_order, save_public_key,
    get_key, flush, save_event
)
from db.models import UavTelemetry, MissionStep, Mission, UavPublicKeys, Uav
from utils import (
    cast_wrapper, generate_forbidden_zones_string,
    get_sha256_hex
)
from .mqtt_handlers import mqtt_publish_flight_state, mqtt_publish_ping, mqtt_publish_forbidden_zones, mqtt_publish_auth
    
def key_kos_exchange_handler(id: str, n: str, e: str):
    """
    Обрабатывает обмен ключами с KOS.

    Args:
        id (str): Идентификатор БПЛА.
        n (str): Модуль открытого ключа.
        e (str): Экспонента открытого ключа.

    Returns:
        str: Строка с открытым ключом ORVD.
    """
    n, e = str(int(n, 16)), str(int(e, 16))
    key_entity = get_entity_by_key(UavPublicKeys, id)
    if key_entity is None:
        save_public_key(n, e, f'kos{id}')
    orvd_n, orvd_e = get_key('orvd', private=False)
    str_to_send = f'$Key: {hex(orvd_n)[2:]} {hex(orvd_e)[2:]}'
    return str_to_send


def auth_handler(id: str):
    """
    Обрабатывает аутентификацию БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Строка подтверждения аутентификации.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        uav_entity = Uav(id=id, is_armed=False, state='В сети', kill_switch_state=False)
        add_and_commit(uav_entity)
    else:
        uav_entity.is_armed = False
        uav_entity.state = 'В сети'
        uav_entity.kill_switch_state = False
        commit_changes()
        
    flush()
    mqtt_publish_flight_state(id)
    scheduler.add_interval_task(
        task_name=f"ping_{id}",
        seconds=uav_entity.delay,
        func=mqtt_publish_ping,
        args=(id,)
    )
    mqtt_publish_forbidden_zones()
    mqtt_publish_auth(id)
    
    return f'$Auth id={id}'

def arm_handler(id: str, **kwargs):
    """
    Обрабатывает запрос на арм БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Статус арма БПЛА.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    elif uav_entity.is_armed:
        return f'$Arm {ARMED}$Delay {uav_entity.delay}' 
    else:
        mission = get_entity_by_key(Mission, id)
        if mission and mission.is_accepted:
            context.arm_queue.add(id)
            uav_entity.state = 'Ожидает'
            commit_changes()
            decision = _arm_wait_decision(id)
            if decision == ARMED:
                uav_entity.state = 'В полете'
            else:
                uav_entity.state = 'В сети'
            commit_changes()
            flush()
            mqtt_publish_flight_state(id)
            return f'$Arm {decision}$Delay {uav_entity.delay}'
        else:
            return f'$Arm {DISARMED}$Delay {uav_entity.delay}'


def _arm_wait_decision(id: str):
    """
    Ожидает решения об арме БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Решение об арме (ARMED или DISARMED).
    """
    while id in context.arm_queue:
        time.sleep(0.1)
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity.is_armed:
        return ARMED
    else:
        return DISARMED


def flight_info_handler(id: str) -> str:
    """
    Обрабатывает запрос на проверку информации полета БПЛА.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Состояние полета БПЛА.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return NOT_FOUND
    else:
        forbidden_zones_hash = get_forbidden_zones_hash_handler(id=id)
        delay = f'$Delay {uav_entity.delay}'
        if uav_entity.kill_switch_state:
            status = '$Flight -1'
        elif uav_entity.is_armed:
            status = '$Flight 0'
        else:
            status = '$Flight 1'
        return ''.join([status, forbidden_zones_hash, delay])


def telemetry_handler(id: str, lat: float, lon: float, alt: float,
                      azimuth: float, dop: float, sats: float, speed: float, **kwargs):
    """
    Обрабатывает телеметрию БПЛА.

    Args:
        id (str): Идентификатор БПЛА.
        lat (float): Широта.
        lon (float): Долгота.
        alt (float): Высота.
        azimuth (float): Азимут.
        dop (float): Снижение точности.
        sats (float): Количество спутников.
        speed (float): Скорость.

    Returns:
        str: Статус арма БПЛА.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity and context.display_only:
        uav_entity = Uav(id=id, is_armed=False, state='В сети', kill_switch_state=False)
        add_and_commit(uav_entity)
        
    if not uav_entity:
        return NOT_FOUND
    else:
        lat = cast_wrapper(lat, float)
        if lat:
            lat /= 1e7
        lon = cast_wrapper(lon, float)
        if lon:
            lon /= 1e7
        alt = cast_wrapper(alt, float)
        if alt:
            alt /= 1e2
        azimuth = cast_wrapper(azimuth, float)
        if azimuth:
            azimuth /= 1e7
        dop = cast_wrapper(dop, float)
        sats = cast_wrapper(sats, int)
        speed = cast_wrapper(speed, float)
        record_time = datetime.datetime.now(datetime.timezone.utc)
        uav_telemetry_entity = get_entity_by_key(UavTelemetry, (uav_entity.id, record_time))
        if not uav_telemetry_entity:
            uav_telemetry_entity = UavTelemetry(uav_id=uav_entity.id, lat=lat, lon=lon, alt=alt,
                                                azimuth=azimuth, dop=dop, sats=sats, speed=speed, record_time=record_time)
            add_and_commit(uav_telemetry_entity)
        else:
            uav_telemetry_entity.lat = lat
            uav_telemetry_entity.lon = lon
            uav_telemetry_entity.alt = alt
            uav_telemetry_entity.azimuth = azimuth
            uav_telemetry_entity.dop = dop
            uav_telemetry_entity.sats = sats
            uav_telemetry_entity.speed = speed
            commit_changes()
        if not uav_entity.is_armed:
            return f'$Arm: {DISARMED}'
        else:
            return f'$Arm: {ARMED}'
    

def fmission_kos_handler(id: str):
    """
    Обрабатывает запрос на получение полетного задания для KOS.

    Args:
        id (str): Идентификатор БПЛА.

    Returns:
        str: Строка с полетным заданием или NOT_FOUND.
    """
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        mission = get_entity_by_key(Mission, id)
        if mission and mission.is_accepted:
            mission_steps = get_entities_by_field_with_order(MissionStep, MissionStep.mission_id, id, order_by_field=MissionStep.step)
            if mission_steps and mission_steps.count() != 0:
                mission_steps = list(map(lambda e: e.operation, mission_steps))
                return f'$FlightMission {"&".join(mission_steps)}'
    return NOT_FOUND

            
def get_all_forbidden_zones_handler(*args, **kwargs):
    """
    Обрабатывает запрос на получение всех запрещенных для полета зон.

    Returns:
        str: Строка с информацией о запрещенных зонах или NOT_FOUND.
    """
    try:
        with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
            forbidden_zones = json.load(f)
            result_str = generate_forbidden_zones_string(forbidden_zones)
            return result_str

    except Exception as e:
        print(e)
        return NOT_FOUND


def get_forbidden_zones_delta_handler(*args, **kwargs):
    """
    Обрабатывает запрос на получение дельты изменений в запрещенных для полета зонах.

    Returns:
        str: Строка с дельтой изменений в запрещенных зонах или NOT_FOUND.
    """
    try:
        with open(FORBIDDEN_ZONES_DELTA_PATH, 'r', encoding='utf-8') as f:
            delta_zones = json.load(f)
        
        delta_str = f'$ForbiddenZonesDelta {len(delta_zones["features"])}'
        for zone in delta_zones['features']:
            name = zone['properties']['name']
            change_type = zone['properties']['change_type']
            coordinates = zone['geometry']['coordinates'][0]
            delta_str += f'&{name}&{change_type}&{len(coordinates)}&{"&".join(list(map(lambda e: f"{e[1]:.7f}_{e[0]:.7f}", coordinates)))}'
        
        return delta_str
    except Exception as e:
        print(e)
        return NOT_FOUND
    

def get_forbidden_zones_hash_handler(*args, **kwargs):
    """
    Обрабатывает запрос на получение SHA-256 хэша строки запрещенных зон.

    Returns:
        str: SHA-256 хэш строки запрещенных зон или NOT_FOUND.
    """
    try:
        with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
            forbidden_zones = json.load(f)
            result_str = generate_forbidden_zones_string(forbidden_zones)
            hash_value = get_sha256_hex(result_str)
            return f'$ForbiddenZonesHash {hash_value}'

    except Exception as e:
        print(e)
        return NOT_FOUND


def revise_mission_handler(id: str, mission: str, **kwargs):
    mission_list = mission.split('*')
    
    mission_entity = get_entity_by_key(Mission, id)
    if mission_entity:
        get_entities_by_field(MissionStep, MissionStep.mission_id, id).delete()
        delete_entity(mission_entity)
        commit_changes()
    
    mission_entity = Mission(uav_id=id, is_accepted=context.auto_mission_approval)
    add_changes(mission_entity)
    for idx, cmd in enumerate(mission_list):
        mission_step_entity = MissionStep(mission_id=id, step=idx, operation=cmd)
        add_changes(mission_step_entity)
    commit_changes()
    
    if context.auto_mission_approval:
        return '$Approve 0'

    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        uav_entity.is_armed = False
        uav_entity.state = 'Ожидает'
        commit_changes()
        flush()
        mqtt_publish_flight_state(id)
        
    context.revise_mission_queue.add(id)
    while id in context.revise_mission_queue:
        time.sleep(0.1)
        
    mission_entity = get_entity_by_key(Mission, id)
    if mission_entity:
        if mission_entity.is_accepted:
            return '$Approve 0'
        else:
            return '$Approve 1'
    return '$Approve 1'


def save_logs_handler(id: str, log: str, **kwargs):
    """
    Обрабатывает запрос на сохранение логов БПЛА.

    Args:
        id (str): Идентификатор БПЛА.
        log (str): Строка с логами для сохранения.

    Returns:
        str: OK в случае успешного сохранения.
    """
    try:
        if not os.path.exists(LOGS_PATH):
            os.makedirs(LOGS_PATH)
        with open(f'{LOGS_PATH}/{id}.txt', 'a') as f:
            f.write(f'\n{log}')
    except Exception as e:
        print(e)
    return OK

def save_events_handler(id: str, log_message: str, **kwargs):
    """
    Обрабатывает запрос на сохранение событий БПЛА в БД и логах.

    Args:
        id (str): Идентификатор БПЛА.
        log_string (str): Строка с логом для сохранения.

    Returns:
        str: OK в случае успешного сохранения.
    """
    try:
        save_event(uav_id=id, log_message=log_message)
        save_logs_handler(id=id, log=log_message)
    except Exception as e:
        print(e)
    return OK