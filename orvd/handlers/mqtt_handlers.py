import json
from extensions import mqtt_client as mqtt
from context import context
from utils import sign, generate_forbidden_zones_string
from db.dao import get_entity_by_key, get_entities_by_field_with_order
from db.models import Uav, Mission, MissionStep
from constants import MQTTTopic, KeyGroup, FORBIDDEN_ZONES_PATH

def mqtt_publish_flight_state(id: str, *args, **kwargs):
    if not context.flight_info_response:
        return
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return
    else:
        if uav_entity.kill_switch_state:
            message = '$Flight -1'
        elif uav_entity.is_armed:
            message = '$Flight 0'
        else:
            message = '$Flight 1'
            
    message = f'{message}#{hex(sign(message, KeyGroup.ORVD))[2:]}'
    mqtt.publish_message(MQTTTopic.FLIGHT_STATUS.format(id=id), message)
    
def mqtt_publish_ping(id: str, *args, **kwargs):
    if not context.flight_info_response:
        return
    uav_entity = get_entity_by_key(Uav, id)
    if not uav_entity:
        return
    else:
        message = f'$Delay {uav_entity.delay}'
    message = f'{message}#{hex(sign(message, KeyGroup.ORVD))[2:]}'
    mqtt.publish_message(MQTTTopic.PING.format(id=id), message)
    
def mqtt_publish_auth(id: str, *args, **kwargs):
    if not context.flight_info_response:
        return
    message = f'$Auth {id}'
    signed_message = f'{message}#{hex(sign(message, KeyGroup.ORVD))[2:]}'
    mqtt.publish_message(MQTTTopic.AUTH.format(id=id), signed_message)

def mqtt_publish_forbidden_zones(*args, **kwargs):
    if not context.flight_info_response:
        return
    try:
        with open(FORBIDDEN_ZONES_PATH, 'r', encoding='utf-8') as f:
            forbidden_zones = json.load(f)
            message = generate_forbidden_zones_string(forbidden_zones)
            message = f'{message}#{hex(sign(message, KeyGroup.ORVD))[2:]}'
            mqtt.publish_message(MQTTTopic.FORBIDDEN_ZONES, message)

    except Exception as e:
        print(e)
        return
    
def mqtt_send_mission(id: str, *args, **kwargs):
    if not context.flight_info_response:
        return
    uav_entity = get_entity_by_key(Uav, id)
    if uav_entity:
        mission = get_entity_by_key(Mission, id)
        if mission and mission.is_accepted:
            mission_steps = get_entities_by_field_with_order(MissionStep, MissionStep.mission_id, id, order_by_field=MissionStep.step)
            if mission_steps and mission_steps.count() != 0:
                mission_steps = list(map(lambda e: e.operation, mission_steps))
                message = f'$FlightMission {"&".join(mission_steps)}'
                message = f'{message}#{hex(sign(message, KeyGroup.ORVD))[2:]}'
                mqtt.publish_message(MQTTTopic.FMISSION_KOS.format(id=id), message)
                
def mqtt_publish_connection_status(*args, **kwargs):
    message = str(context.flight_info_response)
    mqtt.publish_message(MQTTTopic.CONNECTION_STATUS, message)
