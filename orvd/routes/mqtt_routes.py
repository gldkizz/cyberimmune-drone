import json
from urllib.parse import parse_qs
from context import context
from constants import MQTTTopic, APIRoute, KeyGroup
from extensions import mqtt_client as mqtt
from utils import verify, sign, signed_request
from handlers.general_handlers import fmission_ms_handler
from handlers.api_handlers import (
    telemetry_handler, arm_handler, save_logs_handler, revise_mission_handler,
    save_events_handler
)

def extract_id_from_kwargs(kwargs):
    id = kwargs.get('id')
    if id:
        return id
    else:
        raise Exception("No id provided.")

@mqtt.topic(MQTTTopic.TELEMETRY)
def telemetry(client, userdata, msg, **kwargs):
    try:
        query_string = msg.payload.decode()
        query_params = parse_qs(query_string)
        payload = {k: v[0] for k, v in query_params.items()}
        payload['id'] = extract_id_from_kwargs(kwargs)
        telemetry_handler(**payload)
    except Exception as e:
        print(f"Error handling telemetry message: {e}")

@mqtt.topic(MQTTTopic.FMISSION_MS)
def mission(client, userdata, msg, **kwargs):
    try:
        payload_str = msg.payload.decode()
        payload = json.loads(payload_str)
        payload['id'] = extract_id_from_kwargs(kwargs)
        fmission_ms_handler(**payload)
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON from mission message: {e}. Payload: {payload_str}")
    except Exception as e:
        print(f"Error handling mission message: {e}")


@mqtt.topic(MQTTTopic.ARM_REQUEST)
def arm_request(client, userdata, msg, **kwargs):
    """
    Обрабатывает запрос на арм от БПЛА.
    """
    try:
        query_string = msg.payload.decode()
        query_params = parse_qs(query_string)
        payload = {k: v[0] for k, v in query_params.items()}
        id = extract_id_from_kwargs(kwargs)
        
        response = signed_request(handler_func=arm_handler, verifier_func=verify, signer_func=sign,
                            query_str=f"{APIRoute.ARM}?id={id}", key_group=f"{KeyGroup.KOS}{id}", sig=payload['sig'], id=id)
        if not context.flight_info_response:
            return
        if len(response) == 2 and response[1] == 200:
            mqtt.publish_message(MQTTTopic.ARM_RESPONSE.format(id=id), response[0])
    except Exception as e:
        print(f"Error handling mission message: {e}")

@mqtt.topic(MQTTTopic.LOGS)
def save_logs(client, userdata, msg, **kwargs):
    """
    Сохраняет лог для указанного БПЛА.
    """
    query_string = msg.payload.decode()
    query_params = parse_qs(query_string)
    payload = {k: v[0] for k, v in query_params.items()}
    payload['id'] = extract_id_from_kwargs(kwargs)
    save_logs_handler(**payload)
    
@mqtt.topic(MQTTTopic.EVENTS)
def save_events(client, userdata, msg, **kwargs):
    """
    Сохраняет событие для указанного БПЛА.
    """
    payload = {
        'log_message': msg.payload.decode(),
        'id': extract_id_from_kwargs(kwargs)
    }
    save_events_handler(**payload)

@mqtt.topic(MQTTTopic.NMISSION_REQUEST)
def revise_mission(client, userdata, msg, **kwargs):
    try:
        query_string = msg.payload.decode()
        query_params = parse_qs(query_string)
        payload = {k: v[0] for k, v in query_params.items()}
        id = extract_id_from_kwargs(kwargs)
        
        response = signed_request(handler_func=revise_mission_handler, verifier_func=verify, signer_func=sign,
                                    query_str=f"{APIRoute.NMISSION}?id={id}&mission={payload.get('mission')}",
                                    key_group=f'{KeyGroup.KOS}{id}', sig=payload['sig'], id=id, mission=payload.get('mission'))
        if not context.flight_info_response:
            return
        if len(response) == 2 and response[1] == 200:
            mqtt.publish_message(MQTTTopic.NMISSION_RESPONSE.format(id=id), response[0])
    except Exception as e:
        print(f"Error handling mission message: {e}")