import os
import re
import paho.mqtt.client as mqtt
from paho.mqtt.client import topic_matches_sub

class MQTTClientWrapper:
    def __init__(self, app=None):
        self.client = None
        self._topic_handlers = {}
        self.app = app

    def _generate_mqtt_subscription_pattern(self, user_pattern):
        mqtt_pattern = re.sub(r'\{[^}]+\}', '+', user_pattern)
        return mqtt_pattern

    def topic(self, user_pattern):
        def decorator(handler_func):
            mqtt_subscription_pattern = self._generate_mqtt_subscription_pattern(user_pattern)
            self._topic_handlers[user_pattern] = {
                'mqtt_pattern': mqtt_subscription_pattern,
                'handler': handler_func
            }
            return handler_func
        return decorator

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected successfully to MQTT Broker.")
            for user_pattern, details in self._topic_handlers.items():
                mqtt_pattern_to_subscribe = details['mqtt_pattern']
                client.subscribe(mqtt_pattern_to_subscribe)
                print(f"Subscribed to MQTT pattern: {mqtt_pattern_to_subscribe} (derived from user pattern: {user_pattern})")
        else:
            print(f"Failed to connect to MQTT Broker, return code {rc}")

    def _on_message(self, client, userdata, msg):
        message_handled = False
        for user_pattern, details in self._topic_handlers.items():
            mqtt_subscription_pattern = details['mqtt_pattern']
            handler_func = details['handler']

            if topic_matches_sub(mqtt_subscription_pattern, msg.topic):
                topic_parts = msg.topic.split('/')
                user_pattern_parts = user_pattern.split('/')
                
                wildcards = []
                hash_path_parts = []
                
                path_params = {}

                for i, part in enumerate(user_pattern_parts):
                    if part == '+':
                        if i < len(topic_parts):
                            wildcards.append(topic_parts[i])
                    elif part == '#':
                        if i < len(topic_parts):
                            hash_path_parts = topic_parts[i:]
                        break
                    elif part.startswith('{') and part.endswith('}'):
                        param_name = part[1:-1]
                        if i < len(topic_parts):
                            path_params[param_name] = topic_parts[i]
                
                kwargs = {}
                if wildcards:
                    kwargs['wildcards'] = wildcards
                if hash_path_parts:
                    kwargs['hash_path'] = "/".join(hash_path_parts)
                kwargs.update(path_params)

                try:
                    if self.app:
                        with self.app.app_context():
                            handler_func(client, userdata, msg, **kwargs)
                    else:
                        handler_func(client, userdata, msg, **kwargs)
                    message_handled = True
                    break
                except Exception as e:
                    print(f"Error processing message on topic {msg.topic} with handler for user pattern {user_pattern}: {e}")
        
        if not message_handled:
            print(f"No suitable handler registered or error in all handlers for topic {msg.topic}")

    def init_client(self):
        MQTT_BROKER = os.environ.get("MQTT_HOST", "localhost")
        MQTT_PORT = int(os.environ.get("MQTT_PORT", 1883))
        MQTT_CLIENT_ID = os.environ.get("MQTT_CLIENT_ID", f"orvd_server_mqtt_client_{os.getpid()}")
        MQTT_USERNAME = os.environ.get("MQTT_USERNAME")
        MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD")

        self.client = mqtt.Client(client_id=MQTT_CLIENT_ID)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.username_pw_set(username=MQTT_USERNAME, password=MQTT_PASSWORD)

        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            print(f"Attempting to connect to MQTT Broker at {MQTT_BROKER}:{MQTT_PORT}")
        except Exception as e:
            print(f"Failed to connect to MQTT Broker: {e}")
            return False

        self.client.loop_start()
        print("MQTT client loop started.")
        return True

    def publish_message(self, topic, payload, qos=0, retain=False):
        if self.client and self.client.is_connected():
            self.client.publish(topic, payload, qos, retain)
            print(f"Published message to topic {topic}: {payload}")
        else:
            print("Client not connected. Cannot publish message.")
                
    def disconnect(self):
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            print("MQTT client disconnected.")

    def init_app(self, app):
        self.app = app

        if self.init_client():
            if app:
                app.mqtt_client_wrapper = self
        else:
            print("MQTT client initialization failed.")
        return self
