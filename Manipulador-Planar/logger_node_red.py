import json
import threading

try:
    import paho.mqtt.client as mqtt
except Exception:
    mqtt = None

try:
    import requests
except Exception:
    requests = None


class LoggerNodeRed:
    def __init__(self, mqtt_broker=None, mqtt_topic=None, http_endpoint=None):
        self.mqtt_broker = mqtt_broker
        self.mqtt_topic = mqtt_topic
        self.http_endpoint = http_endpoint
        self.client = None
        if mqtt_broker and mqtt is not None:
            self.client = mqtt.Client()
            try:
                self.client.connect(self.mqtt_broker, 1883, 60)
                self.client.loop_start()
            except Exception:
                self.client = None

    def send(self, payload: dict):
        data = json.dumps(payload)
        if self.client and self.mqtt_topic:
            try:
                self.client.publish(self.mqtt_topic, data)
            except Exception:
                pass

        if self.http_endpoint:
            # send asynchronously so we don't block simulation loop
            threading.Thread(target=self._post, args=(data,)).start()

    def _post(self, data):
        if requests is None:
            return
        try:
            requests.post(self.http_endpoint, data=data, headers={"Content-Type": "application/json"}, timeout=1.0)
        except Exception:
            pass
