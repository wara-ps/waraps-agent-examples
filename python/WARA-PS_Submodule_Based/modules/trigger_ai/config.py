import os
class TriggerAIConfig:
    
    WARAPS_BROKER: str     = "broker.waraps.org"
    WARAPS_PORT: int       = 8883

    WARAPS_TLS_CONNECTION: bool = True
    WARAPS_USERNAME: str   = os.getenv('WARAPS_USERNAME')
    WARAPS_PASSWORD: str   = os.getenv('WARAPS_PASSWORD')

    PUSH_VIDEOSTREAM_TOPIC: str = "watch/push/start"
    PUSH_VIDEOSTREAM_LISTEN_BASETOPIC = f"waraps/service/virtual/real/watch/sensors"
    START_AI_TOPIC: str = "p2w-test/tasks"