# FILE: alignment_core/utils/telemetry.py
import json
import time
from datetime import datetime

class TelemetryLogger:
    def __init__(self, log_to_file=True):
        self.log_to_file = log_to_file
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = f"logs/telemetry_{self.session_id}.jsonl"
        
        if self.log_to_file:
            import os
            os.makedirs("logs", exist_ok=True)

    def record(self, intent, audit_result):
        """Captures a single 'frame' of AI thinking."""
        telemetry_frame = {
            "timestamp": time.time(),
            "intent": intent,
            "decision": {
                "authorized": audit_result["authorized"],
                "summary": audit_result["summary"]
            },
            "physics_state": audit_result["kernels"]
        }

        # Print a clean summary to the console
        status = "✅ PASS" if audit_result["authorized"] else "❌ VETO"
        print(f"[{status}] V:{intent['v']:.1f}m/s | R:{intent['r']:.1f}m | {audit_result['summary']}")

        if self.log_to_file:
            with open(self.log_path, "a") as f:
                f.write(json.dumps(telemetry_frame) + "\n")