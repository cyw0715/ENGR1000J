import json
import urllib.request

IP = "192.168.137.19"
for endpoint in ("health", "json"):
    with urllib.request.urlopen(f"http://{IP}/{endpoint}", timeout=5) as response:
        payload = json.loads(response.read().decode("utf-8"))
    print(endpoint.upper() + "=" + json.dumps(payload, separators=(",", ":")))
