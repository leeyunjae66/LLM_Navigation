"""Provider interface and local Ollama implementation for plan generation."""

import json
import urllib.error
import urllib.request
from typing import Protocol


class LLMClient(Protocol):
    def generate_plan(self, prompt: str) -> str:
        """Return the model response containing a plan JSON object."""


class OllamaClient:
    def __init__(self, base_url: str, model: str, timeout_sec: float = 60.0):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.timeout_sec = timeout_sec

    def generate_plan(self, prompt: str) -> str:
        payload = json.dumps({
            "model": self.model,
            "prompt": prompt,
            "stream": False,
            "format": "json",
            "options": {"temperature": 0},
        }).encode("utf-8")
        request = urllib.request.Request(
            f"{self.base_url}/api/generate",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                body = json.loads(response.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"Ollama request failed: {exc}") from exc
        text = body.get("response") if isinstance(body, dict) else None
        if not isinstance(text, str):
            raise RuntimeError("Ollama response did not contain a text response")
        return text
