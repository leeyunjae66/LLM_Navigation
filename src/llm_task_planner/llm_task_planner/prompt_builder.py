"""Build a model-neutral prompt for the initial single-skill planner."""

import json


def build_prompt(command: str, locations: dict[str, str]) -> str:
    available = [{"name": name, "description": description}
                 for name, description in locations.items()]
    return f"""You convert a user's robot command into exactly one JSON object.
Return only JSON with exactly these string fields: skill, target.
Allowed skill: NAVIGATE or UNKNOWN.
For NAVIGATE, target must exactly match one listed location name. The output schema only permits those names.
Never translate a location into a new identifier, invent locations or skills, or output coordinates.
Use UNKNOWN with an empty target if the requested place is not represented by a listed location or cannot be identified confidently.

Available semantic locations:
{json.dumps(available, ensure_ascii=False)}

User command:
{command}
"""
