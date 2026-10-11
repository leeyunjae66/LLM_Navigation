"""Build a model-neutral prompt for the initial single-skill planner."""

import json


def build_prompt(command: str, locations: dict[str, str]) -> str:
    available = [{"name": name, "description": description}
                 for name, description in locations.items()]
    return f"""You convert a user's robot command into exactly one JSON object.
Return only JSON with exactly these string fields: skill, target.
Allowed skill: NAVIGATE or UNKNOWN.
Choose a target only from the available semantic locations below.
Never output coordinates or invent skills or locations.
If the command cannot be mapped confidently to one available location, return {{"skill":"UNKNOWN","target":""}}.

Available semantic locations:
{json.dumps(available, ensure_ascii=False)}

User command:
{command}
"""
