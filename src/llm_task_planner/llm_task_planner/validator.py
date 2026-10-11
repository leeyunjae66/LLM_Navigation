"""Deterministic execution-safety checks for model-produced plans."""

from .schemas import RobotPlan

ALLOWED_SKILLS = {"NAVIGATE", "UNKNOWN"}


class PlanValidationError(ValueError):
    pass


def validate_plan(plan: RobotPlan, locations: set[str]) -> RobotPlan:
    skill = plan.skill.strip().upper()
    target = plan.target.strip()
    if skill not in ALLOWED_SKILLS:
        raise PlanValidationError(f"Unsupported skill: {plan.skill!r}")
    if skill == "UNKNOWN":
        if target:
            raise PlanValidationError("UNKNOWN plans must have an empty target")
        return RobotPlan(skill="UNKNOWN", target="")
    if not target:
        raise PlanValidationError("NAVIGATE requires a target")
    if target not in locations:
        raise PlanValidationError(f"Target is not in semantic map: {target!r}")
    return RobotPlan(skill="NAVIGATE", target=target)
