"""TwinLiteNetPlus-based drivable-area inference for plan_env."""

__all__ = ["DrivableTwinLiteNetPlus", "InferenceEngine"]


def __getattr__(name):
    if name == "DrivableTwinLiteNetPlus":
        from .model import DrivableTwinLiteNetPlus
        return DrivableTwinLiteNetPlus
    if name == "InferenceEngine":
        from .inference import InferenceEngine
        return InferenceEngine
    raise AttributeError(name)
