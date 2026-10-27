"""Preprocessing, TwinLiteNetPlus inference and probability postprocessing."""

import time

import cv2
import numpy as np
import torch
import torch.nn.functional as functional

from .model import DrivableTwinLiteNetPlus


def image_message_to_bgr(message):
    """Convert the supported color sensor_msgs/Image encodings to BGR."""
    height, width, step = message.height, message.width, message.step
    encoding = message.encoding.lower()
    formats = {
        "bgr8": (3, False),
        "rgb8": (3, True),
        "bgra8": (4, False),
        "rgba8": (4, True),
    }
    if encoding in formats:
        channels, rgb_order = formats[encoding]
        rows = np.frombuffer(message.data, np.uint8).reshape(height, step)
        image = rows[:, :width * channels].reshape(height, width, channels)
        if channels == 4:
            conversion = cv2.COLOR_RGBA2BGR if rgb_order else cv2.COLOR_BGRA2BGR
            return cv2.cvtColor(image, conversion)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if rgb_order else image.copy()
    if encoding in ("mono8", "8uc1"):
        rows = np.frombuffer(message.data, np.uint8).reshape(height, step)
        return cv2.cvtColor(rows[:, :width], cv2.COLOR_GRAY2BGR)
    raise ValueError(
        "unsupported color encoding '%s'; expected rgb8, bgr8, rgba8, bgra8 or mono8" %
        message.encoding)


def letterbox(image, image_size, color=(114, 114, 114)):
    """Resize while preserving aspect ratio and pad to a multiple of 32."""
    original_height, original_width = image.shape[:2]
    ratio = min(image_size / original_height, image_size / original_width)
    resized_width = int(round(original_width * ratio))
    resized_height = int(round(original_height * ratio))
    pad_width = (image_size - resized_width) % 32
    pad_height = (image_size - resized_height) % 32
    half_width = pad_width / 2
    half_height = pad_height / 2
    if (resized_width, resized_height) != (original_width, original_height):
        image = cv2.resize(
            image, (resized_width, resized_height), interpolation=cv2.INTER_AREA)
    top = int(round(half_height - 0.1))
    bottom = int(round(half_height + 0.1))
    left = int(round(half_width - 0.1))
    right = int(round(half_width + 0.1))
    image = cv2.copyMakeBorder(
        image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return image, (left, top, right, bottom)


class InferenceEngine:
    def __init__(self, model_config, weight_path, image_size=640,
                 device="auto", use_half=True, enable_lane=False):
        self.image_size = image_size
        self.enable_lane = bool(enable_lane)
        if image_size <= 0 or image_size % 32 != 0:
            raise ValueError("image_size must be a positive multiple of 32")
        if device in ("", "auto"):
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA device requested but torch.cuda.is_available() is false")

        model = DrivableTwinLiteNetPlus(model_config, enable_lane=self.enable_lane)
        try:
            checkpoint = torch.load(weight_path, map_location=self.device, weights_only=True)
        except TypeError:
            checkpoint = torch.load(weight_path, map_location=self.device)
        if isinstance(checkpoint, dict) and "state_dict" in checkpoint:
            checkpoint = checkpoint["state_dict"]
        incompatible = model.load_state_dict(checkpoint, strict=False)
        if incompatible.missing_keys:
            raise RuntimeError(
                "checkpoint misses drivable model parameters: %s" %
                ", ".join(incompatible.missing_keys[:8]))
        allowed_prefixes = ("up_1_ll.", "up_2_ll.", "out_ll.")
        unexpected = [
            key for key in incompatible.unexpected_keys
            if not key.startswith(allowed_prefixes)
        ]
        if unexpected:
            raise RuntimeError(
                "checkpoint contains incompatible parameters: %s" %
                ", ".join(unexpected[:8]))

        self.model = model.to(self.device).eval()
        self.use_half = bool(use_half and self.device.type == "cuda")
        if self.use_half:
            self.model.half()
        warmup = torch.zeros(
            (1, 3, image_size, image_size), device=self.device,
            dtype=torch.float16 if self.use_half else torch.float32)
        with torch.inference_mode():
            self.model(warmup)

    def infer(self, bgr_image):
        original_height, original_width = bgr_image.shape[:2]
        resized, padding = letterbox(bgr_image, self.image_size)
        rgb_chw = resized[:, :, ::-1].transpose(2, 0, 1).copy()
        tensor = torch.from_numpy(rgb_chw).unsqueeze(0).to(self.device)
        tensor = tensor.half() if self.use_half else tensor.float()
        tensor /= 255.0

        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        started = time.perf_counter()
        with torch.inference_mode():
            outputs = self.model(tensor)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        inference_ms = (time.perf_counter() - started) * 1000.0

        if self.enable_lane:
            drivable_logits, lane_logits = outputs
        else:
            drivable_logits, lane_logits = outputs, None

        left, top, right, bottom = padding
        height, width = resized.shape[:2]
        drivable_logits = drivable_logits[
            :, :, top:height - bottom, left:width - right]
        drivable_logits = functional.interpolate(
            drivable_logits, size=(original_height, original_width), mode="bilinear",
            align_corners=False)
        drivable_probability = functional.softmax(
            drivable_logits.float(), dim=1)[:, 1]
        drivable_probability = (
            drivable_probability.squeeze(0).cpu().numpy().astype(np.float32))

        lane_probability = None
        if lane_logits is not None:
            lane_logits = lane_logits[:, :, top:height - bottom, left:width - right]
            lane_logits = functional.interpolate(
                lane_logits, size=(original_height, original_width), mode="bilinear",
                align_corners=False)
            lane_probability = functional.softmax(lane_logits.float(), dim=1)[:, 1]
            lane_probability = (
                lane_probability.squeeze(0).cpu().numpy().astype(np.float32))
            lane_probability = np.ascontiguousarray(lane_probability)
        return (np.ascontiguousarray(drivable_probability), lane_probability,
                inference_ms)
