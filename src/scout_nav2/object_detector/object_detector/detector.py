"""Object-detector abstraction.

The detector is hidden behind a small interface so the ROS node and the
projection/DB/marker pipeline never import torch directly. The default
implementation is YOLOE (ultralytics; the open-vocabulary successor to
YOLO-World, same ultralytics API). ultralytics is imported lazily inside the
constructor so the rest of the package (and its unit tests) work without torch
installed.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence


@dataclass
class Detection:
    """A single 2D detection in image coordinates."""

    label: str
    confidence: float
    # Axis-aligned bounding box in pixels (x1, y1, x2, y2).
    bbox: tuple


class BaseDetector:
    """Interface every detector implementation must satisfy."""

    def detect(self, rgb) -> List[Detection]:  # pragma: no cover - interface
        raise NotImplementedError


class YoloeDetector(BaseDetector):
    """Open-vocabulary detector using ultralytics YOLOE.

    `ultralytics` is imported here (not at module import time) so that
    `import object_detector.detector` never requires torch. Weights are loaded
    from `model_path` if present, otherwise ultralytics downloads them once at
    construction time (never inside the inference loop).

    Text vocabulary handling:
      * PyTorch weights (`.pt`): the vocabulary is set at runtime via
        `set_classes(names, get_text_pe(names))` (YOLOE's text-embedding form).
      * TensorRT engines (`.engine`): the vocabulary is fixed into the engine at
        export time, so set_classes is skipped (see SPEC 2.2 deployment note).
    """

    def __init__(
        self,
        model_path: str,
        target_classes: Sequence[str],
        device: str = "",
        imgsz: int = 640,
        prompts: Sequence[str] = None,
    ):
        try:
            from ultralytics import YOLOE  # noqa: WPS433 (intentional lazy import)
        except ImportError as exc:  # pragma: no cover - depends on env
            raise ImportError(
                "ultralytics is required for YoloeDetector. "
                "Install it (pulls torch) with: pip install ultralytics"
            ) from exc

        # Inference device: "" -> auto (cuda:0 if a CUDA torch build sees a GPU,
        # else cpu). Pass e.g. "cuda:0" or "cpu" to force.
        self._device = self._resolve_device(device)
        # Inference resolution. Larger (e.g. 1280) greatly improves recall on
        # small/distant objects at the cost of speed (cheap on GPU).
        self._imgsz = int(imgsz)

        # Resolve the YOLOE text encoder (mobileclip_blt.ts, ~570MB) next to the
        # model weights so it is found regardless of process cwd. ultralytics'
        # attempt_download_asset checks cwd then SETTINGS["weights_dir"]; pointing
        # weights_dir at the model's own dir avoids a silent re-download.
        import os

        from ultralytics.utils import SETTINGS  # noqa: WPS433 (lazy)

        weights_dir = os.path.dirname(os.path.abspath(model_path))
        if weights_dir and SETTINGS.get("weights_dir") != weights_dir:
            SETTINGS.update({"weights_dir": weights_dir})

        # Canonical labels stored/queried downstream (e.g. "fire extinguisher").
        self._labels = list(target_classes)
        # Detection prompts fed to YOLOE. These may be richer/visual descriptions
        # that match the sim renders better than the canonical label (e.g. the
        # sim extinguisher mesh scores far higher on "red metal cylinder" than on
        # "fire extinguisher"). cls_id indexes this list; we map it back to the
        # parallel canonical label so the stored label stays clean.
        self._prompts = list(prompts) if prompts else list(self._labels)
        # model_path may be a local file or a known weight name that ultralytics
        # resolves/downloads (e.g. "yoloe-11s-seg.pt"). Either way this runs once
        # at startup, outside the callback/worker loop.
        self._model = YOLOE(model_path)
        self._is_engine = str(model_path).endswith(".engine")
        # Move PyTorch weights to the chosen device (engines carry their own).
        if not self._is_engine:
            try:
                self._model.to(self._device)
            except Exception:  # pragma: no cover - device/runtime dependent
                pass
        if self._prompts and not self._is_engine:
            self._set_classes(self._prompts)

    @staticmethod
    def _resolve_device(device: str) -> str:
        if device:
            return device
        try:
            import torch  # noqa: WPS433 (lazy)

            return "cuda:0" if torch.cuda.is_available() else "cpu"
        except ImportError:  # pragma: no cover
            return "cpu"

    def _set_classes(self, names):
        """Apply the open-vocabulary classes, tolerating API variants.

        YOLOE's documented form takes precomputed text embeddings
        (`set_classes(names, get_text_pe(names))`); some builds also accept the
        names-only form. Try the embedding form first, fall back to names-only.
        """
        try:
            self._model.set_classes(names, self._model.get_text_pe(names))
        except TypeError:
            self._model.set_classes(names)

    @property
    def device(self) -> str:
        return self._device

    def detect(self, rgb) -> List[Detection]:
        # verbose=False keeps the per-frame ultralytics logging quiet.
        results = self._model.predict(
            rgb, device=self._device, imgsz=self._imgsz, verbose=False
        )
        detections: List[Detection] = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0])
                label = self._canonical_label(result, cls_id)
                conf = float(box.conf[0])
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
                detections.append(
                    Detection(label=label, confidence=conf, bbox=(x1, y1, x2, y2))
                )
        return detections

    def _canonical_label(self, result, cls_id: int) -> str:
        # Map the matched prompt index back to the canonical label; fall back to
        # the model's own class names (e.g. for a TensorRT engine).
        if 0 <= cls_id < len(self._labels):
            return self._labels[cls_id]
        names = getattr(result, "names", None)
        if isinstance(names, dict):
            return names.get(cls_id, str(cls_id))
        if isinstance(names, (list, tuple)) and 0 <= cls_id < len(names):
            return names[cls_id]
        return str(cls_id)
