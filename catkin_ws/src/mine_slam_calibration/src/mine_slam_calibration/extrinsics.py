import math
from typing import Dict, Iterable, List, Set

import yaml


def load_extrinsics(path: str) -> Dict:
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return data or {}


def audit_extrinsics(data: Dict, required_frames: Iterable[str] = None) -> List[str]:
    issues: List[str] = []
    frames = data.get("frames", {})
    transforms = data.get("transforms", [])
    if not isinstance(frames, dict) or not frames:
        issues.append("frames must be a non-empty mapping")
        frames = {}
    if not isinstance(transforms, list) or not transforms:
        issues.append("transforms must be a non-empty list")
        transforms = []

    declared_frames: Set[str] = set(frames.keys())
    required = list(required_frames or data.get("required_frames", []))
    for frame in required:
        if frame not in declared_frames:
            issues.append("required frame '{}' is not declared".format(frame))

    child_to_parent = {}
    for index, transform in enumerate(transforms):
        if not isinstance(transform, dict):
            issues.append("transform[{}] must be a mapping".format(index))
            continue

        parent = transform.get("parent")
        child = transform.get("child")
        if not _valid_frame_name(parent):
            issues.append("transform[{}].parent is missing or invalid".format(index))
        if not _valid_frame_name(child):
            issues.append("transform[{}].child is missing or invalid".format(index))
        if parent == child and parent is not None:
            issues.append("transform[{}] has identical parent and child".format(index))

        for frame, role in ((parent, "parent"), (child, "child")):
            if _valid_frame_name(frame) and frame not in declared_frames:
                issues.append("transform[{}].{} '{}' is not declared".format(index, role, frame))

        if _valid_frame_name(child):
            if child in child_to_parent:
                issues.append(
                    "frame '{}' has multiple parents: '{}' and '{}'".format(
                        child, child_to_parent[child], parent
                    )
                )
            child_to_parent[child] = parent

        _audit_vector(transform, index, "translation", 3, issues)
        _audit_vector(transform, index, "rotation_rpy", 3, issues)

    roots = declared_frames - set(child_to_parent.keys())
    reference = data.get("calibration", {}).get("reference_frame")
    if reference and reference not in declared_frames:
        issues.append("reference_frame '{}' is not declared".format(reference))
    if reference and reference not in roots:
        issues.append("reference_frame '{}' must be a TF root".format(reference))
    if len(roots) != 1:
        issues.append("TF tree must have exactly one root, found {}".format(sorted(roots)))

    issues.extend(_find_cycles(child_to_parent))
    return issues


def _valid_frame_name(value) -> bool:
    return isinstance(value, str) and bool(value.strip()) and value[0] != "/"


def _audit_vector(transform: Dict, index: int, key: str, size: int, issues: List[str]) -> None:
    vector = transform.get(key)
    if not isinstance(vector, list) or len(vector) != size:
        issues.append("transform[{}].{} must be a {} element list".format(index, key, size))
        return
    for offset, value in enumerate(vector):
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            issues.append("transform[{}].{}[{}] must be finite numeric".format(index, key, offset))


def _find_cycles(child_to_parent: Dict[str, str]) -> List[str]:
    issues = []
    for child in child_to_parent:
        seen = set()
        current = child
        while current in child_to_parent:
            if current in seen:
                issues.append("TF tree contains a cycle at '{}'".format(current))
                break
            seen.add(current)
            current = child_to_parent[current]
    return sorted(set(issues))
