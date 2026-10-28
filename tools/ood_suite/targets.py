"""Reproducible target plans for the composed ArtiGrasp evaluation.

Everything except loading the existing ``.npy`` labels uses the Python standard
library.  In particular, inspecting a plan does not import Torch or RaiSim.
Spatial OOD refers to a deliberate extrapolation from the public evaluation
targets; the public repository does not describe the complete training support.
"""

import copy
import hashlib
import json
import math
import random
from pathlib import Path
from xml.etree import ElementTree


ALL_OBJECTS = (
    "box", "laptop", "notebook", "waffleiron", "mixer", "ketchup",
    "espressomachine", "microwave", "capsulemachine", "phone",
)
PAPER_OBJECTS = ALL_OBJECTS[:8]
DEFAULT_PROFILES = (
    "id_control", "ood_height", "ood_translation",
    "ood_articulation_180", "ood_combined",
)


def load_config(path):
    """Load the experiment JSON without requiring scientific dependencies."""
    with Path(path).open(encoding="utf-8") as stream:
        config = json.load(stream)
    if not isinstance(config, dict):
        raise ValueError("Experiment configuration must be a JSON object")
    return config


def stable_seed(seed, *parts):
    """Use an explicit hash, independent of PYTHONHASHSEED or process order."""
    payload = json.dumps([int(seed)] + list(parts), separators=(",", ":"),
                         ensure_ascii=True).encode("ascii")
    return int.from_bytes(hashlib.sha256(payload).digest()[:4], "big") & 0x7fffffff


def joint_limits(repo_root, object_name):
    """Read the one movable object's hinge limit, never changing its URDF."""
    path = Path(repo_root) / "rsc" / "arctic" / object_name / (object_name + ".urdf")
    movable = [joint for joint in ElementTree.parse(str(path)).getroot().findall("joint")
               if joint.get("type") not in ("fixed", "floating")]
    if len(movable) != 1:
        raise ValueError("Expected exactly one articulation joint in {}".format(path))
    joint = movable[0]
    limit = joint.find("limit")
    if joint.get("type") == "continuous" or limit is None:
        raise ValueError("Expected finite articulation limits in {}".format(path))
    bounds = [float(limit.attrib["lower"]), float(limit.attrib["upper"])]
    if not all(math.isfinite(value) for value in bounds) or bounds[0] > bounds[1]:
        raise ValueError("Invalid articulation limits in {}".format(path))
    return bounds


def articulation_is_feasible(angle, limits):
    """Tolerate float32 roundoff at exactly 180 degrees, without clipping."""
    return bool(limits[0] - 1e-7 <= float(angle) <= limits[1] + 1e-7)


def resolve_articulation(target, reset_angle):
    """Evaluate the public target rule using the actual reset articulation.

    The ordinary public evaluator closes an initially open hinge by 0.5--0.6
    radians, or opens an initially closed one to that angle. Ketchup uses an
    absolute 0.5--1.0-radian target.  Absolute OOD requests always stay literal.
    """
    if target.get("articulation_rule", "absolute") == "public_relative":
        delta = float(target["public_delta_rad"])
        return float(reset_angle) - delta if float(reset_angle) > 1.0 else delta
    return float(target["articulation_rad"])


def _range(value, name):
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        bounds = [float(value), float(value)]
    elif isinstance(value, (list, tuple)) and len(value) == 2:
        bounds = [float(item) for item in value]
    else:
        raise ValueError("{} must be a number or [minimum, maximum]".format(name))
    if not all(math.isfinite(item) for item in bounds) or bounds[0] > bounds[1]:
        raise ValueError("{} must have finite, ordered bounds".format(name))
    return bounds


def _sample(rng, value, name):
    low, high = _range(value, name)
    return rng.uniform(low, high)


def sample_target(object_name, sample_seed, profile):
    """Sample a paired baseline, then alter only the requested target axes.

    The profile-independent random stream ensures, for example, that a height
    sweep keeps translation, orientation, and articulation identical to control.
    A separate stream samples the additional OOD values.
    """
    rng = random.Random(stable_seed(sample_seed, "public_target"))
    ketchup = object_name == "ketchup"
    translation = [rng.uniform(-0.1, -0.05), rng.uniform(-0.1, -0.05)]
    delta = rng.uniform(0.5, 1.0 if ketchup else 0.6)
    orientation = rng.uniform(-0.3 if ketchup else -0.4, 0.3 if ketchup else 0.4)
    target = {
        "lift_m": 0.3 if ketchup else 0.15,
        "translation_m": translation,
        "orientation_delta_rad": orientation,
        "articulation_rad": delta,
        "articulation_rule": "absolute" if ketchup else "public_relative",
        "public_delta_rad": delta,
    }
    changed_axes = []
    if "lift_m" in profile:
        rng_axis = random.Random(stable_seed(sample_seed, "ood_lift"))
        target["lift_m"] = _sample(rng_axis, profile["lift_m"], "lift_m")
        if target["lift_m"] < 0:
            raise ValueError("lift_m must be nonnegative")
        changed_axes.append("lift")
    if "translation_distance_m" in profile:
        rng_axis = random.Random(stable_seed(sample_seed, "ood_translation"))
        distance = _sample(rng_axis, profile["translation_distance_m"], "translation_distance_m")
        if distance <= 0:
            raise ValueError("translation_distance_m must be positive")
        # Balanced directions test extrapolation independent of a preferred axis.
        # All directions have exactly the sampled norm; no axis can cancel it.
        direction = rng_axis.uniform(-math.pi, math.pi)
        target["translation_m"] = [distance * math.cos(direction), distance * math.sin(direction)]
        changed_axes.append("translation")
    if "orientation_delta_deg" in profile:
        rng_axis = random.Random(stable_seed(sample_seed, "ood_orientation"))
        target["orientation_delta_rad"] = math.radians(_sample(
            rng_axis, profile["orientation_delta_deg"], "orientation_delta_deg"))
        changed_axes.append("orientation")
    if "articulation_deg" in profile:
        rng_axis = random.Random(stable_seed(sample_seed, "ood_articulation"))
        target["articulation_rad"] = math.radians(_sample(
            rng_axis, profile["articulation_deg"], "articulation_deg"))
        target["articulation_rule"] = "absolute"
        changed_axes.append("articulation")
    target["changed_axes"] = changed_axes
    return target


def _plain_label(label):
    """Normalize NumPy scalar values into stable JSON-compatible label fields."""
    return {
        "seq_num": int(label["seq_num"]),
        "seq_name": str(label["seq_name"]),
        "obj_name": str(label["obj_name"]),
        "init_frame": int(label["init_frame"]),
        "grasp_frame": int(label["grasp_frame"]),
        "kind": str(label["kind"]),
    }


def load_label_sets(repo_root, objects, split="test"):
    """Read existing label dictionaries without importing label_gen's Torch code.

    Selection follows ``label_eval_compose`` for ordinary objects and the
    released ``ketchup_eval`` special approach for ketchup.  The latter uses
    each articulation label again with kind ``right_special`` for grasping.
    ``label_eval_floating`` accidentally removes several objects' articulation
    labels and therefore cannot enumerate this full suite.
    """
    if split not in ("train", "test"):
        raise ValueError("split must be 'train' or 'test'")
    import numpy as np
    root = Path(repo_root) / "rsc" / "mano_double" / "grasp_label" / "final_labels" / (split + "_set")
    result = {}
    for object_name in objects:
        path = root / (object_name + ".npy")
        labels = [_plain_label(item) for item in np.load(str(path), allow_pickle=True)]
        grasp_kinds = {"left_grasp"} if object_name in ("mixer", "ketchup", "phone") else {
            "left_grasp", "right_grasp", "two_grasp"}
        result[object_name] = {
            "grasp": [item for item in labels if item["kind"] in grasp_kinds],
            "articulation": [item for item in labels if item["kind"] == "right_arti"],
        }
        if object_name == "ketchup":
            result[object_name]["grasp"] = [dict(item, kind="right_special")
                                             for item in result[object_name]["articulation"]]
    return result


def build_plan(repo_root, config, objects=None, profiles=None, samples=None,
               seed=None, split=None, label_sets=None):
    """Build a deterministic list of paired episodes; simulation is separate.

    ``label_sets`` is optional so tests can use in-memory labels. Repeated label
    pairs are intentional Monte Carlo samples with independently drawn goals.
    Plans record this sampling design and never pretend all initial states are
    unique. ``articulation_rad`` for a ``public_relative`` target is a provisional
    value for a closed initial hinge; call ``resolve_articulation`` after reset
    and persist the resolved value in each quantitative result.
    """
    selected_objects = list(config.get("objects", ALL_OBJECTS) if objects is None else objects)
    configured_profiles = config.get("profiles", {})
    selected_profiles = list(configured_profiles if profiles is None else profiles)
    samples = config.get("samples_per_object", 100) if samples is None else samples
    seed = config.get("seed", 1) if seed is None else seed
    split = config.get("split", "test") if split is None else split
    if isinstance(samples, bool) or not isinstance(samples, int) or samples <= 0:
        raise ValueError("samples_per_object must be a positive integer")
    if split not in ("train", "test"):
        raise ValueError("split must be 'train' or 'test'")
    if not selected_objects or len(selected_objects) != len(set(selected_objects)):
        raise ValueError("Select at least one object, with no duplicate names")
    if not selected_profiles or len(selected_profiles) != len(set(selected_profiles)):
        raise ValueError("Select at least one profile, with no duplicate names")
    unknown = set(selected_objects) - set(ALL_OBJECTS)
    if unknown:
        raise ValueError("Unknown object(s): {}".format(", ".join(sorted(unknown))))
    unknown = set(selected_profiles) - set(configured_profiles)
    if unknown:
        raise ValueError("Unknown profile(s): {}".format(", ".join(sorted(unknown))))
    if label_sets is None:
        label_sets = load_label_sets(repo_root, selected_objects, split)

    episodes = []
    label_counts = {}
    for object_name in selected_objects:
        pools = label_sets[object_name]
        grasp_labels = [_plain_label(item) for item in pools["grasp"]]
        arti_labels = [_plain_label(item) for item in pools["articulation"]]
        if object_name == "ketchup":
            # ketchup_eval.py uses the same articulation frame for the left
            # grasp and subsequent right-hand articulation; independent left
            # grasp labels produce a materially different controller input.
            grasp_labels = [dict(item, kind="right_special") for item in arti_labels]
        if not grasp_labels or not arti_labels:
            raise ValueError("{} {} split needs both grasp and articulation labels ({} / {})".format(
                object_name, split, len(grasp_labels), len(arti_labels)))
        for label in grasp_labels + arti_labels:
            if label["obj_name"] != object_name:
                raise ValueError("Mismatched object in {} labels".format(object_name))
        limits = joint_limits(repo_root, object_name)
        label_counts[object_name] = {"grasp": len(grasp_labels), "articulation": len(arti_labels),
                                     "possible_pairs": (len(arti_labels) if object_name == "ketchup"
                                                        else len(grasp_labels) * len(arti_labels))}
        if object_name == "ketchup":
            label_counts[object_name]["sampling_note"] = "paired articulation label; right_special grasp approach"
        for sample_index in range(samples):
            sample_seed = stable_seed(seed, split, object_name, sample_index)
            label_rng = random.Random(stable_seed(sample_seed, "labels"))
            if object_name == "ketchup":
                arti_index = label_rng.randrange(len(arti_labels))
                grasp_index = arti_index
            else:
                grasp_index = label_rng.randrange(len(grasp_labels))
                arti_index = label_rng.randrange(len(arti_labels))
            for profile_name in selected_profiles:
                target = sample_target(object_name, sample_seed, configured_profiles[profile_name])
                feasible = articulation_is_feasible(target["articulation_rad"], limits)
                episodes.append({
                    "episode_id": "{}__{}__{:04d}".format(object_name, profile_name, sample_index),
                    "object": object_name,
                    "profile": profile_name,
                    "sample_index": sample_index,
                    "seed": sample_seed,
                    "split": split,
                    "paper_object": object_name in PAPER_OBJECTS,
                    "grasp_label_index": grasp_index,
                    "arti_label_index": arti_index,
                    "grasp_label": copy.deepcopy(grasp_labels[grasp_index]),
                    "arti_label": copy.deepcopy(arti_labels[arti_index]),
                    "target": target,
                    "joint_limits_rad": limits[:],
                    "target_reachable": feasible,
                    "articulation_within_joint_limits": feasible,
                    "target_reachability_basis": "articulation limits only; spatial reachability not inferred",
                    "articulation_requires_reset_resolution": target["articulation_rule"] == "public_relative",
                })
    effective_config = copy.deepcopy(config)
    effective_config.update({
        "objects": selected_objects[:],
        "profiles": {name: copy.deepcopy(configured_profiles[name]) for name in selected_profiles},
        "samples_per_object": samples,
        "seed": int(seed),
        "split": split,
    })
    return {
        "schema_version": 1,
        "config": effective_config,
        "seed": int(seed),
        "split": split,
        "objects": selected_objects,
        "profiles": selected_profiles,
        "samples_per_object": samples,
        "episodes": episodes,
        "label_counts": label_counts,
        "sampling": "label pairs sampled with replacement; independent random targets; paired across profiles",
        "ood_reference": (
            "Spatial targets extrapolate from the public composed-evaluation target envelope. "
            "The full training support is not published. Absolute 180-degree articulation "
            "exceeds the general_two randomized training range [0.5, 1.5] radians. "
            "URDF limits remain unchanged and infeasible requests remain in the denominator."
        ),
    }
