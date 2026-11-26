"""Finite-difference estimates and explicitly named reference-point proxies."""
import math


def metric(value, unit, method, status="valid", reason="", support=None):
    if value is not None and not math.isfinite(value):
        raise ValueError("Non-finite metric")
    if status != "valid":
        value = None
    return dict(value=value, unit=unit, validity=status, reason=reason,
                method=method, support_steps=support or [])


def derivative(track, step, field, order, dt):
    weights = {1: [(-1, -1), (1, 1)],
               2: [(-1, 1), (0, -2), (1, 1)],
               3: [(-2, -1), (-1, 2), (1, -2), (2, 1)]}[order]
    radius = 2 if order == 3 else 1
    support = list(range(step - radius, step + radius + 1))
    method = f"central_difference_order_{order}_unfiltered"
    unit = {1: "m/s", 2: "m/s^2", 3: "m/s^3"}[order]
    if field == "speed_ms":
        unit = {1: "m/s^2", 2: "m/s^3"}[order]
    missing = [k for k in support if k not in track]
    if missing:
        edge = min(support) < min(track) or max(support) > max(track)
        return metric(None, unit, method, "unavailable", "boundary_support" if edge else "missing_sample", support)
    samples = [track[k] for k in support]
    if any(not r["on_lane"] for r in samples):
        return metric(None, unit, method, "invalid", "localization_failure", support)
    if any(not r["same_road"] for r in samples) or len({r["lane"][:2] for r in samples}) != 1:
        return metric(None, unit, method, "invalid", "road_transition", support)
    if any(not math.isclose(r["time_s"], track[step]["time_s"] + (r["step"] - step) * dt,
                            abs_tol=1e-9, rel_tol=0) for r in samples):
        return metric(None, unit, method, "invalid", "nonuniform_time", support)
    denominator = (2 if order in (1, 3) else 1) * dt ** order
    value = sum(weight * track[step + offset][field] for offset, weight in weights) / denominator
    return metric(value, unit, method, support=support)


def actor_features(track, step, dt):
    result = {}
    for name, field, order in [
        ("vx_world", "x", 1), ("vy_world", "y", 1),
        ("ax_world", "x", 2), ("ay_world", "y", 2),
        ("v_longitudinal", "source_longitudinal_m", 1),
        ("a_longitudinal", "source_longitudinal_m", 2),
        ("j_longitudinal", "source_longitudinal_m", 3),
        ("v_lateral", "source_lateral_m", 1),
        ("a_lateral", "source_lateral_m", 2),
        ("j_lateral", "source_lateral_m", 3),
        ("a_speed", "speed_ms", 1), ("j_speed", "speed_ms", 2),
    ]:
        result[name] = derivative(track, step, field, order, dt)
    return result


def pair_features(subject, other, subject_features, other_features):
    support = [subject["step"]]
    usable = all(r["on_lane"] and r["same_road"] for r in [subject, other]) and subject["lane"][:2] == other["lane"][:2]
    status, reason = ("valid", "") if usable else ("invalid", "localization_or_road_mismatch")
    def point(value, unit, method):
        return metric(value, unit, method, status, reason, support)
    ds = other["source_longitudinal_m"] - subject["source_longitudinal_m"]
    out = dict(reference_distance=point(math.hypot(other["x"] - subject["x"], other["y"] - subject["y"]), "m", "reference_point_euclidean"),
               signed_longitudinal_separation=point(ds, "m", "other_s_minus_subject_s"))
    for name in ["vx_world", "vy_world", "v_longitudinal"]:
        a, b = subject_features[name], other_features[name]
        ok = usable and a["validity"] == b["validity"] == "valid"
        out["relative_" + name] = metric(b["value"] - a["value"] if ok else None,
            "m/s", "other_minus_subject_central_difference", "valid" if ok else "unavailable",
            "" if ok else "invalid_or_missing_velocity_support", a["support_steps"])
    # This is longitudinal reference-point alignment, NOT footprint collision TTC.
    alignment = subject["lane"] == other["lane"] and math.cos(subject["heading_rad"] - other["heading_rad"]) > 0
    for name in ["point_following_ttc_proxy", "gross_reference_headway"]:
        method = "projected_reference_gap_over_closing_speed" if "ttc" in name else "projected_reference_gap_over_follower_v_longitudinal"
        st, why, value = "not_applicable", "not_a_same_assigned_lane_forward_pair", None
        if usable and alignment and ds > 0:
            a, b = subject_features["v_longitudinal"], other_features["v_longitudinal"]
            if a["validity"] != "valid" or b["validity"] != "valid":
                st, why = "unavailable", "missing_velocity_support"
            else:
                denominator = a["value"] - b["value"] if "ttc" in name else a["value"]
                if denominator <= 1e-9:
                    st, why = "undefined", "no_positive_closing_speed" if "ttc" in name else "stationary_or_nonforward_follower"
                else:
                    value, st, why = ds / denominator, "valid", "point_proxy_only_not_collision_evidence"
        elif not usable:
            st, why = "invalid", "localization_or_road_mismatch"
        out[name] = metric(value, "s", method, st, why,
                           subject_features["v_longitudinal"]["support_steps"])
    for name, unit, why in [
        ("bumper_gap", "m", "resolved_dimensions_and_reference_offsets_not_recorded"),
        ("minimum_footprint_clearance", "m", "resolved_footprints_not_recorded"),
        ("footprint_2d_ttc", "s", "resolved_footprints_and_prediction_model_unavailable"),
        ("pet", "s", "conflict_zone_and_footprint_occupancy_not_recorded"),
    ]:
        out[name] = metric(None, unit, "not_implemented_without_geometry_contract", "unavailable", why)
    return out
