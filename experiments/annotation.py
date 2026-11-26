import numpy as np


def verify(rows, width):
    subject = [r for r in rows if r["role"] == "subject"]
    if (not subject or abs(subject[0]["source_lateral_m"]) > 0.35
            or any(not r["same_road"] or r["crash"] for r in subject)):
        return {"verified": False}
    if any(np.hypot(b["x"] - a["x"], b["y"] - a["y"]) > 2
           for a, b in zip(subject, subject[1:])):
        return {"verified": False}
    # Fixed source/target geometry, same directed road, settled for >=1 second.
    settled = [r for r in subject if abs(r["target_lateral_m"]) < 0.35
               and abs(r["source_lateral_m"]) > width - 0.35
               and r["same_road"] and not r["crash"]]
    consecutive = []
    for row in settled:
        consecutive = consecutive + [row] if not consecutive or row["step"] == consecutive[-1]["step"] + 1 else [row]
        if len(consecutive) >= 11:
            departure = next(r for r in subject if abs(r["source_lateral_m"]) > 0.35)
            return {"verified": True, "departure_step": departure["step"],
                    "settled_step": consecutive[0]["step"], "confirmed_step": row["step"],
                    "lateral_displacement_m": row["source_lateral_m"] - subject[0]["source_lateral_m"]}
    return {"verified": False}


