# Controller trial and A/B comparison record

- Session/run ID / timestamp / evidence URI:
- Mechanism / hardware ID / configuration hash / criteria ID:
- Physical reference and legal travel verified:
- Trial type: fixed step / speed change / moving target / disturbance recovery.
- Goal/amplitude/speed, units and starting condition:
- Gains kS/kV/kA/kP/kI/kD, motion constraints and units:
- Supply/stator current limits / voltage / temperature / load:
- Configuration apply/readback evidence:

| Repeat/direction | Data valid? Why? | First-settle time | Stayed settled? | Overshoot | Steady mean/RMS | Goal/profile RMS | Saturation/limiter evidence | Verdict |
|---|---|---:|---|---:|---|---|---|---|
| | | | | | | | | |

Unloaded results must not be labeled Fuel-disturbance results. A limit-clipped trial lacks the requested coverage. A stale or interrupted trace is INVALID, not automatically poor tuning.

## One-change comparison

| Candidate | Single changed parameter | Baseline evidence | Candidate evidence | Repeated/reversed-order run | Decision and reason |
|---|---|---|---|---|---|
| | | | | | |

- Is measured motion following the profile, but the profile lagging the final goal?
- Are voltage saturation and actual current-limit indications distinguished?
- Is apparent bias physical backlash/reference error or a repeatable control error?
- What changed after warming, voltage change or cable orientation?
- Human-approved next action / previous configuration to restore:
- Tests still needed before production promotion:
