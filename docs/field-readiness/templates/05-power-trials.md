# Combined-load and electrical comparison record

- Session/date / hardware / code revision / configuration:
- Battery ID / health/charge checks / power-connection inspection:
- PDH ID and channel-map revision / log and DS-log locations:
- Criteria ID / active fixed drive profile / exact applied current limits:
- Start/end temperature / voltage / source-age coverage:

Run isolated loads before combined loads. Never deliberately create a brownout as a success criterion. Stop a trial that exhibits repeated resets, failed movement or abnormal heating and investigate.

| Run | Requested tasks and actual completion | Battery/config | Minimum V / dip duration | DS brownouts / controller resets | Peak/typical supply loads | Intake completion | Shooter recovery / observed throughput | Loop/CAN health |
|---|---|---|---|---|---|---|---|---|
| | | | | | | | | |

## Controlled experiment

- Hypothesis and one parameter being changed:
- Baseline applied values:
- Candidate applied values:
- Baseline/candidate/baseline run IDs:
- Conditions that changed unintentionally:
- Performance regression and improvement:
- Keep/reject candidate, with evidence:

Supply current is battery-side draw; stator current is torque context. Sum coverage and unsensed loads must remain visible. Logger availability is not proof of continuous data. A missing brownout flag is unknown, not false. The analyzer's plotted dip threshold is not the qualification margin.

## Three full-duration runs

| Trial | 150 s completed? | Representative battery | All requested mechanisms completed? | Brownouts/resets/overruns | Scoring/throughput criterion met? | PASS/FAIL/PENDING |
|---|---|---|---|---|---|---|
| 1 | | | | | | |
| 2 | | | | | | |
| 3 | | | | | | |

If any required condition fails, fix the cause and restart the consecutive qualification sequence. Mark target/field limitations separately from electrical success.
