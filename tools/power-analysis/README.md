# Power analysis

Export synchronized WPILib/Phoenix records to CSV with one timestamped row per sample, then run:

```sh
npm test
npm run analyze -- exported.csv output-directory BATTERY_LABEL CONFIGURATION_ID
```

This tool intentionally does not parse binary `.wpilog` or `.hoot` files. Open the log in AdvantageScope, select the required fields, and export the table as CSV. Direct recorder names such as `Power/TimestampSec`, `Power/PDHVoltageV`, `Power/PDHTotalCurrentA`, `Power/RoboRIOBrownedOut`, and `Power/Motor/<name>/SupplyCurrentA` are accepted; the snake-case aliases are listed in `analyze.mjs`. Timestamp input may be seconds or milliseconds. Missing samples remain missing, records are sorted, duplicate timestamps are rejected, and gaps larger than the configured join window are never forward-filled.

The output contains `report.json` and `power.svg`; the SVG shows PDH voltage, PDH total current, and every exported motor supply-current stream on labeled shared axes. Near-limit time is explicitly inferred. PDH aggregate current remains separate from summed motor supply current; stator currents are never treated as battery load.
