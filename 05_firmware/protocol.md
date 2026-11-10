# Serial command protocol (draft)

Baud: 115200 · Line ending: `\n` · Units: degrees, mm

| Command | Meaning | Validation | Reply |
| --- | --- | --- | --- |
| `S <joint> <angle_deg>` | Set one joint | Known joint, numeric, within calibrated limits | `ACK` / `ERR <reason>`, then `DONE` |
| `XY <x_mm> <y_mm>` | Cartesian target | Numeric, reachable, joint + workspace limits, branch policy | Chosen branch + angles, or `ERR` |
| `SPEED <value>` | Set speed/profile | Bounded, explicit units | Active setting |
| `HOME` | Safe homing | Allowed only from defined states | Progress, `DONE` or `FAULT` |
| `STOP` | Stop/hold, clear queue | Always accepted, highest priority | Immediate `ACK` + state |
| `STATUS` | Report state | — | State, positions, speed, last error, link health |
| `RESET_FAULT` | Clear fault | Safe condition + operator action | New state; never replays the failed command |

TODO: error codes, sequence numbers / timestamps, watchdog timeout value.
