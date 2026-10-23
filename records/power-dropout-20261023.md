# Power dropout, 23 October 2026

- Previous live motor trial reached approximately 7.83 V pack telemetry on 22 October; this is an earlier reading, not a measurement of the present state.
- User reported the chassis dark with its power switch ON. With the switch OFF, unplugging and reconnecting the supplied 12.6 V charger did not change its light from green to red; with charger connected and switch ON, the robot remained dark.
- Later the user reported that the robot had started. `ugvpi.local` briefly answered a ping at `192.168.86.45`, but SSH did not connect and the Pi then stopped responding. The user reported the robot dead again.
- No current battery voltage, cell voltages, charge current or charger output were measured. Do not infer full charge or a specific failed component from the charger LED alone.
- Next diagnostic is physical inspection of the three cells, their orientation and contacts, charging connector and charger. A multimeter measurement of each cell and the charger's output would distinguish a depleted/unbalanced pack from a charger or connection fault. Do not resume motor tests until power is stable.
- The adaptive motion software remains local and has not been deployed or physically validated on the Pi.
