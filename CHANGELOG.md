# Changelog

## V20.4 — Smooth RPM

- Added adaptive filtering for the RPM value displayed on the Wi-Fi Dashboard.
- Added continuous browser-side tachometer needle interpolation for smoother movement.
- Kept a separate responsive RPM value for shift-light control so display smoothing does not delay the warning.
- Diagnostic page now distinguishes raw RPM, control RPM and display RPM.
- Retains V20.3.1 tachometer correction using 2 pulses per revolution.
- Retains configurable cold/hot shift-light logic.
- Retains tachometer and coolant-temperature calibration functions.

## V20.3.1 — Tachometer fix

- Corrected base tachometer calculation from 4 to 2 pulses per revolution.
- Idle calibration remains available as a fine correction around 1100 RPM.
- Improved tachometer calibration diagnostics.

## V20.3 — Cold / hot shift-light

- Added configurable cold-engine shift RPM limit.
- Added configurable cold → hot coolant-temperature threshold.
- Added automatic switching to normal hot-engine solid/flashing shift thresholds.
- Default values: 8000 RPM below 60 °C, then 9000 RPM solid / 10000 RPM flashing.

## V20.2.1 — Calibration and restart fixes

- Added 1100 RPM tachometer calibration.
- Added cold temperature calibration from ambient temperature.
- Added hot calibration at radiator-fan activation.
- Added EEPROM persistence for calibration values.
- Fixed delayed Web-interface ESP restart.
- Fixed compilation scope for restart state variables.

Older experimental firmware remains available through the Git history rather than as duplicate files in the main branch.
