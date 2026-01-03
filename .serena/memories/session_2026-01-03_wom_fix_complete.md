# Session Summary: 2026-01-03 - WoM Fix Complete

## Session Outcome: SUCCESS ✓

Wake-on-Motion (WoM) binary sensor now works correctly after datasheet analysis.

## Key Discoveries

### Root Cause of WoM Failure
The WoM sensor wasn't triggering because we were re-enabling the gyroscope after configuration. Per QMI8658C datasheet Table 31, WoM mode **requires**:
- CTRL7: `aEN=1, gEN=0, mEN=0` (accelerometer ONLY)
- Enabling gyroscope exits WoM mode and enters normal IMU mode

### Datasheet Section 9 - Wake on Motion
Critical configuration requirements:
1. Disable all sensors (CTRL7 = 0x00)
2. Set accelerometer sample rate and scale (CTRL2)
3. Set WoM threshold in CAL1_L; interrupt config in CAL1_H
4. Execute CTRL9 command 0x08 (CTRL_CMD_WRITE_WOM_SETTING)
5. Enable accelerometer ONLY in CTRL7

### CAL1_H Register (Table 33)
- Bits 7:6: Interrupt select (00=INT1, 01=INT2)
- Bits 5:0: Blanking time (samples to ignore at startup)

### Interrupt Behavior
- Interrupt line **TOGGLES** on each WoM event (doesn't just go high)
- Must use `INTERRUPT_ANY_EDGE` to catch all events
- Reading STATUS1 clears WoM flag and resets interrupt line

## Fixes Applied (v1.0.7)

| Issue | Before | After |
|-------|--------|-------|
| CTRL7 after WoM | `ACC_EN \| GYR_EN` | `ACC_EN` only |
| Interrupt edge | `RISING_EDGE` | `ANY_EDGE` |
| Blanking time | 0 samples | 4 samples |
| CTRL9 polling | Fixed 10ms delay | Poll STATUSINT.bit7 |
| Motion hold | Immediate clear | 500ms hold time |

## Commit
- Hash: `0cc1f11`
- Message: "Fix Wake-on-Motion (WoM) sensor to work per datasheet requirements"

## Important Trade-off
When WoM is enabled, gyroscope readings stop updating. This is a hardware limitation. If both gyro readings and motion detection are needed, use the software `motion` binary sensor instead.

## Files Modified
- `components/qmi8658/qmi8658.cpp` - WoM configuration and interrupt handling
- `components/qmi8658/qmi8658.h` - Added motion hold timer, constants
- `components/qmi8658/binary_sensor.py` - Added interrupt pin configuration
- `CLAUDE.md` - Project documentation

## Testing Verified
- WoM works on both INT1 and INT2
- Threshold 100mg provides good sensitivity
- Removing WoM from config restores normal gyro operation
- Software motion sensor unaffected when WoM disabled
