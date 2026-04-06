# Sensor Driver Implementation Status

Updated after driver sprint. Cross-reference against ArduPilot sensor driver audit.

## Summary

| Category | Must Have | Done | High | Done | Total Done |
|----------|----------|------|------|------|-----------|
| GPS | 12 | 12 | 7 | 4 | 18/21 |
| RangeFinder | 4 | 4 | 5 | 4 | 10/13 |
| OpticalFlow | 6 | 6 | 1 | 1 | 7/10 |
| Airspeed | 3 | 3 | 4 | 4 | 8/8 |
| Beacon | 2 | 0 | 1 | 0 | 0/8 |
| ExternalAHRS | 8 | 5 | 2 | 1 | 6/14 |
| **Total** | **35** | **30** | **20** | **14** | **49/74** |

**49 of 74 items now complete (66%).** Up from 33/74 (45%).

## Recent Additions

- **MS4525DO** — Most common airspeed sensor. Full I2C driver with 14-bit pressure, temperature, ground calibration.
- **SDP3x** — Sensirion differential pressure. I2C with CRC8 validation, continuous measurement mode.
- **Airspeed Health System** — Probability LPF: bad=*0.90, good=0.98*p+0.02. Auto-disable at p<0.1, re-enable at p>0.95.
- **Benewake TFMini/TF02/TF03** — Serial lidar rangefinders. 9-byte protocol with checksum, signal quality tracking, all 3 variants.
- **LightWare SF10/SF11/SF20/LW20** — I2C lidar. Plus SF40C/SF45B serial binary protocol with CRC16.
- **VL53L0X/VL53L1X** — STM time-of-flight I2C sensors. Init sequences, short/long mode for VL53L1X.
- **PX4Flow** — I2C optical flow. 26-byte integral frame, flow/gyro rate extraction, scale factors, yaw rotation.
- **VectorNav VN-100/200/300** — External AHRS binary protocol. AHRS mode (2 packets) + INS mode (3 packets). CRC16, ASCII config, state parsing.
- **F9P RTK** — VALSET/VALGET CFGv2 protocol, base station survey-in config, rover RTCM3 input config, 12 configuration keys.
- **NAV-RELPOSNED** — GPS yaw from moving baseline. Full 72-byte message parser with heading extraction and validation.
- **RTCM3 Injector** — Fragment reassembly for MAVLink GPS_RTCM_DATA messages (180-byte fragments → complete frames).

## Remaining Gaps (community contribution targets)

| Item | Priority | Difficulty | Notes |
|------|----------|-----------|-------|
| Beacon system (Marvelmind, Pozyx, Nooploop) | Medium | Medium | New crate needed |
| InertialLabs ExternalAHRS | High | Hard | Complex binary protocol |
| MicroStrain GX5/GQ7 ExternalAHRS | Medium | Hard | |
| DroneCAN GPS/rangefinder/airspeed | Low | Medium | Needs CAN bus integration |
| MAVLink DISTANCE_SENSOR rangefinder | Medium | Easy | Protocol handler |
| MAVLink OPTICAL_FLOW | Medium | Easy | Protocol handler |
| PMW3901 SPI optical flow | Medium | Medium | PixArt sensor |
| NMEA VTG/HDT/THS sentences | High | Easy | Extra GPS parsing |
| GPS JitterCorrection | Medium | Medium | UART timestamp correction |
