# Vehicle Signal Schema — Canonical Field Catalog

This document defines the canonical signal set used by the vehicle CAN bridge.

- **Part 1** — Active Signal Catalog: the signals currently defined in
  `config/vehicle_schema.yaml` and published on `/vehicle/decoded_can`.
- **Part 2** — Full Signal Reference: the complete set of canonical signal
  names across all domains, retained for future expansion reference.

---

## Coordinate System

**ISO 8855** vehicle coordinate system:

| Axis  | Direction        | Sign                                         |
| ----- | ---------------- | -------------------------------------------- |
| X     | Forward          | Positive = forward                           |
| Y     | Left             | Positive = left                              |
| Z     | Up               | Positive = up                                |
| Yaw   | Rotation about Z | Positive = left turn (CCW viewed from above) |
| Pitch | Rotation about Y | Positive = nose up                           |
| Roll  | Rotation about X | Positive = right side down                   |

---

## DBC Coverage Legend

| Symbol | DBC                                      |
| ------ | ---------------------------------------- |
| P      | AStuff PACMod v3 (`as_pacmod.dbc`)       |
| T      | commaai Toyota 2017 (`_toyota_2017.dbc`) |
| P+T    | Both DBCs                                |
| \*     | Not available in either DBC              |

Signals without coverage for a loaded DBC are output with `status = STATUS_INITIAL`
in per-domain publishers. They are omitted from the firehose (`/vehicle/decoded_can`).

---

## Shared Enum Definitions

### Turn Signal

| Value | Meaning |
| ----- | ------- |
| 0     | OFF     |
| 1     | LEFT    |
| 2     | RIGHT   |
| 3     | HAZARD  |

---

## Part 1: Active Signal Catalog

Signals published on `/vehicle/decoded_can`.  
Source of truth: `config/vehicle_schema.yaml`.

## Domain: `dynamics` — Vehicle Motion State

| Canonical Name                     | Unit  | Range            | Sign        | DBC | Notes                                            |
| ---------------------------------- | ----- | ---------------- | ----------- | --- | ------------------------------------------------ |
| `dynamics.speed.longitudinal`      | m/s   | [-327, +327]     | forward+    | P+T | Toyota: `SPEED.SPEED` km/h → m/s                 |
| `dynamics.speed.lateral`           | m/s   | —                | left+       | \*  | Rarely available in OEM DBCs                     |
| `dynamics.wheel_speed.front_left`  | m/s   | [0, +90]         | —           | P+T | PACMod: rad/s × wheel_radius; Toyota: km/h → m/s |
| `dynamics.wheel_speed.front_right` | m/s   | [0, +90]         | —           | P+T |                                                  |
| `dynamics.wheel_speed.rear_left`   | m/s   | [0, +90]         | —           | P+T |                                                  |
| `dynamics.wheel_speed.rear_right`  | m/s   | [0, +90]         | —           | P+T |                                                  |
| `dynamics.angular_vel.yaw`         | rad/s | [-32.77, +32.77] | CCW+        | P+T | Toyota: `KINEMATICS.YAW_RATE` deg/s → rad/s      |
| `dynamics.angular_vel.pitch`       | rad/s | [-32.77, +32.77] | nose-up+    | P   | `ANG_VEL_RPT.PITCH_VEL`                          |
| `dynamics.angular_vel.roll`        | rad/s | [-32.77, +32.77] | right-down+ | P   | `ANG_VEL_RPT.ROLL_VEL`                           |

## Domain: `operation` — Actuator Reports

| Canonical Name                       | Unit  | Range            | Sign  | DBC | Notes                                         |
| ------------------------------------ | ----- | ---------------- | ----- | --- | --------------------------------------------- |
| `operation.steering.report.angle`    | rad   | [-32.77, +32.77] | left+ | P+T | Reported output angle from DBW or sensor      |
| `operation.throttle.report.position` | ratio | [0, 1]           | —     | P+T | Toyota: `GAS_PEDAL_HYBRID.GAS_PEDAL`          |
| `operation.brake.report.position`    | ratio | [0, 1]           | —     | P+T | Toyota: normalized `BRAKE.BRAKE_AMOUNT / 255` |

## Domain: `body` — Driver Intent

| Canonical Name            | Unit | DBC | Notes                                                       |
| ------------------------- | ---- | --- | ----------------------------------------------------------- |
| `body.lights.turn_signal` | enum | P+T | See Turn Signal enum; Toyota: `BLINKERS_STATE.TURN_SIGNALS` |

---

## Part 2: Full Signal Reference

Complete canonical signal catalog for all domains.  
Signals not in Part 1 are available for future expansion.

## Coordinate System and Enums

Coordinate system and enums are common to all domains (see top of document).

### Additional Enums

#### Gear

| Value | Meaning                |
| ----- | ---------------------- |
| 0     | UNKNOWN                |
| 1     | PARK                   |
| 2     | REVERSE                |
| 3     | NEUTRAL                |
| 4     | DRIVE                  |
| 5     | LOW / B (engine brake) |

#### Headlight Mode

| Value | Meaning                   |
| ----- | ------------------------- |
| 0     | OFF                       |
| 1     | POSITION (parking lights) |
| 2     | LOW BEAM                  |
| 3     | HIGH BEAM                 |
| 4     | AUTO                      |

#### Safety Function State (PACMod)

| Value | Meaning           |
| ----- | ----------------- |
| 0     | MANUAL            |
| 1     | MANUAL_READY      |
| 2     | AUTO_READY        |
| 3     | AUTO (autonomous) |
| 4     | CRITICAL_STOP_1   |
| 5     | CRITICAL_STOP_2   |

#### Cruise State

| Value | Meaning                     |
| ----- | --------------------------- |
| 0     | OFF                         |
| 1     | STANDBY (main on, not set)  |
| 2     | ACTIVE (speed-hold)         |
| 3     | BRAKING (auto deceleration) |
| 4     | FAULT                       |

---

### Domain: `operation` — Actuator Commands and Reports

Topic: `/vehicle/operation`

| Canonical Name                        | Unit  | Range            | Sign   | DBC | Notes                                         |
| ------------------------------------- | ----- | ---------------- | ------ | --- | --------------------------------------------- |
| `operation.steering.command.angle`    | rad   | [-32.77, +32.77] | left+  | P   | DBW commanded steering angle                  |
| `operation.steering.command.rate`     | rad/s | [0, 65.54]       | left+  | P   | DBW commanded rotation rate                   |
| `operation.steering.report.angle`     | rad   | [-32.77, +32.77] | left+  | P+T | Reported output angle from DBW or sensor      |
| `operation.steering.report.torque`    | Nm    | [-2, +2] (ratio) | —      | P   | Reported output torque ratio                  |
| `operation.steering.manual_input`     | rad   | [-32.77, +32.77] | left+  | P   | Driver hand-wheel angle                       |
| `operation.throttle.command.position` | ratio | [0, 1]           | —      | P   |                                               |
| `operation.throttle.report.position`  | ratio | [0, 1]           | —      | P+T | Toyota: `GAS_PEDAL_HYBRID.GAS_PEDAL`          |
| `operation.throttle.manual_input`     | ratio | [0, 1]           | —      | P   |                                               |
| `operation.brake.command.position`    | ratio | [0, 1]           | —      | P   |                                               |
| `operation.brake.report.position`     | ratio | [0, 1]           | —      | P+T | Toyota: normalized `BRAKE.BRAKE_AMOUNT / 255` |
| `operation.brake.manual_input`        | ratio | [0, 1]           | —      | P   |                                               |
| `operation.brake.decel_command`       | m/s²  | [0, 10]          | decel+ | P   | XBR deceleration demand                       |
| `operation.brake.pressed`             | bool  | {0,1}            | —      | T   | `BRAKE_2.BRAKE_PRESSED`                       |
| `operation.shift.command`             | enum  | [0, 5]           | —      | P   | See Gear enum                                 |
| `operation.shift.report`              | enum  | [0, 5]           | —      | P+T | Toyota: `GEAR_PACKET.GEAR` remapped           |
| `operation.shift.manual_input`        | enum  | [0, 5]           | —      | P   |                                               |
| `operation.parking_brake.command`     | bool  | {0,1}            | —      | P   | 1=apply                                       |
| `operation.parking_brake.report`      | enum  | {0,1,2}          | —      | P   | 0=released, 1=applied, 2=transitioning        |
| `operation.engaged`                   | bool  | {0,1}            | —      | P   | 1 = DBW actively controlling vehicle          |

**Toyota Gear Remap** (`GEAR_PACKET.GEAR`):

| Raw | Gear             | Canonical   |
| --- | ---------------- | ----------- |
| 0   | —                | 0 (UNKNOWN) |
| 1   | DRIVE (D)        | 4           |
| 2   | NEUTRAL (N)      | 3           |
| 8   | PARK (P)         | 1           |
| 16  | REVERSE (R)      | 2           |
| 32  | B (engine brake) | 5           |

**PACMod Shift Remap** (`SHIFT_RPT.OUTPUT_VALUE`):

| Raw | Gear    | Canonical |
| --- | ------- | --------- |
| 0   | PARK    | 1         |
| 1   | REVERSE | 2         |
| 2   | NEUTRAL | 3         |
| 3   | DRIVE   | 4         |
| 4   | LOW     | 5         |

---

### Domain: `dynamics` — Vehicle Motion State

Topic: `/vehicle/dynamics`

| Canonical Name                     | Unit  | Range            | Sign        | DBC | Notes                                            |
| ---------------------------------- | ----- | ---------------- | ----------- | --- | ------------------------------------------------ |
| `dynamics.speed.longitudinal`      | m/s   | [-327, +327]     | forward+    | P+T | Toyota: `SPEED.SPEED` km/h → m/s                 |
| `dynamics.speed.lateral`           | m/s   | —                | left+       | \*  | Rarely available in OEM DBCs                     |
| `dynamics.wheel_speed.front_left`  | m/s   | [0, +90]         | —           | P+T | PACMod: rad/s × wheel_radius; Toyota: km/h → m/s |
| `dynamics.wheel_speed.front_right` | m/s   | [0, +90]         | —           | P+T |                                                  |
| `dynamics.wheel_speed.rear_left`   | m/s   | [0, +90]         | —           | P+T |                                                  |
| `dynamics.wheel_speed.rear_right`  | m/s   | [0, +90]         | —           | P+T |                                                  |
| `dynamics.accel.longitudinal`      | m/s²  | [-327, +327]     | forward+    | P+T | Toyota: `KINEMATICS.ACCEL_X`                     |
| `dynamics.accel.lateral`           | m/s²  | [-327, +327]     | left+       | P+T | Toyota: `KINEMATICS.ACCEL_Y`                     |
| `dynamics.accel.vertical`          | m/s²  | [-327, +327]     | up+         | P   | `LINEAR_ACCEL_RPT.VERTICAL_ACCEL`                |
| `dynamics.angular_vel.yaw`         | rad/s | [-32.77, +32.77] | CCW+        | P+T | Toyota: `KINEMATICS.YAW_RATE` deg/s → rad/s      |
| `dynamics.angular_vel.pitch`       | rad/s | [-32.77, +32.77] | nose-up+    | P   | `ANG_VEL_RPT.PITCH_VEL`                          |
| `dynamics.angular_vel.roll`        | rad/s | [-32.77, +32.77] | right-down+ | P   | `ANG_VEL_RPT.ROLL_VEL`                           |
| `dynamics.steering.angle`          | rad   | [-8.73, +8.73]   | left+       | T   | `STEER_ANGLE_SENSOR.STEER_ANGLE` deg → rad       |
| `dynamics.steering.rate`           | rad/s | [-34.9, +34.9]   | left+       | T   | `STEER_ANGLE_SENSOR.STEER_RATE` deg/s → rad/s    |
| `dynamics.steering.torque_driver`  | —     | [-32768, +32767] | —           | T   | `STEER_TORQUE_SENSOR.STEER_TORQUE_DRIVER` (raw)  |
| `dynamics.steering.torque_eps`     | —     | [-32768, +32767] | —           | T   | `STEER_TORQUE_SENSOR.STEER_TORQUE_EPS` (raw)     |

> **Note**: `operation.steering.report.angle` and `dynamics.steering.angle` may both represent the steering angle but from different sources:
>
> - PACMod: DBW output position (post-actuator)
> - Toyota `STEER_ANGLE_SENSOR`: column sensor (pre-actuator / driver input)

---

### Domain: `powertrain` — Engine and Drivetrain State

Topic: `/vehicle/powertrain`

| Canonical Name                        | Unit  | Range        | DBC | Notes                                                  |
| ------------------------------------- | ----- | ------------ | --- | ------------------------------------------------------ |
| `powertrain.engine.running`           | bool  | {0,1}        | T   | `ENGINE_RPM.ENGINE_RUNNING`                            |
| `powertrain.engine.rpm`               | rpm   | [0, 16383]   | P+T | Toyota: `ENGINE_RPM.RPM`                               |
| `powertrain.engine.torque_actual`     | Nm    | [0, 4095]    | P   | `ENGINE_AUX_RPT.ENGINE_TORQUE`                         |
| `powertrain.engine.torque_demanded`   | %     | [-125, +125] | P   | `ENGINE_LOAD_FACTOR_RPT.DRVR_DEMANDED_ENG_TORQUE`      |
| `powertrain.engine.load`              | %     | [0, 250]     | P   | `ENGINE_LOAD_FACTOR_RPT.ENG_LOAD_AT_CURRENT_SPEED`     |
| `powertrain.engine.coolant_temp`      | deg_C | [-40, +210]  | P   | `ENGINE_AUX_RPT.ENGINE_COOLANT_TEMP`                   |
| `powertrain.fuel.level`               | ratio | [0, 1]       | P   | `ENGINE_AUX_RPT.FUEL_LEVEL`                            |
| `powertrain.fuel.consumption_rate`    | L/h   | [0, 3213]    | P   | `ENGINE_AUX_RPT_2.FUEL_RATE`                           |
| `powertrain.oil.level`                | ratio | [0, 1]       | P   | `ENGINE_AUX_RPT_2.OIL_LEVEL`                           |
| `powertrain.oil.pressure`             | kPa   | [0, 1000]    | P   | `ENGINE_AUX_RPT_2.OIL_PRESSURE`                        |
| `powertrain.transmission.gear_actual` | enum  | [0, 5]       | P+T | Same as Gear enum; Toyota: `GEAR_PACKET.GEAR` remapped |
| `powertrain.transmission.gear_ratio`  | ratio | [0, 65.5]    | P   | `SHIFT_AUX_RPT.ACTUAL_GEAR_RATIO`                      |
| `powertrain.battery.voltage_12v`      | V     | [0, 50]      | P   | `BATTERY_VOLTAGE_LEVEL_RPT_1.BATTERY_VOLTAGE_1`        |

---

### Domain: `chassis` — Mechanical and Active Safety State

Topic: `/vehicle/chassis`

| Canonical Name                      | Unit | Range     | DBC | Notes                                             |
| ----------------------------------- | ---- | --------- | --- | ------------------------------------------------- |
| `chassis.tire_pressure.front_left`  | kPa  | [0, 1020] | P   |                                                   |
| `chassis.tire_pressure.front_right` | kPa  | [0, 1020] | P   |                                                   |
| `chassis.tire_pressure.rear_left`   | kPa  | [0, 1020] | P   |                                                   |
| `chassis.tire_pressure.rear_right`  | kPa  | [0, 1020] | P   |                                                   |
| `chassis.brake_pressure.circuit_1`  | kPa  | [0, 2000] | P   | `AIR_PRESSURE_RPT.BRAKE_CIRCUIT_1_PRESSURE`       |
| `chassis.brake_pressure.circuit_2`  | kPa  | [0, 2000] | P   |                                                   |
| `chassis.air_pressure.supply`       | kPa  | [0, 2000] | P   | Pneumatic trucks only                             |
| `chassis.safety.abs_active`         | bool | {0,1}     | P   | `DRIVE_TRAIN_FEATURE_RPT.ANTILOCK_BRAKE_ACTIVE`   |
| `chassis.safety.tcs_active`         | bool | {0,1}     | P   | `DRIVE_TRAIN_FEATURE_RPT.TRACTION_CONTROL_ACTIVE` |
| `chassis.safety.esc_active`         | bool | {0,1}     | P   | ESP/ESC                                           |
| `chassis.safety.abs_disabled`       | bool | {0,1}     | P   | `DRIVE_TRAIN_FEATURE_RPT.ANTILOCK_BRAKE_DISABLED` |
| `chassis.safety.tcs_disabled`       | bool | {0,1}     | T   | `ESP_CONTROL.TC_DISABLED`                         |
| `chassis.safety.vsc_disabled`       | bool | {0,1}     | T   | `ESP_CONTROL.VSC_DISABLED`                        |

---

### Domain: `body` — Body, Cabin, Lighting, Climate

Topic: `/vehicle/body`

| Canonical Name                  | Unit  | DBC | Notes                                                         |
| ------------------------------- | ----- | --- | ------------------------------------------------------------- |
| `body.door.driver_open`         | bool  | P+T | `DOOR_RPT.DRIVER_DOOR_OPEN` / `BODY_CONTROL_STATE`            |
| `body.door.passenger_open`      | bool  | P+T |                                                               |
| `body.door.rear_driver_open`    | bool  | P+T |                                                               |
| `body.door.rear_passenger_open` | bool  | P+T |                                                               |
| `body.door.hood_open`           | bool  | P   |                                                               |
| `body.door.trunk_open`          | bool  | P   |                                                               |
| `body.door.fuel_door_open`      | bool  | P   |                                                               |
| `body.seatbelt.driver`          | bool  | P   | `OCCUPANCY_RPT.DRIVER_SEATBELT_BUCKLED`                       |
| `body.seatbelt.passenger`       | bool  | P   |                                                               |
| `body.seatbelt.rear_driver`     | bool  | P   |                                                               |
| `body.seatbelt.rear_passenger`  | bool  | P   |                                                               |
| `body.occupancy.driver_seat`    | bool  | P   | `OCCUPANCY_RPT.DRIVER_SEAT_OCCUPIED`                          |
| `body.occupancy.passenger_seat` | bool  | P   |                                                               |
| `body.occupancy.rear_seat`      | bool  | P   |                                                               |
| `body.lights.turn_signal`       | enum  | P+T | See Turn Signal enum; Toyota: `BLINKERS_STATE.TURN_SIGNALS`   |
| `body.lights.headlight_mode`    | enum  | P+T | See Headlight Mode enum; Toyota: `LIGHT_STALK.HEADLIGHT_MODE` |
| `body.lights.fog_lights_front`  | bool  | P+T | `HEADLIGHT_AUX_RPT.FOG_LIGHTS_ON` / `LIGHT_STALK.FRONT_FOG`   |
| `body.lights.brake_lights`      | bool  | P   | `REAR_LIGHTS_RPT.BRAKE_LIGHTS_ON`                             |
| `body.lights.reverse_lights`    | bool  | P   | `REAR_LIGHTS_RPT.REVERSE_LIGHTS_ON`                           |
| `body.lights.daytime_running`   | bool  | T   | `LIGHT_STALK.DAYTIME_RUNNING_LIGHT`                           |
| `body.climate.interior_temp`    | deg_C | P   | `AMBIENT_CONDITION_RPT.INTERIOR_AMBIENT_TEMP`                 |
| `body.climate.set_point`        | deg_C | P+T | `CABIN_TEMP_RPT` / `AIR_CONDITIONER.CLIMATE_SET_POINT`        |
| `body.climate.ac_on`            | bool  | T   | `AIR_CONDITIONER.AIR_CONDITIONER_STATE` > 0                   |
| `body.climate.defrost_on`       | bool  | P+T | `CABIN_CLIMATE_RPT` / `AIR_CONDITIONER_2.WINDSCREEN_DEFOG`    |

---

### Domain: `adas` — Advanced Driver Assistance Features

Topic: `/vehicle/adas`

| Canonical Name                         | Unit | Range        | DBC | Notes                                                 |
| -------------------------------------- | ---- | ------------ | --- | ----------------------------------------------------- |
| `adas.cruise.main_on`                  | bool | {0,1}        | P+T | Toyota: `PCM_CRUISE_2.MAIN_ON`                        |
| `adas.cruise.state`                    | enum | [0,4]        | T   | `PCM_CRUISE_SM.CRUISE_CONTROL_STATE` remapped         |
| `adas.cruise.set_speed`                | m/s  | [0, 70]      | P+T | Toyota: `PCM_CRUISE_2.SET_SPEED` km/h → m/s           |
| `adas.acc.distance_setting`            | enum | [1,4]        | T   | `PCM_CRUISE_2.PCM_FOLLOW_DISTANCE`                    |
| `adas.acc.lead_vehicle.speed`          | m/s  | [-100, +100] | T   | `LEAD_INFO.LEAD_REL_SPEED` (relative → add ego speed) |
| `adas.acc.lead_vehicle.distance`       | m    | [0, 300]     | T   | `LEAD_INFO.LEAD_LONG_DIST`                            |
| `adas.acc.lead_vehicle.detected`       | bool | {0,1}        | P   | `DETECTED_OBJECT_RPT` distance < threshold            |
| `adas.acc.accel_command`               | m/s² | [-20, +20]   | T   | `ACC_CONTROL.ACCEL_CMD`                               |
| `adas.lane_keeping.active`             | bool | {0,1}        | T   | Derived from `LKAS_HUD` state                         |
| `adas.lane_keeping.lda_state`          | enum | —            | T   | `LKAS_HUD` — vehicle-specific                         |
| `adas.collision.fcw_active`            | bool | {0,1}        | T   | `PCS_HUD.FCW`                                         |
| `adas.collision.pcs_braking`           | bool | {0,1}        | T   | `PCS_HUD.PCS_INDICATOR`                               |
| `adas.collision.front_object_distance` | m    | [0, 16777]   | P   | `DETECTED_OBJECT_RPT.FRONT_OBJECT_DISTANCE_HIGH_RES`  |
| `adas.road_sign.speed_limit`           | km/h | [0, 255]     | T   | `RSA1.SPDVAL1`                                        |

---

### Domain: `system` — Autonomy and Safety System Health

Topic: `/vehicle/system`

| Canonical Name                    | Unit | DBC | Notes                                           |
| --------------------------------- | ---- | --- | ----------------------------------------------- |
| `system.autonomy.enabled`         | bool | P   | `GLOBAL_RPT_2.SYSTEM_READY`                     |
| `system.autonomy.engaged`         | bool | P   | `GLOBAL_RPT_2.SYSTEM_ENABLED`                   |
| `system.autonomy.override_active` | bool | P   | `GLOBAL_RPT_2.SYSTEM_OVERRIDE_ACTIVE`           |
| `system.autonomy.fault_active`    | bool | P   | `GLOBAL_RPT_2.SYSTEM_FAULT_ACTIVE`              |
| `system.safety_function.state`    | enum | P   | `SAFETY_FUNC_RPT.STATE` — see Safety State enum |
| `system.estop.active`             | bool | P   | `ESTOP_RPT.ESTOP`                               |
| `system.remote_stop.active`       | bool | P   | `REMOTE_STOP_RPT.REMOTE_STOP_COMMAND` > 0       |
| `system.fault.engine`             | bool | P   | `VEHICLE_FAULT_RPT.ENGINE_CHECK_LIGHT`          |
| `system.fault.brake`              | bool | P   | `VEHICLE_FAULT_RPT.BRAKE_FAULT`                 |
| `system.fault.steering`           | bool | P   | `VEHICLE_FAULT_RPT.STEERING_LOSS_STOP_SAFELY`   |
| `system.fault.transmission`       | bool | P   | `VEHICLE_FAULT_RPT.XMSN_FAULT_SERVICE_NOW`      |
| `system.fault.abs`                | bool | P   | `VEHICLE_FAULT_RPT.ANTILOCK_BRAKE_FAULT_LIGHT`  |
| `system.fault.traction_control`   | bool | P   | `VEHICLE_FAULT_RPT.TRC_FAULT_LIGHT`             |
| `system.fault.airbag`             | bool | P   | `VEHICLE_FAULT_RPT.AIR_BAGS_FAULT_LIGHT`        |
| `system.fault.tire_pressure`      | bool | P   | `VEHICLE_FAULT_RPT.TIRE_FAULT_LIGHT`            |

---

### Domain: `location` — GNSS Position and Time

Topic: `/vehicle/location`

| Canonical Name            | Unit | Range        | DBC | Notes                          |
| ------------------------- | ---- | ------------ | --- | ------------------------------ |
| `location.gnss.latitude`  | deg  | [-90, +90]   | P   | `LAT_LON_HEADING_RPT` DMS → DD |
| `location.gnss.longitude` | deg  | [-180, +180] | P   |                                |
| `location.gnss.heading`   | deg  | [0, 360)     | P   | True north = 0, East = 90      |
| `location.time.year`      | yr   | [2000, 2255] | P   | `DATE_TIME_RPT.DATE_YEAR`      |
| `location.time.month`     | mon  | [1, 12]      | P   |                                |
| `location.time.day`       | dy   | [1, 31]      | P   |                                |
| `location.time.hour`      | hr   | [0, 23]      | P   |                                |
| `location.time.minute`    | min  | [0, 59]      | P   |                                |
| `location.time.second`    | sec  | [0, 60]      | P   |                                |
| `location.odometer`       | km   | [0, 1048575] | T   | `UI_SETTING.ODOMETER`          |

---

## Adding a New DBC

1. Run `tools/dbc_to_yaml.py --dbc <file>` to generate a signal catalog
2. Compare the catalog against this document to find matching signals
3. Add `aliases.*` mappings (DBC signal name → canonical name)
4. Add `transforms.*` entries for unit/sign conversions
5. Signals without a match will automatically output as `STATUS_INITIAL`
6. Write tests verifying canonical values for representative CAN frames
