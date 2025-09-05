# OpenAMS Architecture Overview

This document provides a clear overview of the OpenAMS system components and their relationships.

## Core Components

### 1. FPS (Filament Pressure Sensor) - `fps.py`
**Purpose**: Monitors filament pressure and interfaces with OAMS units

**Key State Variables**:
- `fps_value: float` - Current pressure sensor reading (0.0-1.0)
- `extruder` - Reference to the associated extruder object
- `oams: List[OAMS]` - List of OAMS units this FPS can work with

**Configuration**:
- `pin` - ADC pin for pressure sensor
- `extruder` - Associated extruder name
- `oams` - Comma-separated list of OAMS units

### 2. OAMS (OpenAMS Hardware) - `oams.py`
**Purpose**: Controls filament spool hardware - loading, unloading, and following

**Key State Variables**:
- `current_spool: Optional[int]` - Currently loaded spool index (0-3), None if unloaded
- `f1s_hes_value: List[int]` - Filament sensor readings [bay0, bay1, bay2, bay3]
- `hub_hes_value: List[int]` - Hub sensor readings [bay0, bay1, bay2, bay3]
- `fps_value: float` - Current pressure sensor reading
- `encoder_clicks: int` - Current encoder position

**Hardware Status Constants**:
- `OAMSStatus.LOADING` - Currently loading filament
- `OAMSStatus.UNLOADING` - Currently unloading filament
- `OAMSStatus.FORWARD_FOLLOWING` - Following extruder forward
- `OAMSStatus.REVERSE_FOLLOWING` - Following extruder reverse
- `OAMSStatus.STOPPED` - Motor stopped, idle state
- `OAMSStatus.ERROR` - Error state requiring intervention

**Key Methods**:
- `is_bay_ready(bay_index)` - Check if spool bay has filament ready
- `is_bay_loaded(bay_index)` - Check if spool bay has filament loaded
- `load_spool(spool_idx)` - Load specific spool bay
- `unload_spool()` - Unload current spool
- `set_oams_follower(enable, direction)` - Control follower mode

### 3. FilamentGroup - `filament_group.py`
**Purpose**: Defines groups of interchangeable filament spools for automatic switching

**Key State Variables**:
- `group_name: str` - Name of the group (e.g., "T0", "T1", "T2")
- `bays: List[Tuple[OAMS, int]]` - List of (oams_object, bay_index) tuples
- `oams: List[OAMS]` - List of unique OAMS units referenced by this group

**Configuration Format**:
```ini
[filament_group T0]
group = "oams1-0", "oams1-1", "oams2-0"
```

**Key Methods**:
- `is_any_spool_loaded()` - Check if any spool in group is loaded
- `get_available_spools()` - Get list of ready-to-load spools
- `get_loaded_spool()` - Get currently loaded spool from group
- `get_next_available_spool()` - Get next spool for automatic loading
- `get_status()` - Get comprehensive group status

### 4. OAMSManager - `oams_manager.py`
**Purpose**: Main coordinator managing multiple FPS units, OAMS hardware, and filament groups

**Key Collections**:
- `fpss: Dict[str, FPS]` - FPS name → FPS object
- `oams: Dict[str, OAMS]` - OAMS name → OAMS object
- `filament_groups: Dict[str, FilamentGroup]` - Group name → FilamentGroup object
- `current_state: OAMSState` - Tracks state of all FPS units

## State Management

### FPS Load States (`FPSLoadState`)
- `UNLOADED` - No filament loaded
- `LOADED` - Filament loaded and ready
- `LOADING` - Currently loading filament
- `UNLOADING` - Currently unloading filament

### FPS State Tracking (`FPSState`)
**Per-FPS State Variables**:
- `state_name: str` - Current loading state (FPSLoadState)
- `current_group: Optional[str]` - Filament group name (e.g., "T0", "T1")
- `current_oams: Optional[str]` - OAMS unit name currently loaded
- `current_spool_idx: Optional[int]` - Spool bay index (0-3) currently loaded
- `following: bool` - Whether follower mode is active
- `direction: int` - Follower direction (0=forward, 1=reverse)
- `since: Optional[float]` - Timestamp when current state began
- `encoder_samples: deque` - Recent encoder readings for motion detection

### Runout Detection (`OAMSRunoutMonitor`)
**Runout States (`OAMSRunoutState`)**:
- `STOPPED` - Monitor is disabled
- `MONITORING` - Actively watching for runout
- `DETECTED` - Runout detected, pausing before coast
- `COASTING` - Follower coasting, preparing next spool
- `RELOADING` - Loading next spool in sequence
- `PAUSED` - Monitor paused due to error/manual intervention

## System Flow

### 1. Initialization
1. `OAMSManager` discovers all FPS and OAMS units
2. Creates `FPSState` tracking for each FPS
3. Determines current hardware state
4. Starts monitoring timers

### 2. Normal Operation
1. FPS monitors pressure continuously
2. `OAMSRunoutMonitor` watches for filament runout
3. OAMS units maintain follower mode during printing
4. State tracking keeps current load status updated

### 3. Runout Handling
1. `MONITORING` → `DETECTED` when hub sensor goes inactive
2. `DETECTED` → `COASTING` after pause distance traveled
3. `COASTING` → `RELOADING` when filament path cleared
4. Automatic loading of next spool in filament group

## Configuration Example

```ini
# FPS configuration - monitors pressure and connects to OAMS units
[fps extruder]
pin: analog1
extruder: extruder
oams: oams1, oams2

# OAMS hardware configuration - controls 4 spool bays each
[oams oams1]
mcu: oams1_mcu
oams_idx: 0
fps_upper_threshold: 0.8
fps_lower_threshold: 0.2
# Hall effect sensor thresholds for each bay
f1s_hes_on: 2.5, 2.5, 2.5, 2.5
hub_hes_on: 2.5, 2.5, 2.5, 2.5
# ... other OAMS config

[oams oams2]
mcu: oams2_mcu
oams_idx: 1
# ... similar config

# Filament group configuration - defines interchangeable spools
[filament_group T0]
group = "oams1-0", "oams1-1", "oams2-0"

[filament_group T1] 
group = "oams1-2", "oams1-3", "oams2-1"

# OAMS Manager configuration
[oams_manager]
reload_before_toolhead_distance: 10.0
```

## Key Concepts

- **FPS** manages one extruder and can work with multiple OAMS units
- **OAMS** manages 4 spool bays with sensors and motor control
- **Filament Groups** (T0, T1, etc.) map to specific OAMS bays
- **Runout Detection** automatically switches to next spool in group
- **State Tracking** maintains current load status across system
- **Follower Mode** allows OAMS to follow extruder movement
