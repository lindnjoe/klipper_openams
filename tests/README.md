# OpenAMS Test Framework

This test framework provides comprehensive testing of the OpenAMS system with fully mocked Klipper components.

## Overview

The test framework mocks all real-time and hardware dependencies to enable testing without a running Klipper system:

- **MockReactor**: Simulates Klipper's event reactor with timer management
- **MockMCU**: Simulates MCU communication for OAMS hardware
- **MockExtruder**: Simulates extruder movement and position tracking
- **MockPrinter**: Coordinates all mocked components
- **MockConfig**: Provides configuration parsing

## Test Scenarios

### 1. Load Test (`test_load_success`)
**Purpose**: Verify successful filament loading from AMS to toolhead

**Process**:
1. Sets up unloaded state
2. Triggers load command for specific filament group
3. Mocks successful OAMS response
4. Verifies state transitions and command execution

### 2. Unload Test (`test_unload_success`) 
**Purpose**: Verify successful filament unloading from toolhead back to AMS

**Process**:
1. Sets up loaded state with active spool
2. Triggers unload command
3. Mocks successful OAMS response
4. Verifies state transitions and cleanup

### 3. Load Encoder Stuck (`test_load_encoder_stuck`)
**Purpose**: Verify detection when encoder shows no progress during loading

**Process**:
1. Sets up loading state
2. Simulates stuck encoder (no movement)
3. Advances time past monitoring threshold
4. Verifies encoder monitoring detects lack of progress

### 4. Unload Encoder Stuck (`test_unload_encoder_stuck`)
**Purpose**: Verify detection when encoder shows no progress during unloading

**Process**:
1. Sets up unloading state
2. Simulates stuck encoder (no movement)
3. Advances time past monitoring threshold
4. Verifies encoder monitoring detects lack of progress

### 5. Runout Successful Replacement (`test_runout_successful_replacement`)
**Purpose**: Verify automatic filament switching when runout is detected

**Process**:
1. Sets up printing state with loaded filament
2. Configures backup spools in same filament group
3. Simulates runout (hub sensor goes inactive)
4. Verifies state transitions: MONITORING → DETECTED → COASTING → RELOADING
5. Simulates extruder movement through pause and coast distances
6. Verifies automatic reload is triggered

### 6. Runout No Backup Pause (`test_runout_no_backup_pause`)
**Purpose**: Verify printer pause when runout occurs with no backup filament

**Process**:
1. Sets up printing state with no backup spools available
2. Simulates runout condition
3. Advances through full runout sequence
4. Verifies printer pause command is issued

## Running Tests

### Method 1: Direct execution
```bash
cd tests
python test_oams_system.py
```

### Method 2: Using runner script
```bash
cd tests
python run_tests.py
```

### Method 3: With unittest module
```bash
cd tests
python -m unittest test_oams_system.OAMSTestSuite
```

## Mock Component Details

### MockReactor
- **Time Management**: Simulates time advancement and timer firing
- **Timer Registration**: Tracks active timers and their callbacks
- **Event Scheduling**: Processes timer events in chronological order

```python
reactor.advance_time(5.0)  # Advance 5 seconds and fire timers
```

### MockMCU
- **Command Mocking**: Captures MCU commands for verification
- **Response Simulation**: Provides canned responses for queries
- **Config Tracking**: Records configuration commands

### MockExtruder
- **Position Tracking**: Maintains current extruder position
- **Movement Simulation**: Allows advancing position for testing

```python
extruder.advance_position(100.0)  # Simulate 100mm movement
```

### Test State Setup
Each test sets up realistic component states:

```python
# OAMS with spools ready
self.oams1.f1s_hes_value = [1, 1, 0, 0]  # Bays 0,1 have filament
self.oams1.hub_hes_value = [0, 0, 0, 0]  # No spools loaded

# FPS connected to OAMS
self.fps.oams = [self.oams1]

# Filament groups
self.group_t0.bays = [(self.oams1, 0), (self.oams1, 1)]
```

## Key Testing Features

### State Verification
Tests verify state transitions through the complete OAMS state machine:
- FPS states: UNLOADED → LOADING → LOADED → UNLOADING
- Runout states: MONITORING → DETECTED → COASTING → RELOADING

### Timer Testing
The MockReactor enables testing time-dependent behavior:
- Encoder monitoring delays
- Runout detection timing
- Pause distance calculations

### Error Condition Testing
Tests verify proper handling of error conditions:
- Stuck encoders
- Missing backup filament
- Hardware communication failures

### Integration Testing
Tests verify coordination between components:
- FPS ↔ OAMS communication
- FilamentGroup ↔ OAMS coordination
- OAMSManager orchestration

## Extending Tests

To add new test scenarios:

1. **Create test method** following naming convention `test_scenario_name`
2. **Set up initial state** using mock objects
3. **Simulate events** using mock methods and time advancement
4. **Verify results** using assertions

Example:
```python
def test_new_scenario(self):
    # Setup
    fps_state = self.manager.current_state.fps_state['fps extruder']
    fps_state.state_name = FPSLoadState.LOADED
    
    # Simulate event
    self.simulate_some_condition()
    self.printer.reactor.advance_time(5.0)
    
    # Verify
    self.assertEqual(fps_state.state_name, FPSLoadState.EXPECTED)
```

## Benefits

1. **No Hardware Required**: Tests run without physical OAMS hardware
2. **Deterministic**: Mocked timing enables repeatable tests
3. **Comprehensive**: Tests cover normal and error scenarios
4. **Fast**: No real-time delays, tests complete quickly
5. **Isolated**: Each test starts with clean state
6. **Debuggable**: Full visibility into component interactions
