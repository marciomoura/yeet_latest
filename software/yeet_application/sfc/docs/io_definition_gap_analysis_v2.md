# SFC Slow IO Definition - Gap Analysis

**Date:** October 12, 2025  
**Scope:** Slow IO interface for TCS, Excitation Controller, and Auxiliary Systems  
**Document:** Analysis of `sfc_io_definition.md`

---

## Important: Scope of This IO Definition

### What IS Included (Slow IO)
✅ Digital and analog signals for external systems:
- **Turbine Control System (TCS)** - operational commands and status feedback
- **External Excitation Controller** - field control interface  
- **Auxiliary PLCs** - cooling fans, breakers, protection relays
- **Customer interfaces** - supervisory monitoring and control

**Update Rates:** 10-100 ms (slow supervisory level)

### What is NOT Included (handled elsewhere)
❌ **High-speed control signals** - handled by real-time embedded controller:
- Grid voltage measurements (ADC, 20 kHz sampling)
- DC link current (ADC, 20 kHz sampling)
- Machine voltage measurements (ADC, 20 kHz sampling)
- Thyristor firing pulses (hardware pulse generators)
- Commutation monitoring (real-time logic)
- Fast protection (hardware interlocks, < 1 ms response)

**Update Rates:** 20-50 kHz (real-time control loop)

---

## Executive Summary

### Overall Assessment: ✅ EXCELLENT - Nearly Complete

The slow IO definition is **well-structured and comprehensive** for TCS and excitation controller interface. Key findings:

**Strengths:**
- ✅ Comprehensive TCS command interface (start, stop, modes)
- ✅ Good protection signal coverage (temps, trips, alarms)
- ✅ Excitation controller command interface (on/off, mode)
- ✅ Excitation analog feedback (current, voltage) - sufficient for deriving field establishment
- ✅ Auxiliary system control (fans, breakers)

**Minor Issues Identified:**
1. ⚠️ **Naming:** DO19/DO20 English-Chinese translation mismatch (documentation fix)
2. ⚠️ **Unclear:** Excitation analog output (AO1 missing, only AO2 defined - needs clarification)
3. ⚠️ **Organization:** Some protection signals split across multiple modules (optional consolidation)

**Resolved (No Action Needed):**
- ✅ **Excitation field established:** Can be derived from AI1 (current) + AI2 (voltage) using event lambda
- ✅ **DI3 can remain SPARE** - no additional digital input required

**Risk Level:** VERY LOW - All critical interfaces present, only minor documentation clarifications needed

---

## 1. Digital Inputs - Detailed Analysis

### 1.1 Module 1 (TCS Commands) - ✅ COMPLETE

**Assessment:** Excellent coverage of turbine control interface

| Signal Group        | Signals | Status     |
| ------------------- | ------- | ---------- |
| Start/Stop Commands | DI1-2   | ✅ Complete |
| Speed/Mode Commands | DI3-10  | ✅ Complete |
| Safety Interlocks   | DI13-16 | ✅ Complete |

**No gaps identified** - this module is production-ready.

---

### 1.2 Module 2 (Excitation & Protection) - ✅ COMPLETE (NO ADDITIONAL DI NEEDED)

#### Excitation Controller Interface

| Signal                    | Current Def | Assessment                   |
| ------------------------- | ----------- | ---------------------------- |
| DI1: EXC. RUN/STOP STATUS | ✅ Defined   | Exciter is running           |
| DI2: EXC. ACTIVE          | ✅ Defined   | Exciter voltage applied      |
| DI3: SPARE                | ✅ Available | Can remain spare (see below) |

**State Machine Integration:**

The state machine now has this logic (from recent code changes):
```cpp
// OPERATION_OFF state
if (inputs.start_command.is_true() && inputs.excitation_is_ready.is_true()) {
    return _operation_on_state;  // Only transition when field is ready
}
```

**How to Implement `excitation_is_ready` (Two Options):**

**Option A: Derive from Analog Inputs ✅ RECOMMENDED**

Since `sfc_state_event` uses `std::function`, we can create the event from existing analog inputs:

```cpp
// In state machine runner initialization
sfc_state_inputs inputs{
    // ... other events ...
    
    // Derive excitation ready from analog measurements (AI1, AI2)
    sfc_state_event("excitation_is_ready", [this]() {
        // Read analog inputs
        float exc_current = analog_inputs.exc_current_feedback.read();  // AI1
        float exc_voltage = analog_inputs.exc_voltage_feedback.read();  // AI2
        
        // Field is established when both exceed thresholds
        const float current_threshold = 0.10;  // 10% of rated field current
        const float voltage_threshold = 0.80;  // 80% of rated field voltage
        
        bool current_ok = (exc_current > current_threshold);
        bool voltage_ok = (exc_voltage > voltage_threshold);
        
        return current_ok && voltage_ok;
    })
};
```

**Advantages:**
- ✅ No additional hardware signal required
- ✅ Uses existing analog feedback (AI1: Exc. Current, AI2: Exc. Voltage)
- ✅ Thresholds configurable in software
- ✅ DI3 remains available for future use
- ✅ More robust (checks both voltage AND current)

**Option B: Use Dedicated Digital Input (If Exciter Provides It)**

If the external exciter has a dedicated "Field Established" discrete output:
```
DI3: EXCITATION FIELD ESTABLISHED (励磁建场完成) → Optional
  Description: Discrete signal from exciter indicating field is ready
  Source: External Excitation Controller
  Maps to: state_machine.inputs.excitation_is_ready
```

Then use simple event:
```cpp
sfc_state_event("excitation_is_ready", [this]() {
    return digital_inputs.excitation_field_established.read();  // DI3
})
```

**Recommendation:** 
- **Use Option A** (derive from AI1, AI2) - more flexible, no additional wiring
- **DI3 remains SPARE** unless exciter vendor specifically provides discrete field-ready output
- **No IO definition change required** - existing analog inputs are sufficient

---

#### Circuit Breaker Feedback - ✅ COMPLETE

| Signal                       | Status    | Assessment |
| ---------------------------- | --------- | ---------- |
| DI5-6: ICB Position Feedback | ✅ Present | Complete   |
| DI7: ICB Tripping Status     | ✅ Present | Complete   |

**No gaps** - breaker interface is well-defined.

---

#### Protection System Inputs - ✅ COMPREHENSIVE

| Signal Group           | Signals      | Coverage                        |
| ---------------------- | ------------ | ------------------------------- |
| Transformer Protection | DI8, DI11-12 | Pressure + Buchholz             |
| Thermal Protection     | DI9-10       | Alarm + Trip levels             |
| Emergency / Faults     | DI13-15      | E-stop, HW trip, External fault |

**Assessment:** Protection coverage is **excellent** for slow IO level. Fast hardware protection (overcurrent, overvoltage within microseconds) is correctly handled elsewhere.

---

### 1.3 Power Cabinet Module - ⚠️ ORGANIZATION ISSUE

| Signal                | Current Location     | Recommendation                                  |
| --------------------- | -------------------- | ----------------------------------------------- |
| DI1: Overvoltage Fuse | Power Cabinet Module | Could move to Module 2 (consolidation)          |
| DI2: Fan Stop Alarm   | Power Cabinet Module | Could move to Module 2 (consolidation)          |
| DI4-5: Reactor Temp   | Power Cabinet Module | Keep here (physically located in power cabinet) |

**Not a functional gap**, but consider consolidating **all protection trip signals** into Module 2 for easier software integration.

---

## 2. Digital Outputs - Detailed Analysis

### 2.1 Auxiliary Control - ✅ COMPLETE

| Function        | Signals     | Status                         |
| --------------- | ----------- | ------------------------------ |
| Fan Control     | DO1-4       | ✅ Complete (4 fans)            |
| Breaker Control | DO5-6, DO27 | ✅ Complete (close, open, trip) |

**No gaps** - auxiliary control is well-defined.

---

### 2.2 TCS Feedback Signals - ⚠️ GAPS AND NAMING ISSUES

#### Status Feedback

| Signal                 | Current Definition             | Issue           |
| ---------------------- | ------------------------------ | --------------- |
| DO19: OPERATION OFF FB | Chinese: "SFC已运行" (Running) | ❌ **BACKWARDS** |
| DO20: OPERATION ON FB  | Chinese: "SFC已停止" (Stopped) | ❌ **BACKWARDS** |

**Critical Naming Error:**
- English: "OPERATION OFF FB" should mean "system is OFF"
- Chinese: "SFC已运行" means "SFC is RUNNING"
- **These are contradictory!**

**Correct Mapping Should Be:**
```
DO19: OPERATION OFF FB → "SFC已停止" (SFC Stopped)
  True when state machine is in OPERATION_OFF or earlier states
  
DO20: OPERATION ON FB → "SFC已运行" (SFC Running)
  True when state machine is in OPERATION_ON state
```

---

#### Missing TCS Feedback for Excitation

| Needed Signal              | Currently     | Recommendation                    |
| -------------------------- | ------------- | --------------------------------- |
| Field establishment status | ❌ Not defined | Add DO-NEW: FIELD ESTABLISHED FB  |
| Pre-fluxing indication     | ❌ Not defined | Optional: PRE-FLUXING IN PROGRESS |

**Proposed Addition:**
```
DO-NEW (use spare): EXCITATION FIELD ESTABLISHED (励磁建场完成) → To TCS
  Description: Field winding flux established, system ready for operation
  Source: Read DI3 (excitation_is_ready) and relay to TCS
  Purpose: Inform operator of excitation establishment progress
```

This allows TCS to display: "Waiting for field establishment..." during the 8-10 second pre-fluxing period.

---

### 2.3 Excitation Controller Interface - ⚠️ NEEDS CLARIFICATION

#### Command Outputs

| Signal                  | Status    | Purpose                     |
| ----------------------- | --------- | --------------------------- |
| DO28: ON CMD            | ✅ Defined | Start excitation            |
| DO29: OFF CMD           | ✅ Defined | Stop excitation             |
| DO30: EXC. CONTROL MODE | ⚠️ Unclear | **What does this control?** |

**Question for exciter vendor:** What is DO30 used for?
- Voltage mode vs. current mode selection?
- Auto mode vs. manual mode?
- Local vs. remote control?

**Recommendation:** Document the purpose of DO30 in the IO definition.

---

## 3. Analog I/O - Detailed Analysis

### 3.1 Analog Inputs - ✅ APPROPRIATE FOR SLOW IO

| Signal                     | Status    | Purpose                  |
| -------------------------- | --------- | ------------------------ |
| AI1: Exc. Current Feedback | ✅ Present | Field current monitoring |
| AI2: Exc. Voltage Feedback | ✅ Present | Field voltage monitoring |

**Assessment:** These signals are **correctly scoped** for supervisory monitoring of the external excitation controller. They provide:
- Operator display values
- Trend logging
- Alarm monitoring (field current too high/low)

**Update rate:** 10-100 ms (adequate for slow IO)

**Note:** High-speed measurements (grid voltage, DC current, machine voltage) are correctly handled by the real-time controller's ADC system, not through this slow IO interface.

**No gaps identified.**

---

### 3.2 Analog Outputs - ⚠️ MISSING AO1 + UNCLEAR MODE

| Signal                 | Status        | Gap Analysis                         |
| ---------------------- | ------------- | ------------------------------------ |
| **AO1:**               | ❌ Not defined | **Why does numbering start at AO2?** |
| AO2: Exc. Current Ref. | ✅ Defined     | Field current command                |

**Issue 1: Missing AO1**

Possible explanations:
1. AO1 reserved for voltage reference (see Issue 2)
2. AO1 used internally or for another purpose
3. Numbering error in documentation

**Issue 2: Voltage vs. Current Control Mode**

The `sfc_excitation_controller` algorithm outputs:
```cpp
voltage_pu_t get_excitation_voltage_setpoint_pu() const noexcept;
```

But the IO definition only has:
```
AO2: Exc. Current Ref. (励磁电流给定)
```

**Possible scenarios:**

**Scenario A: Exciter is current-controlled**
- SFC controller outputs voltage setpoint (internal)
- Conversion: voltage_pu → current_pu using machine parameters
- Output AO2: current reference to exciter
- **Action:** Document the voltage-to-current conversion in software

**Scenario B: Exciter supports voltage mode**
- AO1: Voltage reference (primary mode)
- AO2: Current reference (backup mode)
- DO30: Mode selector
- **Action:** Add AO1 definition to IO document

**Scenario C: Dual outputs for different control strategies**
- Both AO1 and AO2 are active simultaneously
- Exciter uses one or both depending on mode
- **Action:** Clarify with exciter vendor

**Recommendation:**
1. **Immediate:** Document what AO1 is (or should be)
2. **Clarify:** What is the exciter's primary control mode?
3. **If voltage mode:** Add `AO1: Exc. Voltage Ref.` to IO definition
4. **If current mode:** Document the voltage→current conversion algorithm

---

## 4. State Machine Integration

### 4.1 Required Signals for State Machine Operation

Based on `sfc_state_machine.cpp` implementation:

| State Machine Input       | IO Signal                        | Status          |
| ------------------------- | -------------------------------- | --------------- |
| `auxiliaries_are_on`      | DI14-15 (composite)              | ✅ Mapped        |
| `breakers_are_on`         | DI5 (CLOSED FB)                  | ✅ Mapped        |
| `start_command`           | DI1 (OPERATION ON CMD)           | ✅ Mapped        |
| `stop_command`            | DI2 (OPERATION OFF CMD)          | ✅ Mapped        |
| `shutdown_request`        | DI13 (EMERGENCY OFF)             | ✅ Mapped        |
| `is_current_extinguished` | (internal real-time measurement) | ✅ Correct scope |
| **`excitation_is_ready`** | **DI3 (assign SPARE)**           | ❌ **GAP**       |

| State Machine Output    | IO Signal                      | Status          |
| ----------------------- | ------------------------------ | --------------- |
| `pulse_releases`        | (internal to pulse generators) | ✅ Correct scope |
| `freeze_integrator`     | (internal to controllers)      | ✅ Correct scope |
| **`enable_excitation`** | DO28 (ON CMD)                  | ✅ Mapped        |

**One critical gap:** Assign DI3 to excitation field established feedback.

---

### 4.2 Auxiliary Systems Health Aggregation

**Current Inputs:**
- DI14: LV-SUPERVISION
- DI15: SUPERVISION
- DI16: MCB OPEN ALARM
- DI13: FRONT DOOR OPEN
- Power Cabinet DI2: Fan Stop

**State machine needs:**
```cpp
bool auxiliaries_are_on = compute_system_healthy();
```

**Recommendation:** Create composite logic in software:
```cpp
bool compute_auxiliaries_healthy() {
    bool power_ok = read_di(DI15_SUPERVISION);
    bool lv_ok = read_di(DI14_LV_SUPERVISION);
    bool fans_ok = !read_di(POWER_CABINET_DI2_FAN_STOP);
    bool mcb_ok = !read_di(DI16_MCB_OPEN);
    bool door_ok = !read_di(DI13_DOOR_OPEN);
    bool no_hw_trip = !read_di(DI14_COMMON_HW_TRIP);
    
    return power_ok && lv_ok && fans_ok && mcb_ok && door_ok && !no_hw_trip;
}
```

Map this composite signal to `state_machine.inputs.auxiliaries_are_on`.

---

## 5. Summary and Recommendations

### 5.1 Critical Actions (Must Fix)

| Priority       | Action                                   | Impact                                   | Effort |
| -------------- | ---------------------------------------- | ---------------------------------------- | ------ |
| 🔴 **CRITICAL** | Fix DO19/DO20 naming (English ↔ Chinese) | Operator confusion, wrong status display | 5 min  |
| 🟠 **HIGH**     | Clarify AO1 purpose (voltage reference?) | Controller output not connected          | 1 hour |

---

### 5.2 Important Actions (Should Fix)

| Priority     | Action                                                   | Impact                             | Effort  |
| ------------ | -------------------------------------------------------- | ---------------------------------- | ------- |
| 🟠 **HIGH**   | Document DO30 (EXC. CONTROL MODE) purpose                | Unclear interface to exciter       | 30 min  |
| 🟡 **MEDIUM** | Add TCS feedback: Field Established status (optional DO) | Operator visibility during startup | 1 hour  |
| 🟡 **MEDIUM** | Consider consolidating protection signals                | Easier software integration        | 2 hours |

---

### 5.3 Optional Enhancements

| Priority  | Action                              | Benefit                                   |
| --------- | ----------------------------------- | ----------------------------------------- |
| 🟢 **LOW** | Add individual fan running feedback | Detect "commanded but not running" faults |
| 🟢 **LOW** | Add pre-fluxing indication to TCS   | Better operator awareness                 |

---

### 5.4 Resolved (No Action Required) ✅

| Item                               | Resolution                                                                                         |
| ---------------------------------- | -------------------------------------------------------------------------------------------------- |
| **Excitation Field Established**   | ✅ Derive from AI1 (Exc. Current) + AI2 (Exc. Voltage) using event lambda - no additional DI needed |
| **DI3 Status**                     | ✅ Can remain SPARE - not required for excitation field detection                                   |
| **State Machine Transition Logic** | ✅ Implemented using `std::function` event with analog input thresholds                             |

---

## 6. Implementation Example: Excitation Ready Event

### Recommended Implementation in State Machine Runner

```cpp
// In sfc_state_machine_runner.hpp
class sfc_state_machine_runner {
private:
    // Analog input references
    signal_input<float>& _exc_current_feedback;  // AI1
    signal_input<float>& _exc_voltage_feedback;  // AI2
    
    // Configurable thresholds
    parameter<float> _exc_current_threshold{0.10f};  // 10% of rated
    parameter<float> _exc_voltage_threshold{0.80f};  // 80% of rated
    
public:
    sfc_state_machine_runner()
        : _state_machine_inputs{
              sfc_state_event("auxiliaries_are_on", [this]() { 
                  return _inputs.auxiliaries_are_on.read(); 
              }),
              sfc_state_event("breakers_are_on", [this]() { 
                  return _inputs.breakers_are_on.read(); 
              }),
              // ... other events ...
              
              // Excitation ready derived from analog inputs
              sfc_state_event("excitation_is_ready", [this]() {
                  float current = _exc_current_feedback.read();  // AI1
                  float voltage = _exc_voltage_feedback.read();  // AI2
                  
                  bool current_ok = (current > _exc_current_threshold.value());
                  bool voltage_ok = (voltage > _exc_voltage_threshold.value());
                  
                  return current_ok && voltage_ok;
              })
          }
    {
    }
};
```

### Alternative: If Using Discrete DI3 Input

```cpp
// If external exciter provides dedicated field-ready output on DI3
sfc_state_event("excitation_is_ready", [this]() {
    return _inputs.excitation_field_established.read();  // DI3
})
```

---

## 7. Corrected IO Definition (Key Changes)

### Digital Input Module 2 (No Changes Required)

| Input | English Signal Definition | Chinese Signal Definition | From       |
| ----- | ------------------------- | ------------------------- | ---------- |
| DI1   | EXC. RUN/STOP STATUS      | 励磁已运行/停止           | Excitation |
| DI2   | EXC. ACTIVE               | 励磁已投励                | Excitation |
| DI3   | SPARE                     | 备用                      |            |
| DI4   | SPARE                     | 备用                      |            |
| ...   | (rest unchanged)          |                           |            |

**Note:** DI3 can remain SPARE - excitation field readiness is derived from AI1 + AI2 in software.

### Digital Output Module (Corrections)

| Output | English Signal Definition   | Chinese Signal Definition | To  |
| ------ | --------------------------- | ------------------------- | --- |
| ...    | (earlier outputs unchanged) |                           |     |
| DO19   | OPERATION OFF FB            | **SFC已停止** (corrected) | TCS |
| DO20   | OPERATION ON FB             | **SFC已运行** (corrected) | TCS |
| ...    | (rest unchanged)            |                           |     |

### Analog Output Module (Clarification Needed)

| Output  | English Signal Definition   | Chinese Signal Definition | To             |
| ------- | --------------------------- | ------------------------- | -------------- |
| **AO1** | **Exc. Voltage Ref. (TBD)** | **励磁电压给定 (待确认)** | **Excitation** |
| AO2     | Exc. Current Ref.           | 励磁电流给定              | Excitation     |

**Note:** AO1 purpose needs clarification with exciter vendor.

---

## 8. Validation Checklist

Before declaring IO definition complete, verify:

- [x] ~~DI3 assigned to excitation field established feedback~~ → NOT NEEDED (derived from AI1+AI2)
- [ ] Implement excitation_is_ready event using AI1 (current) + AI2 (voltage) thresholds
- [ ] DO19/DO20 Chinese translations corrected in all documentation
- [ ] AO1 purpose clarified (voltage ref or not used?)
- [ ] DO30 (EXC. CONTROL MODE) purpose documented
- [ ] State machine can successfully transition OPERATION_OFF → OPERATION_ON
- [ ] TCS receives correct status feedback during all states
- [ ] Excitation controller voltage setpoint properly converted to AO2 current reference (if applicable)
- [ ] Test excitation_is_ready event triggers correctly when field is established

---

**End of Gap Analysis**
