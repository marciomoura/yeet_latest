
---

### **LCI Drive Control: The Pulse-Mode Startup Strategy**

#### 1. The Challenge: Starting a Synchronous Machine from Zero Speed

A Load-Commutated Inverter (LCI) is a simple, robust, and efficient topology for high-power motor drives. However, it has one fundamental limitation: its operation depends on a voltage source from the load to function.

This process, known as **load commutation**, relies on the synchronous machine's **back-EMF**—an AC voltage the machine generates internally, which is directly proportional to its rotational speed.

*   **At Medium to High Speed:** The machine generates a strong back-EMF. The LCI inverter uses this voltage to naturally turn off, or "commutate," the outgoing thyristors as new ones are fired. This is the drive's normal, efficient **Load-Commutated Mode**.
*   **At or Near Zero Speed:** The machine generates zero or negligible back-EMF. Without this commutating voltage, a conducting thyristor cannot be turned off, preventing the inverter from sequencing to its next state.

This inherent characteristic means a standard LCI cannot start a motor from a standstill using its normal operating principle.

#### 2. The Solution: Forced Commutation via Pulse Mode

**Pulse Mode** (also known as DC Chopper Mode or Forced Commutation Mode) is a special startup strategy that overcomes this limitation. The core concept is to cleverly **shift the responsibility of commutation from the machine-side inverter to the grid-side rectifier.**

Instead of relying on the machine's back-EMF, the system uses the grid-side rectifier to force the entire DC link current down to zero between each switching step of the inverter. When the current falls to zero, the machine-side thyristors naturally turn off, allowing the inverter to safely select the next pair of thyristors to conduct.

#### 3. Advanced Implementation: High-Performance Closed-Loop Control

While simple timed pulses can work, a modern, high-performance drive employs a closed-loop strategy for a startup that is smooth, robust, and independent of load variations. This is achieved through a **cascaded control structure** and a clear **decoupling of converter responsibilities**.

##### 3.1. Control Philosophy: Decoupling Torque and Position

The advanced pulse-mode strategy separates the control of torque magnitude from the control of the motor's rotor position.

| Feature                    | Pulse Mode (Low Speed, < 10-20%)                        | Load-Commutated Mode (High Speed)                    |
| :------------------------- | :------------------------------------------------------ | :--------------------------------------------------- |
| **Commutation Method**     | **Forced Commutation**                                  | **Load (Natural) Commutation**                       |
| **Who Commutates?**        | **Grid-side Rectifier** (by forcing DC current to zero) | **Machine-side Inverter** (using machine's back-EMF) |
| **DC Link Current**        | Discontinuous (pulsed)                                  | Continuous and smooth                                |
| **Machine Current**        | Blocky, discontinuous pulses                            | Quasi-square wave, continuous                        |
| **Torque**                 | Pulsating (smoothed by inertia)                         | Smooth                                               |
| **Required for Operation** | **Rotor Position Sensor** (Encoder/Resolver)            | Machine Back-EMF                                     |
| **Primary Use**            | Startup from 0 Hz and low-speed operation               | Normal medium and high-speed operation               |

##### 3.2. The Cascaded Control Architecture

The system is commanded by a high-level speed controller whose output is a **DC current setpoint** (which is a proxy for torque). This setpoint is fed to the pulse-mode controller, which contains a fast-acting, inner PI loop to regulate the current.

```
                  +---------+         +--------------------------------------------+
Speed Setpoint -->| Speed   |  DC     |          sfc_pulse_mode_controller         |
          (+)   ->| PI      |--Current->|             (Inner Current Loop)         |--> Grid Firing Angle
           -      | Ctrl    |  Setpoint|                                          |    (α_grid)
Measured Speed <--|         |         |  +-----------------+  Measured DC        |
                  +---------+         |  | Managed Current |<-Link Current         |
    (Outer Loop)                      |  |   PI Controller |                       |
                                      |  +-----------------+                       |
                                      +--------------------------------------------+

Rotor Position ----------------------------------------------------------------------> Machine Firing Angle
                                                                                         (α_machine)
```

##### 3.3. Managing the PI Controller for Pulse Duty

A standard PI controller cannot be used directly in this intermittent process due to the risk of integrator windup. Therefore, the controller must be **managed** by the main state machine. This involves enabling, disabling, and resetting the PI controller's state at precise moments.

**Required PI Controller Features:**
*   `.update(error)`: Standard PI calculation.
*   `.enable()` / `.disable()`: To start or stop the controller's output.
*   `.reset()`: To reset the integrator term to zero.
*   `.set_output_limits(min, max)`: To clamp the output firing angle.

The state machine orchestrates the PI controller to ensure it is only active when building a current pulse and is reset before every new pulse begins.

#### 4. The Pulse Cycle: A Detailed Walkthrough

1.  **State: `idle`**
    *   The system waits for an `enable` command and a non-zero `commanded_dc_current` from the outer speed loop. The PI controller is disabled and reset.

2.  **State: `setup_inverter`**
    *   **Trigger:** The `enable` command is active.
    *   **Action:**
        *   The controller inspects the absolute `rotor_position` to determine the next 60-degree electrical sector for the rotor to enter.
        *   It sets the `machine_firing_angle` to fire the thyristor pair that will produce torque in the correct direction.
        *   It resets the PI controller via `_pi_controller.reset()` to ensure a clean state.
        *   It transitions immediately to the `pulse_on` state.

3.  **State: `pulse_on`**
    *   **Trigger:** Entry from the `setup_inverter` state.
    *   **Action:**
        *   The PI controller is **enabled**. It actively regulates the `dc_link_current` to match the `commanded_dc_current` by dynamically adjusting the `grid_firing_angle`.
        *   The controller monitors the `rotor_position`, waiting for it to move into the target sector.
    *   **Exit Condition:** The rotor has successfully moved by one sector, confirming the pulse has done its work.

4.  **State: `force_commutation`**
    *   **Trigger:** The rotor position confirms the mechanical step is complete.
    *   **Action:**
        *   The PI controller is **disabled**. Its output is ignored.
        *   The `grid_firing_angle` is overridden with a deep inverting angle (e.g., 150°) to rapidly drive the DC link current to zero.
        *   The controller monitors the `dc_link_current`.
    *   **Exit Condition:** The `dc_link_current` drops below a zero threshold. The controller then transitions back to `setup_inverter` to begin the next pulse. A safety timeout also guards this state to detect hardware faults.

#### 5. The Handover to Load-Commutated Mode

As this pulse process accelerates the motor, its back-EMF increases. When the speed reaches a pre-defined threshold (typically 10-20% of nominal speed), the back-EMF is strong enough to support natural commutation.

At this point, the control system performs a seamless **handover**:
1.  The pulse-mode controller is disabled.
2.  The grid-side rectifier switches to continuous DC link current control.
3.  The machine-side inverter takes over commutation responsibility, using the machine's back-EMF, and its firing angle is now controlled relative to the PLL's angle output.

The drive is now in its normal, efficient **Load-Commutated Mode**, ready to accelerate to full speed.