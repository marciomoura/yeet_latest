# Independent Watchdog (IWDG) - Critical Safety Feature

## What is a Watchdog Timer?

A **watchdog timer** is a hardware-based safety mechanism that monitors software execution. It acts as a "failsafe" that automatically resets the microcontroller if the software stops working correctly.

Think of it like a deadman's switch on a train - the operator must periodically press a button to prove they're still in control. If they stop pressing the button (because they're incapacitated), the train automatically stops.

## How It Works

```
1. Initialize watchdog with a timeout (e.g., 1000ms)
2. Enable watchdog (THIS CANNOT BE UNDONE!)
3. Normal operation loop:
   ┌──────────────────────────────────┐
   │  Do application work             │
   │  Feed watchdog (reset counter)   │ ← Must happen before timeout!
   │  Continue working...              │
   └──────────────────────────────────┘
4. If watchdog is NOT fed within timeout period:
   → HARDWARE RESET triggered automatically
   → System restarts from beginning
```

## Why Is It Critical?

The watchdog protects against **catastrophic software failures** that would otherwise leave the system completely unresponsive:

### 1. **Infinite Loops**
```c
// BUG: Infinite loop due to logic error
while (sensor_value > threshold) {
    // Forgot to update sensor_value!
    // System is STUCK here forever
}
// Without watchdog: System hangs forever
// With watchdog: System resets after timeout, recovers
```

### 2. **CPU Lockup / Hardware Faults**
- Memory corruption causing invalid instruction execution
- Stack overflow overwriting critical data
- Hardware fault (electromagnetic interference, voltage glitch)
- Cosmic ray bit flip (yes, this really happens!)

### 3. **Deadlock**
```c
// Thread 1 waits for resource A
// Thread 2 waits for resource B
// Thread 1 holds B, Thread 2 holds A
// DEADLOCK - both threads stuck forever
// Watchdog detects no progress and resets
```

### 4. **Runaway Code**
```c
// Pointer bug causes execution jump to invalid address
void (*func_ptr)(void) = NULL;
func_ptr();  // CPU crashes or executes random memory
// Without watchdog: System hangs or executes garbage
// With watchdog: Automatic recovery via reset
```

## What Happens If You DON'T Feed the Watchdog?

### Scenario: Power Electronics Controller (Your Application)

Imagine you're controlling a **high-voltage power converter** for a motor drive:

```c
void main_control_loop(void) {
    while(1) {
        read_sensors();
        calculate_control_output();
        update_pwm_outputs();
        
        // CRITICAL: Feed watchdog to prove we're still alive
        IWDG_Feed(&watchdog, get_timestamp());
        
        delay(1);  // 1ms loop time
    }
}
```

**If the software hangs and stops feeding the watchdog:**

1. **T = 0ms**: Software enters infinite loop or crashes
2. **T = 0-1000ms**: Watchdog counter counts down (timeout = 1000ms)
3. **T = 1000ms**: Watchdog timeout expires
4. **T = 1001ms**: **HARDWARE RESET TRIGGERED**
5. **T = 1002ms**: System reboots, runs initialization code
6. **T = 1050ms**: System operational again, safely shut down or recovered

**Without a watchdog:**
- Motor keeps spinning at last commanded speed (uncontrolled!)
- Power converter stays on with potentially dangerous output
- No way to recover without manual power cycle
- **SAFETY HAZARD** - equipment damage, injury risk

### Real-World Consequences Without Watchdog

**Industrial Control System:**
- Stuck motor could overheat and catch fire
- Runaway conveyor belt crushes products
- Pump continues running, floods factory floor
- **Financial loss**: Thousands to millions of dollars

**Medical Device:**
- Insulin pump delivers incorrect dose
- Ventilator stops responding
- Defibrillator fails to operate
- **Life-threatening situation**

**Automotive:**
- Electronic throttle stuck open
- Brake-by-wire system unresponsive
- Power steering locked up
- **Crash risk**

## Protection Strategy in Your Refactored Driver

The new `iwdg_driver.h/c` provides several safety features:

### 1. **Statistics Tracking**
```c
const iwdg_stats_t* stats = IWDG_GetStats(&watchdog);
printf("Max interval: %lu ms\n", stats->max_feed_interval);
// If max_feed_interval approaches timeout, you have a timing problem!
```

### 2. **Proactive Monitoring**
```c
// Check if watchdog needs feeding soon (with 100ms safety margin)
if (IWDG_NeedsFeeding(&watchdog, current_time, 100)) {
    // WARNING: We're cutting it close!
    log_warning("Watchdog feed interval too long!");
}
```

### 3. **Reset Detection**
```c
if (IWDG_WasResetByWatchdog(&watchdog)) {
    // Last reset was caused by watchdog timeout
    // This means software crashed!
    log_critical("WATCHDOG RESET DETECTED - Software failure!");
    // Send alert, log diagnostics, etc.
}
```

### 4. **Flexible Configuration**
```c
iwdg_config_t config = {
    .prescaler = IWDG_PRESCALER_32,
    .timeout_ms = 1000,  // 1 second timeout
    .stop_in_debug = true  // Don't reset while debugging
};
```

## Best Practices

### DO:
✅ Feed the watchdog from your **main control loop** only
✅ Set timeout **longer than worst-case loop time**
✅ Monitor feed interval statistics to detect timing issues
✅ Check for watchdog reset at startup
✅ Use watchdog in all production deployments

### DON'T:
❌ Feed watchdog from interrupt handlers (masks failures)
❌ Feed watchdog from multiple places (defeats the purpose)
❌ Set timeout too short (causes false resets)
❌ Disable watchdog in production code
❌ Forget that watchdog CANNOT be disabled once enabled

## Example: Complete System

```c
#include "iwdg_driver.h"

iwdg_instance_t watchdog;

void system_init(void) {
    // Configure watchdog
    iwdg_config_t config;
    IWDG_GetDefaultConfig(&config);  // 1000ms timeout
    
    IWDG_Init(&watchdog, &config);
    
    // Check if we were reset by watchdog
    if (IWDG_WasResetByWatchdog(&watchdog)) {
        // Log critical error - software failure detected!
        error_log("WATCHDOG RESET - Software crashed!");
        
        // Could implement:
        // - Send alert to monitoring system
        // - Enter safe mode
        // - Increment persistent failure counter
    }
    
    IWDG_ClearResetFlag(&watchdog);
    
    // Enable watchdog (CANNOT BE UNDONE!)
    IWDG_Enable(&watchdog);
}

void main_loop(void) {
    uint32_t timestamp = 0;
    
    while(1) {
        // Do critical work
        control_motor();
        monitor_safety();
        process_communications();
        
        // Feed watchdog - proves we're still alive
        IWDG_Feed(&watchdog, timestamp++);
        
        // Check if we're cutting it close
        if (IWDG_NeedsFeeding(&watchdog, timestamp, 200)) {
            // Warning: Less than 200ms margin!
            error_log("Watchdog feed interval too long!");
        }
        
        delay_ms(1);
    }
}
```

## Summary

The watchdog is a **last line of defense** against software failures. It's like having a guardian angel that will reset your system if something goes catastrophically wrong.

**Without it:** Your system could hang indefinitely, potentially causing:
- Equipment damage
- Safety hazards
- Financial losses
- Loss of life (in critical applications)

**With it:** Even if your software completely crashes, the hardware watchdog ensures:
- Automatic recovery
- System comes back online
- Failures are logged
- Safety is maintained

**The watchdog timer is not optional for safety-critical systems - it's mandatory.**
