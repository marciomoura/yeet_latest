/**
 ******************************************************************************
 * SIMPLE WATCHDOG USAGE EXAMPLES
 * 
 * This document shows how to use the simplified watchdog driver.
 ******************************************************************************
 */

## BASIC USAGE

### Example 1: Simple initialization with default 1 second timeout

```c
#include "watchdog_simple.h"

int main(void)
{
    // System initialization
    HAL_Init();
    SystemClock_Config();
    
    // Initialize watchdog with 1 second timeout
    Watchdog_Init();
    
    // Main loop
    while (1)
    {
        // Your application code here
        Do_Something();
        
        // Feed watchdog every 10ms (100 times per second)
        HAL_Delay(10);
        Watchdog_Feed();
    }
}
```

### Example 2: Custom timeout (100ms)

```c
#include "watchdog_simple.h"

int main(void)
{
    // System initialization
    HAL_Init();
    SystemClock_Config();
    
    // Initialize watchdog with 100ms timeout
    Watchdog_Init_Custom(100);
    
    // Main loop
    while (1)
    {
        // Your application code here
        Process_Data();
        
        // Feed watchdog every 1ms
        HAL_Delay(1);
        Watchdog_Feed();
    }
}
```

### Example 3: Feeding from timer interrupt

```c
#include "watchdog_simple.h"

// Timer interrupt handler (called every 10ms)
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        // Feed watchdog from interrupt
        Watchdog_Feed();
    }
}

int main(void)
{
    // System initialization
    HAL_Init();
    SystemClock_Config();
    
    // Setup timer to fire every 10ms
    Setup_10ms_Timer();
    
    // Initialize watchdog with 500ms timeout
    Watchdog_Init_Custom(500);
    
    // Main loop - watchdog is fed by timer interrupt
    while (1)
    {
        // Your application code here
        // No need to call Watchdog_Feed() in main loop
        Process_Application();
    }
}
```

### Example 4: Detecting watchdog reset

```c
#include "watchdog_simple.h"

int main(void)
{
    // System initialization
    HAL_Init();
    SystemClock_Config();
    
    // Check if last reset was caused by watchdog
    if (Watchdog_WasLastResetByWatchdog())
    {
        // Watchdog caused the reset - software hung!
        Log_Error("System recovered from watchdog reset!");
        
        // Clear the flag
        Watchdog_ClearResetFlag();
        
        // Take corrective action
        Enter_Safe_Mode();
    }
    
    // Initialize watchdog
    Watchdog_Init();
    
    // Main loop
    while (1)
    {
        // Application code
        Run_Application();
        
        HAL_Delay(10);
        Watchdog_Feed();
    }
}
```

### Example 5: Monitoring feed count

```c
#include "watchdog_simple.h"

int main(void)
{
    // System initialization
    HAL_Init();
    SystemClock_Config();
    
    // Initialize watchdog
    Watchdog_Init();
    
    uint32_t last_reported_count = 0;
    
    // Main loop
    while (1)
    {
        // Application code
        Process_Data();
        
        // Feed watchdog every 10ms
        HAL_Delay(10);
        Watchdog_Feed();
        
        // Report feed count every 10 seconds
        uint32_t count = Watchdog_GetFeedCount();
        if (count - last_reported_count >= 1000)  // 1000 feeds * 10ms = 10 seconds
        {
            printf("Watchdog fed %lu times\n", count);
            last_reported_count = count;
        }
    }
}
```

## COMMON TIMEOUT VALUES

| Timeout | Feed Interval  | Description                              |
| ------- | -------------- | ---------------------------------------- |
| 100ms   | 1ms or 10ms    | Fast control loops, high-frequency tasks |
| 500ms   | 10ms or 50ms   | Normal control loops                     |
| 1000ms  | 10ms or 100ms  | Standard applications (default)          |
| 2000ms  | 100ms or 500ms | Slow operations, long computations       |

## IMPORTANT RULES

1. **Feed Faster Than Timeout:**
   - If timeout = 1000ms, feed every 10-100ms
   - If timeout = 100ms, feed every 1-10ms
   - Leave safety margin (at least 50% of timeout)

2. **Consistent Feeding:**
   - Feed from main loop OR timer interrupt, not both
   - Feed at regular intervals
   - Don't skip feeds

3. **One-Time Initialization:**
   - Call `Watchdog_Init()` or `Watchdog_Init_Custom()` ONCE
   - Cannot disable after enabling
   - Cannot change timeout after initialization

4. **Feed Location:**
   - Feed from main loop for simple applications
   - Feed from timer interrupt for complex applications
   - Safe to feed from interrupts

## WHAT HAPPENS IF YOU FORGET TO FEED?

```
Time 0ms:    Watchdog_Init()          - Watchdog starts counting
Time 10ms:   Watchdog_Feed()          - Counter resets to 1000ms
Time 20ms:   Watchdog_Feed()          - Counter resets to 1000ms
Time 30ms:   Watchdog_Feed()          - Counter resets to 1000ms
...
Time 500ms:  [INFINITE LOOP STARTS]   - Software hangs!
Time 600ms:  [No feed!]               - Counter still counting down
Time 700ms:  [No feed!]               - Counter at ~300ms
Time 800ms:  [No feed!]               - Counter at ~200ms
Time 900ms:  [No feed!]               - Counter at ~100ms
Time 1000ms: [SYSTEM RESET!]          - Watchdog triggers reset
Time 1001ms: main() starts again      - System recovers
```

## COMPARISON WITH OLD DRIVER

### Old Driver (watchdog.h/c):
```c
WatchDog_Init(1000);  // Initialize with timeout
Feed_Dog();           // Feed the watchdog
```

### New Simple Driver (watchdog_simple.h/c):
```c
Watchdog_Init();              // Initialize with default timeout
Watchdog_Init_Custom(1000);   // OR initialize with custom timeout
Watchdog_Feed();              // Feed the watchdog

// Bonus features:
bool was_reset = Watchdog_WasLastResetByWatchdog();
uint32_t feeds = Watchdog_GetFeedCount();
Watchdog_ClearResetFlag();
```

## MIGRATION FROM OLD DRIVER

```c
// OLD CODE:
WatchDog_Init(1000);
Feed_Dog();

// NEW CODE (drop-in replacement):
Watchdog_Init_Custom(1000);
Watchdog_Feed();
```

## API SUMMARY

```c
// Initialization (call once)
void Watchdog_Init(void);                    // Default 1000ms timeout
void Watchdog_Init_Custom(uint16_t timeout); // Custom timeout (1-4095ms)

// Operation (call regularly)
void Watchdog_Feed(void);                    // Feed to prevent reset

// Status (optional)
bool Watchdog_WasLastResetByWatchdog(void);  // Check reset cause
void Watchdog_ClearResetFlag(void);          // Clear reset flag
uint32_t Watchdog_GetFeedCount(void);        // Get feed counter
```

## THAT'S IT!

Just three simple steps:
1. **Initialize once:** `Watchdog_Init();`
2. **Feed regularly:** `Watchdog_Feed();` (every 1ms, 10ms, or 100ms)
3. **Done!** The watchdog protects your system automatically.

No complex structures, no configuration options, no statistics tracking.
Just simple, straightforward watchdog protection! 🐕
