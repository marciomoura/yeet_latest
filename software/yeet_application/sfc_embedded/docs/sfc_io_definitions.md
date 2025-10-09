This document provides a comprehensive mapping of the microcontroller unit's (MCU) peripherals to the processor package and printed circuit board (PCB) terminals.

### Digital Inputs (DI)

The system utilizes 48 digital inputs, DI1 through DI48.

*   **DI1 - DI32:** These inputs are connected to the DI module via a specialized cable and are routed through the FPGA to the CPU. The CPU accesses these inputs through a parallel bus, treating the FPGA as external SRAM.
*   **DI33 - DI48:** These inputs are connected to the ICAM at terminal X6. The communication between the FPGA and the CPLD is handled via the SPI protocol.

| DI Number | DIDO module    | DIDO module TO ICAM | ICAM Terminal number | CPLD                                         | FPGA pin | fpga to cpu                                                                       | CPU address      |
| :-------- | :------------- | :------------------ | :------------------- | :------------------------------------------- | :------- | :-------------------------------------------------------------------------------- | :--------------- |
| DI1       | DI module-DI1  | Specialized cable   | P3                   | no                                           | F17      | Parallel bus (the CPU treats the FPGA as external SRAM and uses FSMC for control) | 0x6C000192\_bit0 |
| DI2       | DI module-DI2  |                     |                      |                                              | F18      |                                                                                   | 0x6C000192\_bit1 |
| DI3       | DI module-DI3  |                     |                      |                                              | E16      |                                                                                   | 0x6C000192\_bit2 |
| DI4       | DI module-DI4  |                     |                      |                                              | E18      |                                                                                   | 0x6C000192\_bit3 |
| DI5       | DI module-DI5  |                     |                      |                                              | D18      |                                                                                   | 0x6C000192\_bit4 |
| DI6       | DI module-DI6  |                     |                      |                                              | D17      |                                                                                   | 0x6C000192\_bit5 |
| DI7       | DI module-DI7  |                     |                      |                                              | C18      |                                                                                   | 0x6C000192\_bit6 |
| DI8       | DI module-DI8  |                     |                      |                                              | C17      |                                                                                   | 0x6C000192\_bit7 |
| DI9       | DI module-DI9  |                     |                      |                                              | H18      |                                                                                   | 0x6C000193\_bit0 |
| DI10      | DI module-DI10 |                     |                      |                                              | G13      |                                                                                   | 0x6C000193\_bit1 |
| DI11      | DI module-DI11 |                     |                      |                                              | G14      |                                                                                   | 0x6C000193\_bit2 |
| DI12      | DI module-DI12 |                     |                      |                                              | G16      |                                                                                   | 0x6C000193\_bit3 |
| DI13      | DI module-DI13 |                     |                      |                                              | G18      |                                                                                   | 0x6C000193\_bit4 |
| DI14      | DI module-DI14 |                     |                      |                                              | F14      |                                                                                   | 0x6C000193\_bit5 |
| DI15      | DI module-DI15 |                     |                      |                                              | F15      |                                                                                   | 0x6C000193\_bit6 |
| DI16      | DI module-DI16 |                     |                      |                                              | F16      |                                                                                   | 0x6C000193\_bit7 |
| DI17      | DI module-DI17 |                     |                      |                                              | J16      |                                                                                   | 0x6C000194\_bit0 |
| DI18      | DI module-DI18 |                     |                      |                                              | J18      |                                                                                   | 0x6C000194\_bit1 |
| DI19      | DI module-DI19 |                     |                      |                                              | H12      |                                                                                   | 0x6C000194\_bit2 |
| DI20      | DI module-DI20 |                     |                      |                                              | H13      |                                                                                   | 0x6C000194\_bit3 |
| DI21      | DI module-DI21 |                     |                      |                                              | H14      |                                                                                   | 0x6C000194\_bit4 |
| DI22      | DI module-DI22 |                     |                      |                                              | H15      |                                                                                   | 0x6C000194\_bit5 |
| DI23      | DI module-DI23 |                     |                      |                                              | H16      |                                                                                   | 0x6C000194\_bit6 |
| DI24      | DI module-DI24 |                     |                      |                                              | H17      |                                                                                   | 0x6C000194\_bit7 |
| DI25      | DI module-DI25 |                     |                      |                                              | K12      |                                                                                   | 0x6C000195\_bit0 |
| DI26      | DI module-DI26 |                     |                      |                                              | K13      |                                                                                   | 0x6C000195\_bit1 |
| DI27      | DI module-DI27 |                     |                      |                                              | K14      |                                                                                   | 0x6C000195\_bit2 |
| DI28      | DI module-DI28 |                     |                      |                                              | K15      |                                                                                   | 0x6C000195\_bit3 |
| DI29      | DI module-DI29 |                     |                      |                                              | K17      |                                                                                   | 0x6C000195\_bit4 |
| DI30      | DI module-DI30 |                     |                      |                                              | K18      |                                                                                   | 0x6C000195\_bit5 |
| DI31      | DI module-DI31 |                     |                      |                                              | K16      |                                                                                   | 0x6C000195\_bit6 |
| DI32      | DI module-DI32 |                     |                      |                                              | J13      |                                                                                   | 0x6C000195\_bit7 |
| DI33      | no             | no                  | X6-3                 | The FPGA communicates with the CPLD via SPI. |          |                                                                                   | 0x6C000196\_bit0 |
| DI34      |                |                     | X6-4                 |                                              |          |                                                                                   | 0x6C000196\_bit1 |
| DI35      |                |                     | X6-5                 |                                              |          |                                                                                   | 0x6C000196\_bit2 |
| DI36      |                |                     | X6-6                 |                                              |          |                                                                                   | 0x6C000196\_bit3 |
| DI37      |                |                     | X6-7                 |                                              |          |                                                                                   | 0x6C000196\_bit4 |
| DI38      |                |                     | X6-8                 |                                              |          |                                                                                   | 0x6C000196\_bit5 |
| DI39      |                |                     | X6-9                 |                                              |          |                                                                                   | 0x6C000196\_bit6 |
| DI40      |                |                     | X6-10                |                                              |          |                                                                                   | 0x6C000196\_bit7 |
| DI41      |                |                     | X6-12                |                                              |          |                                                                                   | 0x6C000197\_bit0 |
| DI42      |                |                     | X6-13                |                                              |          |                                                                                   | 0x6C000197\_bit1 |
| DI43      |                |                     | X6-14                |                                              |          |                                                                                   | 0x6C000197\_bit2 |
| DI44      |                |                     | X6-15                |                                              |          |                                                                                   | 0x6C000197\_bit3 |
| DI45      |                |                     | X6-16                |                                              |          |                                                                                   | 0x6C000197\_bit4 |
| DI46      |                |                     | X6-17                |                                              |          |                                                                                   | 0x6C000197\_bit5 |
| DI47      |                |                     | X6-18                |                                              |          |                                                                                   | 0x6C000197\_bit6 |
| DI48      |                |                     | X6-19                |                                              |          |                                                                                   | 0x6C000197\_bit7 |

### Digital Outputs (DO)

The system features 48 digital outputs, DO1 through DO48.

*   **DO1 - DO32:** These outputs are connected to the DO module via a specialized cable and are controlled by the FPGA.
*   **DO33 - DO48:** These outputs are connected to the ICAM at terminal X7. The FPGA communicates with the CPLD via SPI to control these outputs.

| DO Number | DIDO module    | DIDO module TO ICAM | ICAM Terminal number | CPLD                                         | FPGA pin | CPU address      |
| :-------- | :------------- | :------------------ | :------------------- | :------------------------------------------- | :------- | :--------------- |
| DO1       | DO module-DO1  | Specialized cable   | P4                   | no                                           | N18      | 0x6C000006\_bit0 |
| DO2       | DO module-DO2  |                     |                      |                                              | M13      | 0x6C000006\_bit1 |
| DO3       | DO module-DO3  |                     |                      |                                              | M14      | 0x6C000006\_bit2 |
| DO4       | DO module-DO4  |                     |                      |                                              | M16      | 0x6C000006\_bit3 |
| DO5       | DO module-DO5  |                     |                      |                                              | M18      | 0x6C000006\_bit4 |
| DO6       | DO module-DO6  |                     |                      |                                              | L12      | 0x6C000006\_bit5 |
| DO7       | DO module-DO7  |                     |                      |                                              | L13      | 0x6C000006\_bit6 |
| DO8       | DO module-DO8  |                     |                      |                                              | L14      | 0x6C000006\_bit7 |
| DO9       | DO module-DO9  |                     |                      |                                              | T18      | 0x6C000007\_bit0 |
| DO10      | DO module-DO10 |                     |                      |                                              | P16      | 0x6C000007\_bit1 |
| DO11      | DO module-DO11 |                     |                      |                                              | P17      | 0x6C000007\_bit2 |
| DO12      | DO module-DO12 |                     |                      |                                              | P18      | 0x6C000007\_bit3 |
| DO13      | DO module-DO13 |                     |                      |                                              | N14      | 0x6C000007\_bit4 |
| DO14      | DO module-DO14 |                     |                      |                                              | N15      | 0x6C000007\_bit5 |
| DO15      | DO module-DO15 |                     |                      |                                              | N16      | 0x6C000007\_bit6 |
| DO16      | DO module-DO16 |                     |                      |                                              | N17      | 0x6C000007\_bit7 |
| DO17      | DO module-DO17 |                     |                      |                                              | T11      | 0x6C000008\_bit0 |
| DO18      | DO module-DO18 |                     |                      |                                              | V13      | 0x6C000008\_bit1 |
| DO19      | DO module-DO19 |                     |                      |                                              | U13      | 0x6C000008\_bit2 |
| DO20      | DO module-DO20 |                     |                      |                                              | V14      | 0x6C000008\_bit3 |
| DO21      | DO module-DO21 |                     |                      |                                              | T14      | 0x6C000008\_bit4 |
| DO22      | DO module-DO22 |                     |                      |                                              | U17      | 0x6C000008\_bit5 |
| DO23      | DO module-DO23 |                     |                      |                                              | U18      | 0x6C000008\_bit6 |
| DO24      | DO module-DO24 |                     |                      |                                              | T17      | 0x6C000008\_bit7 |
| DO25      | DO module-DO25 |                     |                      |                                              | V16      | 0x6C000009\_bit0 |
| DO26      | DO module-DO26 |                     |                      |                                              | V15      | 0x6C000009\_bit1 |
| DO27      | DO module-DO27 |                     |                      |                                              | U15      | 0x6C000009\_bit2 |
| DO28      | DO module-DO28 |                     |                      |                                              | M11      | 0x6C000009\_bit3 |
| DO29      | DO module-DO29 |                     |                      |                                              | V12      | 0x6C000009\_bit4 |
| DO30      | DO module-DO30 |                     |                      |                                              | V11      | 0x6C000009\_bit5 |
| DO31      | DO module-DI31 |                     |                      |                                              | U11      | 0x6C000009\_bit6 |
| DO32      | DO module-DO32 |                     |                      |                                              | U10      | 0x6C000009\_bit7 |
| DO33      | no             | no                  | X7-3                 | The FPGA communicates with the CPLD via SPI. |          | 0x6C00000A\_bit0 |
| DO34      |                |                     | X7-4                 |                                              |          | 0x6C00000A\_bit1 |
| DO35      |                |                     | X7-5                 |                                              |          | 0x6C00000A\_bit2 |
| DO36      |                |                     | X7-6                 |                                              |          | 0x6C00000A\_bit3 |
| DO37      |                |                     | X7-7                 |                                              |          | 0x6C00000A\_bit4 |
| DO38      |                |                     | X7-8                 |                                              |          | 0x6C00000A\_bit5 |
| DO39      |                |                     | X7-9                 |                                              |          | 0x6C00000A\_bit6 |
| DO40      |                |                     | X7-10                |                                              |          | 0x6C00000A\_bit7 |
| DO41      |                |                     | X7-12                |                                              |          | 0x6C00000B\_bit0 |
| DO42      |                |                     | X7-13                |                                              |          | 0x6C00000B\_bit1 |
| DO43      |                |                     | X7-14                |                                              |          | 0x6C00000B\_bit2 |
| DO44      |                |                     | X7-15                |                                              |          | 0x6C00000B\_bit3 |
| DO45      |                |                     | X7-16                |                                              |          | 0x6C00000B\_bit4 |
| DO46      |                |                     | X7-17                |                                              |          | 0x6C00000B\_bit5 |
| DO47      |                |                     | X7-18                |                                              |          | 0x6C00000B\_bit6 |
| DO48      |                |                     | X7-19                |                                              |          | 0x6C00000B\_bit7 |

### Analog Inputs (AI)

There are 12 analog inputs, AI1 through AI12, used for monitoring various signals of the excitation system.

| AI Number | Signal definition( Excitation system) | ASI Terminal number | Ratio(ASI)                                                  | Range(ASI) | ICAM Terminal number | Ratio(ICAM)                               | Range(ICAM) |
| :-------- | :------------------------------------ | :------------------ | :---------------------------------------------------------- | :--------- | :------------------- | :---------------------------------------- | :---------- |
| AI1       | Vab                                   | X4-1                | 100V（input）3.5V（output）                                 | ±200V      | X1-1                 | 10V corresponds to a code value of 32767. | -10V ~ +10V |
| AI2       | Vbc                                   | X4-2                |                                                             |            | X1-2                 |                                           |             |
| AI3       | Vca                                   | X4-3                |                                                             |            | X1-3                 |                                           |             |
| AI4       | white noise                           | no                  |                                                             |            | X1-4                 |                                           |             |
| AI5       | Ia                                    | X7-1                | TWO TYPE: 1A（input）5V（output）OR 5A（input）5V（output） | ±7.5A      | X1-5                 |                                           |             |
| AI6       | Ib                                    | X7-3                |                                                             |            | X1-6                 |                                           |             |
| AI7       | Ic                                    | X7-3                |                                                             |            | X1-7                 |                                           |             |
| AI8       | If                                    | X6-1、X6-3、X6-5    | anode current（input）exciting current（output）            |            | X1-8                 |                                           |             |
| AI9       | Grid voltage                          | X4-4                | 100V（input）3.5V（output）                                 | ±200V      | X1-9                 | 10V corresponds to a code value of 4095.  | 0~ 10V      |
| AI10      | spare                                 |                     |                                                             |            | X1-10                |                                           |             |
| AI11      | spare                                 |                     |                                                             |            | X1-11                |                                           |             |
| AI12      | spare                                 |                     |                                                             |            | X1-12                |                                           |             |

### Analog Outputs (AO)

There are 4 analog outputs, AO1 through AO4.

| AO Number | ICAM Terminal number | Ratio(ICAM)                                                                            | Range(ICAM) |
| :-------- | :------------------- | :------------------------------------------------------------------------------------- | :---------- |
| AO1       | X1-14                | "The corresponding output for 0, 32767, and 65535 is -10V, 0V, and +10V respectively." | ±10V        |
| AO2       | X1-15                |                                                                                        |             |
| AO3       | X1-16                |                                                                                        |             |
| AO4       | X1-17                |                                                                                        |             |

### PWM Signals

The system provides 6 PWM signals for gate pulses, directly controlled by the STM32 MCU.

| Gate Pulses | STM32 pin |
| :---------- | :-------- |
| PWM1        | PC6       |
| PWM2        | PC7       |
| PWM3        | PC8       |
| PWM4        | PC9       |
| PWM5        | PB0       |
| PWM6        | PB1       |