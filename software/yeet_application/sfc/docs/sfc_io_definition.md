Of course. Here is the data organized into separate tables, one under the other, with the Chinese translations included.

### **SFC IO Sheet 20250730(Sheet1)**

---

#### **Digital Input Module 1**

| Input | English Signal Definition            | Chinese Signal Definition                                      | From     |
| :---- | :----------------------------------- | :------------------------------------------------------------- | :------- |
| DI1   | OPERATION ON CMD                     | 起动命令 (Start Command)                                       | TCS      |
| DI2   | OPERATION OFF CMD                    | 退出命令 (Stop Command)                                        | TCS      |
| DI3   | HIGH SPEED SPIN REQUEST              | 升速信号 (Speed Up Signal)                                     | TCS      |
| DI4   | HIGH SPEED KEEP REQUEST              | 高盘保持 (High Speed Hold)                                     | TCS      |
| DI5   | LOW SPEED KEEP REQUEST               | 点火保持 (Ignition Hold)                                       | TCS      |
| DI6   | SFC ACCELERATION REQUEST             | 全速升速 (Full Speed Acceleration)                             | TCS      |
| DI7   | COLD-DRAG START COMMAND              | 冷拖起动命令 (Cold-Drag Start Command)                         | TCS      |
| DI8   | ACCELERATION REQUEST                 | 升速命令(各阶段) (Acceleration Command (All Stages))           | TCS      |
| DI9   | SPEED-POWER CONTROL SWITCHING SIGNAL | 转速控制与功率控制切换信号 (Speed/Power Control Switch Signal) | TCS      |
| DI10  | START HSB CMD                        | 高盘起动命令 (High Speed Start Command)                        | TCS      |
| DI11  | SPARE                                | 备用 (Spare)                                                   | TCS      |
| DI12  | SPARE                                | 备用 (Spare)                                                   | TCS      |
| DI13  | FRONT DOOR OPEN                      | 开门报警 (Door Open Alarm)                                     | Internal |
| DI14  | LV-SUPERVISION                       | LV监视 (LV Supervision)                                        | Internal |
| DI15  | SUPERVISION                          | 电源监视 (Power Supervision)                                   | Internal |
| DI16  | MCB OPEN ALARM                       | 空开未合 (MCB Not Closed)                                      | Internal |

---

#### **Digital Input Module 2**

| Input | English Signal Definition | Chinese Signal Definition                    | From       |
| :---- | :------------------------ | :------------------------------------------- | :--------- |
| DI1   | EXC. RUN/STOP STATUS      | 励磁已运行/停止 (Excitation Running/Stopped) | Excitation |
| DI2   | EXC. ACTIVE               | 励磁已投励 (Excitation Active)               | Excitation |
| DI3   | SPARE                     | 备用 (Spare)                                 |            |
| DI4   | SPARE                     | 备用 (Spare)                                 |            |
| DI5   | CLOSED FB                 | ICB已合 (ICB Closed)                         | ICB        |
| DI6   | OPENED FB                 | ICB已分 (ICB Opened)                         | ICB        |
| DI7   | ICB IS TRIPPING           | ICB已退出 (ICB Tripped)                      | ICB        |
| DI8   | PRESS. RELIEF TRIP        | 压力释放跳闸 (Pressure Relief Trip)          | TLS        |
| DI9   | WINDING TEMP ALARM        | 绕组超温报警 (Winding Temp Alarm)            | TLS        |
| DI10  | WINDING TEMP TRIP         | 绕组超温跳闸 (Winding Temp Trip)             | TLS        |
| DI11  | BUCHHOLZ ALARM            | 变压器瓦斯报警 (Transformer Gas Alarm)       | TLS        |
| DI12  | BUCHHOLZ TRIP             | 变压器瓦斯跳闸 (Transformer Gas Trip)        | TLS        |
| DI13  | EMERGENCY OFF PRESSED     | 紧停按钮 (Emergency Stop Pressed)            | Internal   |
| DI14  | COMMON HW-TRIP            | 软件跳闸 (Software Trip)                     | Internal   |
| DI15  | EXTERNAL FAULT            | 外部故障 (External Fault)                    | Internal   |
| DI16  |                           |                                              |            |

---

#### **Power Cabinet Digital Input Module 1**

| Input | English Signal Definition | Chinese Signal Definition                        | From |
| :---- | :------------------------ | :----------------------------------------------- | :--- |
| DI1   | OVERVOLTAGE MONIT         | 过压快熔熔断 (Overvoltage Fuse Blown)            |      |
| DI2   | US MONIT CONVERTER        | 风机停风 (Fan Stop)                              |      |
| DI3   | MCB MONIT                 | 空开未合 (MCB Not Closed)                        |      |
| DI4   | COOL REACTOR A            | 电抗器温度过高1 (Reactor Temperature Too High 1) |      |
| DI5   | COOL REACTOR T            | 电抗器温度过高2 (Reactor Temperature Too High 2) |      |

---

#### **Digital Output Module**

| Output | English Signal Definition | Chinese Signal Definition                          | To         |
| :----- | :------------------------ | :------------------------------------------------- | :--------- |
| DO1    | ON CMD ACU REACT FAN      | 合电抗器柜风机 (Close Reactor Cabinet Fan)         | Internal   |
| DO2    | ON CMD ACU CONV1 FAN      | 合网桥风机 (Close Grid Bridge Fan)                 | Internal   |
| DO3    | ON CMD ACU CONV2 FAN      | 合机桥风机 (Close Machine Bridge Fan)              | Internal   |
| DO4    | ON CMD ACU CTR FAN        | 合控制柜风机 (Close Control Cabinet Fan)           | Internal   |
| DO5    | ICB CLOSE CMD/ICB Control | ICB合闸/ICB控制信号 (ICB Close/ICB Control Signal) | Internal   |
| DO6    | ICB OPEN CMD              | ICB分闸 (ICB Open)                                 | Internal   |
| DO17   | LCI IS HEALTHY            | LCI 状态正常 (LCI Status Normal)                   | TCS        |
| DO18   | MV ON FB                  | 中压接通 (Medium Voltage On)                       | TCS        |
| DO19   | OPERATION OFF FB          | SFC已运行 (SFC Running)                            | TCS        |
| DO20   | OPERATION ON FB           | SFC已停止 (SFC Stopped)                            | TCS        |
| DO21   | SFC IS READY              | SFC准备就绪 (SFC Ready)                            | TCS        |
| DO22   | SPEED CONTROL SELECTED    | 转速控制 (Speed Control Selected)                  | TCS        |
| DO23   | POWER CONTROL SELECTED    | 功率控制 (Power Control Selected)                  | TCS        |
| DO24   | START PROFILE SELECTED    | 启动配置控制 (Start Profile Selected)              | TCS        |
| DO25   | COMMON ALARM              | 报警 (Alarm)                                       | TCS        |
| DO26   | COMMON TRIP               | 跳闸 (Trip)                                        | TCS        |
| DO27   | TRIP/Emergency stop       | 紧急退出ICB (Emergency Stop ICB)                   | ICB        |
| DO28   | ON CMD                    | 励磁投入 (Excitation On)                           | Excitation |
| DO29   | OFF CMD                   | 励磁退出 (Excitation Off)                          | Excitation |
| DO30   | EXC. CONTROL MODE         | 励磁工作模式 (Excitation Control Mode)             | Excitation |
| DO31   | SW TRIP                   | 软件跳闸 (Software Trip)                           | Internal   |
| DO32   | EMERGENCY OFF RESET       | 紧停复位 (Emergency Stop Reset)                    | Internal   |

---

#### **Analog Input Module**

| Input | English Signal Definition | Chinese Signal Definition                  | From       |
| :---- | :------------------------ | :----------------------------------------- | :--------- |
| AI1   | Exc. Current Feedback     | 励磁电流反馈 (Excitation Current Feedback) | Excitation |
| AI2   | Exc. Voltage Feedback     | 励磁电压反馈 (Excitation Voltage Feedback) | Excitation |

---

#### **Analog Output Module**

| Output | English Signal Definition | Chinese Signal Definition                   | To         |
| :----- | :------------------------ | :------------------------------------------ | :--------- |
| AO2    | Exc. Current Ref.         | 励磁电流给定 (Excitation Current Reference) | Excitation |