# Future ideas

Smaller?
Probably need to drop a module.

Software toggle shunt? Or charging IC that handles all of this.

Battery bypass LDO? (Just esp?) USB must not.

Add power switch? 

Add current monitor? (INA219/226?)

Add an LP-GPIO wake button so a deep sleep can be interrupted. Deep sleep
is timer-only, so the 5 min clamp on 0x13 is the only thing keeping the
board reachable.

Route the module's Wi-Fi/BT RF port to an antenna. It reaches test point
BLE1 and stops there on the board as drawn, so the 2.4 GHz side has no
antenna despite the module bringing the port out.

Battery sense divider. There is none, so telemetry cannot report cell
voltage without a board change.

Route GPS EXTINT to the MCU, and TIMEPULSE for PPS discipline. Neither is
connected today; backup mode still wakes on UART traffic, but PPS is simply
unavailable.