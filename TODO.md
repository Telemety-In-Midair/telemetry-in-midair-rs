Wi-Fi Server. Hotspot style.

Need fw-upload to trigger turning on the WIO.

Esp needs to sleep

Decrease packet size on wio to maximize range?

Swap to WIO-S3?

Reduce packet size by adding gps precision from a point. (Given a point in toml or BLE. Find the difference and send that to a given precision)
Now worth doing: nano-mesh padded every payload to 32 bytes, so shrinking
one saved nothing on air. Payloads now go out at their true length.

Try a slow preset now that nothing caps the listen window. SF12 is about
12 dB over SF7 and was unreachable while the mesh needed a listen period
longer than one packet's air time.


WIO-E5 (Maybe usb bridge too (CH32?)) listener. (Dongle?)


Need antenna for BLE.
Remove W.FL antenna add wire antenna (31 mm).

Go mode. Should sleep cycle the ESP, and send the gps coordinates over the radio.

SD card for 

SD logging to file?


Wake-on-radio: SetRxDutyCycle (0x94), exposed by the HAL as
set_rx_duty_cycle. The radio cycles sleep/RX on its own and only wakes the
MCU when a real preamble arrives, instead of the MCU holding continuous RX.
Biggest battery win available on a leaf that mostly listens. Needs the
receive loop restructured and the sleep/RX ratio picked against the beacon
interval: too long asleep and a whole broadcast passes unheard, so the two
have to be chosen together.

CAD auto-transitions: SetCadParams (0x88) with ExitMode. Detect a preamble
and let the chip drop straight into RX to catch the payload, or find the
channel clear and go straight to TX. Cheaper than a full RX window for
listen-before-talk, and it would give repeaters a real collision check
before forwarding rather than the current random jitter.

Will loading the program with the RADIO.CFG auto overwrite flags (Adress)?  


GPS:
    Ultra-high sensitivity mode
    Navigation input filters
    More lowe power?

    Airborne <2g --> Stationary (When on ground). Timer?

Beeper?



The PMode::Boost value: the code writes 0x97, which the HAL documents as best sensitivity, but Semtech's datasheet documents only 0x94 and 0x96. I left the behavior alone and corrected the note in NOTES.md that claimed 0x96 was being written — worth a look at RM0453 if you want certainty.

Since the sync word is a flag day, reflash every node before testing — a partially updated fleet will look exactly like the range problem you started with.
