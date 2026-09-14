- Add shunt for current `Battery I Sense`.
- Add voltage divider for current `Battery V Sense`.

- Swap SD card pins on S3.

- Add software power toggle for:
    - JST SH header.
    - GPS.
    - Shunt (bypass).
    - 


- ~~Location for ceramic patch?~~ Or keep off board?

- Fit a 32.768 kHz crystal on ESP_GPIO15/ESP_GPIO16 (the S3's XTAL_32K_P/N,
  currently unrouted pads). Without it RTC_SLOW_CLK is the internal 150 kHz
  RC oscillator, which free-runs uncorrected through a deep sleep and drifts
  at the percent level with temperature - so a sleeping board cannot keep an
  appointment. A schedule shared before a sleep is good for under 2 s at 1%
  drift and for an hour at 20 ppm, which is the difference between a wake
  that has to be hunted for and one that is simply kept. See
  `docs/WAKE-ON-LORA.md`.
