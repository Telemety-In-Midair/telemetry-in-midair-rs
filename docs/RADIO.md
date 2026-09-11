# The radio

The config, where it lives, the modulation, the slot clock, what goes out
and what comes back. `RADIO.example.toml` is the key-by-key reference and is
generated from the firmware's own table (`cd proto && cargo run --example
radio_example`); this is the reasoning behind it.

## Configuring a board

A config reaches a board as a push over USB (`pixi run board-config`) or
over BLE (the bulk characteristic; the app's Radio page), which applies it
live and writes it to the board's own flash, where the next boot reads it.
All keys are optional; an absent key is the default, except under
`[power]`, where absent means "leave the board's live value alone" (see
`docs/POWER.md`).

A push replaces the whole config: keys absent from what is sent revert to
their defaults rather than keeping the board's current values. The USB tool
cannot read a config back off a board, so start from a file holding your
settings (`--file`) if the board is not on stock ones. Over BLE the board
reports its current config, so the app can read it back - its Radio page has
a "Load from board" that fills the editor from the board itself.

A push that did not reach flash says so on the status line, and
`board-config` exits non-zero when that happens.

### Where a config lives

One store: a record in the `nvs` partition, the config text behind a
length and a crc, one sector past the settings record, written when a
config is pushed and read at boot. A board that has never taken a push
runs the firmware defaults, node address included. Until 2026-09-11 there
was a second store, `RADIO.CFG` on an SD card, which won at boot; the card
is no longer driven (see `docs/HARDWARE.md`). The record is what makes a
push survive a power cycle: the
node address is the one setting nothing can guess back, since two senders
sharing an address are mutually deaf.

An invalid `RADIO.CFG` falls through to the backup rather than to defaults,
and says so on the console. A record that fails its crc reads as nothing
stored, which is also what an interrupted write leaves behind - the length
and the crc sit in front of the text precisely so that a half-written record
cannot be read as a whole one.

The backup is not erased by `cargo run` - only `otadata` is - so a reflash
keeps a board's address. Erasing the whole chip does take it, along with the
duty cycle and the board's name, since both records live in `nvs`.

## Modulation

The default is SF12 at 500 kHz on a single carrier at 915 MHz, with the
network's slot clock running under it.

That band gives a transmitter two ways to be legal, and the default takes
the simpler one. A 500 kHz signal counts as a digital modulation and may
hold one carrier indefinitely, with no dwell or duty cycle ceiling, so
`interval_s = 1` is legal without hopping anywhere. Anything narrower has
to hop: at least 50 channels, no more than 0.4 s on any one of them per
20 s, and receivers hopping in step with the sender.

Hopping is therefore a setting rather than the default. Raising
`hop_channels` buys two things - the narrower modulations, and diversity,
since a fade or an interferer parked on one carrier then costs one beacon
in `hop_channels` rather than every beacon. It does not buy link budget:
the 0.4 s dwell caps time on air, time on air is what buys sensitivity,
and the best a legal hopped plan manages against the default is about a
decibel (SF10 at 125 kHz, 330 ms on air). What it costs is below.

SF12 at 500 kHz keeps the frame inside a slot. The default beacon is 289 ms
on air, and the same frame at BW125 is 1.15 s - past the 0.4 s a visit may
occupy a channel, and past the 800 ms window a 1 s slot leaves after its
guards. The narrower modulations are still config keys, and the estimate on
the app's Radio page says whether a frame fits; a frame that does not is
still sent, since receivers hold their hop for a frame in progress, but it
is one channel held longer than a hop is meant to be.

Somewhere the band rules differ - EU 868, say, where there is no minimum
bandwidth and the constraint is a duty cycle - `spreading_factor = 9`,
`bandwidth_khz = 62` and `power_dbm = 14` get the narrow modulation back,
and `hop_channels = 50` makes it legal where hopping is what is asked for.

## The slot clock

Every node keeps a slot clock: time cut into `hop_dwell_ms` slots, and in
slot `s` every node is on the `s mod n`-th channel of a permutation of all
`n`, reshuffled every cycle of `n` slots from the cycle number. At the
default `hop_channels = 1` that permutation has one entry and nothing ever
retunes; what is left is the clock, which is what gives each node its own
turn to transmit in. There is no config without a plan: a `0` in the file
is read as `1`.

With `hop_channels` raised, a node that beacons every slot uses each
channel once a cycle; one that beacons every twentieth slot still lands
somewhere different each time, because the order under it changes. The
reshuffle is also what lets two nodes that do not agree on the time find
each other at all: their channels coincide in about one slot in `n`, where
a fixed order at a fixed offset never meets.

The clock has three sources, in order of trust, and every frame says which
its sender is on:

- **GPS time**, for a node with a fix: the slot is the second of the day.
  Every node with a fix agrees without hearing anyone. Stratum 0.
- **A heard frame.** Every frame carries a four-byte sync word - the slot
  it went out in, how far into the slot, and the sender's stratum. The
  frame's length and modulation fix its time on air, so the receiver knows
  to a few milliseconds when the sender's slot began, and adopts the clock
  if it is better than its own: a lower stratum, or the same stratum from a
  lower address. It then sits one stratum below.
- **Nothing.** A node with neither runs its own free-running clock at
  stratum 15, so a follower can still be in step with it, and takes the
  first better clock it hears.

A clock nobody has refreshed ages one stratum every ten minutes, so a
network cut off from its GPS reference reorganizes around the lowest
address instead of every node insisting it is still stratum 1, and a node
that gets a fix back outranks everyone again at once. Crystal drift is a
few milliseconds per ten minutes, against a 100 ms guard at each end of the
slot, so an aged clock is still a usable one.

What hopping costs is the join, and it is why the default plan is one
channel wide. A node that knows nobody's clock hears the network only when
its channel happens to coincide, so it waits on average `hop_channels x
interval / nodes transmitting` seconds - 50 s across fifty channels with
one node beaconing every second, 250 s if that node has no fix and is
pinging every five. It goes on paying after the join, too: a follower is a
stratum below and re-anchors on what it hears, and every disagreement about
where a slot began puts it on the wrong channel for a frame. In simulation
a fixless listener loses about a quarter of the traffic that way across
fifty channels and none of it across one. A node with a fix never waits,
and a node that has synced once stays synced through fix loss, a config
push, a standby and a brownout of the radio. The base station on a desk is
the case to know about on a hopping network: give it a fix, or a short
interval on the nodes it is waiting for.

On the air, nodes take turns inside a slot. The window (the slot less a
100 ms guard at each end) is cut into as many lean beacons as fit back to
back - two at the default modulation - and a node's address picks its
turn; with `interval_s` longer than a slot the address picks the slot of
the interval first. So addresses `1` and `2` may both beacon every second
without ever overlapping, and `2 x interval_s` consecutive addresses
never overlap at any interval. Past that, two nodes share a turn and
overlap on every transmission; they cannot hear each other to notice, so
a node that hears both says `hop: node N shares this node's turn`, and the
app's Radio page shows how many addresses a plan carries.

A transmission is planned for a random point inside the turn and made when
that instant arrives (`midair_proto::beacon`, walked by the state space
tests). A plan that has gone stale - the clock re-anchored on a frame heard
since, or a frame arriving held the transmit past its turn - is remade
rather than waited for, and a plan still ahead is kept across the slots
that are other nodes' turns. A receiver that has seen a preamble holds its
hop until the frame lands - for the header time until a header follows,
since the detector fires on noise, then for the longest frame the
modulation allows - and a node holds a transmit for the same reason. The
console reports `hop: clock on gps time` and `hop: clock from node N
(stratum K)` as the clock changes hands, the periodic status line carries
the channel and stratum, and the telemetry characteristic reports both to
the app's Status page.

`tools/radio_sim.py` simulates a few boards on this plan - the hop clock,
the receiver, the GPS UART and the BLE notifier - and
`docs/RADIO-AUDIT.md` is what it found; `pixi run radio-sim` runs it.

None of this is a certification. At the default the node holds one 500 kHz
carrier, which is the band's digital-modulation route rather than its
hopping one; raised to fifty channels the plan follows the shape of the
hopping rule - each channel used equally on average, receivers in step.
Whether a given board and antenna comply either way is a measurement.

## Beacon payload

`fields` decides what goes on the air. The default is position only: about
half the air time of a full GPS packet on every broadcast. Altitude, speed,
course, satellite count and time still reach a connected phone over BLE
whether or not they are transmitted - the choice is only about what a
*remote* receiver gets.

`lat` and `lon` are required; a config that omits either is rejected. The
selected set travels in the frame as a one-byte mask, so nodes configured
differently interoperate: a receiver decodes whatever each sender chose to
include, and fields nobody sent read back as zero.

Air time is the scarce resource on a shared band, and it grows with the
spreading factor - at SF12 a field costs about 32 times what it does at
SF7. Add fields when a receiver needs them, not by default.

## No-fix ping

A node with no fix has no position to broadcast, and a silent node looks
exactly like one out of range or one that is dead. So the beacon goes out
anyway, carrying a 4-byte ping instead: uptime in seconds, whether the GPS
module is talking at all, and whether a fix was ever held. A receiver then
knows the node is up, roughly how long it has been searching, and whether
to look at the sky or at the board.

It goes out on its own, slower period, `ping_interval_s` - every 5 s
against the position's every second - since a receiver still searching has
nothing new to report between pings, and a ping is smaller than the leanest
position. The moment a fix lands, the next transmission is a position on
the beacon interval rather than a ping waiting out its own.
`ping_interval_s = 0` turns the ping off by itself; `interval_s = 0` and
`role = "rx_only"` silence the node altogether, pings included.

A node that hears a ping reports it as a status line (`node 3 ping: rssi
-97, up 214s, gps ok`) rather than a position, and out on the node-ping
characteristic as data, so an app can show the node as alive-without-a-fix.
The RSSI in either form is what makes a ping useful as a range check.

## Leaves and repeaters

Every transmission is a broadcast and every node listens continuously, so
a fleet of plain leaves already works: each hears whichever others are in
direct range, and the defaults need no changing. Setting `role =
"repeater"` on one node makes it retransmit what it hears, which is how
you cover ground no pair of leaves can reach across directly. Give a
repeater the elevation and the antenna - that, not the protocol, is where
the range comes from.

`max_hops` belongs to the *sender*: it is the number of retransmissions
that node's own broadcasts are allowed, stamped into each frame as it goes
out. A repeater forwards anything still carrying hops, so dropping one
into an existing fleet works without reconfiguring the nodes already
deployed. Frames are identified by sender and sequence number, so a frame
that arrives twice is handled once and two repeaters cannot bounce one
back and forth (`midair_proto::dedup`; `dedup_ttl_s` must stay under the
time the sequence number takes to wrap, and the parser refuses a value
that does not). A repeat is queued with a random delay and then moved into
this node's own turn, so two repeaters that heard the same frame do not
collide on it.

Each hop is another full transmission of the same frame on a shared
channel. `max_hops = 1` is the setting that pays; past 2 the traffic grows
faster than the coverage.

A frame is a 3-byte header, the 4-byte sync word and the payload at its
true length, with no padding. Spreading factor is the largest range knob
here (SF7 to SF12 is roughly 12 dB) and every byte of framing is paid for
at whichever one is in use, so the framing is kept small: at the SF12
default a header byte costs 32 times what it would at SF7.

Nodes transmit on the private LoRa sync word (0x1424), not the public
LoRaWAN one, so a receiver does not lock onto LoRaWAN preambles it can never
decode. It is not configurable, and nodes on different sync words cannot
hear each other at all.

`rx_boost` is the one link-budget key that is not symmetric: it buys
roughly +2 dB of sensitivity on the node it is set on, and does nothing
for what that node transmits. Range is set by the worse of the two
directions, so it only helps where the receiving end is the weak one -
setting it on both nodes is the usual answer, at a few mA each while
listening. Every other radio key has to match across nodes to link at
all; this one does not.

A node that transmits re-checks the radio before it keys up: the SX1262 is
a separate chip with its own supply, and one that browned out and came
back is at its power-up defaults - antenna switch unpowered - where a
transmit ramps +22 dBm into an isolated port. `docs/HARDWARE.md` has the
RF path.

## The GPS settings

The `[gps]` settings are pushed to the module as a single UBX-CFG-VALSET at
boot and again whenever a new config is applied (constellation, power and
model changes take effect live). Defaults match the M10 factory set, so an
absent section is a no-op.

The same frame turns off the NMEA sentences the firmware does not read (GLL,
GSA, GSV, VTG), leaving RMC and GGA. The link to the module is 9600 baud, or
960 bytes a second, and GSV alone can exceed that in one epoch once several
constellations are enabled - which delays the two sentences that carry the
fix behind sentences nothing parses. The module acknowledges the frame; a
push that lands before it has finished starting, or after a wake from backup
that took the RAM settings with it, is retried on the first sentence.
