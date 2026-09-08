//! Reference vectors for the Python radio simulator's port of the hop
//! clock (`tools/radio_sim.py`).
//!
//! The simulator has to hop exactly as the firmware does or its collision
//! and sync figures describe some other network. It cannot link this
//! crate, so it checks itself against what this prints:
//!
//! ```text
//! cargo run --example hop_vectors > ../tools/hop_vectors.json
//! python3 ../tools/radio_sim.py --selftest
//! ```
//!
//! Hand-rolled JSON: the crate has no serde dependency and the shapes are
//! flat lists of integers.

use midair_proto::hop::{Clock, Plan, SyncWord};
use midair_proto::radiocfg::RadioConfig;

fn list<I: IntoIterator<Item = T>, T: std::fmt::Display>(items: I) -> String {
    let parts: Vec<String> = items.into_iter().map(|v| v.to_string()).collect();
    format!("[{}]", parts.join(","))
}

fn main() {
    // Fifty channels, not the one-channel default: the simulator's
    // scenarios hop, and vectors from a plan with nowhere to hop to would
    // check the permutation against a column of zeroes.
    let cfg = RadioConfig { hop_channels: 50, ..RadioConfig::default() };
    let plan = Plan::from_config(&cfg);
    let mut out = String::from("{\n");

    out += &format!(
        "\"plan\": {{\"channels\": {}, \"step_khz\": {}, \"center_hz\": {}, \"dwell_ms\": {}, \"sub_slots\": {}, \"guard_ms\": {}, \"window_ms\": {}, \"sub_slot_ms\": {}}},\n",
        plan.channels, plan.step_khz, plan.center_hz, plan.dwell_ms, plan.sub_slots,
        plan.guard_ms(), plan.window_ms(), plan.sub_slot_ms()
    );
    out += &format!("\"channel_hz\": {},\n", list((0..plan.channels).map(|i| plan.channel_hz(i))));
    out += &format!("\"index_for_slot\": {},\n", list((0..250u32).map(|s| plan.index_for_slot(s))));
    out += &format!(
        "\"index_for_slot_high\": {},\n",
        list((0..60u32).map(|s| plan.index_for_slot(0xF_FF00 + s)))
    );
    out += &format!(
        "\"start_range_for\": {},\n",
        list((1..=6u8).flat_map(|a| {
            [1u32, 5].into_iter().flat_map(move |n| {
                [289u32, 240, 370, 450, 900].into_iter().flat_map(move |t| {
                    let (lo, hi) = plan.start_range_for(a, n, t);
                    [u32::from(a), n, t, lo, hi]
                })
            })
        }))
    );
    out += &format!(
        "\"turn_slot\": {},\n",
        list((1..=12u8).flat_map(|a| {
            [1u32, 5, 20]
                .into_iter()
                .flat_map(move |n| [plan.turn_slot(a, n), u32::from(plan.sub_slot_of(a, n)), plan.turns(n)])
        }))
    );

    // Time on air across the modulations the sim exercises.
    let mut toa = Vec::new();
    for (sf, bw, cr) in [(12u8, 500u16, 5u8), (11, 500, 5), (10, 500, 5), (9, 62, 5), (7, 125, 5), (12, 125, 5), (12, 500, 8)] {
        let c = RadioConfig {
            spreading_factor: sf,
            bandwidth_khz: bw,
            coding_rate: cr,
            ..RadioConfig::default()
        };
        for len in [0usize, 7, 11, 17, 28, 39] {
            toa.extend([u32::from(sf), u32::from(bw), u32::from(cr), len as u32, c.time_on_air_us(len)]);
        }
    }
    out += &format!("\"toa\": {},\n", list(toa));

    // Free-running clocks: the seed picks the origin slot and the jitter.
    let mut clocks = Vec::new();
    for seed in [1u32, 2, 3, 7, 200] {
        let mut c = Clock::new(1000, 0, seed);
        clocks.push(seed);
        clocks.push(c.slot(0));
        clocks.push(c.phase_ms(0));
        clocks.push(u32::from(c.stratum(0)));
        for i in 0..12u64 {
            let now = 10_000 + i * 37;
            let start = c.tx_start(&plan, (seed % 3) as u8 + 1, 1000 * (1 + seed % 2), now, 289);
            clocks.push(start as u32);
        }
        clocks.push(c.word_at(12_345).to_u32());
    }
    out += &format!("\"clocks\": {},\n", list(clocks));

    // GPS discipline and a follower adopting a heard frame.
    let mut a = Clock::new(1000, 0, 1);
    a.discipline_gps(1_000_000, 10_000);
    let word = a.word_at(20_300);
    let mut b = Clock::new(1000, 3, 2);
    let adopted = b.offer(word, 1, 2, 77_300, 77_589);
    let mut c = Clock::new(1000, 5_000, 7);
    c.discipline_gps(45_296_250, 5_000);
    out += &format!(
        "\"sync\": {},\n",
        list([
            word.to_u32(),
            u32::from(word.slot == a.slot(20_300)),
            match adopted {
                midair_proto::hop::Offer::Adopted(s) => u32::from(s),
                midair_proto::hop::Offer::Kept => 99,
            },
            b.slot(77_300),
            b.phase_ms(77_300),
            u32::from(b.stratum(77_589)),
            b.word_at(77_800).to_u32(),
            c.slot(5_000),
            c.phase_ms(5_000),
            c.slot(5_750),
            u32::from(c.turn_due(&plan, 1, Some(10_100), 1000, 10_300)),
            u32::from(c.turn_due(&plan, 1, Some(10_100), 1000, 11_000)),
            u32::from(c.turn_due(&plan, 3, None, 5000, 10_500)),
            u32::from(c.turn_due(&plan, 3, None, 5000, 11_500)),
            c.wait_for_window_ms(&plan, 1, 1000, 10_300, 289),
            c.wait_for_window_ms(&plan, 2, 1000, 10_300, 289),
            c.wait_for_window_ms(&plan, 2, 5000, 10_300, 289),
            c.wait_for_window_ms(&plan, 7, 5000, 10_300, 289),
            c.next_slot_start_ms(10_300) as u32,
            SyncWord { slot: 0xABCDE, stratum: 9, phase: 200 }.to_u32(),
        ])
    );
    out += &format!(
        "\"unit_ms\": {}, \"header_ms\": {}\n}}\n",
        cfg.hop_unit_airtime_us().div_ceil(1000),
        cfg.header_time_us().div_ceil(1000)
    );
    print!("{out}");
}
