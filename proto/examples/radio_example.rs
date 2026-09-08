//! The reference config file, printed from the key table.
//!
//! `RADIO.example.toml` at the repository root is this program's output,
//! and a test holds it to that - so the file, the parser's ranges and the
//! app's field help cannot drift apart, because they are one table.
//!
//! ```text
//! cargo run --example radio_example > ../RADIO.example.toml
//! ```

fn main() {
    let mut out = String::new();
    midair_proto::radiocfg::write_example(&mut out).expect("a String does not fail");
    print!("{out}");
}
