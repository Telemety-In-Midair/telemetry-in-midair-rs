fn main() {
    emit_ble_address();
    // make sure linkall.x is the last linker script (otherwise might cause problems with flip-link)
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}

/// Validate an optional `BLE_ADDRESS` override and hand it to the firmware.
///
/// The address is a static-random one written most-significant octet first,
/// e.g. "FF:C6:A1:53:50:47"; `tools/gen_ble_address.py` prints one. Absent,
/// nothing is emitted and the firmware derives a per-chip address from the
/// eFuse MAC (`option_env!` sees `None`), which is what makes every board
/// unique out of the box. Validating here means a bad override fails the
/// build rather than flashing a radio that will not advertise.
fn emit_ble_address() {
    println!("cargo:rerun-if-env-changed=BLE_ADDRESS");
    let Ok(raw) = std::env::var("BLE_ADDRESS") else {
        return;
    };

    let octets: Vec<u8> = raw
        .split(':')
        .map(|o| {
            u8::from_str_radix(o, 16)
                .unwrap_or_else(|_| panic!("BLE_ADDRESS octet {o:?} is not two hex digits"))
        })
        .collect();
    if octets.len() != 6 {
        panic!(
            "BLE_ADDRESS must be 6 colon-separated octets, got {}",
            octets.len()
        );
    }
    // Static-random requires the two most-significant bits of the MSB set.
    if octets[0] & 0xC0 != 0xC0 {
        panic!(
            "BLE_ADDRESS {raw:?} is not a static-random address: the top two bits of the first \
             octet must be set (first octet 0xC0-0xFF)"
        );
    }

    let normalized = octets
        .iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(":");
    println!("cargo:rustc-env=BLE_ADDRESS={normalized}");
}

// The retired ESP32-C6 firmware also installed a `--error-handling-script`
// here, to translate undefined-symbol errors into the missing dependency they
// usually mean. That is an lld flag, and this target links through
// xtensa-esp32s3-elf-gcc, which rejects it outright.
