fn main() {
    // make sure linkall.x is the last linker script (otherwise might cause problems with flip-link)
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}

// The esp32c6-gps crate also installs a `--error-handling-script` here to
// translate undefined-symbol errors into the missing dependency they
// usually mean. That is an lld flag, and this target links through
// xtensa-esp32s3-elf-gcc, which rejects it outright.
