# Vendored crates

`esp-radio` is the crates.io 0.17.0 release plus `esp-radio.patch`: the
`TxPower` re-export and the BLE controller's modem sleep, which upstream
leaves unimplemented. The `[patch.crates-io]` in the firmware's `Cargo.toml`
points at it.

`esp-radio-patch.sh` keeps the two level:

```sh
./esp-radio-patch.sh check   # is vendor/esp-radio the release plus the patch?
./esp-radio-patch.sh make    # record edits to vendor/esp-radio in the patch
./esp-radio-patch.sh apply   # rebuild vendor/esp-radio from the release plus the patch
```

Review the patch, not the tree: it is a thousand lines against a crate of
tens of thousands. Moving to a newer esp-radio is bump `VERSION` in the
script, `cargo fetch`, `apply`, fix whatever hunks no longer land, `make`.
