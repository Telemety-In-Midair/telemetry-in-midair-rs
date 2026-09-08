#!/bin/sh
# The vendored esp-radio as a patch against the crates.io release.
#
#   ./esp-radio-patch.sh make    regenerate esp-radio.patch from vendor/esp-radio
#   ./esp-radio-patch.sh apply   rebuild vendor/esp-radio from the registry copy plus the patch
#   ./esp-radio-patch.sh check   verify vendor/esp-radio equals the registry copy plus the patch
#
# The registry copy is whatever `cargo fetch` put under ~/.cargo for the
# pinned version; `apply` and `check` need it there, `make` too. Moving to a
# newer esp-radio is: bump VERSION, `cargo fetch`, `apply`, fix the hunks
# that no longer land, then `make` to record the result.
set -eu

VERSION=0.17.0
HERE=$(cd "$(dirname "$0")" && pwd)
PATCH="$HERE/esp-radio.patch"
VENDORED="$HERE/esp-radio"
REGISTRY=$(ls -d "$HOME"/.cargo/registry/src/*/esp-radio-"$VERSION" 2>/dev/null | head -n 1)
if [ -z "$REGISTRY" ]; then
    echo "esp-radio $VERSION is not in the cargo registry; run cargo fetch first" >&2
    exit 1
fi

stage() {
    TMP=$(mktemp -d)
    trap 'rm -rf "$TMP"' EXIT
    mkdir -p "$TMP/a" "$TMP/b"
    cp -r "$REGISTRY" "$TMP/a/esp-radio"
    # The registry's own bookkeeping, which the patch is not about.
    rm -f "$TMP/a/esp-radio/.cargo-ok" "$TMP/a/esp-radio/.cargo_vcs_info.json" "$TMP/a/esp-radio/Cargo.toml.orig"
}

case "${1:-}" in
    make)
        stage
        cp -r "$VENDORED" "$TMP/b/esp-radio"
        rm -f "$TMP/b/esp-radio/Cargo.toml.orig"
        # diff exits 1 when the trees differ, which is the point.
        (cd "$TMP" && diff -ruN a b > "$PATCH") || true
        echo "wrote $(wc -l < "$PATCH") lines to $PATCH"
        ;;
    apply)
        stage
        (cd "$TMP/a" && patch -p1 < "$PATCH")
        rm -rf "$VENDORED"
        cp -r "$TMP/a/esp-radio" "$VENDORED"
        echo "rebuilt $VENDORED from esp-radio $VERSION plus the patch"
        ;;
    check)
        stage
        (cd "$TMP/a" && patch -p1 --silent < "$PATCH")
        if diff -r -x Cargo.toml.orig "$TMP/a/esp-radio" "$VENDORED" > /dev/null; then
            echo "vendor/esp-radio is esp-radio $VERSION plus esp-radio.patch"
        else
            echo "vendor/esp-radio differs from esp-radio $VERSION plus esp-radio.patch; run make" >&2
            diff -r -x Cargo.toml.orig "$TMP/a/esp-radio" "$VENDORED" | head -n 20 >&2
            exit 1
        fi
        ;;
    *)
        echo "usage: $0 make|apply|check" >&2
        exit 2
        ;;
esac
