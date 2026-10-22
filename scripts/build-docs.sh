#!/usr/bin/env bash
# Build the rolcb documentation site (rustdoc + guide chapters) into target/site.
#
# Only the library is documented; `rolcb-hub` usage lives in the hub guide
# chapter. The crate's pages are moved to the site root (scripts/flatten-docs.py)
# so URLs read /rolcb/node/... rather than /rolcb/rolcb/node/...
#
#   scripts/build-docs.sh            # build
#   scripts/build-docs.sh --open     # build and open in a browser
#
# Uses a nightly toolchain with `--cfg docsrs` when one is installed, so the
# "Available on crate feature X only" badges render exactly as on docs.rs.
# Falls back to stable (no badges) otherwise. Warnings are errors either way.
set -euo pipefail

cd "$(dirname "$0")/.."
export PATH="$HOME/.cargo/bin:$PATH"

OPEN=0
for arg in "$@"; do
    case "$arg" in
        --open) OPEN=1 ;;
        -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done

rm -rf target/doc
FLAGS="-D warnings -D rustdoc::broken_intra_doc_links"
if rustup run nightly rustc --version >/dev/null 2>&1; then
    echo "==> building docs with nightly (doc_cfg feature badges enabled)"
    RUSTDOCFLAGS="--cfg docsrs $FLAGS" cargo +nightly doc --no-deps --lib --all-features
else
    echo "==> nightly not installed; building docs with stable (no feature badges)"
    RUSTDOCFLAGS="$FLAGS" cargo doc --no-deps --lib --all-features
fi

python3 scripts/flatten-docs.py target/doc rolcb target/site
echo "==> site written to $(pwd)/target/site/index.html"

if [ "$OPEN" = 1 ]; then
    if command -v xdg-open >/dev/null; then xdg-open target/site/index.html
    elif command -v open >/dev/null; then open target/site/index.html
    else echo "open target/site/index.html in your browser"; fi
fi
