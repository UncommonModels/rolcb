#!/usr/bin/env bash
# Build the rolcb documentation site (rustdoc + guide chapters) into target/doc.
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
        -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done

FLAGS="-D warnings -D rustdoc::broken_intra_doc_links"
if rustup run nightly rustc --version >/dev/null 2>&1; then
    echo "==> building docs with nightly (doc_cfg feature badges enabled)"
    RUSTDOCFLAGS="--cfg docsrs $FLAGS" cargo +nightly doc --no-deps --all-features
else
    echo "==> nightly not installed; building docs with stable (no feature badges)"
    RUSTDOCFLAGS="$FLAGS" cargo doc --no-deps --all-features
fi

# Root redirect so the site can be served from target/doc directly
# (e.g. GitHub Pages).
cat > target/doc/index.html <<'HTML'
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>rolcb documentation</title>
<meta http-equiv="refresh" content="0; url=rolcb/index.html">
<link rel="canonical" href="rolcb/index.html">
</head>
<body>
<p>Redirecting to <a href="rolcb/index.html">rolcb/index.html</a>…</p>
</body>
</html>
HTML
touch target/doc/.nojekyll

echo "==> docs written to $(pwd)/target/doc/rolcb/index.html"

if [ "$OPEN" = 1 ]; then
    if command -v xdg-open >/dev/null; then xdg-open target/doc/index.html
    elif command -v open >/dev/null; then open target/doc/index.html
    else echo "open target/doc/index.html in your browser"; fi
fi
