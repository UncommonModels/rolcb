#!/usr/bin/env python3
"""Turn rustdoc output into a site whose root *is* the crate.

rustdoc writes `target/doc/<crate>/...` next to shared files
(`static.files/`, `search.index/`, `src/`, ...). Published as-is on a site
that is already named after the crate, every URL reads `/rolcb/rolcb/...`.

This copies the crate's pages to the site root, rewrites the links that
point above the crate directory, and leaves small redirect pages at the old
`<crate>/...` paths (rustdoc's search and source pages build links that way).

    flatten-docs.py target/doc rolcb target/site
"""

import re
import shutil
import sys
from pathlib import Path

ATTR = re.compile(r'(=")((?:\.\./)+)')

REDIRECT = """<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>Redirecting…</title>
<meta http-equiv="refresh" content="0; url={target}">
<link rel="canonical" href="{target}">
<script>location.replace("{target}" + location.search + location.hash);</script>
</head><body><a href="{target}">Moved here</a></body></html>
"""


def main() -> None:
    doc, crate, out = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3])
    crate_dir = doc / crate
    if not crate_dir.is_dir():
        sys.exit(f"{crate_dir} not found; run cargo doc first")
    if out.exists():
        shutil.rmtree(out)

    # Shared files keep their place relative to the doc root.
    top = doc.resolve()
    shutil.copytree(
        doc,
        out,
        ignore=lambda d, names: [n for n in names if Path(d).resolve() == top and n in (crate, ".lock")],
    )

    for src in crate_dir.rglob("*"):
        rel = src.relative_to(crate_dir)
        dst = out / rel
        if src.is_dir():
            dst.mkdir(parents=True, exist_ok=True)
            continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        if src.suffix != ".html":
            shutil.copy2(src, dst)
            continue

        # A page `depth` directories below the crate root reaches the doc
        # root with depth+1 "../"; at the site root it needs one fewer.
        depth = len(rel.parts) - 1
        above = "../" * (depth + 1)

        def fix(m: re.Match) -> str:
            ups = m.group(2)
            if ups.startswith(above):
                ups = ups[3:]
            return m.group(1) + ups

        html = ATTR.sub(fix, src.read_text(encoding="utf-8"))
        # rustdoc wants a non-empty root path.
        html = html.replace('data-root-path=""', 'data-root-path="./"')
        dst.write_text(html, encoding="utf-8")

        # Redirect stub at the old location.
        stub = out / crate / rel
        stub.parent.mkdir(parents=True, exist_ok=True)
        stub.write_text(REDIRECT.format(target="../" * (depth + 1) + rel.as_posix()), encoding="utf-8")

    (out / ".nojekyll").touch()
    print(f"site written to {out}")


if __name__ == "__main__":
    main()
