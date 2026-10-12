"""Zip the CAD source, STEP files, print STLs, drawings and renders for a reviewer."""
import pathlib, zipfile
root = pathlib.Path(__file__).resolve().parent.parent
out = root / "review" / "WK10-cad-package.zip"
globs = ["README.md", "cad/wk10.py", "cad/sheets.py", "review/data.json", "cad/out/*.step",
         "cad/out/print/*.stl", "review/sheets/*.svg", "cad/out/render_*.png"]
files = [p for g in globs for p in sorted(root.glob(g))]
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
    for p in files:
        z.write(p, "WK10/" + p.relative_to(root).as_posix())
print(len(files), "files,", out.stat().st_size // 1024, "KB ->", out)
