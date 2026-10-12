"""Inline data.json and the drawing sheets into template.html -> index.html."""
import json, pathlib
here = pathlib.Path(__file__).parent
esc = lambda o: json.dumps(o, separators=(",", ":")).replace("</", "<\\/")
data = json.loads((here / "data.json").read_text(encoding="utf-8"))
sheets = {p.stem.split("-")[1]: p.read_text(encoding="utf-8") for p in sorted((here / "sheets").glob("WK10-*.svg"))}
html = (here / "template.html").read_text(encoding="utf-8").replace("{{DATA}}", esc(data)).replace("{{SHEETS}}", esc(sheets))
assert "{{" not in html
(here / "index.html").write_text(html, encoding="utf-8")
# artifacts serve no .stl, so the meshes ship as base64 inside one JSON file
import base64
meshes = {n: base64.b64encode((here / "models" / f"{n}.stl").read_bytes()).decode() for n in data["models"]}
(here / "models.json").write_text(json.dumps(meshes), encoding="utf-8")
print(f"index.html {len(html) // 1024} KB, {len(sheets)} sheets, {len(data['models'])} models")
