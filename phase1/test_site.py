"""Open the Phase 1 site in local Chrome (GPU, X11), let physics run, and check stability and speed.

Usage: DISPLAY=:1 python test_site.py [url]
Prints JSON with load time, fps, physics step time and any bodies that fell, exploded or never settled.
"""

import json
import sys
import time

from playwright.sync_api import sync_playwright

CHROME = "/home/dttai1/Documents/construction-site-simulation/phase0/tools/chrome-deb/root/opt/google/chrome/chrome"
URL = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8731/web/index.html"

with sync_playwright() as pw:
    browser = pw.chromium.launch(
        executable_path=CHROME,
        headless=False,
        args=["--enable-unsafe-webgpu", "--enable-features=Vulkan", "--use-angle=vulkan", "--window-size=1600,900"],
    )
    page = browser.new_page(viewport={"width": 1600, "height": 900})
    errors = []
    page.on("console", lambda m: m.type == "error" and errors.append(m.text))
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(URL)
    page.wait_for_function("window.__site && window.__site.stats.loadMs > 0", timeout=120_000)
    gl = page.evaluate(
        "(() => { const c = document.querySelector('canvas'); const g = c.getContext('webgl2'); const d = g.getExtension('WEBGL_debug_renderer_info'); return d ? g.getParameter(d.UNMASKED_RENDERER_WEBGL) : 'unknown'; })()"
    )
    time.sleep(8)  # settle
    settle = page.evaluate("window.__site.snapshot()")
    stats1 = page.evaluate("window.__site.stats")
    page.screenshot(path="data/site_settled.png")

    # stress: drop 6 crates and the beam, run 8 s
    page.click("#btn-drop")
    page.click("#btn-beam")
    time.sleep(8)
    after = page.evaluate("window.__site.snapshot()")
    stats2 = page.evaluate("window.__site.stats")
    page.screenshot(path="data/site_after_drops.png")
    page.click("#btn-hulls")
    time.sleep(1)
    page.screenshot(path="data/site_colliders.png")
    browser.close()


def check(snap):
    return {
        "bodies": len(snap),
        "fell_below_ground": sorted({b["asset"] for b in snap if b["y"] < -0.3}),
        "moving_over_0.2_mps": sorted({b["asset"] for b in snap if b["speed"] > 0.2}),
        "max_speed_mps": round(max(b["speed"] for b in snap), 3),
    }


print(
    json.dumps(
        {
            "renderer": gl,
            "load_s": round(stats1["loadMs"] / 1000, 2),
            "after_8s": {**check(settle), "fps": round(stats1["fps"], 1), "step_ms": round(stats1["stepMs"], 2)},
            "after_drops_8s": {**check(after), "fps": round(stats2["fps"], 1), "step_ms": round(stats2["stepMs"], 2), "bodies_total": stats2["bodies"]},
            "console_errors": errors[:10],
        },
        indent=2,
    )
)
