"""End-to-end check: soil server -> WebSocket -> three.js site in GPU Chrome.

Usage (with soil_server.py running and the site served on 127.0.0.1:8731):
    DISPLAY=:1 python test_stream.py
Runs the automatic dig cycle for ~22 s, samples what the browser receives, and saves screenshots.
"""

import json
import time

from playwright.sync_api import sync_playwright

CHROME = "/home/dttai1/Documents/construction-site-simulation/phase0/tools/chrome-deb/root/opt/google/chrome/chrome"
URL = "http://127.0.0.1:8731/web/index.html"
SAMPLE = """(() => { const f = window.__soil?.lastFrame; const s = window.__soil?.stats; if (!f) return null;
  return { t: +f.simTime.toFixed(2), stepMs: +f.stepMs.toFixed(1), rtf: +f.rtf.toFixed(2), grains: f.grains.count,
    fill: Math.round(f.fill), fx: Math.round(f.force[0]), fy: Math.round(f.force[1]), cut: +f.cut.toFixed(3), fee: Math.round(f.fee),
    bucketY: +f.pos[1].toFixed(2), recvFps: +(s?.fps ?? 0).toFixed(1), mbps: +(s?.mbps ?? 0).toFixed(2),
    siteFps: Math.round(window.__site.stats.fps) }; })()"""

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
    page.wait_for_function("window.__soil && window.__soil.lastFrame", timeout=120_000)
    page.click("#soil-reset")
    time.sleep(1)
    page.click("#soil-view")
    page.click("#soil-auto")
    samples = []
    shots = {4.5: "dig", 9.5: "lift", 13.8: "dump"}
    t0 = time.time()
    while time.time() - t0 < 22:
        time.sleep(0.5)
        s = page.evaluate(SAMPLE)
        if s:
            samples.append(s)
        el = time.time() - t0
        for k in list(shots):
            if el >= k:
                page.screenshot(path=f"logs/stream_{shots.pop(k)}.png")
    # surface after one cycle, and a prop resting on the sand?
    page.click("#soil-auto")
    time.sleep(2)
    page.screenshot(path="logs/stream_after_cycle.png")
    status = page.inner_text("#soil-status")
    browser.close()

print(json.dumps({"status": status, "console_errors": errors[:10], "samples": samples}, indent=1))
