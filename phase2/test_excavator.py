"""End-to-end check of the excavator: excavator_server.py -> WebSocket -> three.js site in GPU Chrome.

Usage (excavator_server.py running, site served on 127.0.0.1:8731):
    DISPLAY=:1 python test_excavator.py
1. auto dig-swing-load cycle for ~60 s, sampling what the browser shows (incl. truck load)
2. manual: hold keys to drive the bucket down into the hard floor and check the stall indicator
3. manual at 40 % relief pressure: crowd the stick through the sand and check it slows/stalls
"""

import json
import time

from playwright.sync_api import sync_playwright

CHROME = "/home/dttai1/Documents/construction-site-simulation/phase0/tools/chrome-deb/root/opt/google/chrome/chrome"
URL = "http://127.0.0.1:8731/web/index.html"
SAMPLE = """(() => { const f = window.__soil?.lastFrame; if (!f || f.kind !== 'excavator') return null;
  const d = (v) => Math.round(v * 180 / Math.PI);
  return { t: +f.simTime.toFixed(2), rtf: +f.rtf.toFixed(2), stage: f.stage, q: f.q.map(d), qd: f.qd.map(v => +v.toFixed(2)),
    press: f.press.map(v => +v.toFixed(2)), fill: Math.round(f.fill), fx: Math.round(f.force[0]), fy: Math.round(f.force[1]),
    grains: f.grains.count, truck: Math.round(f.truck), stalled: window.__soil.stalled, recvFps: +window.__soil.stats.fps.toFixed(1), model: window.__soil.model, siteFps: Math.round(window.__site.stats.fps) }; })()"""


def sample_for(page, seconds, every=0.5, shots=None, prefix=""):
    out = []
    t0 = time.time()
    shots = dict(shots or {})
    while time.time() - t0 < seconds:
        time.sleep(every)
        s = page.evaluate(SAMPLE)
        if s:
            out.append(s)
        for k in list(shots):
            if time.time() - t0 >= k:
                page.screenshot(path=f"logs/{prefix}{shots.pop(k)}.png")
    return out


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
    page.wait_for_function("window.__soil && window.__soil.lastFrame && window.__soil.lastFrame.kind === 'excavator'", timeout=120_000)
    page.eval_on_selector("#relief", "el => { el.value = 100; el.dispatchEvent(new Event('input')); }")
    page.click("#soil-reset")
    time.sleep(1.5)
    page.click("#soil-view")
    page.click("#soil-auto")
    auto = sample_for(page, 62, shots={5: "drag", 11: "lift", 15: "swing", 19: "dump", 60: "after_two_loads"}, prefix="load_")
    page.click("#soil-auto")  # stop
    time.sleep(1)
    # manual swing with the arrow keys
    page.mouse.click(1000, 450)
    swing_before = page.evaluate(SAMPLE)
    page.keyboard.down("ArrowLeft")
    swing_hold = sample_for(page, 3)
    page.keyboard.up("ArrowLeft")
    swing_after = page.evaluate(SAMPLE)

    # manual: boom down + bucket dump-ish until the bucket meets the hard floor -> stall
    page.click("#soil-reset")
    time.sleep(1.5)
    page.mouse.click(1000, 450)  # focus the canvas so key events reach the window
    page.keyboard.down("s")  # boom lower
    page.keyboard.down("d")  # stick out
    floor = sample_for(page, 7, shots={6: "floor_stall"}, prefix="exc_")
    page.keyboard.up("s")
    page.keyboard.up("d")
    time.sleep(1)

    # manual at 40 % relief: stick in (A) through the sand
    page.eval_on_selector("#relief", "el => { el.value = 40; el.dispatchEvent(new Event('input')); }")
    page.click("#soil-reset")
    time.sleep(1.5)
    page.mouse.click(1000, 450)
    page.keyboard.down("s")
    weak_down = sample_for(page, 3)
    page.keyboard.up("s")
    page.keyboard.down("a")  # stick in: drag toward the machine
    weak = sample_for(page, 6, shots={5: "weak_stall"}, prefix="exc_")
    page.keyboard.up("a")
    status = page.inner_text("#soil-status")
    browser.close()

print(json.dumps({"status": status, "console_errors": errors[:10], "auto": auto, "swing_before": swing_before, "swing_hold": swing_hold, "swing_after": swing_after, "floor": floor, "weak_down": weak_down, "weak": weak}, indent=1))
