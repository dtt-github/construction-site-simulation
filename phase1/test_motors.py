"""Check the polished site: terrain contact, Jolt masses, fork lift with payload, driving, ramps.

Usage: DISPLAY=:1 python test_motors.py   (run from phase1/, site served on 127.0.0.1:8731)
"""

import json
import time

from playwright.sync_api import sync_playwright

CHROME = "/home/dttai1/Documents/construction-site-simulation/phase0/tools/chrome-deb/root/opt/google/chrome/chrome"
URL = "http://127.0.0.1:8731/web/index.html"
FORK = "joint_sm_fork_a01_joint_00"
STEER = "joint_sm_ruddebase_a01_joint_00"
RAMP = "joint_rampa1_c01_joint_00"
FK = "vehicle_forklift_orange_a01"
LIFT = "vehicleSupport_scissor_lift_c01"


def set_range(page, sel, value):
    page.eval_on_selector(sel, "(el, v) => { el.value = v; el.dispatchEvent(new Event('input')); }", value)


def bodies(page, asset=None, name=None):
    snap = page.evaluate("window.__site.snapshot()")
    return [b for b in snap if (asset is None or b["asset"] == asset) and (name is None or b["name"] == name)]


out = {}
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
    time.sleep(6)

    snap = bodies(page)
    out["settle"] = {
        "bodies": len(snap),
        "below_terrain": sorted({b["asset"] for b in snap if b["y"] < b["ground"] - 0.3}),
        "moving_over_0.2": sorted({b["asset"] for b in snap if b["speed"] > 0.2}),
        "stats": page.evaluate("window.__site.stats"),
    }
    mc = page.evaluate("window.__site.massCheck()")
    out["mass_check_max_rel_error"] = round(max(abs(m["jolt"] - m["intended"]) / m["intended"] for m in mc), 4)
    page.screenshot(path="data/polish_overview.png")

    # payload on forks
    pallet0 = [b for b in bodies(page, "pallet_a01") if abs(b["x"] + 7.2) < 0.6 and abs(b["z"] - 2) < 0.6]
    crate0 = [b for b in bodies(page, "crate_plywood_b01") if abs(b["x"] + 7.2) < 0.6 and abs(b["z"] - 2) < 0.6]
    set_range(page, "#ctl-fork", 1.2)
    time.sleep(8)
    fork_pos = page.evaluate(f"window.__site.jointValue('{FK}', '{FORK}')")
    pallet1 = [b for b in bodies(page, "pallet_a01") if abs(b["x"] + 7.2) < 0.8 and abs(b["z"] - 2) < 0.8]
    crate1 = [b for b in bodies(page, "crate_plywood_b01") if abs(b["x"] + 7.2) < 0.8 and abs(b["z"] - 2) < 0.8]
    out["fork_lift"] = {
        "target_m": 1.2,
        "fork_joint_m": round(fork_pos, 3),
        "pallet_y_before_after": [round(pallet0[0]["y"], 3) if pallet0 else None, round(pallet1[0]["y"], 3) if pallet1 else None],
        "crate_y_before_after": [round(crate0[0]["y"], 3) if crate0 else None, round(crate1[0]["y"], 3) if crate1 else None],
    }
    page.screenshot(path="data/polish_fork_raised.png")
    set_range(page, "#ctl-fork", 0.1)
    time.sleep(8)

    # drive forward 4 s, then steer test
    body0 = bodies(page, FK, "sm_body_a01_obj_00")[0]
    page.dispatch_event("#ctl-fwd", "pointerdown")
    time.sleep(4)
    page.dispatch_event("#ctl-fwd", "pointerup")
    time.sleep(1.5)
    body1 = bodies(page, FK, "sm_body_a01_obj_00")[0]
    out["drive_forward_4s"] = {
        "moved_m": round(((body1["x"] - body0["x"]) ** 2 + (body1["z"] - body0["z"]) ** 2) ** 0.5, 3),
        "dx": round(body1["x"] - body0["x"], 3),
        "dz": round(body1["z"] - body0["z"], 3),
    }
    set_range(page, "#ctl-steer", 40)
    time.sleep(2)
    out["steer_target_deg"] = 40
    out["steer_joint_deg"] = round(page.evaluate(f"window.__site.jointValue('{FK}', '{STEER}')") * 57.2958, 1)

    # ramps
    set_range(page, "#ctl-ramp", 30)
    time.sleep(3)
    out["ramp_target_deg"] = 30
    out["ramp_joint_deg"] = round(page.evaluate(f"window.__site.jointValue('{LIFT}', '{RAMP}')") * 57.2958, 1)
    page.screenshot(path="data/polish_ramps.png")

    # colliders view of fence row and scaffold
    page.click("#btn-hulls")
    time.sleep(0.5)
    page.screenshot(path="data/polish_colliders.png")
    out["final_stats"] = page.evaluate("window.__site.stats")
    out["console_errors"] = errors[:10]
    browser.close()

print(json.dumps(out, indent=2))
