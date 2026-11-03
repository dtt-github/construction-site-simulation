"""Open webgpu/index.html in the locally unpacked Chrome under several flag sets and print results."""

import http.server
import json
import os
import sys
import threading
from functools import partial

from playwright.sync_api import sync_playwright

HERE = os.path.dirname(os.path.abspath(__file__))
CHROME = os.path.join(HERE, "..", "tools", "chrome-deb", "root", "opt", "google", "chrome", "chrome")

CONFIGS = {
    "default (headed, X11)": dict(headless=False, args=[]),
    "unsafe-webgpu (headed, X11)": dict(headless=False, args=["--enable-unsafe-webgpu"]),
    "unsafe-webgpu + Vulkan (headed, X11)": dict(
        headless=False, args=["--enable-unsafe-webgpu", "--enable-features=Vulkan", "--use-angle=vulkan"]
    ),
    "unsafe-webgpu + Vulkan (headless)": dict(
        headless=True, args=["--enable-unsafe-webgpu", "--enable-features=Vulkan", "--use-angle=vulkan"]
    ),
}


def main():
    handler = partial(http.server.SimpleHTTPRequestHandler, directory=HERE)
    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    port = httpd.server_address[1]
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{port}/index.html"

    results = {}
    with sync_playwright() as pw:
        for name, cfg in CONFIGS.items():
            browser = pw.chromium.launch(executable_path=CHROME, headless=cfg["headless"], args=cfg["args"])
            page = browser.new_page()
            page.goto(url)
            try:
                page.wait_for_function("document.title === 'DONE'", timeout=120_000)
                res = page.evaluate("window.__result")
            except Exception as e:  # noqa: BLE001
                res = {"ok": False, "error": f"timeout or crash: {e}"}
            gpu_page = browser.new_page()
            try:
                gpu_page.goto("chrome://gpu")
                text = gpu_page.inner_text("body")
                status = [ln for ln in text.splitlines() if ln.startswith(("WebGPU", "Vulkan", "WebGL"))][:6]
            except Exception:  # noqa: BLE001
                status = []
            res["chrome_gpu_status"] = status
            results[name] = res
            browser.close()
            print(f"== {name}\n{json.dumps(res, indent=2)}", flush=True)

    httpd.shutdown()
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "logs", "chrome_webgpu.json")
    with open(out, "w") as fh:
        json.dump(results, fh, indent=2)


if __name__ == "__main__":
    main()
