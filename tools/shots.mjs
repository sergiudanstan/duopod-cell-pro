// Deterministic Playwright stills of the cell, plus console errors and an fps reading.
// Usage: node tools/shots.mjs [--quick] [--cad=0]
//   --quick  iso and pick at 4.2 s only; --cad=0 forces the procedural robot
import { spawn } from "node:child_process";
import { mkdirSync } from "node:fs";
import { homedir } from "node:os";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const { chromium } = await import("/opt/homebrew/lib/node_modules/playwright/index.mjs");

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const outDir = join(homedir(), "Downloads/duopod_cell_pro/shots");
const cad0 = process.argv.includes("--cad=0");
const quick = process.argv.includes("--quick");
const views = ["iso", "front", "side", "top", "robot", "pick"];
const times = [3.45, 4.2, 4.7];
const shots = quick ? [["iso", 4.2], ["pick", 3.45]] : views.flatMap((v) => times.map((t) => [v, t]));

mkdirSync(outDir, { recursive: true });

const server = spawn("python3", ["-m", "http.server", "8777", "--bind", "127.0.0.1"], {
  cwd: root,
  stdio: ["ignore", "pipe", "pipe"],
});
let serverLog = "";
server.stdout.on("data", (d) => { serverLog += d; });
server.stderr.on("data", (d) => { serverLog += d; });

const errors = [];
let browser;
function ignore(text) {
  return false;
}

try {
  await new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error("http.server did not answer on 127.0.0.1:8777")), 8000);
    const ping = async () => {
      if (server.exitCode != null) {
        clearTimeout(timer);
        reject(new Error("http.server exited\n" + serverLog));
        return;
      }
      try {
        const r = await fetch("http://127.0.0.1:8777/");
        if (r.status < 500) { clearTimeout(timer); resolve(); return; }
      } catch { /* not up yet */ }
      setTimeout(ping, 100);
    };
    ping();
  });

  browser = await chromium.launch({
    channel: "chrome",
    headless: false,
    args: ["--window-size=1700,1050"],
  });
  const page = await browser.newPage({ viewport: { width: 1600, height: 900 } });
  page.on("console", (msg) => {
    if (msg.type() !== "error") return;
    const text = msg.text();
    if (ignore(text)) return;
    errors.push(text);
  });
  page.on("pageerror", (err) => {
    const text = String(err);
    if (ignore(text)) return;
    errors.push(text);
  });

  await page.goto("http://127.0.0.1:8777/index.html" + (cad0 ? "?cad=0" : ""), { waitUntil: "domcontentloaded", timeout: 60000 });
  await page.waitForFunction(() => window.pcReady === true, { timeout: 45000 });
  if (!cad0) await page.waitForFunction(() => window.pc.cad === true, { timeout: 30000 });
  // The HQ watchdog closes after the first 3 s. Wait it out, then lock HQ so the stills show AO.
  await page.waitForTimeout(3200);
  await page.evaluate(() => window.setQuality(true));

  const paths = [];
  for (const [v, t] of shots) {
    await page.evaluate(({ v, t }) => {
      window.pc.pause();
      window.pc.seek(t);
      window.pc.setView(v);
    }, { v, t });
    await page.waitForTimeout(1200);
    const file = join(outDir, `${cad0 ? "proc_" : ""}${v}_${t.toFixed(2)}.png`);
    await page.screenshot({ path: file });
    paths.push(file);
    console.log(file);
  }

  await page.evaluate(() => {
    window.pc.setView("iso"); window.pc.seek(4.2);
    document.getElementById("guards").checked = false;
    document.getElementById("guards").dispatchEvent(new Event("change", { bubbles: true }));
  });
  await page.waitForTimeout(1200);
  const guardsOff = join(outDir, `${cad0 ? "proc_" : ""}guards_off_4.20.png`);
  await page.screenshot({ path: guardsOff });
  console.log(guardsOff);
  await page.evaluate(() => {
    document.getElementById("guards").checked = true;
    document.getElementById("guards").dispatchEvent(new Event("change", { bubbles: true }));
  });

  const checks = await page.evaluate(() => {
    const out = [];
    for (const t of [0.2, 1.0, 4.2, 7.3]) { window.pc.seek(t); window.pc.render(); out.push(window.pc.cadCheck()); }
    return { cad: window.pc.cad, cadMM: out, hq: window.__hqStats ?? null };
  });
  console.log("cad loaded", checks.cad, "cadCheck mm", checks.cadMM.map((v) => (v == null ? "n/a" : v.toFixed(2))).join(" "));
  if (checks.cad && checks.cadMM.some((v) => v == null || v > 0.5)) errors.push("CAD plate does not follow the logic flange: " + checks.cadMM.join(" "));
  if (checks.hq) console.log("watchdog avgMs", checks.hq.avgMs.toFixed(1), "tripped", checks.hq.tripped);

  async function fps(on) {
    await page.evaluate((hq) => {
      window.setQuality(hq);
      window.pc.setView("iso");
      window.pc.play();
    }, on);
    await page.waitForTimeout(900);
    return page.evaluate(() => new Promise((resolve) => {
      const t0 = performance.now();
      let frames = 0;
      const step = (now) => {
        frames += 1;
        if (now - t0 >= 3000) resolve(frames * 1000 / (now - t0));
        else requestAnimationFrame(step);
      };
      requestAnimationFrame(step);
    }));
  }
  const fpsHq = await fps(true);
  const fpsOff = await fps(false);
  console.log(`fps HQ ${fpsHq.toFixed(1)}`);
  console.log(`fps noHQ ${fpsOff.toFixed(1)}`);

  if (errors.length) {
    console.error(`console errors: ${errors.length}`);
    for (const e of errors) console.error(e);
    process.exitCode = 1;
  } else {
    console.log("console errors: 0");
  }
} finally {
  if (browser) await browser.close();
  server.kill("SIGTERM");
}
