// Architecture role: browser-side presentation and report parser.
// The UI requests runs from scripts/simulation_ui_server.py, parses the stable
// TEST CASE/SUMMARY text emitted by each suite, and renders it. No estimator,
// controller, electronics, actuator, or RocketPy behavior is implemented here.

const SUITE_META = {
  flight: { label: "Flight Control Suite", symbol: "↟" },
  rocketpy: { label: "RocketPy Physics Suite", symbol: "◉" },
  electronics: { label: "Electronics Bring-up Suite", symbol: "▣" },
  actuator: { label: "Actuator Suite", symbol: "⌁" }
};

const baselineData = {
  mode: "baseline",
  timestamp: null,
  durationSeconds: null,
  raw: "",
  suites: {
    flight: {
      overall: "PASS",
      scenarios: [
        scenario("baseline", "PASS", "Nominal noisy IMU/barometer data with a strong motor and normal actuator speed.", "Estimator remains healthy, reaches AirbrakeActive, commands full deployment, and rejects no normal barometer reads.", "Estimator stayed healthy, entered AirbrakeActive, and commanded full deployment.", {
          "true apogee": "2736 ft", "target error": "-264 ft", "predicted apogee": "3070 ft", "peak command": "100.0%", "peak actual deploy": "100.0%", "rejected baro reads": "0", "final phase": "AirbrakeActive"
        }, "WARN - true apogee is outside the +/-100 ft mission tolerance."),
        scenario("baro spike", "PASS", "A one-time +120 m pressure-altitude spike at 2.6 s.", "The EKF rejects at least one barometer read and does not enter Fault.", "The injected spike was rejected and the flight computer stayed out of Fault.", { "rejected baro reads": "1", "final phase": "AirbrakeActive" }),
        scenario("imu +1.5mps2 bias", "PASS", "The accelerometer reports 1.5 m/s² too high for the whole flight.", "The estimator stays healthy, commands remain bounded, and no Fault phase appears.", "IMU bias did not produce an invalid command or force a Fault phase.", { "peak command": "100.0%", "estimator healthy": "yes" }),
        scenario("slow actuator", "PASS", "The physical airbrake moves only 0.35 deployment-fraction per second.", "Full software command happens before full physical deployment.", "Software command led the virtual mechanism, making actuator lag visible.", { "full command time": "1.900 s", "full deploy time": "4.616 s", "true apogee": "2809 ft" }),
        scenario("weak motor", "PASS", "Lower thrust and shorter burn produce a flight that should not need deployment.", "Command and deployment stay near 0%, and the phase remains Coast.", "The weak-motor flight stayed in Coast with deployment inhibited.", { "true apogee": "2042 ft", "peak command": "0.0%", "final phase": "Coast" })
      ]
    },
    rocketpy: {
      overall: "FAIL",
      scenarios: [
        scenario("M5 passive reference", "FAIL", "RocketPy runs the June 14 M5 launch conditions and stabilizing-fin geometry with the selected AeroTech J420R and no airbrake deployment.", "RocketPy completes and passive apogee remains within 10% of the current 3379 ft M5 OpenRocket value.", "The provisional model predicts 3955 ft, 17.1% above the current report value.", { "RocketPy passive apogee": "3955 ft", "M5 OpenRocket apogee": "3379 ft", "difference": "+17.1%", "maximum Mach": "0.495", "rail exit velocity": "42.7 ft/s" }, "FAIL is intentional: current mass, CG, inertia, and drag data are missing, so the model was not retuned to force agreement."),
        scenario("C++ closed-loop airbrakes", "PASS", "RocketPy feeds virtual IMU/barometer measurements into the real C++ AMBAR flight computer and applies its rate-limited airbrake command.", "The estimator stays healthy, commands remain bounded, deployment begins after burnout plus margin, commands occur only in AirbrakeActive, and apogee is reduced by more than 50 ft.", "The controller reduced this provisional trajectory from 3955 ft to 3016 ft without commanding before the post-burn enable time.", { "closed-loop apogee": "3016 ft", "target error": "+16 ft", "apogee reduction": "940 ft", "motor burn end": "1.64 s", "minimum deploy time": "1.74 s", "first C++ command": "1.78 s", "C++ command range": "0.0% to 100.0%", "estimator healthy": "yes" }, "Controller coupling passed, but target accuracy is not validated because the passive vehicle model failed its source comparison."),
        scenario("M5 flight envelope checks", "FAIL", "The closed-loop RocketPy trajectory is checked against the M5 subsonic and minimum rail-exit requirements.", "Maximum Mach is no greater than 1.0 and rail-exit velocity is at least 52 ft/s.", "The trajectory stayed subsonic but left the 72-inch rail too slowly.", { "maximum Mach": "0.495", "Mach limit": "1.0", "minimum rail exit velocity": "42.7 ft/s", "rail exit minimum": "52.0 ft/s", "M5 reported rail exit velocity": "75.5 ft/s" })
      ]
    },
    electronics: {
      overall: "ARMABLE",
      warnings: [],
      scenarios: [
        scenario("nominal V3 board", "PASS", "All modeled chips respond with current V3 constants and M5 sensor roles are kept separate.", "All modeled startup checks pass and the decision becomes ARMABLE.", "Observed decision: ARMABLE.", { "passing checks": "12", "warnings": "0", "failures": "0" }, "The airbrake board uses LIS2MDL; independent recovery GPS remains a vehicle-level requirement."),
        scenario("BMP388 SDO high", "PASS", "The barometer responds at 0x77 instead of expected 0x76.", "The BMP388 check fails and boot becomes BLOCKED.", "Injected address fault produced a BLOCKED decision.", { "observed decision": "BLOCKED" }),
        scenario("3V3 overloaded", "PASS", "The 3V3 rail is modeled at 720 mA against a 600 mA limit.", "The regulator check fails and boot becomes BLOCKED.", "Injected overload produced a BLOCKED decision.", { "observed load": "720 mA", "limit": "600 mA" }),
        scenario("wrong flash fitted", "PASS", "The flash JEDEC device ID does not match W25Q64JV.", "The flash check fails and boot becomes BLOCKED.", "The wrong virtual device was detected.", { "observed decision": "BLOCKED" }),
        scenario("radio BUSY stuck", "PASS", "The SX1280 BUSY pin never clears after a command.", "The radio check fails and boot becomes BLOCKED.", "The stuck BUSY line produced a BLOCKED decision.", { "observed decision": "BLOCKED" }),
        scenario("motor VBUS missing", "PASS", "The TMC5240 responds digitally but motor supply is absent.", "The motor check fails and boot becomes BLOCKED.", "Missing motor power produced a BLOCKED decision.", { "observed decision": "BLOCKED" })
      ]
    },
    actuator: {
      overall: "PASS",
      scenarios: [
        scenario("nominal actuator", "PASS", "A homed actuator receives deploy, partial retract, then inhibit commands.", "It deploys near 100% within one second, retracts, and never faults.", "Reached near-full deployment within the M5 limit and returned retracted.", { "deploy duration": "0.642 s", "limit": "1.000 s", "peak deployment": "100.0%", "peak current": "420.0 mA" }),
        scenario("slow motor", "PASS", "The motor moves at 25% of placeholder nominal step rate.", "It cannot reach full deployment during the command window but does not fault.", "The slower motor reached only partial travel.", { "peak deployment": "73.8%", "fault raised": "no" }),
        scenario("not homed", "PASS", "The actuator has not found its zero position before deployment.", "It refuses to move and raises a deploy-before-homing fault.", "Unhomed deployment was blocked.", { "peak deployment": "0.0%", "fault reason": "deploy command before homing" }),
        scenario("jam at 45 percent", "PASS", "The virtual mechanism stalls near 45% deployment.", "Travel stops near the jam and the current/fault path triggers.", "The jam stopped motion and raised the expected fault.", { "peak deployment": "45.2%", "peak current": "1200.0 mA", "fault raised": "yes" })
      ]
    }
  }
};

function scenario(name, status, condition, passRule, result, measurements = {}, note = "") {
  return { name, status, condition, passRule, result, measurements, note };
}

const state = {
  data: structuredClone(baselineData),
  activeView: "overview",
  selectedSuite: "flight",
  selectedScenarioIndex: 0,
  running: false,
  runStatus: "ready",
  inspectorOpen: true,
  consoleOpen: false
};

const el = (id) => document.getElementById(id);

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

function normalizeStatus(status) {
  const value = String(status || "").toUpperCase();
  if (value.includes("WARN")) return "WARN";
  if (value.includes("FAIL") || value.includes("BLOCKED")) return "FAIL";
  return "PASS";
}

function statusClass(status) {
  const normalized = normalizeStatus(status);
  return normalized === "PASS" ? "status-pass" : normalized === "WARN" ? "status-warn" : "status-fail";
}

function statusSymbol(status) {
  const normalized = normalizeStatus(status);
  return normalized === "PASS" ? "✓" : normalized === "WARN" ? "!" : "×";
}

function suiteOverallClass(overall) {
  return overall.includes("WARN") ? "status-warn" : overall.includes("BLOCK") || overall.includes("FAIL") ? "status-fail" : "status-pass";
}

function render() {
  renderNavigation();
  renderSummary();
  renderWorkspace();
  renderInspector();
  renderConsole();
  renderRunMeta();
}

function renderNavigation() {
  document.querySelectorAll(".nav-item[data-view]").forEach((button) => {
    button.classList.toggle("selected", button.dataset.view === state.activeView);
  });
}

function renderSummary() {
  const suites = state.data.suites;
  const summaryItems = Object.entries(suites).map(([key, suite]) => {
    const passCount = suite.scenarios.filter((item) => item.status === "PASS").length;
    const overall = suite.overall;
    const klass = suiteOverallClass(overall);
    return `
      <div class="summary-metric">
        <span class="status-ring ${klass}">${statusSymbol(overall)}</span>
        <div>
          <div class="summary-label">${escapeHtml(SUITE_META[key].label.replace(" Suite", ""))}</div>
          <div class="summary-value ${klass}">${escapeHtml(overall)}</div>
          <div class="summary-count">${passCount} / ${suite.scenarios.length} scenarios passed</div>
        </div>
      </div>`;
  }).join("");
  el("summary-band").innerHTML = summaryItems;
  const findings = [
    "RocketPy mass and drag values remain provisional",
    "M5 2.4 GHz requirement matches SX1280 hardware",
    "Recovery GPS remains separate from the airbrake magnetometer",
    "Actuator steps/mm and current are placeholders"
  ];
  el("open-findings").innerHTML = findings.map((finding) => `<span class="finding">${escapeHtml(finding)}</span>`).join("");
  el("data-mode").textContent = state.data.mode === "live" ? "Live local run" : "Baseline snapshot";
  el("summary-subtitle").textContent = state.data.mode === "live"
    ? "Results parsed from the local C++ sandboxes and RocketPy physics backend."
    : "Reviewed baseline results. Run the suite to refresh from the local executables.";
}

function renderWorkspace() {
  const sourcesVisible = state.activeView === "sources";
  el("summary-section").classList.toggle("hidden", sourcesVisible);
  el("suite-content").classList.toggle("hidden", sourcesVisible);
  el("sources-content").classList.toggle("hidden", !sourcesVisible);

  if (sourcesVisible) {
    renderSources();
    return;
  }

  if (state.running) {
    el("suite-content").innerHTML = `<div class="suite-panel"><div class="loading-row"><span class="spinner"></span>Running local simulation executables…</div></div>`;
    return;
  }

  const keys = state.activeView === "overview" ? ["flight", "rocketpy", "electronics", "actuator"] : [state.activeView];
  el("suite-content").innerHTML = keys.map(renderSuite).join("");
  bindScenarioRows();
}

function renderSuite(key) {
  const suite = state.data.suites[key];
  const meta = SUITE_META[key];
  const rows = suite.scenarios.map((item, index) => {
    const isSelected = state.selectedSuite === key && state.selectedScenarioIndex === index;
    return `
      <tr class="${isSelected ? "selected" : ""}" data-suite="${key}" data-index="${index}" tabindex="0">
        <td><span class="scenario-number">${String(index + 1).padStart(2, "0")}</span>${escapeHtml(item.name)}</td>
        <td>${escapeHtml(item.condition)}</td>
        <td>${escapeHtml(item.passRule)}</td>
        <td><span class="scenario-status ${statusClass(item.status)}"><span class="mini-status">${statusSymbol(item.status)}</span>${escapeHtml(item.status)}</span></td>
      </tr>`;
  }).join("");

  return `
    <article class="suite-panel">
      <header class="suite-header">
        <div class="suite-title"><span class="suite-symbol" aria-hidden="true">${meta.symbol}</span><h2>${meta.label}</h2></div>
        <span class="suite-overall ${suiteOverallClass(suite.overall)}">Status: ${escapeHtml(suite.overall)}</span>
      </header>
      <table class="scenario-table">
        <thead><tr><th class="scenario-col">Scenario</th><th class="condition-col">Condition / Input</th><th class="rule-col">Pass Rule</th><th class="result-col">Result</th></tr></thead>
        <tbody>${rows}</tbody>
      </table>
    </article>`;
}

function bindScenarioRows() {
  document.querySelectorAll(".scenario-table tbody tr").forEach((row) => {
    const select = () => {
      state.selectedSuite = row.dataset.suite;
      state.selectedScenarioIndex = Number(row.dataset.index);
      state.inspectorOpen = true;
      el("inspector").classList.remove("hidden");
      renderWorkspace();
      renderInspector();
    };
    row.addEventListener("click", select);
    row.addEventListener("keydown", (event) => {
      if (event.key === "Enter" || event.key === " ") select();
    });
  });
}

function renderInspector() {
  el("inspector").classList.toggle("hidden", !state.inspectorOpen || state.activeView === "sources");
  if (!state.inspectorOpen || state.activeView === "sources") return;

  const suite = state.data.suites[state.selectedSuite];
  const item = suite.scenarios[state.selectedScenarioIndex] || suite.scenarios[0];
  el("inspector-title").textContent = item.name;
  const measurements = Object.entries(item.measurements || {}).map(([key, value]) => `<dt>${escapeHtml(key)}</dt><dd>${escapeHtml(value)}</dd>`).join("");
  const proves = whatThisProves(state.selectedSuite, item);
  el("inspector-body").innerHTML = `
    <section class="inspector-section">
      <div class="status-banner ${statusClass(item.status)}"><span class="status-ring ${statusClass(item.status)}">${statusSymbol(item.status)}</span>${escapeHtml(item.status)}</div>
    </section>
    <section class="inspector-section"><h3>Condition</h3><p>${escapeHtml(item.condition)}</p></section>
    <section class="inspector-section"><h3>Pass Rule</h3><p>${escapeHtml(item.passRule)}</p></section>
    <section class="inspector-section"><h3>Observed Result</h3><p>${escapeHtml(item.result)}</p></section>
    ${measurements ? `<section class="inspector-section"><h3>Measurements</h3><dl class="measurements">${measurements}</dl></section>` : ""}
    <section class="inspector-section"><h3>What This Proves</h3><p>${escapeHtml(proves)}</p></section>
    ${item.note ? `<section class="inspector-section"><h3>Important Note</h3><p class="status-warn">${escapeHtml(item.note)}</p></section>` : ""}`;
}

function whatThisProves(suite, item) {
  if (suite === "flight") return `This verifies the estimator/controller response for the injected virtual condition. It does not prove the aerodynamic model or final apogee accuracy. ${item.result}`;
  if (suite === "rocketpy") return `This verifies RocketPy-to-C++ closed-loop integration using the versioned M5 reference model. Final prediction accuracy still depends on measured mass properties and aerodynamic data. ${item.result}`;
  if (suite === "electronics") return `This verifies the modeled boot-decision logic and constant checks. It does not measure the physical PCB, power integrity, or bus waveforms. ${item.result}`;
  return `This verifies the virtual actuator safety behavior using the current model. Final motor torque, current, travel, and timing still require bench measurements. ${item.result}`;
}

function renderSources() {
  const sections = [
    {
      title: "Source-backed inputs",
      rows: [
        ["June 14 M5 report", "3379 ft passive apogee, 579 ft/s maximum velocity, 75.5 ft/s rail exit, launch conditions, fin geometry, airbrake loads, and 430 mA logic budget", "EXTRACTED", "status-pass"],
        ["M5 project requirements", "3000 ft target, ±100 ft tolerance, 2.4 GHz radio, J420R motor selection, and separate recovery GPS", "IN CODE", "status-pass"],
        ["KiCad hardware map", "STM32H562, BMP388, LSM6DSV32X, LIS2MDL, SX1280, W25Q64, TMC5240", "LOCAL SOURCE", "status-pass"],
        ["RocketPy physics", "RocketPy 1.12.1, certified J420R thrust curve, standard atmosphere, and real C++ controller bridge", "IN CODE", "status-pass"],
        ["Sensor architecture", "Airbrake board uses magnetometer; recovery/tracking GPS is an independent subsystem", "VERIFIED", "status-pass"]
      ]
    },
    {
      title: "Open engineering inputs",
      rows: [
        ["OpenRocket reconstruction", "Current .ork, mass properties, component positions, and drag curves are needed to resolve 3955 ft versus 3379 ft", "FAILING", "status-warn"],
        ["Aerodynamic calibration", "Drag versus Mach/deployment with a declared reference area", "PROVISIONAL", "status-warn"],
        ["Mass properties", "Measured flight-ready mass, center of gravity, and inertia", "PLACEHOLDER", "status-warn"],
        ["Actuator calibration", "Final step/mm, current limit, homing switch, friction, and stall threshold", "PLACEHOLDER", "status-warn"],
        ["Recovery GPS", "Required for vehicle recovery but intentionally outside the airbrake PCB and controller", "SEPARATE SYSTEM", "status-pass"],
        ["Bench measurements", "3V3 current, I2C/SPI timing, signal integrity, and static-port response", "MISSING", "status-warn"]
      ]
    }
  ];
  el("sources-content").innerHTML = sections.map((section) => `
    <section class="source-section">
      <h2>${escapeHtml(section.title)}</h2>
      ${section.rows.map(([name, detail, status, klass]) => `<div class="source-row"><div><div class="source-name">${escapeHtml(name)}</div><div class="source-detail">${escapeHtml(detail)}</div></div><div class="source-state ${klass}">${escapeHtml(status)}</div></div>`).join("")}
    </section>`).join("");
}

function renderConsole() {
  el("console-panel").classList.toggle("collapsed", !state.consoleOpen);
  el("console-toggle").setAttribute("aria-expanded", String(state.consoleOpen));
  el("console-chevron").textContent = state.consoleOpen ? "⌄" : "⌃";
  el("raw-output").textContent = state.data.raw || "Run a simulation suite to view the complete terminal output here.";
  el("console-run-meta").textContent = state.data.timestamp
    ? `Last run ${formatDate(state.data.timestamp)} · ${formatDuration(state.data.durationSeconds)}`
    : "No live run captured";
}

function renderRunMeta() {
  el("last-run").textContent = state.data.timestamp ? formatDate(state.data.timestamp) : "Not run in UI";
  const status = el("build-status");
  const statusText = state.runStatus === "running" ? "Running" : state.runStatus === "failed" ? "Failed" : "Ready";
  const statusClassName = state.runStatus === "running" ? "status-running" : state.runStatus === "failed" ? "status-failed" : "status-ready";
  status.textContent = statusText;
  status.className = `status-inline ${statusClassName}`;
  el("run-all").disabled = state.running;
  el("run-suite").disabled = state.running || state.activeView === "overview" || state.activeView === "sources";
}

function formatDate(value) {
  return new Intl.DateTimeFormat(undefined, { month: "short", day: "numeric", hour: "numeric", minute: "2-digit", second: "2-digit" }).format(new Date(value));
}

function formatDuration(seconds) {
  if (seconds == null) return "duration unavailable";
  return `${Number(seconds).toFixed(1)} s`;
}

async function runSimulation(suite = "all") {
  state.running = true;
  state.runStatus = "running";
  render();
  try {
    const response = await fetch("/api/run", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ suite, rebuild: el("rebuild-toggle").checked })
    });
    const payload = await response.json();
    if (!response.ok || payload.exitCode !== 0) throw new Error(payload.error || `Simulation exited with code ${payload.exitCode}`);
    const parsed = parseSimulationOutput(payload.output, suite);
    if (suite === "all") {
      state.data = { ...parsed, mode: "live", timestamp: payload.timestamp, durationSeconds: payload.durationSeconds, raw: payload.output };
    } else {
      state.data.suites[suite] = parsed.suites[suite];
      state.data.mode = "live";
      state.data.timestamp = payload.timestamp;
      state.data.durationSeconds = payload.durationSeconds;
      state.data.raw = payload.output;
    }
    state.runStatus = "ready";
    showToast(`${suite === "all" ? "All suites" : SUITE_META[suite].label} completed successfully.`);
  } catch (error) {
    state.runStatus = "failed";
    state.consoleOpen = true;
    state.data.raw = `${state.data.raw || ""}\n\nUI ERROR: ${error.message}`.trim();
    showToast(`Run failed: ${error.message}`);
  } finally {
    state.running = false;
    render();
  }
}

async function hydrateLastRun() {
  try {
    const response = await fetch("/api/last-run", { cache: "no-store" });
    const payload = await response.json();
    if (!response.ok || payload.exitCode !== 0 || !payload.output || !payload.suite) return;

    const parsed = parseSimulationOutput(payload.output, payload.suite);
    if (payload.suite === "all") {
      state.data = { ...parsed, mode: "live", timestamp: payload.timestamp, durationSeconds: payload.durationSeconds, raw: payload.output };
    } else if (state.data.suites[payload.suite] && parsed.suites[payload.suite]) {
      state.data.suites[payload.suite] = parsed.suites[payload.suite];
      state.data.mode = "live";
      state.data.timestamp = payload.timestamp;
      state.data.durationSeconds = payload.durationSeconds;
      state.data.raw = payload.output;
    }
    render();
  } catch {
    // The baseline snapshot remains usable if no prior local run can be loaded.
  }
}

function parseSimulationOutput(raw, requestedSuite) {
  // Run All concatenates several executable reports. Split them at the printed
  // suite banners, then feed each section to the same per-case parser used for
  // an individual Run Suite request.
  const data = structuredClone(baselineData);
  data.raw = raw;
  const suiteChunks = requestedSuite === "all"
      ? {
        flight: between(raw, "Running sim_flight_sandbox.exe", "Running sim_electronics_sandbox.exe"),
        electronics: between(raw, "Running sim_electronics_sandbox.exe", "Running sim_actuator_sandbox.exe"),
        actuator: between(raw, "Running sim_actuator_sandbox.exe", "Running sim_rocketpy_physics"),
        rocketpy: raw.split("Running sim_rocketpy_physics")[1] || ""
      }
    : { [requestedSuite]: raw };

  Object.entries(suiteChunks).forEach(([key, chunk]) => {
    const scenarios = parseTestCases(chunk, key);
    if (scenarios.length) {
      data.suites[key].scenarios = scenarios;
      data.suites[key].overall = deriveOverall(key, chunk, scenarios);
      if (key === "electronics") data.suites[key].warnings = extractElectronicsWarnings(chunk);
    }
  });
  return data;
}

function between(text, start, end) {
  const afterStart = text.split(start)[1] || "";
  return end ? afterStart.split(end)[0] : afterStart;
}

function parseTestCases(chunk, suite) {
  const matches = [...chunk.matchAll(/TEST CASE\s+\d+:\s*([^\r\n]+)\r?\n([\s\S]*?)(?=\r?\nTEST CASE\s+\d+:|\r?\nSUMMARY)/g)];
  return matches.map((match) => {
    const block = match[2];
    const resultMatch = block.match(/Result:\s*(PASS|FAIL|WARN)\s*-\s*([^\r\n]+)/);
    const condition = lineValue(block, "Condition being tested") || "Condition not parsed";
    const passRule = lineValue(block, "Pass rule") || "Pass rule not parsed";
    const status = resultMatch?.[1] || "FAIL";
    const result = resultMatch?.[2] || "Result details not parsed";
    const measurements = parseMeasurements(block);
    if (suite === "electronics") {
      measurements["expected decision"] = lineValue(block, "Expected boot decision") || "";
      measurements["observed decision"] = lineValue(block, "Observed boot decision") || "";
      measurements["check counts"] = lineValue(block, "Check counts") || "";
      Object.keys(measurements).forEach((key) => { if (!measurements[key]) delete measurements[key]; });
    }
    const targetNote = block.match(/Target note:\s*([^\r\n]+)/)?.[1] || "";
    return scenario(match[1].trim(), status, condition, passRule, result, measurements, targetNote);
  });
}

function lineValue(block, label) {
  const escaped = label.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
  return block.match(new RegExp(`^${escaped}:\\s*(.+)$`, "mi"))?.[1]?.trim();
}

function parseMeasurements(block) {
  const section = block.match(/Measurements:\s*\r?\n([\s\S]*?)(?=\r?\nTarget note:|\r?\nDetailed check log:|$)/)?.[1] || "";
  const values = {};
  section.split(/\r?\n/).forEach((line) => {
    const match = line.match(/^\s{2}([^:]+):\s*(.+)$/);
    if (match) values[match[1].trim()] = match[2].trim();
  });
  return values;
}

function deriveOverall(key, chunk, scenarios) {
  if (key === "electronics") {
    const decision = chunk.match(/TEST CASE 1:[\s\S]*?Observed boot decision:\s*([^\r\n]+)/)?.[1]?.trim();
    return decision || (scenarios.every((item) => item.status === "PASS") ? "PASS" : "FAIL");
  }
  if (key === "rocketpy") return scenarios.every((item) => item.status === "PASS") ? "PASS_WITH_MODEL_WARNINGS" : "FAIL";
  return scenarios.every((item) => item.status === "PASS") ? "PASS" : "FAIL";
}

function extractElectronicsWarnings(chunk) {
  return [...chunk.matchAll(/\[WARN\]\s*([^\r\n]+)[\s\S]*?Meaning:\s*([^\r\n]+)/g)].map((match) => `${match[1].trim()}: ${match[2].trim()}`);
}

function showToast(message) {
  const toast = el("toast");
  toast.textContent = message;
  toast.classList.add("visible");
  clearTimeout(showToast.timeout);
  showToast.timeout = setTimeout(() => toast.classList.remove("visible"), 3200);
}

function setupEvents() {
  el("run-all").addEventListener("click", () => runSimulation("all"));
  el("run-suite").addEventListener("click", () => runSimulation(state.activeView));
  el("navigation").addEventListener("click", (event) => {
    const button = event.target.closest("[data-view]");
    if (!button) return;
    state.activeView = button.dataset.view;
    if (SUITE_META[state.activeView]) {
      state.selectedSuite = state.activeView;
      state.selectedScenarioIndex = 0;
      state.inspectorOpen = true;
    }
    render();
  });
  el("close-inspector").addEventListener("click", () => { state.inspectorOpen = false; renderInspector(); });
  el("console-toggle").addEventListener("click", () => { state.consoleOpen = !state.consoleOpen; renderConsole(); });
  el("raw-nav").addEventListener("click", () => { state.consoleOpen = true; renderConsole(); });
  el("copy-output").addEventListener("click", async () => {
    await navigator.clipboard.writeText(state.data.raw || "");
    showToast("Raw output copied.");
  });
  el("download-output").addEventListener("click", () => {
    const blob = new Blob([state.data.raw || ""], { type: "text/plain" });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = `ambar-simulation-${new Date().toISOString().replaceAll(":", "-")}.txt`;
    link.click();
    URL.revokeObjectURL(url);
  });
  el("clear-output").addEventListener("click", () => { state.data.raw = ""; renderConsole(); });
}

setupEvents();
render();
hydrateLastRun();
