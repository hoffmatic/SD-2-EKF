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

const METERS_TO_FEET = 3.280839895;
const PHASE_COLORS = {
  PadIdle: "#f2f4f7",
  Boost: "#fff4d6",
  Coast: "#e9f7fb",
  AirbrakeActive: "#e9f0ff",
  Recovery: "#e8f7ef",
  Fault: "#fdecec"
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
        scenario("M5 passive reference", "FAIL", "RocketPy runs the June 2 OpenRocket geometry and June 14 M5 launch conditions with the selected AeroTech J420R and no airbrake deployment.", "RocketPy completes and passive apogee remains within 10% of the current 3379 ft M5 OpenRocket value.", "The provisional model predicts 3851 ft, 14.0% above the current report value.", { "RocketPy passive apogee": "3851 ft", "M5 OpenRocket apogee": "3379 ft", "difference": "+14.0%", "maximum Mach": "0.494", "rail exit velocity": "42.7 ft/s" }, "FAIL is intentional: current mass, CG, inertia, and drag data are unresolved, so the model was not retuned to force agreement."),
        scenario("C++ closed-loop airbrakes", "PASS", "RocketPy feeds delayed, noisy virtual IMU/barometer measurements into the real C++ AMBAR flight computer and applies its rate-limited airbrake command.", "The estimator stays healthy, commands remain bounded, deployment begins after burnout plus margin, commands occur only in AirbrakeActive, and apogee is reduced by more than 50 ft.", "The controller reduced this provisional trajectory from 3851 ft to 2973 ft without commanding before the post-burn enable time.", { "closed-loop apogee": "2973 ft", "target error": "-27 ft", "apogee reduction": "879 ft", "motor burn end": "1.64 s", "minimum deploy time": "1.74 s", "first C++ command": "1.82 s", "C++ command range": "0.0% to 100.0%", "maximum altitude error": "3.64 m", "maximum velocity error": "6.54 m/s", "estimator healthy": "yes" }, "Controller coupling passed, but target accuracy is not validated because the passive vehicle model failed its source comparison."),
        scenario("closed-loop target attainment", "PASS", "The provisional closed-loop apogee is compared directly with 3000 +/-100 ft.", "Closed-loop apogee is between 2900 ft and 3100 ft.", "The provisional result is inside the band, but source-model failures prevent a validation claim.", { "closed-loop apogee": "2973 ft", "target error": "-27 ft" }, "This is a necessary target-band check, not sufficient validation."),
        scenario("M5 flight envelope checks", "FAIL", "The closed-loop RocketPy trajectory is checked against the M5 subsonic and minimum rail-exit requirements.", "Maximum Mach is no greater than 1.0 and rail-exit velocity is at least 52 ft/s.", "The trajectory stayed subsonic but left the 72-inch rail too slowly.", { "maximum Mach": "0.494", "Mach limit": "1.0", "minimum rail exit velocity": "42.7 ft/s", "rail exit minimum": "52.0 ft/s", "M5 reported rail exit velocity": "75.5 ft/s" }),
        scenario("flight-data integrity and recovery observation", "PASS", "The structured log is checked through a short post-apogee controller observation window.", "Timestamps and values remain valid, Recovery is observed, barometer samples are sparse, and deployment retracts below 2%.", "The time history passed its integrity checks and showed PadIdle, Boost, AirbrakeActive, and Recovery.", { "controller samples": "1727", "barometer samples": "864", "final deployment": "0.0%", "integrity errors": "0" }, "Parachutes, recovery electronics, landing, and deployment loads are not modeled.")
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
  consoleOpen: false,
  inputFields: [],
  inputBaseline: {},
  inputValues: {},
  fixedCriteria: {},
  inputMode: "report-baseline",
  flightData: null
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
  el("data-mode").textContent = state.inputMode === "experimental-overrides"
    ? "Experimental inputs"
    : state.data.mode === "live" ? "Live local run" : "Baseline snapshot";
  el("summary-subtitle").textContent = state.data.mode === "live"
    ? "Results parsed from the local C++ sandboxes and RocketPy physics backend."
    : "Reviewed baseline results. Run the suite to refresh from the local executables.";
}

function renderWorkspace() {
  const sourcesVisible = state.activeView === "sources";
  const inputsVisible = state.activeView === "inputs";
  const flightDataVisible = state.activeView === "flightdata";
  const specialViewVisible = sourcesVisible || inputsVisible || flightDataVisible;
  el("app").classList.toggle("workspace-wide", specialViewVisible);
  el("summary-section").classList.toggle("hidden", specialViewVisible);
  el("suite-content").classList.toggle("hidden", specialViewVisible);
  el("flight-data-content").classList.toggle("hidden", !flightDataVisible);
  el("inputs-content").classList.toggle("hidden", !inputsVisible);
  el("sources-content").classList.toggle("hidden", !sourcesVisible);

  if (sourcesVisible) {
    renderSources();
    return;
  }

  if (inputsVisible) {
    renderInputs();
    return;
  }

  if (flightDataVisible) {
    renderFlightData();
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
  const specialViewVisible = state.activeView === "sources" || state.activeView === "inputs" || state.activeView === "flightdata";
  el("inspector").classList.toggle("hidden", !state.inspectorOpen || specialViewVisible);
  if (!state.inspectorOpen || specialViewVisible) return;

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

function deriveClientPhaseTransitions(log) {
  const transitions = [];
  log.forEach((sample) => {
    if (!transitions.length || transitions[transitions.length - 1].phase !== sample.phase) {
      transitions.push({
        time_s: sample.time_s,
        phase: sample.phase,
        altitude_m: sample.truth_altitude_m,
        vertical_velocity_mps: sample.truth_velocity_mps,
        deployment_fraction: sample.actual_deployment_fraction
      });
    }
  });
  return transitions;
}

function finiteMaximum(values, fallback = 0) {
  const finite = values.filter(Number.isFinite);
  return finite.length ? Math.max(...finite) : fallback;
}

function flightChartDefinitions(data) {
  const log = data.controllerLog || [];
  const targetFt = Number(data.appliedConfig?.requirements?.target_apogee_ft ?? 3000);
  const times = log.map((sample) => Number(sample.time_s));
  const series = (label, color, values, options = {}) => ({ label, color, values, ...options });
  return [
    {
      id: "altitude",
      title: "Altitude and predicted apogee",
      subtitle: "RocketPy truth, sparse barometer samples, C++ EKF estimate, and controller prediction",
      unit: "ft AGL",
      includeZero: true,
      times,
      series: [
        series("RocketPy truth", "#175cd3", log.map((sample) => sample.truth_altitude_m * METERS_TO_FEET)),
        series("Barometer sample", "#b54708", log.map((sample) => sample.measured_altitude_m == null ? null : sample.measured_altitude_m * METERS_TO_FEET), { pointsOnly: true }),
        series("C++ EKF estimate", "#067647", log.map((sample) => sample.estimated_altitude_m * METERS_TO_FEET)),
        series("Predicted apogee", "#6941c6", log.map((sample) => sample.predicted_apogee_m * METERS_TO_FEET), { dash: [5, 4] }),
        series("Mission target", "#b42318", log.map(() => targetFt), { dash: [2, 4] })
      ]
    },
    {
      id: "speed",
      title: "Speed and vertical velocity",
      subtitle: "Total 3D speed is RocketPy truth; vertical channels show estimator tracking and descent sign",
      unit: "ft/s",
      includeZero: true,
      times,
      series: [
        series("Total speed", "#175cd3", log.map((sample) => Number(sample.truth_speed_mps ?? Math.abs(sample.truth_velocity_mps)) * METERS_TO_FEET)),
        series("True vertical velocity", "#b54708", log.map((sample) => sample.truth_velocity_mps * METERS_TO_FEET), { dash: [5, 4] }),
        series("Estimated vertical velocity", "#067647", log.map((sample) => sample.estimated_velocity_mps * METERS_TO_FEET))
      ]
    },
    {
      id: "acceleration",
      title: "Vertical acceleration",
      subtitle: "Launch-frame net acceleration after assumed alignment and gravity compensation, not raw body-axis IMU data",
      unit: "ft/s^2",
      includeZero: true,
      times,
      series: [
        series("RocketPy vertical truth", "#175cd3", log.map((sample) => sample.truth_acceleration_mps2 * METERS_TO_FEET)),
        series("Virtual sensor supplied to C++", "#b54708", log.map((sample) => sample.measured_acceleration_mps2 * METERS_TO_FEET))
      ]
    },
    {
      id: "deployment",
      title: "Airbrake command and deployment",
      subtitle: "Controller request compared with the rate-limited deployment actually applied to RocketPy",
      unit: "%",
      includeZero: true,
      fixedMaximum: 100,
      times,
      series: [
        series("C++ command", "#b54708", log.map((sample) => sample.command_fraction * 100), { dash: [5, 4] }),
        series("Applied deployment", "#175cd3", log.map((sample) => sample.actual_deployment_fraction * 100))
      ]
    }
  ];
}

function renderFlightData() {
  const container = el("flight-data-content");
  const data = state.flightData;
  const log = data?.controllerLog || [];
  if (!data || !log.length) {
    container.innerHTML = `
      <section class="flight-empty">
        <h1>No flight time history is loaded</h1>
        <p>Run RocketPy to generate altitude, speed, acceleration, phase, command, and deployment data from the current inputs.</p>
        <button id="run-flight-data" class="button button-primary" type="button">Run RocketPy</button>
      </section>`;
    el("run-flight-data").addEventListener("click", () => runSimulation("rocketpy"));
    return;
  }

  const closed = data.closedLoop || {};
  const transitions = data.phaseTransitions?.length ? data.phaseTransitions : deriveClientPhaseTransitions(log);
  const maximumSpeedMps = Number(closed.maximumSpeedMps ?? finiteMaximum(log.map((sample) => Number(sample.truth_speed_mps)), 0));
  const maximumAccelerationMps2 = Number(closed.maximumVerticalAccelerationMps2 ?? finiteMaximum(log.map((sample) => Math.abs(Number(sample.truth_acceleration_mps2))), 0));
  const chartDefinitions = flightChartDefinitions(data);
  const modelStatus = data.modelStatus || "Model maturity was not recorded.";
  const limitations = data.limitations || [];
  const integrityPass = Boolean(data.acceptance?.timeHistoryPass);
  const inputMode = data.inputMode === "experimental-overrides" ? "Experimental input run" : "Report baseline run";

  container.innerHTML = `
    <header class="flight-data-heading">
      <div>
        <div class="flight-data-kicker">RocketPy + C++ closed loop</div>
        <h1>Flight Time History</h1>
        <p>Follow the simulated vehicle from pad through boost, controlled coast, apogee, Recovery entry, and airbrake retraction.</p>
      </div>
      <div class="flight-data-actions">
        <button id="run-flight-data" class="button button-primary" type="button">Run RocketPy</button>
        <button id="download-flight-csv" class="button button-secondary" type="button">Download CSV</button>
        <button id="download-flight-json" class="button button-secondary" type="button">Download JSON</button>
      </div>
    </header>
    <section class="model-disclosure">
      <div><strong>${escapeHtml(inputMode)}</strong><span>${escapeHtml(modelStatus)}</span></div>
      <span class="integrity-badge ${integrityPass ? "status-pass" : "status-fail"}">${integrityPass ? "DATA CHECK PASS" : "DATA CHECK FAIL"}</span>
    </section>
    <section class="flight-metrics" aria-label="Flight summary">
      <div><span>Closed-loop apogee</span><strong>${Number(closed.apogeeFt ?? 0).toFixed(0)} ft</strong><small>${Number(closed.targetErrorFt ?? 0) >= 0 ? "+" : ""}${Number(closed.targetErrorFt ?? 0).toFixed(0)} ft from target</small></div>
      <div><span>Maximum 3D speed</span><strong>${(maximumSpeedMps * METERS_TO_FEET).toFixed(1)} ft/s</strong><small>RocketPy truth</small></div>
      <div><span>Peak vertical acceleration</span><strong>${(maximumAccelerationMps2 * METERS_TO_FEET).toFixed(1)} ft/s²</strong><small>Absolute net vertical value</small></div>
      <div><span>Peak deployment</span><strong>${(Number(closed.peakDeploymentFraction ?? 0) * 100).toFixed(1)}%</strong><small>${Number(closed.finalDeploymentFraction ?? 0) <= 0.02 ? "Retracted after apogee" : "Still deployed at window end"}</small></div>
      <div><span>Apogee time</span><strong>${Number(closed.apogeeTimeS ?? 0).toFixed(2)} s</strong><small>Observation ends at ${Number(closed.observationEndTimeS ?? log.at(-1)?.time_s ?? 0).toFixed(2)} s</small></div>
    </section>
    <section class="data-key">
      <div><strong>Truth</strong><span>RocketPy trajectory and aerodynamic response.</span></div>
      <div><strong>Sensor</strong><span>Delayed, biased, noisy, and quantized virtual measurements.</span></div>
      <div><strong>Estimate</strong><span>Output from the repository's C++ vertical EKF.</span></div>
    </section>
    <section class="flight-chart-grid">
      ${chartDefinitions.map((definition) => `
        <article class="flight-chart-panel">
          <header><div><h2>${escapeHtml(definition.title)}</h2><p>${escapeHtml(definition.subtitle)}</p></div><span>${escapeHtml(definition.unit)}</span></header>
          <div class="chart-legend">${definition.series.map((item) => `<span><i style="--series-color:${item.color}"></i>${escapeHtml(item.label)}</span>`).join("")}</div>
          <div class="chart-stage">
            <canvas class="flight-chart-canvas" data-chart-id="${definition.id}" aria-label="${escapeHtml(definition.title)} line graph"></canvas>
            <div class="chart-tooltip" role="status"></div>
          </div>
        </article>`).join("")}
    </section>
    <section class="phase-panel">
      <header><h2>Flight phase timeline</h2><p>Phase names come from the real C++ flight-phase tracker, not from UI inference.</p></header>
      <div class="phase-strip">${transitions.map((transition, index) => {
        const endTime = transitions[index + 1]?.time_s ?? log.at(-1).time_s;
        const duration = Math.max(0.01, endTime - transition.time_s);
        return `<span style="--phase-color:${PHASE_COLORS[transition.phase] || "#f2f4f7"};--phase-grow:${duration}" title="${escapeHtml(transition.phase)} from ${Number(transition.time_s).toFixed(2)} s"><b>${escapeHtml(transition.phase)}</b><small>${Number(transition.time_s).toFixed(2)} s</small></span>`;
      }).join("")}</div>
      <div class="phase-table-wrap"><table class="phase-table"><thead><tr><th>Phase entered</th><th>Time</th><th>Altitude</th><th>Vertical velocity</th><th>Deployment</th></tr></thead><tbody>
        ${transitions.map((transition) => `<tr><td>${escapeHtml(transition.phase)}</td><td>${Number(transition.time_s).toFixed(2)} s</td><td>${(Number(transition.altitude_m) * METERS_TO_FEET).toFixed(1)} ft</td><td>${(Number(transition.vertical_velocity_mps) * METERS_TO_FEET).toFixed(1)} ft/s</td><td>${(Number(transition.deployment_fraction) * 100).toFixed(1)}%</td></tr>`).join("")}
      </tbody></table></div>
    </section>
    <section class="limitations-panel"><h2>What this run does not prove</h2>${limitations.map((limitation) => `<p>${escapeHtml(limitation)}</p>`).join("")}</section>`;

  el("run-flight-data").addEventListener("click", () => runSimulation("rocketpy"));
  el("download-flight-csv").addEventListener("click", () => downloadFlightData("csv"));
  el("download-flight-json").addEventListener("click", () => downloadFlightData("json"));
  requestAnimationFrame(renderFlightCharts);
}

function drawFlightChart(canvas, definition, transitions, hoverIndex = null) {
  const width = Math.max(320, Math.floor(canvas.clientWidth));
  const height = Math.max(230, Math.floor(canvas.clientHeight));
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.floor(width * ratio);
  canvas.height = Math.floor(height * ratio);
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);

  const margin = { left: 58, right: 16, top: 12, bottom: 34 };
  const plotWidth = width - margin.left - margin.right;
  const plotHeight = height - margin.top - margin.bottom;
  const xMin = definition.times[0] ?? 0;
  const xMax = definition.times.at(-1) ?? 1;
  const finiteValues = definition.series.flatMap((item) => item.values.filter(Number.isFinite));
  let yMin = finiteValues.length ? Math.min(...finiteValues) : 0;
  let yMax = definition.fixedMaximum ?? (finiteValues.length ? Math.max(...finiteValues) : 1);
  if (definition.includeZero) {
    yMin = Math.min(0, yMin);
    yMax = Math.max(0, yMax);
  }
  if (Math.abs(yMax - yMin) < 1e-9) yMax = yMin + 1;
  const padding = (yMax - yMin) * 0.08;
  if (definition.fixedMaximum == null) yMax += padding;
  yMin -= padding;

  const mapX = (value) => margin.left + ((value - xMin) / Math.max(1e-9, xMax - xMin)) * plotWidth;
  const mapY = (value) => margin.top + (1 - (value - yMin) / (yMax - yMin)) * plotHeight;

  transitions.forEach((transition, index) => {
    const end = transitions[index + 1]?.time_s ?? xMax;
    context.fillStyle = PHASE_COLORS[transition.phase] || "#f8f9fb";
    context.fillRect(mapX(transition.time_s), margin.top, Math.max(1, mapX(end) - mapX(transition.time_s)), plotHeight);
  });

  context.font = "10px Segoe UI, sans-serif";
  context.fillStyle = "#667085";
  context.strokeStyle = "#dfe4ec";
  context.lineWidth = 1;
  for (let tick = 0; tick <= 4; tick += 1) {
    const fraction = tick / 4;
    const y = margin.top + fraction * plotHeight;
    const value = yMax - fraction * (yMax - yMin);
    context.beginPath();
    context.moveTo(margin.left, y);
    context.lineTo(width - margin.right, y);
    context.stroke();
    context.textAlign = "right";
    context.textBaseline = "middle";
    context.fillText(value.toFixed(Math.abs(value) >= 100 ? 0 : 1), margin.left - 7, y);
  }
  for (let tick = 0; tick <= 5; tick += 1) {
    const fraction = tick / 5;
    const x = margin.left + fraction * plotWidth;
    const value = xMin + fraction * (xMax - xMin);
    context.beginPath();
    context.moveTo(x, margin.top);
    context.lineTo(x, margin.top + plotHeight);
    context.stroke();
    context.textAlign = "center";
    context.textBaseline = "top";
    context.fillText(`${value.toFixed(1)} s`, x, margin.top + plotHeight + 8);
  }

  definition.series.forEach((item) => {
    context.strokeStyle = item.color;
    context.fillStyle = item.color;
    context.lineWidth = item.pointsOnly ? 1 : 1.8;
    context.setLineDash(item.dash || []);
    if (item.pointsOnly) {
      item.values.forEach((value, index) => {
        if (!Number.isFinite(value)) return;
        context.beginPath();
        context.arc(mapX(definition.times[index]), mapY(value), 1.4, 0, Math.PI * 2);
        context.fill();
      });
      return;
    }
    context.beginPath();
    let drawing = false;
    item.values.forEach((value, index) => {
      if (!Number.isFinite(value)) {
        drawing = false;
        return;
      }
      const x = mapX(definition.times[index]);
      const y = mapY(value);
      if (!drawing) context.moveTo(x, y);
      else context.lineTo(x, y);
      drawing = true;
    });
    context.stroke();
  });
  context.setLineDash([]);

  if (hoverIndex != null && definition.times[hoverIndex] != null) {
    const x = mapX(definition.times[hoverIndex]);
    context.strokeStyle = "#344054";
    context.lineWidth = 1;
    context.beginPath();
    context.moveTo(x, margin.top);
    context.lineTo(x, margin.top + plotHeight);
    context.stroke();
    definition.series.forEach((item) => {
      const value = item.values[hoverIndex];
      if (!Number.isFinite(value)) return;
      context.fillStyle = item.color;
      context.beginPath();
      context.arc(x, mapY(value), 3, 0, Math.PI * 2);
      context.fill();
    });
  }
  canvas._flightPlot = { definition, transitions, margin, plotWidth, xMin, xMax };
}

function nearestTimeIndex(times, target) {
  let low = 0;
  let high = times.length - 1;
  while (low < high) {
    const middle = Math.floor((low + high) / 2);
    if (times[middle] < target) low = middle + 1;
    else high = middle;
  }
  if (low > 0 && Math.abs(times[low - 1] - target) < Math.abs(times[low] - target)) return low - 1;
  return low;
}

function renderFlightCharts() {
  if (state.activeView !== "flightdata" || !state.flightData?.controllerLog?.length) return;
  const transitions = state.flightData.phaseTransitions?.length
    ? state.flightData.phaseTransitions
    : deriveClientPhaseTransitions(state.flightData.controllerLog);
  const definitions = flightChartDefinitions(state.flightData);
  definitions.forEach((definition) => {
    const canvas = document.querySelector(`[data-chart-id="${definition.id}"]`);
    if (!canvas) return;
    const tooltip = canvas.parentElement.querySelector(".chart-tooltip");
    drawFlightChart(canvas, definition, transitions);
    canvas.addEventListener("mousemove", (event) => {
      const bounds = canvas.getBoundingClientRect();
      const plot = canvas._flightPlot;
      const relative = Math.max(0, Math.min(1, (event.clientX - bounds.left - plot.margin.left) / plot.plotWidth));
      const targetTime = plot.xMin + relative * (plot.xMax - plot.xMin);
      const index = nearestTimeIndex(definition.times, targetTime);
      drawFlightChart(canvas, definition, transitions, index);
      const phase = state.flightData.controllerLog[index]?.phase || "Unknown";
      const values = definition.series
        .filter((item) => Number.isFinite(item.values[index]))
        .map((item) => `<span><i style="--series-color:${item.color}"></i>${escapeHtml(item.label)}: <b>${Number(item.values[index]).toFixed(1)} ${escapeHtml(definition.unit)}</b></span>`)
        .join("");
      tooltip.innerHTML = `<strong>${definition.times[index].toFixed(2)} s · ${escapeHtml(phase)}</strong>${values}`;
      tooltip.style.left = `${Math.max(8, Math.min(bounds.width - 220, event.clientX - bounds.left + 12))}px`;
      tooltip.style.top = `${Math.max(8, event.clientY - bounds.top - 12)}px`;
      tooltip.classList.add("visible");
    });
    canvas.addEventListener("mouseleave", () => {
      tooltip.classList.remove("visible");
      drawFlightChart(canvas, definition, transitions);
    });
  });
}

function downloadFlightData(format) {
  const data = state.flightData;
  if (!data?.controllerLog?.length) return;
  let body;
  let type;
  let extension;
  if (format === "csv") {
    const fields = Object.keys(data.controllerLog[0]);
    const rows = data.controllerLog.map((sample) => fields.map((field) => {
      const value = sample[field];
      if (value == null) return "";
      const textValue = String(value);
      return /[",\n]/.test(textValue) ? `"${textValue.replaceAll('"', '""')}"` : textValue;
    }).join(","));
    body = [fields.join(","), ...rows].join("\n");
    type = "text/csv";
    extension = "csv";
  } else {
    body = JSON.stringify(data, null, 2);
    type = "application/json";
    extension = "json";
  }
  const url = URL.createObjectURL(new Blob([body], { type }));
  const link = document.createElement("a");
  link.href = url;
  link.download = `ambar-flight-data.${extension}`;
  link.click();
  URL.revokeObjectURL(url);
}

function inputIsModified(fieldId) {
  const current = Number(state.inputValues[fieldId]);
  const baseline = Number(state.inputBaseline[fieldId]);
  return Number.isFinite(current) && Number.isFinite(baseline) && Math.abs(current - baseline) > 1e-9;
}

function renderInputs() {
  if (!state.inputFields.length) {
    el("inputs-content").innerHTML = `<div class="suite-panel"><div class="loading-row"><span class="spinner"></span>Loading simulation inputs...</div></div>`;
    return;
  }

  const groups = [...new Set(state.inputFields.map((field) => field.group))];
  const modifiedCount = state.inputFields.filter((field) => inputIsModified(field.id)).length;
  const criteria = state.fixedCriteria;
  const groupMarkup = groups.map((group) => {
    const fields = state.inputFields.filter((field) => field.group === group);
    return `
      <section class="input-group">
        <header class="input-group-header"><h2>${escapeHtml(group)}</h2><span>${fields.length} controls</span></header>
        <div class="input-list">
          ${fields.map((field) => {
            const modified = inputIsModified(field.id);
            const baseline = state.inputBaseline[field.id];
            return `
              <label class="input-row ${modified ? "modified" : ""}" for="input-${escapeHtml(field.id)}">
                <span class="input-description">
                  <span class="input-label">${escapeHtml(field.label)}</span>
                  <span class="input-source">${escapeHtml(field.source)} baseline: ${escapeHtml(baseline)} ${escapeHtml(field.unit)}</span>
                </span>
                <span class="number-control">
                  <input id="input-${escapeHtml(field.id)}" data-input-id="${escapeHtml(field.id)}" type="number"
                    value="${escapeHtml(state.inputValues[field.id])}" min="${field.minimum}" max="${field.maximum}" step="${field.step}">
                  <span>${escapeHtml(field.unit)}</span>
                </span>
                <span class="input-state">${modified ? "Modified" : "Baseline"}</span>
              </label>`;
          }).join("")}
        </div>
      </section>`;
  }).join("");

  el("inputs-content").innerHTML = `
    <div class="inputs-heading">
      <div>
        <h1>RocketPy Simulation Inputs</h1>
        <p>Change one or more assumptions, then run RocketPy. These values affect only the temporary simulation run and do not rewrite the reviewed project baseline.</p>
      </div>
      <div class="inputs-actions">
        <button id="reset-inputs" class="button button-secondary" type="button" ${modifiedCount ? "" : "disabled"}>Reset Baseline</button>
        <button id="run-inputs" class="button button-primary" type="button" ${state.running ? "disabled" : ""}>Run RocketPy</button>
      </div>
    </div>
    <div class="input-warning">
      <strong>${modifiedCount ? `${modifiedCount} experimental input${modifiedCount === 1 ? "" : "s"} changed.` : "Report baseline selected."}</strong>
      ${modifiedCount ? " Results will be labeled experimental and should be compared against the baseline, not substituted for measured data." : " Placeholder values remain provisional until replaced by measured vehicle data."}
    </div>
    <section class="criteria-strip" aria-label="Fixed pass and fail criteria">
      <div><span>Maximum Mach</span><strong>${escapeHtml(criteria.maximumMach)}</strong><small>Fixed envelope limit</small></div>
      <div><span>Minimum rail exit</span><strong>${escapeHtml(criteria.minimumRailExitVelocityFps)} ft/s</strong><small>Fixed safety criterion</small></div>
      <div><span>M5 passive reference</span><strong>${escapeHtml(criteria.reportPassiveApogeeFt)} ft</strong><small>Comparison source</small></div>
      <div><span>Target tolerance</span><strong>+/-${escapeHtml(criteria.targetToleranceFt)} ft</strong><small>Fixed mission criterion</small></div>
    </section>
    <div class="input-groups">${groupMarkup}</div>`;
  bindInputControls();
}

function bindInputControls() {
  document.querySelectorAll("[data-input-id]").forEach((control) => {
    control.addEventListener("input", () => {
      state.inputValues[control.dataset.inputId] = Number(control.value);
      updateInputIndicators();
    });
    control.addEventListener("change", () => {
      state.inputValues[control.dataset.inputId] = Number(control.value);
      renderInputs();
      renderRunMeta();
    });
  });
  el("reset-inputs").addEventListener("click", () => {
    state.inputValues = structuredClone(state.inputBaseline);
    renderInputs();
    showToast("Simulation inputs reset to the reviewed baseline.");
  });
  el("run-inputs").addEventListener("click", () => runSimulation("rocketpy", true));
}

function updateInputIndicators() {
  const modifiedFields = state.inputFields.filter((field) => inputIsModified(field.id));
  state.inputFields.forEach((field) => {
    const control = document.querySelector(`[data-input-id="${field.id}"]`);
    const row = control?.closest(".input-row");
    const modified = inputIsModified(field.id);
    row?.classList.toggle("modified", modified);
    const indicator = row?.querySelector(".input-state");
    if (indicator) indicator.textContent = modified ? "Modified" : "Baseline";
  });
  const reset = el("reset-inputs");
  if (reset) reset.disabled = modifiedFields.length === 0;
  const warning = document.querySelector(".input-warning");
  if (warning) {
    const count = modifiedFields.length;
    warning.innerHTML = count
      ? `<strong>${count} experimental input${count === 1 ? "" : "s"} changed.</strong> Results will be labeled experimental and should be compared against the baseline, not substituted for measured data.`
      : "<strong>Report baseline selected.</strong> Placeholder values remain provisional until replaced by measured vehicle data.";
  }
}

function renderSources() {
  const sections = [
    {
      title: "Source-backed inputs",
      rows: [
        ["June 14 M5 report", "3379 ft passive apogee, 579 ft/s maximum velocity, 75.5 ft/s rail exit, launch conditions, fin geometry, airbrake loads, and 430 mA logic budget", "EXTRACTED", "status-pass"],
        ["M5 project requirements", "3000 ft target, ±100 ft tolerance, 2.4 GHz radio, J420R motor selection, and separate recovery GPS", "IN CODE", "status-pass"],
        ["KiCad hardware map", "STM32H562, BMP388, LSM6DSV32X, LIS2MDL, SX1280, W25Q64, TMC5240", "LOCAL SOURCE", "status-pass"],
        ["RocketPy physics", "RocketPy 1.12.1, certified J420R thrust curve, standard pressure/temperature, constant M5 wind, provisional sensor errors, and real C++ controller bridge", "IN CODE", "status-pass"],
        ["Sensor architecture", "Airbrake board uses magnetometer; recovery/tracking GPS is an independent subsystem", "VERIFIED", "status-pass"]
      ]
    },
    {
      title: "Open engineering inputs",
      rows: [
        ["OpenRocket reconstruction", "The current .ork geometry is applied, but the selected configuration must be rerun and measured mass properties/drag are needed to resolve 3851 ft versus 3379 ft", "FAILING", "status-warn"],
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
  const statusText = state.runStatus === "running" ? "Running" : state.runStatus === "failed" ? "Checks failed" : "Ready";
  const statusClassName = state.runStatus === "running" ? "status-running" : state.runStatus === "failed" ? "status-failed" : "status-ready";
  status.textContent = statusText;
  status.className = `status-inline ${statusClassName}`;
  el("run-all").disabled = state.running;
  el("run-suite").disabled = state.running || state.activeView === "overview" || state.activeView === "sources" || state.activeView === "inputs" || state.activeView === "flightdata";
}

function formatDate(value) {
  return new Intl.DateTimeFormat(undefined, { month: "short", day: "numeric", hour: "numeric", minute: "2-digit", second: "2-digit" }).format(new Date(value));
}

function formatDuration(seconds) {
  if (seconds == null) return "duration unavailable";
  return `${Number(seconds).toFixed(1)} s`;
}

function captureInputValuesFromDom() {
  document.querySelectorAll("[data-input-id]").forEach((control) => {
    state.inputValues[control.dataset.inputId] = Number(control.value);
  });
}

async function runSimulation(suite = "all", startedFromInputs = false) {
  captureInputValuesFromDom();
  state.running = true;
  state.runStatus = "running";
  render();
  try {
    const requestBody = { suite, rebuild: el("rebuild-toggle").checked };
    if (suite === "all" || suite === "rocketpy") requestBody.inputs = state.inputValues;
    const response = await fetch("/api/run", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(requestBody)
    });
    const payload = await response.json();
    if (!response.ok || !payload.output) throw new Error(payload.error || `Simulation request failed with code ${payload.exitCode}`);
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
    state.inputMode = payload.inputMode || "report-baseline";
    if (payload.appliedInputs) state.inputValues = structuredClone(payload.appliedInputs);
    if (payload.flightData) state.flightData = payload.flightData;
    state.runStatus = payload.exitCode === 0 ? "ready" : "failed";
    if (startedFromInputs) {
      state.activeView = "flightdata";
      state.selectedSuite = "rocketpy";
      state.selectedScenarioIndex = 0;
      state.inspectorOpen = false;
    }
    showToast(payload.exitCode === 0
      ? `${suite === "all" ? "All suites" : SUITE_META[suite].label} completed successfully.`
      : "Simulation completed and reported failed engineering checks. Review the red scenarios.");
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
    if (!response.ok || !payload.output || !payload.suite) return;

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
    state.inputMode = payload.inputMode || "report-baseline";
    if (payload.appliedInputs) state.inputValues = structuredClone(payload.appliedInputs);
    if (payload.flightData) state.flightData = payload.flightData;
    state.runStatus = payload.exitCode === 0 ? "ready" : "failed";
    render();
  } catch {
    // The baseline snapshot remains usable if no prior local run can be loaded.
  }
}

async function hydrateInputs() {
  try {
    const response = await fetch("/api/inputs", { cache: "no-store" });
    const payload = await response.json();
    if (!response.ok || !payload.baseline || !payload.fields) throw new Error("Input schema unavailable.");
    state.inputBaseline = structuredClone(payload.baseline);
    state.inputValues = structuredClone(payload.baseline);
    state.inputFields = payload.fields;
    state.fixedCriteria = payload.fixedCriteria || {};
    render();
  } catch (error) {
    showToast(`Inputs unavailable: ${error.message}`);
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
    } else if (state.activeView === "flightdata") {
      state.inspectorOpen = false;
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
  let resizeFrame = null;
  window.addEventListener("resize", () => {
    if (state.activeView !== "flightdata") return;
    cancelAnimationFrame(resizeFrame);
    resizeFrame = requestAnimationFrame(renderFlightCharts);
  });
}

setupEvents();
render();
hydrateInputs().then(hydrateLastRun);
