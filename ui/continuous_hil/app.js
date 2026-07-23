const COLORS = {
  blue: "#1769aa",
  orange: "#c75d17",
  gold: "#aa7a08",
  red: "#b42318",
  green: "#16734a",
  grid: "#d9e0e7",
  muted: "#667085",
  ink: "#17212b",
  phase: ["#dceaf5", "#fbe4d5", "#fff0b8", "#e2efe7", "#eadff4", "#eceff2"],
};

const state = {
  payload: window.AMBAR_SNAPSHOT_STATE || null,
  source: null,
  refreshPending: false,
  refreshInFlight: false,
  lastRefreshMs: 0,
  eventIndices: new Map(),
  snapshot: Boolean(window.AMBAR_SNAPSHOT_STATE),
};

const byId = (id) => document.getElementById(id);
const safe = (value, fallback = "—") => value === null || value === undefined || value === "" ? fallback : String(value);
const finite = (value) => Number.isFinite(Number(value)) ? Number(value) : null;
const signedFixed = (value, digits = 1, suffix = "") => {
  const numeric = finite(value);
  if (numeric === null) return "N/A";
  return `${numeric >= 0 ? "+" : ""}${numeric.toFixed(digits)}${suffix}`;
};
const fixed = (value, digits = 1, suffix = "") => finite(value) === null ? "—" : `${Number(value).toFixed(digits)}${suffix}`;

function formatCrc(value) {
  const numeric = finite(value);
  if (numeric === null) return "—";
  return `0x${(Number(numeric) >>> 0).toString(16).toUpperCase().padStart(8, "0")}`;
}

function isVariableHil(payload) {
  const session = payload?.session || {};
  const run = payload?.current_run || {};
  const identity = [session.coupling_mode, run.coupling_mode, run.mode]
    .filter((value) => value !== null && value !== undefined)
    .join(" ")
    .toUpperCase();
  if (identity.includes("VARIABLE_HIL") || identity.includes("TMC_RAMP_STATE_COUPLED")) return true;
  return (run.samples || []).some((sample) => finite(sample.physics_applied_percent) !== null);
}

function escapeHtml(value) {
  return safe(value, "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function statusClass(status) {
  const normalized = safe(status, "").toUpperCase();
  if (["PASS", "PASSED", "SUCCESS"].includes(normalized)) return "pass";
  if (["FAIL", "FAILED", "ERROR", "FAULT", "ABORTED", "ABORTED_HOST_CRASH"].includes(normalized)) return "fail";
  return "";
}

function setConnection(label, connectionState) {
  const node = byId("connection-status");
  node.dataset.state = connectionState;
  node.querySelector("strong").textContent = label;
}

function renderSummary(payload) {
  const session = payload.session || {};
  const run = payload.current_run || null;
  const latestSample = run?.samples?.length ? run.samples[run.samples.length - 1] : null;
  const variableHil = isVariableHil(payload);
  byId("session-state").textContent = safe(session.status, "Waiting");
  byId("session-id").textContent = safe(session.session_id, "No session database yet");
  byId("completed-count").textContent = safe(session.completed_runs, "0");
  byId("pass-fail-count").textContent = `${safe(session.passed_runs, "0")} pass · ${safe(session.failed_runs, "0")} fail`;
  byId("current-run").textContent = run ? safe(run.cycle_index, safe(run.hardware_run_id)) : "—";
  byId("current-mode").textContent = run
    ? `${variableHil ? "VARIABLE_HIL" : safe(run.mode, "trajectory")} · ${safe(run.status, "unknown")}`
    : "Waiting for preflight";
  const latestFault = safe(session.latest_fault, "None");
  byId("latest-fault").textContent = latestFault;
  byId("latest-fault").closest(".fault-card").dataset.fault = latestFault === "None" ? "false" : "true";
  byId("dwell-remaining").textContent = fixed(session.dwell_remaining_s, 1, " s");
  byId("dwell-config").textContent = `Configured ${fixed(session.dwell_seconds, 1, " s")}`;
  byId("sample-count").textContent = `${run?.sample_count || 0} samples`;
  byId("freshness").textContent = payload.database?.available ? `SQLite updated ${formatTime(payload.database.modified_utc)}` : "Waiting for SQLite/UDP";
  byId("packet-lag").textContent = fixed(latestSample?.packet_lag_ms, 1, " ms");
  byId("skipped-samples").textContent = safe(latestSample?.skipped_host_samples, "0");
  byId("generated-at").textContent = `Dashboard state ${formatTime(payload.generated_utc)}`;
  byId("pass-fail-count").textContent = `${variableHil ? "Hardware safety" : "Hardware HIL stroke"}: ${safe(session.passed_runs, "0")} pass / ${safe(session.failed_runs, "0")} fail`;

  byId("feedback-age").textContent = fixed(latestSample?.feedback_age_ms, 1, " ms");
  byId("feedback-source").textContent = latestSample?.feedback_source
    ? `${safe(latestSample.feedback_source)}; XACTUAL is not an encoder`
    : "No correlated feedback yet";
  const reachable = latestSample?.target_reachable;
  byId("target-reachability").textContent = reachable === true
    ? "Reachable"
    : reachable === false
      ? "Outside bracket"
      : "—";
  byId("configuration-crc").textContent = formatCrc(
    latestSample?.configuration_crc ?? run?.configuration_crc ?? session.configuration_crc,
  );
  const hardwareVerdict = run?.hardware_safety_verdict ?? (run?.status || "Pending");
  const performanceVerdict = run?.performance_verdict
    ?? (run?.target_band_pass === true ? "PASS" : run?.target_band_pass === false ? "FAIL" : "Pending");
  byId("hardware-safety-verdict").textContent = safe(hardwareVerdict, "Pending");
  byId("hardware-safety-verdict").dataset.status = statusClass(hardwareVerdict);
  byId("performance-verdict").textContent = safe(performanceVerdict, "Pending");
  byId("performance-verdict").dataset.status = statusClass(performanceVerdict);

  byId("physics-channel-title").textContent = variableHil ? "Causal RocketPy physics" : "RocketPy/SIL physics";
  byId("physics-channel-note").textContent = variableHil
    ? "next interval uses last confirmed XACTUAL fraction"
    : "precomputed trajectory truth";
  byId("target-channel-title").textContent = variableHil ? "Motor target fraction" : "Forced HIL actuator command";
  byId("target-channel-note").textContent = variableHil
    ? "normal gated controller-to-actuator path"
    : "0 to full to HOME in TMC ramp counts";
  byId("feedback-channel").hidden = !variableHil;
  const warnings = payload.warnings || [];
  byId("measurement-warning").textContent = warnings[1]
    || "TMC XACTUAL is internal ramp-generator state, not independent mechanical position evidence.";
  byId("causality-warning").textContent = warnings[0]
    || "Controller request, motor target, feedback, and physics-applied deployment are separate evidence channels.";
  byId("deployment-title").textContent = variableHil ? "Causal deployment channels" : "Deployment demand";
  byId("deployment-note").textContent = variableHil
    ? "Request, motor target, XACTUAL, and the fraction applied to RocketPy remain distinct"
    : "Raw controller request remains separate from forced motion";
  byId("deployment-legend-controller").textContent = variableHil ? "Controller request" : "Raw STM32 controller";
  byId("deployment-legend-target").textContent = variableHil ? "Actuator target" : "Applied HIL override";
  byId("deployment-legend-feedback").hidden = !variableHil;
  byId("deployment-legend-physics").hidden = !variableHil;

  const truthApogee = finite(run?.rocketpy_truth_apogee_ft);
  byId("current-truth-apogee").textContent = truthApogee === null
    ? "Pending"
    : `${truthApogee.toFixed(1)} ft`;
  byId("current-truth-apogee-note").textContent = run?.apogee_is_final
    ? "Final RocketPy/SIL truth; not a physical measurement"
    : "Peak so far; final simulated apogee is pending";

  const silEvaluated = finite(session.sil_evaluated_runs) ?? 0;
  const silTargetPassed = finite(session.sil_target_band_passed_runs) ?? 0;
  byId("sil-target-band-count").textContent = `${silTargetPassed}/${silEvaluated}`;
  byId("sil-target-band-note").textContent = "RocketPy/SIL truth inside 3,000 +/-100 ft";

  const chip = byId("run-status-chip");
  chip.textContent = run ? safe(run.status, "Unknown") : "Waiting";
  chip.dataset.status = statusClass(run?.status);
}

function formatTime(value) {
  if (!value) return "not yet";
  const date = new Date(value);
  return Number.isNaN(date.getTime()) ? safe(value) : date.toLocaleTimeString();
}

function normalizeSeries(samples, key) {
  return samples.map((sample) => finite(sample[key]));
}

function validPointCount(series) {
  return series.reduce((count, value) => count + (value === null ? 0 : 1), 0);
}

function drawLineChart(canvas, samples, seriesDefinitions, options = {}) {
  const stage = canvas.closest(".chart-stage");
  const times = samples.map((sample, index) => finite(sample.time_s) ?? index);
  const series = seriesDefinitions.map((definition) => ({
    ...definition,
    values: normalizeSeries(samples, definition.key),
  }));
  const enough = samples.length >= (options.minimumPoints || 8)
    && series.some((item) => validPointCount(item.values) >= (options.minimumPoints || 8));
  stage.dataset.ready = enough ? "true" : "false";

  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(rect.width * ratio));
  canvas.height = Math.max(1, Math.round(rect.height * ratio));
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, rect.width, rect.height);
  if (!enough) return;

  const padding = { left: 54, right: 18, top: 12, bottom: 30 };
  const width = Math.max(1, rect.width - padding.left - padding.right);
  const height = Math.max(1, rect.height - padding.top - padding.bottom);
  const xMin = Math.min(...times);
  const xMaxRaw = Math.max(...times);
  const xMax = xMaxRaw === xMin ? xMin + 1 : xMaxRaw;
  const values = series.flatMap((item) => item.values.filter((value) => value !== null));
  let yMin = options.yMin ?? Math.min(...values);
  let yMax = options.yMax ?? Math.max(...values);
  if (yMin === yMax) {
    yMin -= 1;
    yMax += 1;
  }
  const margin = (yMax - yMin) * 0.08;
  if (options.yMin === undefined) yMin -= margin;
  if (options.yMax === undefined) yMax += margin;

  const x = (value) => padding.left + ((value - xMin) / (xMax - xMin)) * width;
  const y = (value) => padding.top + (1 - (value - yMin) / (yMax - yMin)) * height;

  context.strokeStyle = COLORS.grid;
  context.fillStyle = COLORS.muted;
  context.lineWidth = 1;
  context.font = "11px Segoe UI";
  context.textAlign = "right";
  context.textBaseline = "middle";
  for (let index = 0; index <= 4; index += 1) {
    const value = yMin + ((yMax - yMin) * index) / 4;
    const py = y(value);
    context.beginPath();
    context.moveTo(padding.left, py);
    context.lineTo(padding.left + width, py);
    context.stroke();
    context.fillText(formatAxis(value), padding.left - 7, py);
  }
  context.textAlign = "center";
  context.textBaseline = "top";
  for (let index = 0; index <= 4; index += 1) {
    const value = xMin + ((xMax - xMin) * index) / 4;
    context.fillText(`${formatAxis(value)}${options.xSuffix ?? " s"}`, x(value), padding.top + height + 8);
  }

  for (const item of series) {
    context.strokeStyle = item.color;
    context.lineWidth = item.width || 2.4;
    context.setLineDash(item.dash || []);
    context.lineJoin = "round";
    context.lineCap = "round";
    context.beginPath();
    let drawing = false;
    item.values.forEach((value, index) => {
      if (value === null) {
        drawing = false;
        return;
      }
      const px = x(times[index]);
      const py = y(value);
      if (!drawing) {
        context.moveTo(px, py);
        drawing = true;
      } else {
        context.lineTo(px, py);
      }
    });
    context.stroke();
  }
  context.setLineDash([]);
}

function formatAxis(value) {
  const absolute = Math.abs(value);
  if (absolute >= 1000) return Math.round(value).toLocaleString();
  if (absolute >= 100) return value.toFixed(0);
  if (absolute >= 10) return value.toFixed(1);
  return value.toFixed(2);
}

function drawCampaignAltitudeChart(canvas, campaign) {
  const stage = canvas.closest(".chart-stage");
  const traces = (campaign?.traces || []).filter((trace) => (trace.points || []).length >= 2);
  const average = campaign?.average_trace || [];
  const target = finite(campaign?.target_apogee_ft) ?? 3000;
  const enough = traces.length > 0;
  stage.dataset.ready = enough ? "true" : "false";

  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(rect.width * ratio));
  canvas.height = Math.max(1, Math.round(rect.height * ratio));
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, rect.width, rect.height);
  if (!enough) return;

  const allPoints = traces.flatMap((trace) => trace.points || []);
  const times = allPoints.map((point) => finite(point.time_s)).filter((value) => value !== null);
  const altitudes = allPoints.map((point) => finite(point.altitude_ft)).filter((value) => value !== null);
  const padding = { left: 60, right: 18, top: 12, bottom: 30 };
  const width = Math.max(1, rect.width - padding.left - padding.right);
  const height = Math.max(1, rect.height - padding.top - padding.bottom);
  const xMin = Math.min(...times);
  const rawXMax = Math.max(...times);
  const xMax = rawXMax === xMin ? xMin + 1 : rawXMax;
  const yMin = Math.min(0, ...altitudes, target);
  const rawYMax = Math.max(...altitudes, target);
  const yMax = rawYMax === yMin ? yMin + 1 : rawYMax * 1.05;
  const x = (value) => padding.left + ((value - xMin) / (xMax - xMin)) * width;
  const y = (value) => padding.top + (1 - (value - yMin) / (yMax - yMin)) * height;

  context.strokeStyle = COLORS.grid;
  context.fillStyle = COLORS.muted;
  context.lineWidth = 1;
  context.font = "11px Segoe UI";
  context.textAlign = "right";
  context.textBaseline = "middle";
  for (let index = 0; index <= 4; index += 1) {
    const value = yMin + ((yMax - yMin) * index) / 4;
    const py = y(value);
    context.beginPath();
    context.moveTo(padding.left, py);
    context.lineTo(padding.left + width, py);
    context.stroke();
    context.fillText(formatAxis(value), padding.left - 7, py);
  }
  context.textAlign = "center";
  context.textBaseline = "top";
  for (let index = 0; index <= 4; index += 1) {
    const value = xMin + ((xMax - xMin) * index) / 4;
    context.fillText(`${formatAxis(value)} s`, x(value), padding.top + height + 8);
  }

  const strokePoints = (points) => {
    context.beginPath();
    let drawing = false;
    points.forEach((point) => {
      const time = finite(point.time_s);
      const altitude = finite(point.altitude_ft);
      if (time === null || altitude === null) {
        drawing = false;
        return;
      }
      if (!drawing) {
        context.moveTo(x(time), y(altitude));
        drawing = true;
      } else {
        context.lineTo(x(time), y(altitude));
      }
    });
    context.stroke();
  };

  context.strokeStyle = "rgba(23, 105, 170, 0.20)";
  context.lineWidth = 1.15;
  context.setLineDash([]);
  traces.forEach((trace) => strokePoints(trace.points || []));

  if (average.length >= 2) {
    context.strokeStyle = COLORS.blue;
    context.lineWidth = 3.2;
    strokePoints(average);
  }

  context.strokeStyle = COLORS.gold;
  context.lineWidth = 2.2;
  context.setLineDash([3, 5]);
  context.beginPath();
  context.moveTo(padding.left, y(target));
  context.lineTo(padding.left + width, y(target));
  context.stroke();
  context.setLineDash([]);
}

function renderCampaignSummary(payload) {
  const campaign = payload.campaign_altitude || {};
  byId("campaign-apogee-mean").textContent = fixed(campaign.mean_truth_apogee_ft, 1, " ft");
  const minimum = finite(campaign.minimum_truth_apogee_ft);
  const maximum = finite(campaign.maximum_truth_apogee_ft);
  byId("campaign-apogee-range").textContent = minimum === null || maximum === null
    ? "N/A"
    : `${minimum.toFixed(1)} - ${maximum.toFixed(1)} ft`;
  byId("campaign-target-error").textContent = signedFixed(campaign.mean_target_error_ft, 1, " ft");
  byId("campaign-apogee-count").textContent = `${safe(campaign.final_apogee_count, "0")} runs`;
}

function drawTimeline(canvas, samples) {
  const stage = canvas.closest(".chart-stage");
  const enough = samples.length > 0;
  stage.dataset.ready = enough ? "true" : "false";
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(rect.width * ratio));
  canvas.height = Math.max(1, Math.round(rect.height * ratio));
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, rect.width, rect.height);
  if (!enough) return;

  const padding = { left: 72, right: 18, top: 12, bottom: 28 };
  const width = rect.width - padding.left - padding.right;
  const tracks = [
    { label: "PHASE", key: "phase" },
    { label: "SW ZERO", key: "software_home_active", color: COLORS.blue },
    { label: "SW FULL", key: "software_full_active", color: COLORS.gold },
    { label: "FAULT", key: "fault_active", color: COLORS.red },
  ];
  const trackHeight = (rect.height - padding.top - padding.bottom) / tracks.length;
  const times = samples.map((sample, index) => finite(sample.time_s) ?? index);
  const xMin = Math.min(...times);
  const xMaxRaw = Math.max(...times);
  const xMax = xMaxRaw === xMin ? xMin + 1 : xMaxRaw;
  const x = (value) => padding.left + ((value - xMin) / (xMax - xMin)) * width;
  const phaseColors = new Map();

  tracks.forEach((track, trackIndex) => {
    const top = padding.top + trackIndex * trackHeight;
    context.fillStyle = trackIndex % 2 ? "#fbfcfd" : "#f5f7f9";
    context.fillRect(padding.left, top, width, trackHeight - 2);
    context.fillStyle = COLORS.muted;
    context.font = "11px Segoe UI";
    context.textAlign = "right";
    context.textBaseline = "middle";
    context.fillText(track.label, padding.left - 8, top + trackHeight / 2);

    samples.forEach((sample, index) => {
      const start = x(times[index]);
      const end = index + 1 < samples.length ? x(times[index + 1]) : padding.left + width;
      if (track.key === "phase") {
        const phase = safe(sample.phase, "UNKNOWN");
        if (!phaseColors.has(phase)) phaseColors.set(phase, COLORS.phase[phaseColors.size % COLORS.phase.length]);
        context.fillStyle = phaseColors.get(phase);
        context.fillRect(start, top + 2, Math.max(1, end - start), trackHeight - 6);
      } else if (sample[track.key]) {
        context.fillStyle = track.color;
        context.fillRect(start, top + 4, Math.max(1, end - start), trackHeight - 10);
      }
    });
  });

  context.fillStyle = COLORS.muted;
  context.textAlign = "center";
  context.textBaseline = "top";
  for (let index = 0; index <= 4; index += 1) {
    const value = xMin + ((xMax - xMin) * index) / 4;
    context.fillText(`${formatAxis(value)} s`, x(value), rect.height - padding.bottom + 7);
  }
}

function renderCharts(payload) {
  const samples = payload.current_run?.samples || [];
  const variableHil = isVariableHil(payload);
  drawLineChart(byId("altitude-chart"), samples, [
    { key: "truth_altitude_ft", color: COLORS.blue },
    { key: "stm32_altitude_ft", color: COLORS.orange, dash: [8, 5] },
  ]);
  drawLineChart(byId("velocity-chart"), samples, [
    { key: "truth_velocity_fps", color: COLORS.blue },
    { key: "stm32_velocity_fps", color: COLORS.orange, dash: [8, 5] },
  ]);
  drawLineChart(byId("apogee-chart"), samples, [
    { key: "predicted_apogee_ft", color: COLORS.orange, dash: [8, 5] },
    { key: "closed_predicted_apogee_ft", color: COLORS.green, dash: [5, 4] },
    { key: "full_predicted_apogee_ft", color: COLORS.red, dash: [5, 4] },
    { key: "target_apogee_ft", color: COLORS.gold, dash: [2, 5] },
  ]);
  const deploymentSeries = variableHil
    ? [
      { key: "controller_request_percent", color: COLORS.blue, dash: [8, 5] },
      { key: "actuator_target_percent", color: COLORS.gold },
      { key: "actuator_feedback_percent", color: COLORS.orange, dash: [5, 4] },
      { key: "physics_applied_percent", color: COLORS.green },
    ]
    : [
      { key: "raw_controller_percent", color: COLORS.blue, dash: [8, 5] },
      { key: "applied_hil_percent", color: COLORS.gold },
    ];
  drawLineChart(byId("deployment-chart"), samples, deploymentSeries, { yMin: 0, yMax: 100 });
  drawLineChart(byId("actuator-chart"), samples, [
    { key: "target_rotations", color: COLORS.blue },
    { key: "xactual_rotations", color: COLORS.orange, dash: [8, 5] },
  ], { yMin: 0, yMax: 3.1 });
  drawTimeline(byId("timeline-chart"), samples);
  drawCampaignAltitudeChart(
    byId("campaign-altitude-chart"),
    payload.campaign_altitude || {},
  );

  const trends = payload.trends || [];
  const trendSamples = trends.map((item, index) => ({
    time_s: finite(item.sequence) ?? index + 1,
    open_time_s: finite(item.open_time_s),
    close_time_s: finite(item.close_time_s),
    max_tracking_error_rotations: finite(item.max_tracking_error_rotations),
  }));
  drawLineChart(byId("stroke-chart"), trendSamples, [
    { key: "open_time_s", color: COLORS.blue },
    { key: "close_time_s", color: COLORS.orange, dash: [8, 5] },
  ], { minimumPoints: 2, yMin: 0, xSuffix: " cycle" });
  drawLineChart(byId("tracking-chart"), trendSamples, [
    { key: "max_tracking_error_rotations", color: COLORS.gold },
  ], { minimumPoints: 2, yMin: 0, xSuffix: " cycle" });
}

function renderTable(payload) {
  const rows = payload.recent_runs || [];
  const body = byId("recent-runs");
  if (!rows.length) {
    body.innerHTML = '<tr><td colspan="10" class="empty-cell">No finalized runs yet.</td></tr>';
    return;
  }
  body.innerHTML = rows.map((run) => {
    const klass = statusClass(run.status);
    return `
      <tr>
        <td>${escapeHtml(safe(run.cycle_index))}</td>
        <td>${escapeHtml(safe(run.mode))}</td>
        <td title="${escapeHtml(safe(run.hardware_run_id))}">${escapeHtml(shortId(run.hardware_run_id))}</td>
        <td><span class="run-status ${klass}">${escapeHtml(safe(run.status))}</span></td>
        <td>${escapeHtml(fixed(run.rocketpy_truth_apogee_ft, 1, " ft"))}</td>
        <td>${run.target_band_pass === null || run.target_band_pass === undefined ? "N/A" : `<span class="run-status ${run.target_band_pass ? "pass" : "fail"}">${run.target_band_pass ? "PASS" : "FAIL"}</span>`}</td>
        <td>${escapeHtml(fixed(run.open_time_s, 2, " s"))}</td>
        <td>${escapeHtml(fixed(run.close_time_s, 2, " s"))}</td>
        <td>${escapeHtml(fixed(run.max_tracking_error_rotations, 3, " rot"))}</td>
        <td class="failure" title="${escapeHtml(safe(run.failure_message, ""))}">${escapeHtml(safe(run.failure_message, "—"))}</td>
      </tr>`;
  }).join("");
}

function shortId(value) {
  const text = safe(value);
  return text.length > 18 ? `${text.slice(0, 8)}…${text.slice(-6)}` : text;
}

function render(payload) {
  state.payload = payload;
  renderSummary(payload);
  renderCampaignSummary(payload);
  renderCharts(payload);
  renderTable(payload);
}

async function refreshState(force = false) {
  if (state.snapshot || state.refreshInFlight) return;
  const now = performance.now();
  if (!force && now - state.lastRefreshMs < 450) {
    if (!state.refreshPending) {
      state.refreshPending = true;
      setTimeout(() => {
        state.refreshPending = false;
        refreshState(true);
      }, 460 - (now - state.lastRefreshMs));
    }
    return;
  }
  state.refreshInFlight = true;
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) throw new Error(`state HTTP ${response.status}`);
    const payload = await response.json();
    state.lastRefreshMs = performance.now();
    render(payload);
    setConnection(state.source?.readyState === EventSource.OPEN ? "Live" : "SQLite connected", state.source?.readyState === EventSource.OPEN ? "live" : "waiting");
  } catch (error) {
    setConnection("Dashboard data lost", "lost");
  } finally {
    state.refreshInFlight = false;
  }
}

async function backfillGap(record, previousIndex) {
  const hardwareId = record.hardware_run_id;
  if (!hardwareId) return;
  const query = new URLSearchParams({
    hardware_run_id: hardwareId,
    after_event_index: String(previousIndex),
  });
  try {
    await fetch(`/api/events/backfill?${query}`, { cache: "no-store" });
  } finally {
    refreshState(true);
  }
}

function observeEvent(record) {
  const hardwareId = record.hardware_run_id;
  const eventIndex = finite(record.event_index);
  if (hardwareId && eventIndex !== null) {
    const previous = state.eventIndices.get(hardwareId);
    if (previous !== undefined && eventIndex > previous + 1) {
      backfillGap(record, previous);
    }
    state.eventIndices.set(hardwareId, Math.max(previous ?? -1, eventIndex));
  }
  refreshState();
}

function startEvents() {
  if (state.snapshot) return;
  const source = new EventSource("/api/events");
  state.source = source;
  source.addEventListener("open", () => {
    setConnection("Live", "live");
    refreshState(true);
  });
  source.addEventListener("update", (event) => {
    try {
      observeEvent(JSON.parse(event.data));
    } catch (_error) {
      refreshState();
    }
  });
  source.addEventListener("reset", () => refreshState(true));
  source.addEventListener("error", () => {
    setConnection("Reconnecting", "waiting");
    refreshState(true);
  });
}

let resizeTimer = null;
window.addEventListener("resize", () => {
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(() => state.payload && renderCharts(state.payload), 120);
});

if (state.snapshot) {
  document.body.dataset.snapshot = "true";
  setConnection("Portable snapshot", "waiting");
  render(state.payload);
} else {
  refreshState(true);
  startEvents();
  setInterval(() => refreshState(true), 1000);
}
