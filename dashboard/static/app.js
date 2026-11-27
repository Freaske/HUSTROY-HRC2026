"use strict";

const $ = (selector) => document.querySelector(selector);
const MAX_DUTY = 200;          // MAX_SPEED in include/config.h
const REVERSE_LIMIT = 80;      // LINE_REVERSE_PWM_LIMIT in include/line_follow.h
const FOLLOW_STATES = { TRACKING: "đang bám vạch", SEARCH: "đang xoay tìm vạch", RECOVER: "vừa bắt lại vạch", AMBIGUOUS: "vạch mơ hồ" };
const MODE_NAMES = { MOTOR_TEST: "thử motor", SENSOR_TEST: "đọc cảm biến", LINE_FOLLOW: "bám line", RIGHT_LINE: "thử bánh phải" };
const STOP_REASONS = {
  LINE_LOST_TIMEOUT: "Mất vạch", LINE_AMBIGUOUS: "Vạch mơ hồ", RUN_TIMEOUT: "Hết giờ", EMERGENCY_COMMAND: "Bấm Dừng xe",
  BOOT_STOP: "Nút BOOT", STOP: "Lệnh STOP", DISARM: "Lệnh DISARM", SENSOR_FAULT: "Lỗi cảm biến", STALE_CONTROL: "Trễ nhịp điều khiển",
  MOTOR_FAULT: "Lỗi motor", MODE_CHANGE: "Đổi chế độ", BAD_COMMAND: "Lệnh sai",
};

const S = { state: {}, frame: null, frameAt: 0, chart: [], dead: { straight: null, pivot: null }, ff: null, calCommand: null };

// ---- plumbing -------------------------------------------------------------------------
let toastTimer;
function toast(message) {
  const el = $("#toast");
  el.textContent = message instanceof Error ? message.message : String(message);
  el.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.hidden = true; }, 5000);
}

async function api(path, body) {
  const options = body === undefined ? {} :
    { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) };
  const response = await fetch(path, options);
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.error || `Lỗi ${response.status}`);
  return data;
}
const send = (...lines) => api("/api/send", { lines }).catch(toast);
const emergencyStop = () => { stopDrive(false); finishWizard(null, false); api("/api/stop", {}).catch(toast); };

// ---- connection and status -------------------------------------------------------------
async function loadPorts() {
  const select = $("#port");
  try {
    const ports = await api("/api/ports");
    select.innerHTML = "";
    if (!ports.length) select.add(new Option("Không thấy cổng nào: cắm USB của xe", ""));
    for (const port of ports) {
      select.add(new Option(`${port.device}: ${port.description}`, port.device, false, port.device === S.state.port));
    }
  } catch (error) { toast(error); }
}

function renderStatus() {
  const s = S.state;
  $("#connect").textContent = s.connected ? "Ngắt kết nối" : "Kết nối";
  $("#port").disabled = !!s.connected;
  if (!s.connected) {
    $("#status").textContent = "Chưa kết nối. Chọn cổng của xe (thường là USB-SERIAL CH340) rồi bấm Kết nối.";
  } else {
    const parts = [`<strong>${s.port}</strong>`];
    if (s.mode) parts.push(`chế độ ${MODE_NAMES[s.mode] || s.mode}`);
    if (s.armed === true) parts.push("đã ARM");
    if (s.armed === false) parts.push("chưa ARM");
    if (s.auto === "countdown") parts.push("tự chạy: đang đếm ngược");
    if (s.auto === "on") parts.push("tự chạy: bật");
    $("#status").innerHTML = parts.join(", ");
  }
  const motor = s.connected && s.mode === "MOTOR_TEST";
  document.querySelectorAll(".hold, [data-wizard]").forEach((b) => { b.disabled = !motor; });
  $("#ff-now").textContent = s.params?.FF?.replace("/", " / ") || "gõ GET hoặc vào chế độ thử motor để xem";
}

// ---- the track strip ------------------------------------------------------------------
const SVG = "http://www.w3.org/2000/svg";
const track = { eyes: [], fills: [], bars: [], marks: [], labels: [] };
const eyeX = (i) => 100 + i * 100;

function el(name, attrs, parent) {
  const node = document.createElementNS(SVG, name);
  for (const [key, value] of Object.entries(attrs)) node.setAttribute(key, value);
  parent.appendChild(node);
  return node;
}

function buildTrack() {
  const svg = $("#track");
  const defs = el("defs", {}, svg);
  const hatch = el("pattern", { id: "hatch", width: 10, height: 10, patternUnits: "userSpaceOnUse", patternTransform: "rotate(45)" }, defs);
  el("rect", { width: 10, height: 10, fill: "#f6f7f4" }, hatch);
  el("rect", { width: 5, height: 10, fill: "#202328" }, hatch);
  el("rect", { class: "paper", x: 20, y: 10, width: 560, height: 92, rx: 4 }, svg);
  track.tape = el("rect", { class: "tape", x: 282, y: 10, width: 36, height: 92 }, svg);
  for (let i = 0; i < 5; i++) {
    el("circle", { class: "eye-ring", cx: eyeX(i), cy: 56, r: 21 }, svg);
    track.fills.push(el("circle", { class: "eye-fill", cx: eyeX(i), cy: 56, r: 0 }, svg));
    el("rect", { class: "bar-bg", x: eyeX(i) - 22, y: 122, width: 44, height: 92 }, svg);
    track.bars.push(el("rect", { class: "bar-val", x: eyeX(i) - 22, y: 214, width: 44, height: 0 }, svg));
    track.marks.push(["white", "floor", "black"].map((kind) =>
      el("line", { class: `mark ${kind}`, x1: eyeX(i) - 28, x2: eyeX(i) + 28, y1: -10, y2: -10 }, svg)));
    track.labels.push(el("text", { x: eyeX(i), y: 236, "text-anchor": "middle" }, svg));
    el("text", { class: "small", x: eyeX(i), y: 250, "text-anchor": "middle" }, svg).textContent = `S${i + 1}`;
  }
}

function renderTrack() {
  const f = S.frame;
  const stale = !f || performance.now() - S.frameAt > 1500;
  $("#track").classList.toggle("stale", stale);
  if (!f) return;
  const cal = S.state.cal;
  const tape = track.tape;
  const ambiguous = f.signal && !f.posValid;
  tape.classList.toggle("ambiguous", ambiguous);
  tape.style.display = f.posValid || ambiguous ? "" : "none";
  if (f.posValid) { tape.setAttribute("x", 300 + f.pos * 100 - 18); tape.setAttribute("width", 36); }
  if (ambiguous) { tape.setAttribute("x", 20); tape.setAttribute("width", 560); }
  f.raw.forEach((raw, i) => {
    let level = f.norm?.[i];
    if (level === undefined && cal) level = (raw - cal.white[i]) / Math.max(1, cal.black[i] - cal.white[i]);
    track.fills[i].setAttribute("r", (18 * Math.min(1, Math.max(0, level || 0))).toFixed(1));
    const height = 92 * Math.min(1, raw / 4095);
    track.bars[i].setAttribute("y", 214 - height);
    track.bars[i].setAttribute("height", height);
    track.labels[i].textContent = raw;
    ["white", "floor", "black"].forEach((kind, k) => {
      const y = cal ? 214 - 92 * Math.min(1, cal[kind][i] / 4095) : -10;
      track.marks[i][k].setAttribute("y1", y);
      track.marks[i][k].setAttribute("y2", y);
    });
  });
  const diagOnly = !f.norm?.length;
  $("#r-line").textContent = diagOnly ? "chỉ có giá trị thô" : f.posValid ? "thấy vạch" : f.signal ? "mơ hồ, nhiều vạch" : "không thấy";
  $("#r-pos").textContent = f.posValid ? `${f.pos > 0 ? "+" : ""}${f.pos.toFixed(2)} mắt` : "—";
  $("#r-conf").textContent = diagOnly ? "—" : `${Math.round(f.conf * 100)}%`;
  $("#r-lr").textContent = `${f.l} / ${f.r}`;
  $("#r-mode").textContent = FOLLOW_STATES[f.state] || MODE_NAMES[f.state] || f.state || "—";
}

// ---- live chart of a line-follow run ---------------------------------------------------
function drawChart() {
  const canvas = $("#chart");
  if (canvas.offsetParent === null) return;
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth, height = canvas.clientHeight || 220;
  if (canvas.width !== Math.round(width * ratio)) { canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio); }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  const now = performance.now(), span = 10000, left = 44;
  const x = (t) => left + (width - left) * (1 - (now - t) / span);
  const top = { y: 8, h: height * 0.55 - 12 }, bottom = { y: height * 0.55 + 8, h: height * 0.45 - 16 };
  const posY = (p) => top.y + top.h / 2 - (p / 2) * (top.h / 2);
  const dutyY = (d) => bottom.y + bottom.h / 2 - (d / MAX_DUTY) * (bottom.h / 2);
  ctx.font = "12px 'Be Vietnam Pro', system-ui"; ctx.fillStyle = "#5e646d"; ctx.strokeStyle = "#d8dbd4"; ctx.lineWidth = 1;
  for (const p of [-2, -1, 0, 1, 2]) { const y = posY(p); ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(width, y); ctx.stroke(); ctx.fillText(p > 0 ? `+${p}` : `${p}`, 8, y + 4); }
  for (const d of [-MAX_DUTY, 0, MAX_DUTY]) { const y = dutyY(d); ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(width, y); ctx.stroke(); ctx.fillText(`${d}`, 4, y + 4); }
  const points = S.chart.filter((p) => now - p.t < span);
  const line = (key, toY, color, w) => {
    ctx.strokeStyle = color; ctx.lineWidth = w; ctx.beginPath();
    let open = false;
    for (const p of points) {
      if (p[key] === null) { open = false; continue; }
      const px = x(p.t), py = toY(p[key]);
      if (open) ctx.lineTo(px, py); else ctx.moveTo(px, py);
      open = true;
    }
    ctx.stroke();
  };
  line("pos", posY, "#202328", 2.5);
  line("l", dutyY, "#2e5bff", 1.75);
  line("r", dutyY, "#17a05d", 1.75);
  if (!points.length) { ctx.fillStyle = "#5e646d"; ctx.fillText("Biểu đồ hiện khi xe đang tự bám line.", left + 8, top.y + top.h / 2); }
}

let frameQueued = false;
function scheduleRender() {
  if (frameQueued) return;
  frameQueued = true;
  requestAnimationFrame(() => { frameQueued = false; renderTrack(); drawChart(); });
}
setInterval(scheduleRender, 500); // Keeps the stale fade and the chart scroll moving between frames.

// ---- serial log -------------------------------------------------------------------------
function log(text, kind) {
  if (/^(SET |BOTH |TEST: )/.test(text)) return; // Drive heartbeats would bury everything else.
  const pre = $("#log");
  const row = document.createElement("div");
  row.textContent = kind === "out" ? `> ${text}` : text;
  if (kind === "out") row.className = "out";
  pre.appendChild(row);
  while (pre.childElementCount > 300) pre.firstChild.remove();
  if (pre.scrollHeight - pre.scrollTop - pre.clientHeight < 80) pre.scrollTop = pre.scrollHeight;
}

// ---- events from the server -------------------------------------------------------------
function listen() {
  const events = new EventSource("/api/events");
  events.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "frame") {
      S.frame = message; S.frameAt = performance.now();
      if (FOLLOW_STATES[message.state]) {
        S.chart.push({ t: S.frameAt, pos: message.posValid ? message.pos : null, l: message.l, r: message.r });
        if (S.chart.length > 800) S.chart.splice(0, S.chart.length - 800);
      }
      scheduleRender();
    } else if (message.type === "status") {
      S.state = message.state; renderStatus();
    } else if (message.type === "line") {
      log(message.text, "in");
      if (/^OK SAVED/.test(message.text)) toast("Đã lưu vào xe.");
      if (/^(ERR|ERROR|BLOCKED)/.test(message.text)) toast(message.text);
    } else if (message.type === "sent") {
      log(message.text, "out");
    } else if (message.type === "run") {
      addRun(message.run, true);
    }
  };
}

// ---- motor: hold to drive ---------------------------------------------------------------
const speed = () => +$("#speed").value;
const clampDuty = (value) => Math.max(-MAX_DUTY, Math.min(MAX_DUTY, Math.round(value)));
const DRIVES = {
  forward: () => [speed(), speed()], back: () => [-speed(), -speed()],
  left: () => [-speed(), speed()], right: () => [speed(), -speed()],
  sliders: () => [+$("#left").value, +$("#right").value],
};
let drive = null;

function motorReady() {
  if (S.state.connected && S.state.mode === "MOTOR_TEST") return true;
  toast("Bấm “Vào chế độ thử motor” trước.");
  return false;
}

async function startDrive(name, button) {
  if (drive || wizard || !motorReady()) return;
  const current = drive = { name, button, timer: null, started: false, released: false };
  button?.classList.add("active");
  const tick = () => { if (drive === current) { const [l, r] = DRIVES[current.name]().map(clampDuty); send(`SET ${l} ${r}`); } };
  await send("ARM");                 // ARM itself never moves the wheels.
  if (drive !== current) return;     // Emergency stop or tab change while ARM was in flight.
  tick();
  current.started = true;
  if (current.released) {            // A quick tap: nudge for 0.3 s instead of doing nothing.
    setTimeout(() => { if (drive === current) stopDrive(); }, 300);
    return;
  }
  current.timer = setInterval(tick, 300); // Firmware stops on its own 1 s after the last SET (PULSE=1000).
}

function stopDrive(sendStop = true, allowNudge = true) {
  if (!drive) return;
  if (sendStop && allowNudge && !drive.started) { drive.released = true; return; }
  clearInterval(drive.timer);
  drive.button?.classList.remove("active");
  drive = null;
  if (sendStop) send("STOP");
}

document.querySelectorAll(".hold").forEach((button) => {
  button.addEventListener("pointerdown", (event) => {
    if (button.disabled) return;
    button.setPointerCapture(event.pointerId);
    startDrive(button.dataset.drive, button);
  });
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) button.addEventListener(type, () => stopDrive());
});

for (const id of ["speed", "left", "right"]) {
  const input = $(`#${id}`);
  input.addEventListener("input", () => { $(`#${id}-out`).textContent = input.value; });
}

// ---- motor: dead-band wizard ------------------------------------------------------------
let wizard = null;

async function startWizard(kind) {
  if (drive || wizard || !motorReady()) return;
  wizard = { kind, duty: 30 };
  $("#wizard-duty").textContent = wizard.duty;
  $("#wizard-live").hidden = false;
  $("#wizard-moved").focus();
  await send("ARM");
  if (!wizard) return;
  const command = () => wizard && send(wizard.kind === "straight" ? `BOTH ${wizard.duty}` : `SET ${wizard.duty} -${wizard.duty}`);
  command();
  wizard.timer = setInterval(command, 300);
  wizard.step = setInterval(() => {
    if (!wizard) return;
    if (wizard.duty + 5 > MAX_DUTY) {
      finishWizard(null);
      toast(`Đến ${MAX_DUTY} mà xe vẫn chưa nhích: kiểm tra công tắc pin, dây VM và bánh có bị kẹt không.`);
      return;
    }
    wizard.duty += 5;
    $("#wizard-duty").textContent = wizard.duty;
  }, 1500);
}

function finishWizard(value, sendStop = true) {
  if (!wizard) return;
  clearInterval(wizard.timer); clearInterval(wizard.step);
  const kind = wizard.kind;
  wizard = null;
  $("#wizard-live").hidden = true;
  if (sendStop) send("STOP");
  if (value !== null) { S.dead[kind] = value; renderDeadBand(); }
}

function renderDeadBand() {
  const { straight, pivot } = S.dead;
  $("#dead-straight").textContent = straight ? `${straight}/255` : "chưa đo";
  $("#dead-pivot").textContent = pivot ? `${pivot}/255` : "chưa đo";
  if (!straight || !pivot) return;
  const reverse = Math.min(pivot, REVERSE_LIMIT);
  const start = [straight, reverse, straight, reverse];
  const sustain = start.map((v) => Math.round(v * 0.8));
  S.ff = `FF=${start.join(",")}/${sustain.join(",")}`;
  $("#ff-new").textContent = `${start.join(",")} / ${sustain.join(",")}`;
  $("#ff-note").textContent = pivot > REVERSE_LIMIT
    ? `Xe cần ${pivot} để xoay nhưng firmware chỉ cho bánh lùi tới ${REVERSE_LIMIT}. Cần tăng LINE_REVERSE_PWM_LIMIT trong include/line_follow.h rồi nạp lại.`
    : "Thứ tự: trái tiến, trái lùi, phải tiến, phải lùi. Mức duy trì lấy bằng 80% mức bắt đầu.";
  $("#ff-apply").disabled = false;
}

document.querySelectorAll("[data-wizard]").forEach((button) => button.addEventListener("click", () => startWizard(button.dataset.wizard)));
$("#wizard-moved").addEventListener("click", () => wizard && finishWizard(wizard.duty));
$("#ff-apply").addEventListener("click", () => {
  if (S.ff) send("STOP", S.ff, "SAVE", "GET");
});

$("#motor-mode").addEventListener("click", async () => {
  await api("/api/stop", {}).catch(toast);
  send("MODE MOTOR_TEST", "PULSE=1000", "TRACE=1", "GET");
});

// ---- sensors and calibration ------------------------------------------------------------
const CAL_STEPS = [
  { step: "white", label: "Cả 5 mắt trên giấy trắng, không mắt nào chạm vạch" },
  { step: 0, label: "Mắt S1 (ngoài cùng trái) đè lên vạch" },
  { step: 1, label: "Mắt S2 đè lên vạch" },
  { step: 2, label: "Mắt S3 (giữa) đè lên vạch" },
  { step: 3, label: "Mắt S4 đè lên vạch" },
  { step: 4, label: "Mắt S5 (ngoài cùng phải) đè lên vạch" },
];

function buildSteps() {
  const list = $("#cal-steps");
  list.innerHTML = "";
  for (const item of CAL_STEPS) {
    const row = document.createElement("li");
    row.innerHTML = `<span></span><span class="result">chưa ghi</span><button class="quiet">Ghi</button>`;
    row.firstChild.textContent = item.label;
    const button = row.querySelector("button");
    button.addEventListener("click", async () => {
      button.disabled = true;
      button.textContent = "Đang ghi…";
      try {
        const result = await api("/api/calibration/record", { step: item.step });
        row.classList.add("done");
        row.querySelector(".result").textContent = `${result.count} mẫu: ${result.median.join(" ")}`;
      } catch (error) { toast(error); }
      button.disabled = false;
      button.textContent = "Ghi lại";
      $("#cal-compute").disabled = $("#cal-steps").querySelectorAll("li.done").length < CAL_STEPS.length;
    });
    list.appendChild(row);
  }
  $("#cal-compute").disabled = true;
  $("#cal-apply").disabled = true;
  $("#cal-report").hidden = true;
}

$("#cal-compute").addEventListener("click", async () => {
  try {
    const result = await api("/api/calibration/compute", {});
    const report = $("#cal-report");
    report.hidden = false;
    const ok = !result.problems.length;
    report.className = `report ${ok ? "good" : "bad"}`;
    report.textContent = result.report + result.command + (ok ? "\nĐẠT: bấm Lưu vào xe." : `\nKHÔNG ĐẠT:\n - ${result.problems.join("\n - ")}`);
    S.calCommand = ok ? result.command : null;
    $("#cal-apply").disabled = !ok;
  } catch (error) { toast(error); }
});
$("#cal-apply").addEventListener("click", () => { if (S.calCommand) send(S.calCommand, "CAL?", "SAVE"); });
$("#cal-reset").addEventListener("click", async () => { await api("/api/calibration/reset", {}).catch(toast); buildSteps(); });
$("#sensor-mode").addEventListener("click", async () => {
  await api("/api/stop", {}).catch(toast);
  send("MODE SENSOR_TEST", "TRACE=1", "CAL?");
});

// ---- runs --------------------------------------------------------------------------------
function addRun(run, isNew) {
  const body = $("#runs-body");
  body.querySelector(".empty")?.remove();
  const row = document.createElement("tr");
  if (isNew) row.className = "new";
  const cells = [
    run.time?.slice(5).replace(" ", ", ") ?? "", run.spd, run.kp, run.kd,
    STOP_REASONS[run.reason] || run.reason, run.run_s, `${run.detected_pct}%`, `${run.edge_pct}%`,
    run.line_losses, run.mae,
  ];
  cells.forEach((value, index) => {
    const cell = document.createElement("td");
    cell.textContent = value ?? "";
    if (index >= 5) cell.className = "num";
    if (index === 4) cell.title = run.reason;
    row.appendChild(cell);
  });
  const link = document.createElement("td");
  link.innerHTML = `<a href="/runs/${encodeURIComponent(run.file)}">tải</a>`;
  row.appendChild(link);
  body.prepend(row);
}

async function loadRuns() {
  try {
    const runs = await api("/api/runs");
    runs.forEach((run) => addRun(run, false));
  } catch (error) { toast(error); }
}

$("#auto-on").addEventListener("click", () => { S.chart = []; send("AUTO"); });

// ---- tabs, keys, top bar -----------------------------------------------------------------
function selectTab(tab) {
  document.querySelectorAll("[role=tab]").forEach((other) => {
    const selected = other === tab;
    other.setAttribute("aria-selected", selected);
    other.tabIndex = selected ? 0 : -1;
    $(`#${other.getAttribute("aria-controls")}`).hidden = !selected;
  });
  stopDrive(true, false);
  scheduleRender();
}
document.querySelectorAll("[role=tab]").forEach((tab) => tab.addEventListener("click", () => selectTab(tab)));
$(".tabs").addEventListener("keydown", (event) => {
  const tabs = [...document.querySelectorAll("[role=tab]")];
  const index = tabs.indexOf(document.activeElement);
  if (index < 0 || !["ArrowLeft", "ArrowRight"].includes(event.key)) return;
  event.preventDefault();
  const next = tabs[(index + (event.key === "ArrowRight" ? 1 : tabs.length - 1)) % tabs.length];
  next.focus();
  selectTab(next);
});

const KEY_DRIVES = { ArrowUp: "forward", ArrowDown: "back", ArrowLeft: "left", ArrowRight: "right" };
const typing = (target) => target.closest("input, select, textarea, [role=tab]");
document.addEventListener("keydown", (event) => {
  if (event.code === "Space" && !typing(event.target)) { event.preventDefault(); emergencyStop(); return; }
  if (event.key === "Enter" && wizard && !typing(event.target)) { event.preventDefault(); finishWizard(wizard.duty); return; }
  const name = KEY_DRIVES[event.key];
  if (!name || event.repeat || typing(event.target) || $("#panel-motor").hidden) return;
  event.preventDefault();
  startDrive(name, document.querySelector(`.hold[data-drive=${name}]`));
});
document.addEventListener("keyup", (event) => { if (KEY_DRIVES[event.key] && drive?.name === KEY_DRIVES[event.key]) stopDrive(); });
window.addEventListener("blur", () => { stopDrive(true, false); finishWizard(null); });
document.addEventListener("visibilitychange", () => { if (document.hidden) { stopDrive(true, false); finishWizard(null); } });

$("#stop").addEventListener("click", emergencyStop);
$("#refresh-ports").addEventListener("click", loadPorts);
$("#connect-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  try {
    if (S.state.connected) await api("/api/disconnect", {});
    else if ($("#port").value) await api("/api/connect", { port: $("#port").value });
    else toast("Chưa chọn cổng. Cắm USB của xe rồi bấm Tìm cổng.");
  } catch (error) { toast(`Không mở được cổng: ${error.message}. Đóng Serial Monitor hoặc script đang dùng cổng này.`); }
});
$("#raw-form").addEventListener("submit", (event) => {
  event.preventDefault();
  const value = $("#raw").value.trim().toUpperCase();
  if (value) { send(value); $("#raw").value = ""; }
});

buildTrack();
buildSteps();
renderStatus();
listen();
loadPorts();
loadRuns();
