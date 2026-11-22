const tracks = {
  straight: ["Thẳng 1 m", "Đoạn thẳng từ vị trí xuất phát đến mốc cách 1 m."],
  curve: ["Cua R100", "Cua phải 90° có bán kính 100 mm và mốc sau cua."],
  corner: ["Góc nhọn", "Góc trong 60° với mốc trên đoạn ra."],
  cross: ["Ngã tư", "Hai line giao nhau vuông góc."],
  gap: ["Đứt line", "Line bị ngắt một đoạn 100 mm giữa sa bàn."],
  finish: ["Ô đích", "Xe tiến vào ô tối mô phỏng có kích thước 400 × 300 mm."],
};

const elements = Object.fromEntries([
  "tracks", "track-title", "track-description", "board", "run-state", "arm-state", "play", "play-label",
  "step", "reset", "timeline", "clock", "settings", "settings-note", "kp", "kd", "speed", "sensor-bits",
  "left", "right", "left-meter", "right-meter", "error", "distance", "line", "stop", "checkpoint-key", "alert",
].map(id => [id, document.getElementById(id)]));
const settingsInputs = [elements.kp, elements.kd, elements.speed];
const trackButtons = [...elements.tracks.querySelectorAll("button")];
const lights = [...elements["sensor-bits"].children];
const ctx = elements.board.getContext("2d");
const fixed = new Intl.NumberFormat("vi-VN", { minimumFractionDigits: 2, maximumFractionDigits: 2 });
const integer = new Intl.NumberFormat("vi-VN", { maximumFractionDigits: 0 });

let config;
let run;
let selected = "straight";
let index = 0;
let elapsed = 0;
let started = false;
let playing = false;
let busy = false;
let tuningChanged = false;
let lastTimestamp;
let animation;

function drawLabel(text, x, y, color = "#526782") {
  ctx.fillStyle = color;
  const fontSize = Math.max(17, 9 * elements.board.width / elements.board.clientWidth);
  ctx.font = `700 ${fontSize}px system-ui, sans-serif`;
  ctx.fillText(text, x, y);
}

function drawSurface(track) {
  ctx.fillStyle = "#f8fafd";
  ctx.fillRect(0, 0, track.width, track.height);
  ctx.fillStyle = "#d7e1ee";
  for (let x = 25; x < track.width; x += 25) {
    for (let y = 25; y < track.height; y += 25) {
      ctx.fillRect(x, y, 2, 2);
    }
  }
  ctx.strokeStyle = "#e0e8f1";
  ctx.lineWidth = 2;
  ctx.strokeRect(24, 24, track.width - 48, track.height - 48);
  ctx.lineWidth = track.lineWidth;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.strokeStyle = "#27384d";
  for (const path of track.paths) {
    ctx.beginPath();
    ctx.moveTo(...path[0]);
    for (const point of path.slice(1)) ctx.lineTo(...point);
    ctx.stroke();
  }
  for (const [x, y, width, height] of track.zones) {
    ctx.fillStyle = "#27384d";
    ctx.fillRect(x, y, width, height);
    drawLabel("Ô ĐÍCH · 400 × 300 mm", x + 22, y + 36, "#deebfc");
  }
  const [startX, startY] = track.start;
  ctx.strokeStyle = "#7f96b0";
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 6]);
  ctx.beginPath();
  ctx.moveTo(startX, startY - 61);
  ctx.lineTo(startX, startY + 61);
  ctx.stroke();
  drawLabel("START", startX - 25, startY - 70);
  for (const [x, y, angle] of track.checkpoints) {
    const wideX = 35 * Math.sin(angle);
    const wideY = 35 * Math.cos(angle);
    ctx.strokeStyle = "#18825e";
    ctx.beginPath();
    ctx.moveTo(x + wideX, y - wideY);
    ctx.lineTo(x - wideX, y + wideY);
    ctx.stroke();
    drawLabel("MỐC", x + (wideX > 15 ? 46 : -14), y - (wideX > 15 ? 0 : 48), "#147655");
  }
  ctx.setLineDash([]);
  ctx.strokeStyle = "#7f96b0";
  ctx.beginPath();
  ctx.moveTo(55, track.height - 51);
  ctx.lineTo(155, track.height - 51);
  ctx.moveTo(55, track.height - 57);
  ctx.lineTo(55, track.height - 45);
  ctx.moveTo(155, track.height - 57);
  ctx.lineTo(155, track.height - 45);
  ctx.stroke();
  drawLabel("100 mm", 64, track.height - 62);
}

function drawTrail(frames) {
  ctx.beginPath();
  ctx.moveTo(frames[0].x, frames[0].y);
  for (let i = 1; i <= index; i++) ctx.lineTo(frames[i].x, frames[i].y);
  ctx.strokeStyle = "#67baff";
  ctx.lineWidth = 4;
  ctx.stroke();
}

function roundedRect(x, y, width, height, radius, fill, border) {
  ctx.fillStyle = fill;
  ctx.beginPath();
  ctx.roundRect(x, y, width, height, radius);
  ctx.fill();
  if (border) {
    ctx.lineWidth = 2;
    ctx.strokeStyle = border;
    ctx.stroke();
  }
}

function drawRobot(frame, track, stopped) {
  ctx.save();
  ctx.translate(frame.x, frame.y);
  ctx.rotate(frame.angle);
  const dark = stopped ? "#993d2c" : "#17365f";
  const light = stopped ? "#ffebe4" : "#e0efff";
  const accent = stopped ? "#b95039" : "#185dc6";
  roundedRect(-52, -34, 113, 68, 12, light, dark);
  roundedRect(-23, -track.wheelTrack / 2 - 9, 46, 18, 5, "#27384d");
  roundedRect(-23, track.wheelTrack / 2 - 9, 46, 18, 5, "#27384d");
  roundedRect(-32, -24, 60, 48, 7, "#ffffff", "#adc1db");
  roundedRect(track.sensorAhead - 7, -29, 14, 58, 5, dark);
  ctx.fillStyle = accent;
  ctx.beginPath();
  ctx.moveTo(13, 0);
  ctx.lineTo(-7, -10);
  ctx.lineTo(-7, 10);
  ctx.fill();
  for (const [i, bit] of [...frame.bits].entries()) {
    ctx.beginPath();
    ctx.arc(track.sensorAhead, (i - 2) * track.sensorSpacing, 3.4, 0, 2 * Math.PI);
    ctx.fillStyle = bit === "1" ? "#80edd0" : "#9bacc1";
    ctx.fill();
  }
  ctx.restore();
}

function showError(message) {
  elements.alert.textContent = message;
  elements.alert.hidden = !message;
}

function renderControls() {
  elements.tracks.disabled = busy || !config;
  for (const input of settingsInputs) input.disabled = busy || !config;
  elements.play.disabled = busy || !run;
  elements.step.disabled = busy || !run || playing || index >= run.frames.length - 1;
  elements.reset.disabled = busy || !run || !started;
  elements.timeline.disabled = busy || !run || run.frames.length < 2;
  elements.play.querySelector("span").textContent = playing ? "Ⅱ" : "▶";
  let playLabel = "Chạy mô phỏng";
  if (playing) playLabel = "Tạm dừng";
  else if (started && !tuningChanged && index < run.frames.length - 1) playLabel = "Tiếp tục";
  else if (started) playLabel = "Chạy lại";
  elements["play-label"].textContent = playLabel;
  if (config) {
    elements["settings-note"].textContent = tuningChanged ? "Áp dụng thông số khi bấm chạy." :
      `Giới hạn lượt chạy ${fixed.format(config.runMs / 1000)} s.`;
  }
}

function render() {
  renderControls();
  if (!run) return;
  const frame = run.frames[index];
  const ended = started && index === run.frames.length - 1;
  const success = ended && run.checkpointReached;
  drawSurface(run.track);
  drawTrail(run.frames);
  drawRobot(frame, run.track, ended && !success);

  const state = elements["run-state"];
  const stateText = ended ? (success ? "Qua mốc" : "Xe đã dừng") :
    playing ? "Đang chạy" : started ? "Tạm dừng" : "Sẵn sàng";
  if (state.textContent !== stateText) state.textContent = stateText;
  state.dataset.state = ended ? (success ? "success" : "stopped") : playing ? "running" : "idle";
  elements["arm-state"].textContent = started && frame.armed ? "ARMED" : "DISARMED";
  elements["arm-state"].dataset.armed = Boolean(started && frame.armed);
  elements.timeline.max = run.frames.length - 1;
  elements.timeline.value = index;
  elements.clock.textContent = `${fixed.format(frame.t / 1000)} s`;
  elements.timeline.setAttribute("aria-valuetext", elements.clock.textContent);
  elements["sensor-bits"].setAttribute("aria-label", `Năm cảm biến: ${frame.bits}`);
  for (const [i, light] of lights.entries()) {
    light.textContent = frame.bits[i];
    light.classList.toggle("active", frame.bits[i] === "1");
  }
  for (const side of ["left", "right"]) {
    elements[side].textContent = frame[side];
    elements[`${side}-meter`].style.width = `${Math.abs(frame[side]) * 100 / config.maxSpeed}%`;
  }
  elements.error.textContent = fixed.format(frame.error);
  elements.distance.textContent = `${integer.format(frame.distance)} mm`;
  elements.line.textContent = frame.line;
  elements.stop.textContent = ended ? frame.stop : "–";
  elements["checkpoint-key"].hidden = !run.track.checkpoints.length;
}

function pause() {
  playing = false;
  cancelAnimationFrame(animation);
}

function seek(frameIndex) {
  index = frameIndex;
  elapsed = run.frames[index].t;
  render();
}

function tick(timestamp) {
  elapsed += Math.max(0, timestamp - lastTimestamp);
  lastTimestamp = timestamp;
  while (index + 1 < run.frames.length && run.frames[index + 1].t <= elapsed) index++;
  if (index === run.frames.length - 1) pause();
  render();
  if (playing) animation = requestAnimationFrame(tick);
}

function play() {
  if (index === run.frames.length - 1) {
    index = 0;
    elapsed = 0;
  }
  started = true;
  playing = run.frames.length > 1;
  lastTimestamp = performance.now();
  render();
  if (playing) animation = requestAnimationFrame(tick);
}

async function readJson(url) {
  const response = await fetch(url);
  const body = await response.json();
  if (!response.ok) throw new Error(body.error || "Không tải được dữ liệu mô phỏng.");
  return body;
}

async function loadTrack(track, autoplay = false) {
  if (!elements.settings.reportValidity()) return;
  const query = new URLSearchParams({
    track, kp: elements.kp.value, kd: elements.kd.value, speed: elements.speed.value,
  });
  pause();
  busy = true;
  showError("");
  render();
  try {
    run = await readJson(`/api/run?${query}`);
    selected = track;
    index = 0;
    elapsed = 0;
    started = false;
    tuningChanged = false;
    elements["track-title"].textContent = tracks[track][0];
    elements["track-description"].textContent = tracks[track][1];
    for (const button of trackButtons) button.setAttribute("aria-pressed", button.dataset.track === track);
  } catch (error) {
    showError(error.message);
    if (!run) elements["run-state"].textContent = "Không tải được";
    return;
  } finally {
    busy = false;
    render();
  }
  if (autoplay) play();
}

for (const button of trackButtons) button.addEventListener("click", () => loadTrack(button.dataset.track));
elements.play.addEventListener("click", () => {
  if (playing) { pause(); render(); }
  else if (tuningChanged) loadTrack(selected, true);
  else play();
});
elements.step.addEventListener("click", () => { started = true; seek(index + 1); });
elements.reset.addEventListener("click", () => { pause(); started = false; seek(0); });
elements.timeline.addEventListener("input", () => { pause(); started = true; seek(elements.timeline.valueAsNumber); });
elements.settings.addEventListener("input", () => { tuningChanged = true; renderControls(); });
elements.settings.addEventListener("submit", event => {
  event.preventDefault();
  if (!busy && !playing) loadTrack(selected, true);
});
new ResizeObserver(() => { if (run) render(); }).observe(elements.board);

async function initialize() {
  try {
    config = await readJson("/api/config");
    for (const input of settingsInputs) input.value = config[input.id];
    elements.speed.max = config.maxSpeed;
    elements.step.textContent = `Bước +${config.stepMs} ms`;
    renderControls();
    await loadTrack(selected);
  } catch (error) {
    showError(error.message);
    elements["run-state"].textContent = "Không tải được";
  }
}

initialize();
