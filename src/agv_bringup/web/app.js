/* global ROSLIB */

const $ = (id) => document.getElementById(id);

const els = {
  wsUrl: $("wsUrl"),
  topic: $("topic"),
  lin: $("lin"),
  ang: $("ang"),
  hz: $("hz"),
  btnConnect: $("btnConnect"),
  btnStop: $("btnStop"),
  connDot: $("connDot"),
  connText: $("connText"),
  log: $("log"),
  btnUp: $("btnUp"),
  btnDown: $("btnDown"),
  btnLeft: $("btnLeft"),
  btnRight: $("btnRight"),
};

let ros = null;
let cmdVelTopic = null;
let publishTimer = null;
let currentCmd = { linearX: 0, angularZ: 0 };

function log(line) {
  const ts = new Date().toLocaleTimeString();
  els.log.textContent = `[${ts}] ${line}\n` + els.log.textContent;
}

function setConnected(isConnected) {
  els.connDot.classList.toggle("dot--connected", isConnected);
  els.connDot.classList.toggle("dot--disconnected", !isConnected);
  els.connText.textContent = isConnected ? "Connected" : "Disconnected";
  els.btnConnect.textContent = isConnected ? "Disconnect" : "Connect";
}

function getNumber(el, fallback) {
  const v = Number(el.value);
  return Number.isFinite(v) ? v : fallback;
}

function ensureRoslib() {
  if (typeof ROSLIB === "undefined") {
    throw new Error("roslibjs not loaded. Check your network / CDN access.");
  }
}

function connectOrDisconnect() {
  try {
    ensureRoslib();
  } catch (e) {
    log(String(e?.message ?? e));
    return;
  }

  if (ros) {
    stopPublishing(true);
    try {
      ros.close();
    } catch (_) {}
    ros = null;
    cmdVelTopic = null;
    setConnected(false);
    log("Disconnected.");
    return;
  }

  const url = els.wsUrl.value.trim();
  const topicName = els.topic.value.trim() || "/cmd_vel";

  ros = new ROSLIB.Ros({ url });

  ros.on("connection", () => {
    cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType: "geometry_msgs/Twist",
    });
    setConnected(true);
    log(`Connected to ${url}. Publishing to ${topicName}.`);
  });

  ros.on("error", (err) => {
    setConnected(false);
    log(`Connection error: ${err?.message ?? err}`);
  });

  ros.on("close", () => {
    stopPublishing(true);
    ros = null;
    cmdVelTopic = null;
    setConnected(false);
    log("Connection closed.");
  });
}

function publishOnce(linearX, angularZ) {
  if (!cmdVelTopic) return;
  cmdVelTopic.publish(
    new ROSLIB.Message({
      linear: { x: linearX, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angularZ },
    }),
  );
}

function startPublishing(linearX, angularZ) {
  if (!cmdVelTopic) {
    log("Not connected. Click Connect first.");
    return;
  }

  currentCmd = { linearX, angularZ };

  const hz = Math.max(1, Math.floor(getNumber(els.hz, 20)));
  const periodMs = Math.floor(1000 / hz);

  if (publishTimer) clearInterval(publishTimer);
  publishTimer = setInterval(() => {
    publishOnce(currentCmd.linearX, currentCmd.angularZ);
  }, periodMs);

  // publish immediately so it feels responsive
  publishOnce(currentCmd.linearX, currentCmd.angularZ);
}

function stopPublishing(sendZero) {
  if (publishTimer) {
    clearInterval(publishTimer);
    publishTimer = null;
  }
  currentCmd = { linearX: 0, angularZ: 0 };
  if (sendZero) publishOnce(0, 0);
}

function activateButton(btn, on) {
  btn.classList.toggle("is-active", on);
}

function bindHold(btn, getCmd) {
  const onPress = (evt) => {
    evt.preventDefault();
    const { linearX, angularZ } = getCmd();
    activateButton(btn, true);
    startPublishing(linearX, angularZ);
  };
  const onRelease = (evt) => {
    evt.preventDefault();
    activateButton(btn, false);
    stopPublishing(true);
  };

  btn.addEventListener("pointerdown", onPress);
  btn.addEventListener("pointerup", onRelease);
  btn.addEventListener("pointercancel", onRelease);
  btn.addEventListener("pointerleave", onRelease);
}

function cmds() {
  const lin = Math.max(0, getNumber(els.lin, 0.25));
  const ang = Math.max(0, getNumber(els.ang, 0.8));
  return {
    forward: { linearX: +lin, angularZ: 0 },
    backward: { linearX: -lin, angularZ: 0 },
    left: { linearX: 0, angularZ: +ang },
    right: { linearX: 0, angularZ: -ang },
  };
}

function setup() {
  setConnected(false);

  // Default to the same host serving the UI, but rosbridge is on 9090.
  if (!els.wsUrl.value.trim()) {
    els.wsUrl.value = `ws://${window.location.hostname}:9090`;
  }

  els.btnConnect.addEventListener("click", connectOrDisconnect);
  els.btnStop.addEventListener("click", () => stopPublishing(true));

  bindHold(els.btnUp, () => cmds().forward);
  bindHold(els.btnDown, () => cmds().backward);
  bindHold(els.btnLeft, () => cmds().left);
  bindHold(els.btnRight, () => cmds().right);

  window.addEventListener("keydown", (e) => {
    if (e.repeat) return;
    if (e.code === "Space") {
      e.preventDefault();
      stopPublishing(true);
      return;
    }
    const map = {
      KeyW: els.btnUp,
      KeyS: els.btnDown,
      KeyA: els.btnLeft,
      KeyD: els.btnRight,
    };
    const btn = map[e.code];
    if (!btn) return;
    btn.dispatchEvent(new PointerEvent("pointerdown", { bubbles: true }));
  });

  window.addEventListener("keyup", (e) => {
    const map = {
      KeyW: els.btnUp,
      KeyS: els.btnDown,
      KeyA: els.btnLeft,
      KeyD: els.btnRight,
    };
    const btn = map[e.code];
    if (!btn) return;
    btn.dispatchEvent(new PointerEvent("pointerup", { bubbles: true }));
  });

  log("Ready. Click Connect to use rosbridge on port 9090.");
}

setup();

