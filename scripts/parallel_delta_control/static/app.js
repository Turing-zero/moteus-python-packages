(() => {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const DASH = "—";
  const ACTUAL_COLOR = "#3987e5";
  const TARGET_COLOR = "#d95926";

  const DEFAULT_CONFIG = {
    version: 1,
    mechanism: {
      H: 85, D: 80, e: 0, alpha1_deg: 45, alpha2_deg: 60,
      beta_deg: 45, h_mid: 15, d_stack: 7,
      column_radius: 14, gear_radius: 28, pinion_radius: 8,
      rod_diameter: 8, motor_radius: 14
    },
    transmission: {
      motor_ids: [1, 2, 3], directions: [-1, 1, 1],
      motor_reduction: 8, belt_reduction: 2.22
    },
    safety: {
      roll_limit_deg: 15, pitch_limit_deg: 15, yaw_limit_deg: 60,
      feedback_rpy_margin_deg: 10,
      motor_min_deg: [-1066, -1066, -1066],
      motor_max_deg: [1066, 1066, 1066], max_move_deg: 5000,
      velocity_limit_rev_s: 100, accel_limit_rev_s2: 100,
      maximum_torque_nm: 1, kp_scale: 1, kd_scale: 1,
      watchdog_timeout_s: 0.25, feedback_timeout_s: 0.5,
      cycle_timeout_s: 0.05, control_rate_hz: 100,
      state_publish_hz: 100, min_condition: 0.05,
      fk_tolerance: 1e-9, fk_max_iterations: 300,
      fk_path_step_deg: 2, max_pose_step_deg: 300, max_feedback_step_deg: 1000,
      command_complete_tol_deg: 1
    },
    calibration: {
      velocity_rev_s: 1.5, accel_rev_s2: 3, drive_current_a: 0.5,
      motor_kv: 127.2,
      firmware_max_current_a: 2, max_q_current_a: 6,
      stall_velocity_rev_s: 0.04, stall_position_tol_deg: 2,
      stable_s: 1, settle_s: 0.5, stage_timeout_s: 30, min_span_rev: 0.1,
      max_span_imbalance: 0.1, return_tol_deg: 2
    }
  };

  const CONFIG_SECTIONS = [
    {
      title: "机构几何", key: "mechanism", wide: true,
      description: "理想同轴 3-RRR 球面模型；长度单位均为 mm。",
      fields: [
        { name: "version", path: "version", label: "配置版本", type: "int", readonly: true, hint: "当前后端仅支持版本 1" },
        { name: "H", label: "球心高度 H", unit: "mm", min: 0.000001 },
        { name: "D", label: "任务球直径 D", unit: "mm", min: 0.000001 },
        { name: "e", label: "球心 X 偏移 e", unit: "mm", min: 0 },
        { name: "alpha1_deg", label: "近杆锥角 α₁", unit: "deg", min: 20, max: 160 },
        { name: "alpha2_deg", label: "远杆锥角 α₂", unit: "deg", min: 20, max: 160 },
        { name: "beta_deg", label: "平台安装角 β", unit: "deg", min: 15, max: 80 },
        { name: "h_mid", label: "中层轴线高度", unit: "mm", min: 0.000001 },
        { name: "d_stack", label: "层间高度差", unit: "mm", min: 0.000001 },
        { name: "column_radius", label: "套轴半径", unit: "mm", min: 0.000001 },
        { name: "gear_radius", label: "齿轮半径", unit: "mm", min: 0.000001 },
        { name: "pinion_radius", label: "小齿轮半径", unit: "mm", min: 0.000001 },
        { name: "rod_diameter", label: "连杆直径", unit: "mm", min: 0.000001 },
        { name: "motor_radius", label: "电机半径", unit: "mm", min: 0.000001 }
      ]
    },
    {
      title: "电机与传动", key: "transmission",
      description: "电机 ID 必须互不重复；方向仅允许 +1 或 −1。",
      fields: [
        { name: "motor_ids", label: "电机 ID", type: "triple-int", labels: ["电机 1", "电机 2", "电机 3"], min: 1, max: 126, step: 1 },
        { name: "directions", label: "电机方向", type: "triple-select", labels: ["电机 1", "电机 2", "电机 3"], options: [[1, "+1 正向"], [-1, "−1 反向"]] },
        { name: "motor_reduction", label: "电机减速比", min: 0.000001 },
        { name: "belt_reduction", label: "同步带减速比", min: 0.000001 }
      ]
    },
    {
      title: "安全、限位与解算", key: "safety", wide: true,
      description: "软限位必须包含 0°；可选增益或扭矩留空表示由驱动器使用默认值。",
      fields: [
        { name: "roll_limit_deg", label: "Roll 限位", unit: "±deg", min: 0.000001 },
        { name: "pitch_limit_deg", label: "Pitch 限位", unit: "±deg", min: 0.000001 },
        { name: "yaw_limit_deg", label: "Yaw 限位", unit: "±deg", min: 0.000001 },
        { name: "feedback_rpy_margin_deg", label: "反馈 RPY 边界容差", unit: "deg", min: 0, hint: "仅用于实际反馈超调；目标指令仍严格限位" },
        { name: "motor_min_deg", label: "电机软限位下限", type: "triple", unit: "deg", labels: ["电机 1", "电机 2", "电机 3"] },
        { name: "motor_max_deg", label: "电机软限位上限", type: "triple", unit: "deg", labels: ["电机 1", "电机 2", "电机 3"] },
        { name: "max_move_deg", label: "单次最大位移 max_move", unit: "deg", min: 0.000001 },
        { name: "velocity_limit_rev_s", label: "电机速度限制", unit: "rev/s", min: 0.000001 },
        { name: "accel_limit_rev_s2", label: "电机加速度限制", unit: "rev/s²", min: 0.000001 },
        { name: "maximum_torque_nm", label: "最大扭矩", unit: "N·m", min: 0.000001, nullable: true, hint: "可选，留空为 null" },
        { name: "kp_scale", label: "位置增益 kp_scale", min: 0, nullable: true, hint: "可选，留空为 null" },
        { name: "kd_scale", label: "阻尼增益 kd_scale", min: 0, nullable: true, hint: "可选，留空为 null" },
        { name: "watchdog_timeout_s", label: "指令看门狗", unit: "s", min: 0.000001 },
        { name: "feedback_timeout_s", label: "反馈超时", unit: "s", min: 0.000001 },
        { name: "cycle_timeout_s", label: "循环超时", unit: "s", min: 0.000001 },
        { name: "control_rate_hz", label: "控制频率", unit: "Hz", min: 0.000001 },
        { name: "state_publish_hz", label: "状态发布频率", unit: "Hz", min: 0.000001 },
        { name: "min_condition", label: "最小条件数阈值", min: 0, max: 1 },
        { name: "fk_tolerance", label: "FK 收敛容差", min: 0.000000000000001, step: "any" },
        { name: "fk_max_iterations", label: "FK 最大迭代次数", type: "int", min: 1, step: 1 },
        { name: "fk_path_step_deg", label: "FK 路径步长", unit: "deg", min: 0.000001 },
        { name: "max_pose_step_deg", label: "最大姿态分支跳变", unit: "deg", min: 0.000001 },
        { name: "max_feedback_step_deg", label: "最大反馈跳变", unit: "deg", min: 0.000001 },
        { name: "command_complete_tol_deg", label: "轨迹完成容差", unit: "deg", min: 0.000001 }
      ]
    },
    {
      title: "极限位置标定", key: "calibration", wide: true,
      description: "使用固件纯速度模式，按电机 Kv 将工作电流换算为扭矩上限；102 仅表示达到预期限幅，其他故障仍中止。",
      fields: [
        { name: "velocity_rev_s", label: "标定速度", unit: "rev/s", min: 0.000001 },
        { name: "accel_rev_s2", label: "标定加速度", unit: "rev/s²", min: 0.000001 },
        { name: "drive_current_a", label: "标定工作电流", unit: "A", min: 0.000001 },
        { name: "motor_kv", label: "电机 Kv", unit: "rpm/V", min: 0.000001, nullable: true, hint: "必填真实值；用于把工作电流换算为 moteus 最大扭矩" },
        { name: "firmware_max_current_a", label: "固件 servo.max_current_A", unit: "A", min: 0.000001, nullable: true, hint: "必填真实值；用于计算 ilimit_scale，留空禁止标定" },
        { name: "max_q_current_a", label: "独立过流保护阈值", unit: "A", min: 0.000001, hint: "必须大于工作电流，例如工作 1 A、保护 2 A" },
        { name: "stall_velocity_rev_s", label: "堵转速度阈值", unit: "rev/s", min: 0.000001 },
        { name: "stall_position_tol_deg", label: "稳定位置漂移阈值", unit: "deg", min: 0.000001 },
        { name: "stable_s", label: "稳定采样时长", unit: "s", min: 0.000001 },
        { name: "settle_s", label: "STOPPED 复位时长", unit: "s", min: 0.000001 },
        { name: "stage_timeout_s", label: "单阶段超时", unit: "s", min: 0.000001 },
        { name: "min_span_rev", label: "最小有效行程", unit: "rev", min: 0.000001 },
        { name: "max_span_imbalance", label: "三轴行程最大相对差异", min: 0.000001, max: 1 },
        { name: "return_tol_deg", label: "归零完成容差", unit: "deg", min: 0.000001 }
      ]
    }
  ];

  let latestState = null;
  let configDraft = clone(DEFAULT_CONFIG);
  let configDirty = false;
  let configSubmitting = false;
  let lastConfigSignature = "";
  let socket = null;
  let reconnectTimer = null;
  let pageActive = true;
  let stateRequestInFlight = false;
  let toastTimer = null;
  const actionPending = new Set();

  function clone(value) {
    return JSON.parse(JSON.stringify(value));
  }

  function isObject(value) {
    return value !== null && typeof value === "object" && !Array.isArray(value);
  }

  function mergeConfig(base, update) {
    const result = clone(base);
    if (!isObject(update)) return result;
    Object.keys(result).forEach((key) => {
      if (isObject(result[key]) && isObject(update[key])) {
        result[key] = mergeConfig(result[key], update[key]);
      } else if (Object.prototype.hasOwnProperty.call(update, key)) {
        result[key] = clone(update[key]);
      }
    });
    return result;
  }

  function finite(value) {
    return typeof value === "number" && Number.isFinite(value);
  }

  function numberValue(value) {
    if (value === null || value === undefined || value === "") return null;
    const result = Number(value);
    return Number.isFinite(result) ? result : null;
  }

  function formatNumber(value, digits = 2) {
    const number = numberValue(value);
    if (number === null) return DASH;
    if (Math.abs(number) !== 0 && (Math.abs(number) >= 1e7 || Math.abs(number) < 1e-5)) {
      return number.toExponential(3);
    }
    return number.toLocaleString("zh-CN", { maximumFractionDigits: digits });
  }

  function formatValue(value) {
    if (value === null || value === undefined || value === "") return DASH;
    if (typeof value === "boolean") return value ? "是" : "否";
    if (typeof value === "object") {
      try { return JSON.stringify(value); } catch (_) { return String(value); }
    }
    return String(value);
  }

  function setText(id, value) {
    const element = $(id);
    if (element) element.textContent = value;
  }

  function setBadge(element, text, style = "neutral") {
    if (!element) return;
    element.textContent = text;
    element.className = `badge badge-${style}`;
  }

  function showToast(message) {
    const toast = $("toast");
    toast.textContent = message;
    toast.hidden = false;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { toast.hidden = true; }, 3200);
  }

  function reportError(context, error) {
    const message = error instanceof Error ? error.message : formatValue(error);
    const fullMessage = `${context}：${message}`;
    $("latestAlert").textContent = fullMessage;
    $("latestAlert").hidden = false;
    $("errorPanel").hidden = false;
    const item = document.createElement("li");
    const time = document.createElement("span");
    time.className = "error-time";
    time.textContent = new Date().toLocaleTimeString("zh-CN", { hour12: false });
    item.append(time, document.createTextNode(fullMessage));
    $("errorList").prepend(item);
    while ($("errorList").children.length > 30) $("errorList").lastElementChild.remove();
  }

  function clearErrors() {
    $("errorList").replaceChildren();
    $("errorPanel").hidden = true;
    $("latestAlert").hidden = true;
    $("latestAlert").textContent = "";
  }

  function errorDetail(payload, status) {
    if (payload && Object.prototype.hasOwnProperty.call(payload, "detail")) {
      const detail = payload.detail;
      if (Array.isArray(detail)) {
        return detail.map((entry) => {
          if (isObject(entry)) {
            const location = Array.isArray(entry.loc) ? entry.loc.join(".") + ": " : "";
            return location + (entry.msg || JSON.stringify(entry));
          }
          return String(entry);
        }).join("；");
      }
      return typeof detail === "string" ? detail : JSON.stringify(detail);
    }
    return `HTTP ${status}`;
  }

  async function api(path, options = {}) {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), options.timeout || 10000);
    const fetchOptions = { method: options.method || "GET", signal: controller.signal, headers: {} };
    if (options.body !== undefined) {
      fetchOptions.headers["Content-Type"] = "application/json";
      fetchOptions.body = JSON.stringify(options.body);
    }
    try {
      const response = await fetch(path, fetchOptions);
      const text = await response.text();
      let payload = null;
      if (text) {
        try { payload = JSON.parse(text); } catch (_) { payload = { detail: text }; }
      }
      if (!response.ok) throw new Error(errorDetail(payload, response.status));
      return payload;
    } catch (error) {
      if (error && error.name === "AbortError") throw new Error("请求超时（10 秒）");
      if (error instanceof TypeError) throw new Error("无法连接后端或网络已中断");
      throw error;
    } finally {
      clearTimeout(timeout);
    }
  }

  function consumeResponse(payload) {
    if (!isObject(payload)) return false;
    if (isObject(payload.state) && Array.isArray(payload.state.motors)) {
      applyState(payload.state);
      return true;
    }
    if (Array.isArray(payload.motors) && isObject(payload.config)) {
      applyState(payload);
      return true;
    }
    if (isObject(payload.config)) {
      if (!configDirty) populateConfig(payload.config);
      return false;
    }
    return false;
  }

  async function refreshState() {
    if (stateRequestInFlight) return;
    stateRequestInFlight = true;
    try {
      const state = await api("/api/state");
      if (!isObject(state)) throw new Error("状态响应不是对象");
      applyState(state);
    } catch (error) {
      reportError("获取状态失败", error);
    } finally {
      stateRequestInFlight = false;
    }
  }

  function updateConnection(connected) {
    const badge = $("connectionBadge");
    if (connected) {
      badge.className = "badge badge-good";
      badge.textContent = "● 实时已连接";
    } else {
      badge.className = "badge badge-warning";
      badge.textContent = pageActive ? "● 已断线 · 2 秒重连" : "● 页面已关闭";
    }
  }

  function connectWebSocket() {
    if (!pageActive || (socket && (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING))) return;
    clearTimeout(reconnectTimer);
    const protocol = location.protocol === "https:" ? "wss:" : "ws:";
    try {
      socket = new WebSocket(`${protocol}//${location.host}/api/ws`);
    } catch (error) {
      updateConnection(false);
      reconnectTimer = setTimeout(connectWebSocket, 2000);
      return;
    }
    socket.addEventListener("open", () => updateConnection(true));
    socket.addEventListener("message", (event) => {
      try {
        const state = JSON.parse(event.data);
        if (!isObject(state)) throw new Error("WebSocket 状态不是对象");
        if (isObject(state.state) && Array.isArray(state.state.motors)) applyState(state.state);
        else applyState(state);
      } catch (error) {
        reportError("解析实时状态失败", error);
      }
    });
    socket.addEventListener("close", () => {
      updateConnection(false);
      if (pageActive) {
        clearTimeout(reconnectTimer);
        reconnectTimer = setTimeout(connectWebSocket, 2000);
      }
    });
    socket.addEventListener("error", () => updateConnection(false));
  }

  function stateLabel(value) {
    const labels = {
      idle: "空闲", ready: "就绪", running: "运行中", moving: "运动中",
      holding: "保持", stopped: "已停止", calibrating: "标定中", uncalibrated: "未标定",
      fault: "故障", disabled: "未使能", initializing: "初始化"
    };
    return labels[value] || formatValue(value);
  }

  function updateStatusCards(state) {
    const isHardware = state.mode === "hardware";
    setText("modeStatus", state.mode === "simulation" ? "模拟" : isHardware ? "实物" : DASH);
    setText("modeSubstatus", state.mode === "simulation" ? "仿真后端" : isHardware ? "真实硬件 · 谨慎操作" : "未知模式");
    setText("controlStatus", stateLabel(state.state));
    setText("enabledStatus", state.enabled === true ? "已使能" : state.enabled === false ? "未使能" : "使能状态未知");
    setText("calibrationStatus", state.state === "calibrating" ? "极限标定中" : state.calibrated === true ? "已标定" : state.calibrated === false ? "未标定" : DASH);
    const fault = state.fault !== null && state.fault !== undefined && state.fault !== false && state.fault !== "";
    setText("faultStatus", fault ? "有故障" : state.fault === null || state.fault === false || state.fault === "" ? "无故障" : DASH);
    setText("faultDetail", fault ? formatValue(state.fault) : fault === false ? "无活动故障" : "等待故障信息");

    const cards = document.querySelectorAll(".status-card");
    cards.forEach((card) => card.classList.remove("status-good", "status-warning", "status-critical"));
    cards[0].classList.add(isHardware ? "status-warning" : state.mode === "simulation" ? "status-good" : "status-warning");
    cards[1].classList.add(state.enabled === true && !fault ? "status-good" : fault ? "status-critical" : "status-warning");
    cards[2].classList.add(state.calibrated === true ? "status-good" : "status-warning");
    cards[3].classList.add(fault ? "status-critical" : state.fault === undefined ? "status-warning" : "status-good");

    const message = formatValue(state.message);
    $("stateMessage").textContent = message === DASH ? (fault ? formatValue(state.fault) : "后端未提供状态说明。") : message;
    $("stateMessage").classList.toggle("fault", fault);
  }

  function commandIsAllowed() {
    if (!latestState) return false;
    const noFault = latestState.fault === null || latestState.fault === undefined || latestState.fault === false || latestState.fault === "";
    return latestState.enabled === true && latestState.calibrated === true && noFault;
  }

  function updateActionAvailability() {
    const hasState = Boolean(latestState);
    const allowed = commandIsAllowed();
    $("sendMotorCommand").disabled = !allowed || actionPending.has("command");
    $("sendRpyCommand").disabled = !allowed || actionPending.has("command");
    $("calibrateButton").disabled = !hasState || latestState.state === "calibrating" || actionPending.has("calibrate");
    $("extremeCalibrateButton").disabled = !hasState || latestState.enabled === true || latestState.state === "calibrating" || actionPending.has("extreme-calibrate");
    $("holdButton").disabled = !hasState || actionPending.has("hold");
    $("clearButton").disabled = !hasState || actionPending.has("clear");
    $("stopButton").disabled = false;
    const reason = !hasState ? "等待状态" : latestState.enabled !== true ? "未使能" : latestState.calibrated !== true ? "未标定" : !allowed ? "存在故障" : "";
    $("sendMotorCommand").title = reason ? `不可发送：${reason}` : "发送三个相对电机角";
    $("sendRpyCommand").title = reason ? `不可发送：${reason}` : "发送 Roll / Pitch / Yaw 目标";
  }

  function makeCell(text, className = "") {
    const cell = document.createElement("td");
    cell.textContent = text;
    if (className) cell.className = className;
    if (text === DASH) cell.classList.add("invalid-value");
    return cell;
  }

  function addPoseRow(body, label, target, actual, unit = "") {
    const row = document.createElement("tr");
    const heading = document.createElement("th");
    heading.scope = "row";
    heading.textContent = label;
    row.append(heading, makeCell(target), makeCell(actual), makeCell(unit));
    body.append(row);
  }

  function addPoseGroup(body, label) {
    const row = document.createElement("tr");
    row.className = "row-group";
    const cell = document.createElement("td");
    cell.colSpan = 4;
    cell.textContent = label;
    row.append(cell);
    body.append(row);
  }

  function poseValidText(pose) {
    if (!isObject(pose) || typeof pose.valid !== "boolean") return DASH;
    return pose.valid ? "有效" : "无效";
  }

  function updatePoseTable(state) {
    const target = isObject(state.target) ? state.target : {};
    const actual = isObject(state.actual) ? state.actual : {};
    const body = $("poseTableBody");
    body.replaceChildren();
    addPoseRow(body, "位姿有效性", poseValidText(target), poseValidText(actual));
    const groups = [
      ["相对电机角", "motor_deg", "deg", "电机"],
      ["分支角 q", "q_deg", "deg", "q"],
      ["平台 RPY", "rpy_deg", "deg", "RPY"]
    ];
    groups.forEach(([title, key, unit, prefix]) => {
      addPoseGroup(body, title);
      for (let i = 0; i < 3; i += 1) {
        const label = key === "rpy_deg" ? ["Roll", "Pitch", "Yaw"][i] : `${prefix} ${i + 1}`;
        addPoseRow(body, label, formatNumber(target[key] && target[key][i], 3), formatNumber(actual[key] && actual[key][i], 3), unit);
      }
    });
    addPoseGroup(body, "解算质量");
    addPoseRow(body, "条件数 condition", formatNumber(target.condition, 6), formatNumber(actual.condition, 6));
    addPoseRow(body, "残差 residual", formatNumber(target.residual, 8), formatNumber(actual.residual, 8));
    setText("targetQuality", `${formatNumber(target.condition, 6)} / ${formatNumber(target.residual, 8)}`);
    setText("actualQuality", `${formatNumber(actual.condition, 6)} / ${formatNumber(actual.residual, 8)}`);
  }

  function updateMotorTable(state) {
    const body = $("motorFeedbackBody");
    body.replaceChildren();
    if (!Array.isArray(state.motors) || state.motors.length === 0) {
      const row = document.createElement("tr");
      const cell = makeCell("暂无电机反馈", "empty-cell");
      cell.colSpan = 16;
      row.append(cell);
      body.append(row);
      return;
    }
    state.motors.forEach((motor) => {
      const item = isObject(motor) ? motor : {};
      const row = document.createElement("tr");
      row.append(makeCell(formatValue(item.id)));
      const online = document.createElement("td");
      const dot = document.createElement("span");
      dot.className = `online-dot${item.online === true ? " online" : ""}`;
      dot.setAttribute("aria-hidden", "true");
      online.append(dot, document.createTextNode(item.online === true ? "在线" : item.online === false ? "离线" : DASH));
      row.append(online);
      const mode = item.mode_name !== null && item.mode_name !== undefined
        ? `${formatValue(item.mode_name)} (${formatValue(item.mode)})` : formatValue(item.mode);
      row.append(makeCell(mode));
      const numericFields = [
        ["raw_position_rev", 6], ["raw_target_rev", 6],
        ["motor_position_deg", 3], ["motor_target_deg", 3],
        ["shaft_position_deg", 3], ["shaft_target_deg", 3],
        ["velocity_rev_s", 4], ["torque_nm", 4], ["voltage", 2],
        ["temperature", 1], ["age_ms", 1]
      ];
      numericFields.forEach(([key, digits]) => row.append(makeCell(formatNumber(item[key], digits))));
      row.append(makeCell(item.trajectory_complete === true ? "✓ 是" : item.trajectory_complete === false ? "✗ 否" : DASH));
      const fault = makeCell(formatValue(item.fault), "motor-fault");
      if (item.fault === 102) row.classList.add("has-warning");
      else if (item.fault) row.classList.add("has-fault");
      row.append(fault);
      body.append(row);
    });
  }

  function updateBackend(state) {
    const backend = isObject(state.backend) ? state.backend : {};
    setText("loopHz", finite(numberValue(backend.loop_hz)) ? `${formatNumber(backend.loop_hz, 2)} Hz` : DASH);
    setText("cycleMs", finite(numberValue(backend.cycle_ms)) ? `${formatNumber(backend.cycle_ms, 3)} ms` : DASH);
    setText("timeoutCount", formatValue(backend.timeouts));
    setText("lastUpdate", `更新于 ${new Date().toLocaleTimeString("zh-CN", { hour12: false })}`);
    const config = isObject(state.config) ? state.config : configDraft;
    setText("conditionThreshold", formatNumber(config.safety && config.safety.min_condition, 6));
  }

  function applyState(state) {
    latestState = state;
    updateStatusCards(state);
    updatePoseTable(state);
    updateMotorTable(state);
    updateBackend(state);
    if (isObject(state.config)) {
      const signature = JSON.stringify(state.config);
      if (signature !== lastConfigSignature) {
        lastConfigSignature = signature;
        if (!configDirty && !configSubmitting) populateConfig(state.config);
      }
    }
    updateActionAvailability();
    scheduleDraw();
  }

  function getPath(object, path) {
    return path.split(".").reduce((value, key) => value === null || value === undefined ? undefined : value[key], object);
  }

  function setPath(object, path, value) {
    const keys = path.split(".");
    const last = keys.pop();
    let cursor = object;
    keys.forEach((key) => { cursor = cursor[key]; });
    cursor[last] = value;
  }

  function makeConfigInput(field, path, label) {
    let input;
    if (field.type === "triple-select") {
      input = document.createElement("select");
      field.options.forEach(([value, text]) => {
        const option = document.createElement("option");
        option.value = String(value);
        option.textContent = text;
        input.append(option);
      });
      input.setAttribute("aria-label", `${field.label} ${label}`);
    } else {
      input = document.createElement("input");
      input.type = "number";
      input.step = field.step || "any";
      input.inputMode = "decimal";
      if (field.min !== undefined) input.min = String(field.min);
      if (field.max !== undefined) input.max = String(field.max);
      if (field.readonly) input.readOnly = true;
      if (!field.nullable && !field.readonly) input.required = true;
      if (field.nullable) input.placeholder = "默认（null）";
      input.setAttribute("aria-label", `${field.label}${label ? " " + label : ""}`);
    }
    input.dataset.configPath = path;
    input.dataset.configType = field.type || "number";
    input.dataset.nullable = field.nullable ? "true" : "false";
    input.dataset.label = `${field.label}${label ? " " + label : ""}`;
    return input;
  }

  function buildConfigForm() {
    const container = $("configSections");
    container.replaceChildren();
    CONFIG_SECTIONS.forEach((section) => {
      const article = document.createElement("section");
      article.className = `config-section${section.wide ? " section-wide" : ""}`;
      const header = document.createElement("div");
      header.className = "config-section-header";
      const title = document.createElement("h3");
      title.textContent = section.title;
      const description = document.createElement("p");
      description.textContent = section.description;
      header.append(title, description);
      const fields = document.createElement("div");
      fields.className = "config-fields";
      section.fields.forEach((field) => {
        const basePath = field.path || `${section.key}.${field.name}`;
        if (field.type === "triple" || field.type === "triple-int" || field.type === "triple-select") {
          const wrapper = document.createElement("div");
          wrapper.className = "config-field triple-field";
          const groupLabel = document.createElement("span");
          groupLabel.textContent = `${field.label}${field.unit ? ` (${field.unit})` : ""}`;
          const inputs = document.createElement("div");
          inputs.className = "triple-inputs";
          for (let i = 0; i < 3; i += 1) {
            const label = document.createElement("label");
            label.textContent = field.labels[i];
            label.append(makeConfigInput(field, `${basePath}.${i}`, field.labels[i]));
            inputs.append(label);
          }
          wrapper.append(groupLabel, inputs);
          fields.append(wrapper);
        } else {
          const label = document.createElement("label");
          label.className = "config-field";
          const labelLine = document.createElement("span");
          labelLine.className = "field-label";
          const name = document.createElement("span");
          name.textContent = field.label;
          labelLine.append(name);
          if (field.unit) {
            const unit = document.createElement("span");
            unit.className = "field-unit";
            unit.textContent = field.unit;
            labelLine.append(unit);
          }
          const hint = document.createElement("small");
          hint.textContent = field.hint || field.name;
          label.append(labelLine, makeConfigInput(field, basePath, ""), hint);
          fields.append(label);
        }
      });
      article.append(header, fields);
      container.append(article);
    });
    populateConfig(configDraft);
  }

  function populateConfig(config) {
    configDraft = mergeConfig(DEFAULT_CONFIG, config);
    document.querySelectorAll("[data-config-path]").forEach((input) => {
      const value = getPath(configDraft, input.dataset.configPath);
      input.value = value === null || value === undefined ? "" : String(value);
    });
    configDirty = false;
    updateConfigDirtyUI();
    $("configFormError").hidden = true;
  }

  function updateConfigDirtyUI() {
    $("configDirtyDot").hidden = !configDirty;
    setBadge($("configStatus"), configSubmitting ? "正在提交" : configDirty ? "有未应用修改" : latestState ? "与后端同步" : "本地默认配置",
      configDirty ? "warning" : latestState ? "good" : "neutral");
  }

  function collectConfig() {
    const result = clone(configDraft || DEFAULT_CONFIG);
    const errors = [];
    document.querySelectorAll("[data-config-path]").forEach((input) => {
      const path = input.dataset.configPath;
      const type = input.dataset.configType;
      const nullable = input.dataset.nullable === "true";
      const label = input.dataset.label;
      const raw = input.value.trim();
      if (raw === "") {
        if (nullable) setPath(result, path, null);
        else errors.push(`${label} 不能为空`);
        return;
      }
      const value = Number(raw);
      if (!Number.isFinite(value)) {
        errors.push(`${label} 必须是有限数`);
        return;
      }
      if ((type === "int" || type === "triple-int" || type === "triple-select") && !Number.isInteger(value)) {
        errors.push(`${label} 必须是整数`);
      }
      if (input.min !== "" && value < Number(input.min)) errors.push(`${label} 不能小于 ${input.min}`);
      if (input.max !== "" && value > Number(input.max)) errors.push(`${label} 不能大于 ${input.max}`);
      setPath(result, path, value);
    });
    for (let i = 0; i < 3; i += 1) {
      const lo = result.safety.motor_min_deg[i];
      const hi = result.safety.motor_max_deg[i];
      if (finite(lo) && finite(hi) && (lo >= hi || lo > 0 || hi < 0)) {
        errors.push(`电机 ${i + 1} 软限位必须满足 下限 ≤ 0 ≤ 上限，且下限小于上限`);
      }
    }
    if (new Set(result.transmission.motor_ids).size !== 3) errors.push("三个电机 ID 必须互不重复");
    if (errors.length) throw new Error(errors.join("\n"));
    return result;
  }

  async function submitConfig(save) {
    if (configSubmitting) return;
    let config;
    try {
      config = collectConfig();
    } catch (error) {
      $("configFormError").textContent = error.message;
      $("configFormError").hidden = false;
      reportError("配置校验失败", error);
      return;
    }
    configSubmitting = true;
    updateConfigDirtyUI();
    $("applyConfigButton").disabled = true;
    $("saveConfigButton").disabled = true;
    $("resetConfigButton").disabled = true;
    try {
      const payload = await api(`/api/config?save=${save ? "true" : "false"}`, { method: "POST", body: config });
      configDraft = clone(config);
      configDirty = false;
      lastConfigSignature = JSON.stringify(config);
      populateConfig(config);
      consumeResponse(payload);
      showToast(save ? "配置已应用并保存" : "配置已应用（未保存）");
      await refreshState();
    } catch (error) {
      reportError(save ? "保存配置失败" : "应用配置失败", error);
      $("configFormError").textContent = error.message;
      $("configFormError").hidden = false;
    } finally {
      configSubmitting = false;
      $("applyConfigButton").disabled = false;
      $("saveConfigButton").disabled = false;
      $("resetConfigButton").disabled = false;
      updateConfigDirtyUI();
    }
  }

  async function runAction(name, path, successMessage, confirmation) {
    if (name !== "stop" && actionPending.has(name)) return;
    if (confirmation && !window.confirm(confirmation)) return;
    actionPending.add(name);
    updateActionAvailability();
    try {
      const payload = await api(path, { method: "POST" });
      consumeResponse(payload);
      showToast(successMessage);
      await refreshState();
    } catch (error) {
      reportError(`${successMessage}失败`, error);
    } finally {
      actionPending.delete(name);
      updateActionAvailability();
    }
  }

  function setCommandMode(mode) {
    const motor = mode === "motor";
    $("motorCommandForm").hidden = !motor;
    $("rpyCommandForm").hidden = motor;
    $("motorModeButton").classList.toggle("active", motor);
    $("rpyModeButton").classList.toggle("active", !motor);
    $("motorModeButton").setAttribute("aria-pressed", String(motor));
    $("rpyModeButton").setAttribute("aria-pressed", String(!motor));
  }

  async function sendCommand(kind) {
    if (!commandIsAllowed()) {
      reportError("指令已阻止", new Error("后端未使能、未标定、存在故障或状态未知"));
      return;
    }
    const isMotor = kind === "motor";
    const form = $(isMotor ? "motorCommandForm" : "rpyCommandForm");
    if (!form.reportValidity()) return;
    const values = [0, 1, 2].map((i) => Number($(isMotor ? `motorCommand${i}` : `rpyCommand${i}`).value));
    const duration = Number($(isMotor ? "motorCommandDuration" : "rpyCommandDuration").value);
    if (values.some((value) => !Number.isFinite(value)) || !Number.isFinite(duration) || duration <= 0) {
      reportError("指令校验失败", new Error("三个指令值必须是有限数，到达时间必须是正数"));
      return;
    }
    if (!isMotor && (Math.abs(values[0]) > 15 || Math.abs(values[1]) > 15 || Math.abs(values[2]) > 60)) {
      reportError("指令校验失败", new Error("RPY 超出 ±15° / ±15° / ±60° 范围"));
      return;
    }
    actionPending.add("command");
    updateActionAvailability();
    try {
      const path = isMotor ? "/api/command/motors" : "/api/command/rpy";
      const body = isMotor ? { motor_deg: values, duration_s: duration } : { rpy_deg: values, duration_s: duration };
      const payload = await api(path, { method: "POST", body });
      consumeResponse(payload);
      showToast(`${isMotor ? "正解" : "逆解"}目标已发送，规划时间 ${formatNumber(duration, 2)} 秒`);
    } catch (error) {
      reportError(isMotor ? "发送正解目标失败" : "发送逆解目标失败", error);
    } finally {
      actionPending.delete("command");
      updateActionAvailability();
    }
  }

  // Canvas 2D orthographic orbit view. All geometry remains in world millimetres, Z up.
  const canvas = $("mechanismCanvas");
  const context = canvas.getContext("2d");
  const camera = { azimuth: -0.72, elevation: 0.48, zoom: 1 };
  const requestedView = new URLSearchParams(location.search).get("view");
  if (requestedView === "xy" || requestedView === "x-up") {
    camera.azimuth = requestedView === "x-up" ? Math.PI : -Math.PI / 2;
    camera.elevation = Math.PI / 2;
  }
  let canvasWidth = 800;
  let canvasHeight = 500;
  let drawRequested = false;
  let drag = null;
  let viewBasis = null;
  let viewCenter = [0, 0, 75];
  let viewScale = 2;

  function vector3(value) {
    if (!Array.isArray(value) || value.length !== 3) return null;
    const result = value.map((component) => {
      if (component === null || component === undefined || component === "" || typeof component === "boolean") return NaN;
      return Number(component);
    });
    return result.every(Number.isFinite) ? result : null;
  }

  function matrix3(value) {
    if (!Array.isArray(value) || value.length !== 3) return null;
    const columns = [0, 1, 2].map((column) => vector3([value[0] && value[0][column], value[1] && value[1][column], value[2] && value[2][column]]));
    return columns.every(Boolean) ? columns : null;
  }

  function pointRows(value) {
    if (!Array.isArray(value) || value.length !== 3) return null;
    const rows = value.map(vector3);
    return rows.every(Boolean) ? rows : null;
  }

  function normalizedGeometry(pose) {
    if (!isObject(pose) || pose.valid !== true || !isObject(pose.geometry)) return null;
    const geometry = pose.geometry;
    const normalized = {
      C: vector3(geometry.C), P: vector3(geometry.P), R: matrix3(geometry.R),
      Bu: pointRows(geometry.Bu), Bw: pointRows(geometry.Bw), Bv: pointRows(geometry.Bv),
      roots: pointRows(geometry.roots),
      heights: vector3(geometry.heights)
    };
    const poseFieldsValid = normalized.C && normalized.P && normalized.R && normalized.Bu && normalized.Bw && normalized.Bv && normalized.roots && normalized.heights;
    return poseFieldsValid ? normalized : null;
  }

  function subtract(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function scaleVector(vector, scale) { return vector.map((value) => value * scale); }
  function addVectors(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }

  function project(point) {
    const relative = subtract(point, viewCenter);
    return {
      x: canvasWidth * 0.5 + dot(relative, viewBasis.right) * viewScale,
      y: canvasHeight * 0.52 - dot(relative, viewBasis.up) * viewScale,
      depth: dot(relative, viewBasis.forward)
    };
  }

  function pathPolygon(points) {
    if (!points.length) return;
    context.beginPath();
    context.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i += 1) context.lineTo(points[i].x, points[i].y);
    context.closePath();
  }

  function strokeLine(points, color, width = 1, dash = [], alpha = 1) {
    if (!points.length) return;
    context.save();
    context.globalAlpha = alpha;
    context.strokeStyle = color;
    context.lineWidth = width;
    context.lineCap = "round";
    context.lineJoin = "round";
    context.setLineDash(dash);
    context.beginPath();
    context.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i += 1) context.lineTo(points[i].x, points[i].y);
    context.stroke();
    context.restore();
  }

  function addLinePrimitive(primitives, a, b, color, width, options = {}) {
    const pa = project(a);
    const pb = project(b);
    primitives.push({
      depth: (pa.depth + pb.depth) * 0.5 + (options.depthBias || 0),
      draw: () => {
        const alpha = options.alpha === undefined ? 1 : options.alpha;
        const dash = options.dash || [];
        if (options.outline !== false) strokeLine([pa, pb], "#07100d", width + 2.4, dash, alpha * 0.75);
        strokeLine([pa, pb], color, width, dash, alpha);
      }
    });
  }

  function addPointPrimitive(primitives, point, color, radius, options = {}) {
    const projected = project(point);
    primitives.push({
      depth: projected.depth + 0.01,
      draw: () => {
        context.save();
        context.globalAlpha = options.alpha === undefined ? 1 : options.alpha;
        context.fillStyle = color;
        context.strokeStyle = "#07100d";
        context.lineWidth = 1.5;
        context.beginPath();
        context.arc(projected.x, projected.y, radius, 0, Math.PI * 2);
        context.fill();
        context.stroke();
        if (options.label) {
          context.globalAlpha = 1;
          context.fillStyle = "#c3c2b7";
          context.font = "11px system-ui, sans-serif";
          context.textBaseline = "middle";
          context.fillText(options.label, projected.x + radius + 4, projected.y - 1);
        }
        context.restore();
      }
    });
  }

  function addCylinder(primitives, center, radius, z0, z1, fill, stroke) {
    if (!vector3(center) || !finite(radius) || !finite(z0) || !finite(z1) || radius <= 0 || z1 <= z0) return;
    const segments = 20;
    const lower = [];
    const upper = [];
    for (let i = 0; i < segments; i += 1) {
      const angle = i / segments * Math.PI * 2;
      const x = center[0] + Math.cos(angle) * radius;
      const y = center[1] + Math.sin(angle) * radius;
      lower.push(project([x, y, z0]));
      upper.push(project([x, y, z1]));
    }
    for (let i = 0; i < segments; i += 1) {
      const next = (i + 1) % segments;
      const points = [lower[i], lower[next], upper[next], upper[i]];
      const lightness = 20 + Math.round((Math.cos(i / segments * Math.PI * 2 - camera.azimuth) + 1) * 4);
      primitives.push({
        depth: points.reduce((sum, point) => sum + point.depth, 0) / 4,
        draw: () => {
          pathPolygon(points);
          context.fillStyle = `hsl(155 10% ${lightness}%)`;
          context.fill();
          context.strokeStyle = stroke;
          context.lineWidth = 0.65;
          context.stroke();
        }
      });
    }
    primitives.push({
      depth: upper.reduce((sum, point) => sum + point.depth, 0) / segments + 0.001,
      draw: () => {
        pathPolygon(upper);
        context.fillStyle = fill;
        context.fill();
        context.strokeStyle = stroke;
        context.lineWidth = 1;
        context.stroke();
      }
    });
  }

  function drawGridAndAxes(config) {
    const mechanism = config.mechanism || DEFAULT_CONFIG.mechanism;
    const diameter = finite(numberValue(mechanism.D)) ? Number(mechanism.D) : 100;
    const extent = Math.max(60, diameter * 0.75);
    const divisions = 6;
    for (let i = -divisions; i <= divisions; i += 1) {
      const offset = extent * i / divisions;
      strokeLine([project([-extent, offset, 0]), project([extent, offset, 0])], "#24332d", i === 0 ? 1.2 : 0.7);
      strokeLine([project([offset, -extent, 0]), project([offset, extent, 0])], "#24332d", i === 0 ? 1.2 : 0.7);
    }
    const length = diameter * 0.52;
    const origin = [0, 0, 0];
    const axes = [
      { vector: [length, 0, 0], color: "#e66767", label: "X" },
      { vector: [0, length, 0], color: "#55bd78", label: "Y" },
      { vector: [0, 0, length], color: "#6da7ec", label: "Z" }
    ];
    axes.forEach((axis) => {
      const end = project(axis.vector);
      strokeLine([project(origin), end], axis.color, 1.7);
      context.fillStyle = axis.color;
      context.font = "bold 12px system-ui, sans-serif";
      context.fillText(axis.label, end.x + 5, end.y - 5);
    });
    context.fillStyle = "#78867f";
    context.font = "11px system-ui, sans-serif";
    context.fillText("Z ↑ / mm", 12, canvasHeight - 14);
  }

  function drawBase(primitives, config, actualGeometry) {
    const mechanism = config.mechanism || DEFAULT_CONFIG.mechanism;
    const gearRadius = numberValue(mechanism.gear_radius) || 28;
    const columnRadius = numberValue(mechanism.column_radius) || 14;
    const motorRadius = numberValue(mechanism.motor_radius) || 14;
    const axisX = numberValue(mechanism.e) || 0;
    addCylinder(primitives, [axisX, 0, 0], Math.max(gearRadius * 1.3, motorRadius * 1.5), -6, 0, "#34473f", "#52675d");

    const roots = actualGeometry && actualGeometry.roots;
    let heights = actualGeometry && actualGeometry.heights;
    if (!heights) {
      const mid = numberValue(mechanism.h_mid) || 28;
      const stack = numberValue(mechanism.d_stack) || 10;
      heights = [mid - stack, mid, mid + stack];
    }
    for (let i = 0; i < 3; i += 1) {
      const z = heights[i];
      if (!finite(z) || z <= 0) continue;
      addCylinder(primitives, [axisX, 0, 0], columnRadius, 0, z, "#3d5047", "#5f756a");
      addCylinder(primitives, [axisX, 0, 0], gearRadius, Math.max(0, z - 2.5), z + 2.5, "#455e52", "#70877a");
      if (roots && roots[i]) addPointPrimitive(primitives, roots[i], "#9aa9a1", 2.6, { label: `轴 ${i + 1}` });
    }
  }

  function addPlatform(primitives, geometry, color, alpha, dashed) {
    const points = geometry.Bv.map(project);
    const projectedC = project(geometry.C);
    const projectedP = project(geometry.P);
    primitives.push({
      depth: points.reduce((sum, point) => sum + point.depth, 0) / 3,
      draw: () => {
        context.save();
        context.globalAlpha = alpha * 0.32;
        pathPolygon(points);
        context.fillStyle = color;
        context.fill();
        context.restore();
      }
    });
    for (let i = 0; i < 3; i += 1) {
      addLinePrimitive(primitives, geometry.Bv[i], geometry.Bv[(i + 1) % 3], color, 2.2, { alpha, dash: dashed ? [7, 5] : [] });
    }
    addLinePrimitive(primitives, geometry.C, geometry.P, color, 1.2, { alpha: alpha * 0.75, dash: [3, 4], outline: false });
    addPointPrimitive(primitives, geometry.C, color, 4.5, { alpha, label: dashed ? "" : "球心 C" });
    addPointPrimitive(primitives, geometry.P, color, 4, { alpha, label: dashed ? "" : "平台 P" });
    // A small circle at C and the C-P line make the spherical centre explicit.
    void projectedC;
    void projectedP;
  }

  function addPose(primitives, geometry, kind) {
    const target = kind === "target";
    const color = target ? TARGET_COLOR : ACTUAL_COLOR;
    const alpha = target ? 0.43 : 1;
    const dash = target ? [7, 5] : [];
    for (let i = 0; i < 3; i += 1) {
      addLinePrimitive(primitives, geometry.Bu[i], geometry.Bw[i], color, target ? 5 : 6.2, { alpha, dash });
      addLinePrimitive(primitives, geometry.Bw[i], geometry.Bv[i], color, target ? 2.7 : 3.4, { alpha, dash });
      addLinePrimitive(primitives, geometry.roots ? geometry.roots[i] : geometry.Bu[i], geometry.Bu[i], color, 2.4, { alpha: alpha * 0.8, dash });
      addPointPrimitive(primitives, geometry.Bu[i], color, target ? 3.5 : 4.3, { alpha, label: target ? "" : `M${i + 1}` });
      addPointPrimitive(primitives, geometry.Bw[i], color, target ? 3 : 3.8, { alpha });
      addPointPrimitive(primitives, geometry.Bv[i], color, target ? 3 : 3.8, { alpha });
    }
    addPlatform(primitives, geometry, color, alpha, target);
    if (!target) {
      const labels = ["x", "y", "z"];
      const colors = ["#e66767", "#55bd78", "#6da7ec"];
      for (let i = 0; i < 3; i += 1) {
        const end = addVectors(geometry.P, scaleVector(geometry.R[i], 22));
        addLinePrimitive(primitives, geometry.P, end, colors[i], 1.4, { outline: false, depthBias: 0.02 });
        const p = project(end);
        primitives.push({
          depth: p.depth + 0.03,
          draw: () => {
            context.fillStyle = colors[i];
            context.font = "10px system-ui, sans-serif";
            context.fillText(labels[i], p.x + 3, p.y - 3);
          }
        });
      }
    }
  }

  function drawPrimitives(primitives) {
    primitives.sort((a, b) => a.depth - b.depth);
    primitives.forEach((primitive) => primitive.draw());
  }

  function updateGeometryNotices(actualGeometry, targetGeometry) {
    setText("actualGeometryStatus", `实际几何：${actualGeometry ? "有效" : "无效 / 已隐藏"}`);
    setText("targetGeometryStatus", `目标几何：${targetGeometry ? "有效" : "无效 / 已隐藏"}`);
    const invalid = [];
    if (!actualGeometry) invalid.push("实际几何无效");
    if (!targetGeometry) invalid.push("目标几何无效");
    const notice = $("canvasNotice");
    if (invalid.length) {
      notice.textContent = `${invalid.join("；")}，已隐藏且未归零`;
      notice.hidden = false;
    } else {
      notice.hidden = true;
    }
  }

  function drawScene() {
    drawRequested = false;
    if (!context || canvasWidth <= 0 || canvasHeight <= 0) return;
    context.setTransform(1, 0, 0, 1, 0, 0);
    context.clearRect(0, 0, canvas.width, canvas.height);
    const ratio = canvas.width / canvasWidth;
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.fillStyle = "#0c1211";
    context.fillRect(0, 0, canvasWidth, canvasHeight);

    const config = latestState && isObject(latestState.config) ? mergeConfig(DEFAULT_CONFIG, latestState.config) : configDraft;
    const mechanism = config.mechanism;
    const height = numberValue(mechanism.H) || 150;
    const diameter = numberValue(mechanism.D) || 100;
    viewCenter = [numberValue(mechanism.e) || 0, 0, height * 0.48];
    viewScale = Math.min(canvasWidth, canvasHeight) * 0.82 / Math.max(120, height + diameter * 0.65) * camera.zoom;
    const cosElevation = Math.cos(camera.elevation);
    viewBasis = {
      forward: [cosElevation * Math.cos(camera.azimuth), cosElevation * Math.sin(camera.azimuth), Math.sin(camera.elevation)],
      right: [-Math.sin(camera.azimuth), Math.cos(camera.azimuth), 0],
      up: [-Math.sin(camera.elevation) * Math.cos(camera.azimuth), -Math.sin(camera.elevation) * Math.sin(camera.azimuth), Math.cos(camera.elevation)]
    };

    const actualGeometry = latestState ? normalizedGeometry(latestState.actual) : null;
    const targetGeometry = latestState ? normalizedGeometry(latestState.target) : null;
    updateGeometryNotices(actualGeometry, targetGeometry);
    drawGridAndAxes(config);

    const basePrimitives = [];
    drawBase(basePrimitives, config, actualGeometry || targetGeometry);
    drawPrimitives(basePrimitives);
    if (actualGeometry) {
      const actualPrimitives = [];
      addPose(actualPrimitives, actualGeometry, "actual");
      drawPrimitives(actualPrimitives);
    }
    if (targetGeometry) {
      const targetPrimitives = [];
      addPose(targetPrimitives, targetGeometry, "target");
      drawPrimitives(targetPrimitives);
    }
  }

  function scheduleDraw() {
    if (drawRequested) return;
    drawRequested = true;
    requestAnimationFrame(drawScene);
  }

  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(1, Math.round(rect.width * ratio));
    const height = Math.max(1, Math.round(rect.height * ratio));
    canvasWidth = rect.width;
    canvasHeight = rect.height;
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    scheduleDraw();
  }

  function resetView() {
    camera.azimuth = -0.72;
    camera.elevation = 0.48;
    camera.zoom = 1;
    scheduleDraw();
  }

  function changeZoom(factor) {
    camera.zoom = Math.max(0.45, Math.min(3.5, camera.zoom * factor));
    scheduleDraw();
  }

  function bindCanvasEvents() {
    canvas.addEventListener("pointerdown", (event) => {
      if (event.button !== 0) return;
      drag = { x: event.clientX, y: event.clientY, id: event.pointerId };
      canvas.setPointerCapture(event.pointerId);
      canvas.classList.add("dragging");
    });
    canvas.addEventListener("pointermove", (event) => {
      if (!drag || drag.id !== event.pointerId) return;
      const dx = event.clientX - drag.x;
      const dy = event.clientY - drag.y;
      drag.x = event.clientX;
      drag.y = event.clientY;
      camera.azimuth -= dx * 0.008;
      camera.elevation = Math.max(-1.42, Math.min(1.52, camera.elevation + dy * 0.008));
      scheduleDraw();
    });
    const endDrag = (event) => {
      if (!drag || drag.id !== event.pointerId) return;
      drag = null;
      canvas.classList.remove("dragging");
      if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
    };
    canvas.addEventListener("pointerup", endDrag);
    canvas.addEventListener("pointercancel", endDrag);
    canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      changeZoom(Math.exp(-event.deltaY * 0.001));
    }, { passive: false });
    canvas.addEventListener("keydown", (event) => {
      let handled = true;
      if (event.key === "ArrowLeft") camera.azimuth += 0.09;
      else if (event.key === "ArrowRight") camera.azimuth -= 0.09;
      else if (event.key === "ArrowUp") camera.elevation = Math.min(1.52, camera.elevation + 0.07);
      else if (event.key === "ArrowDown") camera.elevation = Math.max(-1.42, camera.elevation - 0.07);
      else if (event.key === "+" || event.key === "=") changeZoom(1.12);
      else if (event.key === "-") changeZoom(1 / 1.12);
      else if (event.key === "Home") resetView();
      else handled = false;
      if (handled) {
        event.preventDefault();
        scheduleDraw();
      }
    });
    if ("ResizeObserver" in window) new ResizeObserver(resizeCanvas).observe($("canvasContainer"));
    else window.addEventListener("resize", resizeCanvas);
  }

  function switchTab(tab) {
    const control = tab === "control";
    $("controlPanel").hidden = !control;
    $("configPanel").hidden = control;
    $("controlTabButton").classList.toggle("active", control);
    $("configTabButton").classList.toggle("active", !control);
    $("controlTabButton").setAttribute("aria-selected", String(control));
    $("configTabButton").setAttribute("aria-selected", String(!control));
    if (control) requestAnimationFrame(resizeCanvas);
  }

  function bindEvents() {
    document.querySelectorAll("[data-tab]").forEach((button) => button.addEventListener("click", () => switchTab(button.dataset.tab)));
    $("stopButton").addEventListener("click", () => runAction("stop", "/api/stop", "停止指令已发送"));
    $("calibrateButton").addEventListener("click", () => runAction(
      "calibrate", "/api/calibrate", "标定指令已发送",
      "标定会使机构执行寻零或运动。\n\n请确认：机构周围无人、运动范围内无障碍、硬件急停可用。\n\n确定开始标定吗？"
    ));
    $("extremeCalibrateButton").addEventListener("click", () => runAction(
      "extreme-calibrate", "/api/calibrate/extreme", "极限标定已启动",
      "极限标定会让三个电机同时低速正转、反转到机械堵转，再计算中点并归零。\n\n即使限速限流，堵转仍可能过热或损坏机构。请确认：\n• 机械极限和堵转扭矩已经评估；\n• 固件电流限值与标定参数正确；\n• 周围无人、无障碍，机械支撑和独立硬件急停可用；\n• 出现异常会立即使用 Stop 或硬件急停。\n\n确定开始极限标定吗？"
    ));
    $("holdButton").addEventListener("click", () => runAction("hold", "/api/hold", "保持指令已发送"));
    $("clearButton").addEventListener("click", () => runAction("clear", "/api/clear", "故障清除指令已发送"));
    $("clearErrorsButton").addEventListener("click", clearErrors);
    $("refreshButton").addEventListener("click", refreshState);
    $("motorModeButton").addEventListener("click", () => setCommandMode("motor"));
    $("rpyModeButton").addEventListener("click", () => setCommandMode("rpy"));
    $("motorCommandForm").addEventListener("submit", (event) => { event.preventDefault(); sendCommand("motor"); });
    $("rpyCommandForm").addEventListener("submit", (event) => { event.preventDefault(); sendCommand("rpy"); });
    $("configForm").addEventListener("input", (event) => {
      if (!event.target.dataset.configPath || event.target.readOnly) return;
      configDirty = true;
      updateConfigDirtyUI();
    });
    $("configForm").addEventListener("change", (event) => {
      if (!event.target.dataset.configPath) return;
      configDirty = true;
      updateConfigDirtyUI();
    });
    $("configForm").addEventListener("submit", (event) => { event.preventDefault(); submitConfig(true); });
    $("applyConfigButton").addEventListener("click", () => submitConfig(false));
    $("resetConfigButton").addEventListener("click", async () => {
      if (configSubmitting) return;
      if (!window.confirm("恢复后端默认配置？\n\n当前未应用的修改也会被丢弃。该操作可能立即影响机构运动与限位。")) return;
      configSubmitting = true;
      updateConfigDirtyUI();
      $("applyConfigButton").disabled = true;
      $("saveConfigButton").disabled = true;
      $("resetConfigButton").disabled = true;
      try {
        const payload = await api("/api/config/reset", { method: "POST" });
        configDirty = false;
        consumeResponse(payload);
        if (!isObject(payload) || (!isObject(payload.config) && !isObject(payload.state))) populateConfig(DEFAULT_CONFIG);
        await refreshState();
        showToast("已恢复默认配置");
      } catch (error) {
        reportError("恢复默认配置失败", error);
      } finally {
        configSubmitting = false;
        $("applyConfigButton").disabled = false;
        $("saveConfigButton").disabled = false;
        $("resetConfigButton").disabled = false;
        updateConfigDirtyUI();
      }
    });
    $("topViewButton").addEventListener("click", () => { camera.azimuth = Math.PI; camera.elevation = Math.PI / 2; scheduleDraw(); });
    $("resetViewButton").addEventListener("click", resetView);
    $("zoomInButton").addEventListener("click", () => changeZoom(1.18));
    $("zoomOutButton").addEventListener("click", () => changeZoom(1 / 1.18));
    window.addEventListener("beforeunload", () => {
      pageActive = false;
      clearTimeout(reconnectTimer);
      if (socket) socket.close();
    });
  }

  buildConfigForm();
  bindEvents();
  bindCanvasEvents();
  updateActionAvailability();
  resizeCanvas();
  refreshState().finally(connectWebSocket);
})();
