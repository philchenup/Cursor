(function () {
  "use strict";

  const consoleEl = document.getElementById("console");
  const clockEl = document.getElementById("clock");
  const modal = document.getElementById("modal");
  const dialogTitle = document.getElementById("dialogTitle");
  const dialogBody = document.getElementById("dialogBody");

  function now() {
    return new Date().toLocaleTimeString("en-GB", { hour12: false });
  }

  function log(msg, cls) {
    const line = document.createElement("div");
    line.innerHTML = `<span class="ts">[${now()}]</span><span class="${cls || "act"}">${msg}</span>`;
    consoleEl.appendChild(line);
    consoleEl.scrollTop = consoleEl.scrollHeight;
  }

  log("NexusVIT started!", "ok");
  log("Web UI 皮肤已加载 · 原功能入口全部保留", "ok");

  setInterval(() => {
    clockEl.textContent = now();
  }, 1000);
  clockEl.textContent = now();

  document.querySelectorAll(".menu").forEach((menu) => {
    menu.addEventListener("click", (e) => {
      e.stopPropagation();
      const open = menu.classList.contains("open");
      document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
      if (!open) menu.classList.add("open");
    });
  });
  document.addEventListener("click", () => {
    document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
  });

  const dialogs = {
    tcp: {
      title: "TCP 标定",
      html: "<p>保持原 TCP 标定流程。请将工具尖点贴合标定球，采集 4 组以上姿态后计算法兰到 TCP 的变换。</p><p>当前 TCP：X 0.000  Y 0.000  Z 185.0 mm</p>",
    },
    handeye: {
      title: "手眼标定",
      html: "<p>眼在手上标定。采集标定板位姿与机器人法兰位姿，求解相机到法兰的刚体变换。入口与原软件一致。</p>",
    },
    library: {
      title: "工件库",
      html: '<div class="grid-cards"><button class="wp on">法兰盘 A</button><button class="wp">轴承座</button><button class="wp">端盖</button><button class="wp">连杆</button><button class="wp">阀体</button><button class="wp">自定义…</button></div><p>选择抓取模板，不改变后端匹配与抓取规划接口。</p>',
    },
    vision: {
      title: "视觉配置",
      html: "<p>加载无序抓取视觉工程：曝光、ROI、点云滤波、模板匹配阈值。配置文件路径与原 Vision Config 一致。</p>",
    },
    place: {
      title: "摆放设置",
      html: "<p>PlaceConfig：码放起点、行列间距、层高。保存后仍由原摆放逻辑执行。</p>",
    },
    about: {
      title: "关于 NexusVIT",
      html: "<p>NexusVIT Unordered Grasp 的 Web 工作台皮肤。所有按钮、输入与原 Qt 界面一一对应，仅替换布局与视觉语言，不改动控制、视觉、通信功能。</p>",
    },
  };

  function openDialog(key) {
    const d = dialogs[key];
    if (!d) return;
    dialogTitle.textContent = d.title;
    dialogBody.innerHTML = d.html;
    modal.classList.add("open");
    dialogBody.querySelectorAll(".wp").forEach((el) => {
      el.addEventListener("click", () => {
        dialogBody.querySelectorAll(".wp").forEach((x) => x.classList.remove("on"));
        el.classList.add("on");
        log("工件库选择 " + el.textContent, "ok");
      });
    });
  }

  document.getElementById("dialogCancel").onclick = () => modal.classList.remove("open");
  document.getElementById("dialogOk").onclick = () => {
    log(dialogTitle.textContent + " 确认", "ok");
    modal.classList.remove("open");
  };
  modal.addEventListener("click", (e) => {
    if (e.target === modal) modal.classList.remove("open");
  });

  document.querySelectorAll("[data-action]").forEach((el) => {
    el.addEventListener("click", (e) => {
      e.stopPropagation();
      log(el.dataset.action);
      if (el.dataset.dialog) openDialog(el.dataset.dialog);
    });
  });

  const viewport = document.getElementById("viewport");
  document.getElementById("tabScene").onclick = () => {
    viewport.classList.remove("cam");
    document.getElementById("tabScene").classList.add("on");
    document.getElementById("tabCamera").classList.remove("on");
    log("Scene");
  };
  document.getElementById("tabCamera").onclick = () => {
    viewport.classList.add("cam");
    document.getElementById("tabCamera").classList.add("on");
    document.getElementById("tabScene").classList.remove("on");
    log("Camera");
  };

  document.querySelectorAll(".deck-tab").forEach((tab) => {
    tab.onclick = () => {
      document.querySelectorAll(".deck-tab").forEach((t) => t.classList.remove("on"));
      document.querySelectorAll(".panel").forEach((p) => p.classList.remove("on"));
      tab.classList.add("on");
      document.getElementById("panel-" + tab.dataset.panel).classList.add("on");
      log("切换面板 " + tab.textContent.trim());
    };
  });

  const deck = document.getElementById("deck");
  function toggleDeck() {
    deck.classList.toggle("collapsed");
    document.getElementById("toggleDeck").textContent = deck.classList.contains("collapsed") ? "▴ 展开" : "▾ 折叠";
  }
  document.getElementById("toggleDeck").onclick = toggleDeck;
  document.getElementById("toggleDeckMenu").onclick = toggleDeck;

  const workspace = document.getElementById("workspace");
  let leftOn = true;
  let rightOn = true;
  document.getElementById("toggleLeft").onclick = () => {
    leftOn = !leftOn;
    document.getElementById("leftPane").style.display = leftOn ? "" : "none";
    workspace.style.gridTemplateColumns = `${leftOn ? "248px" : "0"} minmax(0,1fr) ${rightOn ? "318px" : "0"}`;
    if (window.robotScene) window.robotScene.resize();
  };
  document.getElementById("toggleRight").onclick = () => {
    rightOn = !rightOn;
    document.getElementById("rightPane").style.display = rightOn ? "" : "none";
    workspace.style.gridTemplateColumns = `${leftOn ? "248px" : "0"} minmax(0,1fr) ${rightOn ? "318px" : "0"}`;
    if (window.robotScene) window.robotScene.resize();
  };

  function bindToggle(btnId, pillId, labels) {
    const btn = document.getElementById(btnId);
    btn.addEventListener("click", () => {
      const on = btn.classList.toggle("on");
      btn.textContent = on ? labels[0] : labels[1];
      const pill = document.getElementById(pillId);
      pill.classList.toggle("on", on);
      pill.childNodes[1].textContent = on ? " " + labels[2] : " " + labels[3];
      log(on ? labels[2] : labels[3], "ok");
    });
  }
  bindToggle("robotConnectBtn", "pillRobot", ["Connected", "Connect", "机器人 已连接", "机器人 待机"]);
  bindToggle("cam_btn_connect", "pillVision", ["Connected", "Connect", "视觉 已连接", "视觉 待机"]);
  bindToggle("connectLaserBtn", "pillScrew", ["Connected", "Connect", "工具 已连接", "工具 待机"]);
  bindToggle("connectCommBtn", "pillComm", ["Connected", "Connect", "通信 监听中", "通信 待机"]);

  document.getElementById("robotEnableBtn").addEventListener("click", function () {
    this.classList.toggle("on");
    this.textContent = this.classList.contains("on") ? "Enabled" : "Enable";
    log(this.classList.contains("on") ? "Enable" : "Disable", "ok");
  });

  document.getElementById("sendCommBtn").addEventListener("click", () => {
    const msg = document.getElementById("sendBox").value;
    document.getElementById("recvBox").value += `[${now()}] ${msg}\n`;
  });

  document.getElementById("clearConsole").addEventListener("click", () => {
    consoleEl.innerHTML = "";
    log("控制台已清除", "warn");
  });

  document.getElementById("robotSpeed").oninput = (e) => {
    document.getElementById("speedVal").textContent = e.target.value;
  };

  const jointNames = ["J0", "J1", "J2", "J3", "J4", "J5"];
  const cartNames = ["X", "Y", "Z", "Rx", "Ry", "Rz"];
  const jointInputs = [];
  const cartInputs = [];

  function fillAxes(container, names, store, xyz) {
    const el = document.getElementById(container);
    names.forEach((n, i) => {
      const wrap = document.createElement("div");
      wrap.className = "axis" + (xyz ? " " + n.toLowerCase().replace("r", "") : "");
      if (xyz && i < 3) wrap.className = "axis " + n.toLowerCase();
      wrap.innerHTML = `<span>${n}</span><input class="num" type="number" step="0.001" value="0.000">`;
      el.appendChild(wrap);
      store.push(wrap.querySelector("input"));
    });
  }
  fillAxes("joints", jointNames, jointInputs, false);
  fillAxes("cart", cartNames, cartInputs, true);

  const propMap = {
    PointCloud: { ID: "PointCloud", 类别: "Cloud", 长度: "1842", 分辨率: "0.50", 体积: "—" },
    SceneBody: { ID: "Workpiece", 类别: "AIS_Shape", 长度: "184.2", 分辨率: "0.50", 体积: "12.4" },
    Robot: { ID: "Robot", 类别: "JAKA Zu12", 长度: "1327 mm", 分辨率: "—", 体积: "—" },
    Bin: { ID: "PartsBin", 类别: "Bin", 长度: "420×320", 分辨率: "—", 体积: "料框" },
  };
  document.getElementById("tree").addEventListener("click", (e) => {
    const eye = e.target.closest(".eye");
    if (eye) {
      eye.classList.toggle("off");
      log((eye.classList.contains("off") ? "隐藏 " : "显示 ") + e.target.closest(".node").dataset.id, "warn");
      return;
    }
    const node = e.target.closest(".node");
    if (!node) return;
    document.querySelectorAll(".node").forEach((n) => n.classList.remove("sel"));
    node.classList.add("sel");
    const data = propMap[node.dataset.id];
    document.getElementById("props").innerHTML = Object.entries(data)
      .map(([k, v]) => `<tr><th>${k}</th><td>${v}</td></tr>`)
      .join("");
    log("选择 " + data.ID);
  });

  const canvas = document.getElementById("sceneCanvas");
  const scene = window.createRobotScene(canvas);
  window.robotScene = scene;

  function writeState(state) {
    state.joints.forEach((v, i) => {
      if (document.activeElement !== jointInputs[i]) jointInputs[i].value = v.toFixed(3);
    });
    const c = state.cart;
    const vals = [c.x, c.y, c.z, c.rx, c.ry, c.rz];
    vals.forEach((v, i) => {
      if (document.activeElement !== cartInputs[i]) cartInputs[i].value = v.toFixed(3);
    });
    document.getElementById("fpsChip").textContent = state.fps + " fps";
  }
  scene.onUpdate(writeState);

  jointInputs.forEach((input, i) => {
    input.addEventListener("change", () => {
      const next = scene.getJoints();
      next[i] = (parseFloat(input.value) || 0) * Math.PI / 180;
      scene.setJoints(next);
      log("点动 " + jointNames[i]);
    });
  });

  document.getElementById("GoHomBtn").addEventListener("click", () => scene.goHome());
  document.getElementById("GoCapBtn").addEventListener("click", () => scene.goCap());
  document.getElementById("setHomBtn").addEventListener("click", () => log("Home 点已记录", "ok"));
  document.getElementById("setCapBtn").addEventListener("click", () => log("Capture 点已记录", "ok"));
  document.getElementById("btnRun").addEventListener("click", () => {
    scene.play();
    document.getElementById("pillRobot").classList.add("warn");
  });
  document.getElementById("btnPause").addEventListener("click", () => {
    scene.pause();
    document.getElementById("pillRobot").classList.remove("warn");
  });
  document.getElementById("btnReset").addEventListener("click", () => {
    scene.reset();
    document.getElementById("graspCount").textContent = "3 件待抓";
    document.getElementById("pillRobot").classList.remove("warn");
  });
  document.getElementById("btnStep").addEventListener("click", () => scene.step());
  document.getElementById("cam_btn_capture").addEventListener("click", () => {
    viewport.classList.add("cam");
    document.getElementById("tabCamera").classList.add("on");
    document.getElementById("tabScene").classList.remove("on");
  });
  document.getElementById("AutoCalibBtn").addEventListener("click", () => {
    log("AutoCalib 开始采集…", "warn");
    scene.goCap();
    setTimeout(() => log("AutoCalib 完成（预览）", "ok"), 900);
  });

  let remaining = 3;
  document.getElementById("btnRun").addEventListener("click", () => {
    remaining = 3;
    const timer = setInterval(() => {
      remaining -= 1;
      document.getElementById("graspCount").textContent = remaining > 0 ? remaining + " 件待抓" : "料框已空";
      if (remaining <= 0) clearInterval(timer);
    }, 2600);
  });
})();
