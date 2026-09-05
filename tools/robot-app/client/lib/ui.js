/** Shared UI helpers: the top bar, robot status pill, and small DOM conveniences. */

const PAGES = [
    { href: "/", label: "Home", match: (p) => p === "/" || p === "/index.html" },
    { href: "/pages/controls-pilot/", label: "Pilot" },
    { href: "/pages/controls-operator/", label: "Operator" },
    { href: "/pages/logs/", label: "Logs" },
    { href: "/pages/power/", label: "Power" },
    { href: "/pages/can/", label: "CAN Bus" },
    { href: "/pages/swerve-align/", label: "Swerve Align" },
];

export function el(tag, attrs = {}, ...children) {
    const node = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) {
        if (v === null || v === undefined || v === false) continue;
        if (k === "class") node.className = v;
        else if (k === "html") node.innerHTML = v;
        else if (k.startsWith("on") && typeof v === "function") node.addEventListener(k.slice(2), v);
        else node.setAttribute(k, v === true ? "" : String(v));
    }
    for (const c of children.flat()) {
        if (c === null || c === undefined || c === false) continue;
        node.append(c instanceof Node ? c : document.createTextNode(String(c)));
    }
    return node;
}

export function mountHeader() {
    const here = location.pathname.replace(/index\.html$/, "");
    const tabs = el(
        "nav",
        { class: "tabs" },
        PAGES.map((p) => {
            const active = p.match ? p.match(here) : here.startsWith(p.href);
            return el("a", { href: p.href, class: active ? "active" : null }, p.label);
        })
    );
    const pill = el("div", { class: "robot-pill", id: "robot-pill", title: "Robot connection" }, el("span", { class: "dot checking" }), el("span", { id: "robot-pill-text" }, "checking…"));
    const header = el(
        "header",
        { class: "topbar" },
        el("div", { class: "brand" }, el("span", { class: "mark" }, "◅ "), "Spectrum", el("span", { class: "sub" }, "2026 offseason bot")),
        tabs,
        pill
    );
    document.body.prepend(header);
    refreshRobotPill();
    return header;
}

let pillTimer = null;
export async function refreshRobotPill() {
    const dot = document.querySelector("#robot-pill .dot");
    const text = document.getElementById("robot-pill-text");
    if (!dot) return;
    try {
        const r = await fetch("/api/robot/probe").then((x) => x.json());
        if (r.best) {
            dot.className = "dot up";
            text.textContent = r.best.label;
            document.getElementById("robot-pill").title = `${r.best.host} responded in ${r.best.ms} ms`;
        } else {
            dot.className = "dot down";
            text.textContent = "no robot";
            document.getElementById("robot-pill").title = "No roboRIO answered on any known address";
        }
    } catch {
        dot.className = "dot down";
        text.textContent = "server?";
    }
    clearTimeout(pillTimer);
    pillTimer = setTimeout(refreshRobotPill, 15000);
}

export function fmtBytes(n) {
    if (n === null || n === undefined) return "—";
    if (n < 1024) return `${n} B`;
    if (n < 1024 ** 2) return `${(n / 1024).toFixed(0)} KB`;
    if (n < 1024 ** 3) return `${(n / 1024 ** 2).toFixed(1)} MB`;
    return `${(n / 1024 ** 3).toFixed(2)} GB`;
}

export function fmtDuration(sec) {
    if (sec === null || sec === undefined) return "—";
    if (sec < 60) return `${sec.toFixed(0)}s`;
    const m = Math.floor(sec / 60);
    return `${m}m ${Math.round(sec - m * 60)}s`;
}

export function fmtDate(ms) {
    if (!ms) return "—";
    return new Date(ms).toLocaleString(undefined, { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" });
}

export async function api(path, opts) {
    const res = await fetch(path, {
        ...opts,
        headers: opts?.body ? { "content-type": "application/json", ...(opts.headers || {}) } : opts?.headers,
    });
    const body = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(body.error || `${res.status} ${res.statusText}`);
    return body;
}
