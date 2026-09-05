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

/**
 * Light is the brand default (the team site is white with a deep-purple nav). Dark is a second,
 * separately stepped palette for pit and queue-line use, remembered per browser.
 */
export function initTheme() {
    let stored = null;
    try {
        stored = localStorage.getItem("spectrum-theme");
    } catch {
        // Private browsing or blocked storage: fall through to the brand default.
    }
    document.documentElement.dataset.theme = stored === "dark" ? "dark" : "light";
}

export function toggleTheme() {
    const next = document.documentElement.dataset.theme === "dark" ? "light" : "dark";
    document.documentElement.dataset.theme = next;
    try {
        localStorage.setItem("spectrum-theme", next);
    } catch {
        // Not persisting the choice is survivable; failing the click is not.
    }
    document.dispatchEvent(new CustomEvent("themechange", { detail: { theme: next } }));
    const btn = document.querySelector(".theme-toggle");
    if (btn) {
        btn.textContent = next === "dark" ? "\u2600" : "\u263D";
        btn.title = next === "dark" ? "Switch to light theme" : "Switch to dark theme";
    }
}

// Applied before first paint so the page never flashes the wrong theme.
initTheme();

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
    const dark = document.documentElement.dataset.theme === "dark";
    const themeBtn = el(
        "button",
        {
            class: "theme-toggle",
            title: dark ? "Switch to light theme" : "Switch to dark theme",
            "aria-label": "Toggle colour theme",
            onclick: toggleTheme,
        },
        dark ? "\u2600" : "\u263D"
    );
    const header = el(
        "header",
        { class: "topbar" },
        el("div", { class: "brand" }, el("span", { class: "mark" }, "\u25C5 "), "Spectrum", el("span", { class: "sub" }, "2026 offseason bot")),
        tabs,
        pill,
        themeBtn
    );
    document.body.prepend(header);
    mountDriftBanner(header);
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

/**
 * Tells the reader when this app has fallen behind the robot code.
 *
 * The app documents a robot it cannot see change. Nothing stops someone renaming a binding or
 * moving a current limit, and every page here would go on confidently describing the old one --
 * which is worse than showing nothing, because a student has no reason to doubt it.
 *
 * So the app says so itself, rather than a build step refusing to compile robot code until a web
 * app is updated. Two strengths, deliberately worded differently: "problems" are contradictions
 * the checker actually parsed, "stale" only means the Java moved more recently than the data file,
 * which is a prompt to look, not proof of anything wrong.
 *
 * Failures here are swallowed on purpose: a drift banner is never worth breaking a page over.
 */
async function mountDriftBanner(header) {
    let r;
    try {
        const res = await fetch("/api/drift");
        if (!res.ok) return;
        r = await res.json();
    } catch {
        return;
    }
    const problems = r?.problems ?? [];
    const stale = r?.stale ?? [];
    if (!problems.length && !stale.length) return;

    const bad = problems.length > 0;
    const lines = [...problems, ...stale.map((s) => s.message)];
    const shown = lines.slice(0, 6);

    const details = el("div", { style: "margin-top:6px;display:none" },
        el("ul", { style: "margin:0;padding-left:18px" }, shown.map((t) => el("li", { style: "margin:2px 0" }, t))),
        lines.length > shown.length ? el("div", { style: "margin-top:4px" }, `…and ${lines.length - shown.length} more.`) : null,
        el("div", { style: "margin-top:8px" },
            "Re-run it yourself with ",
            el("code", {}, "node tools/robot-app/scripts/check-drift.mjs"),
            ". It blocks no build; update the files in ",
            el("code", {}, "tools/robot-app/data/"),
            " when convenient."));

    const toggle = el("a", { href: "#", style: "margin-left:6px" }, "show");
    toggle.addEventListener("click", (e) => {
        e.preventDefault();
        const open = details.style.display !== "none";
        details.style.display = open ? "none" : "block";
        toggle.textContent = open ? "show" : "hide";
    });

    const summary = bad
        ? `This app disagrees with the robot code in ${problems.length} place${problems.length === 1 ? "" : "s"}` +
          (stale.length ? `, and ${stale.length} of its data file${stale.length === 1 ? " is" : "s are"} behind the Java.` : ".")
        : `${stale.length} data file${stale.length === 1 ? "" : "s"} here ${stale.length === 1 ? "has" : "have"} not been updated since the robot code changed.`;

    const banner = el("div", { class: `notice ${bad ? "bad" : "warn"}`, style: "margin:0;border-radius:0" },
        el("strong", {}, bad ? "Out of date. " : "Possibly out of date. "),
        summary,
        " ",
        bad ? "Treat what you read here as suspect until it is checked." : "Worth a look before trusting the details.",
        toggle,
        details);

    header.after(banner);
}
