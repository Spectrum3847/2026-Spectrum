import "../../styles.css";
import "./logs.css";
import { mountHeader, el, api, fmtBytes, fmtDuration, fmtDate } from "../../lib/ui.js";

mountHeader();
const root = document.getElementById("app");

const state = { config: null, host: null, robotLogs: null, localLogs: [], repo: null, selected: new Set(), busy: false };

const sections = {
    robot: el("section", {}),
    local: el("section", {}),
    repo: el("section", {}),
};

root.replaceChildren(
    el("h1", {}, "Logs"),
    el("p", { class: "lede" },
        "Pull ", el("code", {}, ".wpilog"), " files off the robot and index them into the team logs repo. ",
        "Indexing extracts the headline numbers into a manifest that is committed even when the log itself is not, so a season of log history stays in the repo without hundreds of megabytes of binaries."),
    sections.robot,
    sections.local,
    sections.repo
);

// ---------------------------------------------------------------- robot

function renderRobot() {
    const c = state.config;
    const body = [];

    if (!state.probe) {
        body.push(el("div", {}, el("span", { class: "spin" }), " Looking for the robot…"));
    } else if (!state.probe.reachable.length) {
        body.push(
            el("div", { class: "notice warn" },
                el("strong", {}, "No robot found. "),
                "Tried ", state.probe.candidates.map((x) => x.host).join(", "), ". ",
                "Get on the robot radio or plug into the RIO's USB port, then look again."),
            el("button", { onclick: refreshProbe }, "Look again")
        );
    } else {
        const opts = state.probe.reachable.map((r) =>
            el("option", { value: r.host, selected: r.host === state.host }, `${r.label} — ${r.host} (${r.ms} ms)`));
        body.push(
            el("div", { class: "row" },
                el("label", { style: "color:var(--tx-dim);font-size:0.84rem" }, "Robot"),
                el("select", { id: "host-select", onchange: (e) => { state.host = e.target.value; loadRobotLogs(); } }, opts),
                el("button", { onclick: refreshProbe }, "Re-probe"),
                el("div", { class: "spacer" }),
                el("button", { class: "primary", disabled: !state.selected.size || state.busy, onclick: sync },
                    state.busy ? "Syncing…" : `Sync ${state.selected.size || ""} selected`))
        );
        body.push(renderRobotLogs());
    }
    sections.robot.replaceChildren(el("h2", {}, "On the robot"), el("div", { class: "card" }, ...body));
}

function renderRobotLogs() {
    if (!state.robotLogs) return el("div", { class: "empty" }, el("span", { class: "spin" }), " Listing logs…");
    if (state.robotLogs.error) return el("div", { class: "notice bad" }, state.robotLogs.error);

    const localNames = new Set(state.localLogs.map((l) => l.name));
    const rows = [];
    for (const d of state.robotLogs.dirs) {
        if (!d.files.length) continue;
        rows.push(el("tr", { class: "dir-row" }, el("td", { colspan: 5 }, el("code", {}, d.dir), ` — ${d.files.length} log${d.files.length === 1 ? "" : "s"}`)));
        for (const f of d.files) {
            const have = localNames.has(f.name);
            const id = `${d.dir}/${f.name}`;
            rows.push(
                el("tr", { class: have ? "have" : null },
                    el("td", {}, el("input", {
                        type: "checkbox", checked: state.selected.has(id), disabled: have,
                        onchange: (e) => { e.target.checked ? state.selected.add(id) : state.selected.delete(id); renderRobot(); },
                    })),
                    el("td", { class: "mono" }, f.name),
                    el("td", { class: "num" }, fmtBytes(f.bytes)),
                    el("td", {}, fmtDate(f.mtime)),
                    el("td", {}, have ? el("span", { class: "tag ok" }, "already synced") : ""))
            );
        }
    }
    if (!rows.length) return el("div", { class: "empty" }, "No .wpilog files on the robot.");

    return el("div", { class: "table-wrap", style: "margin-top:14px" },
        el("table", {},
            el("thead", {}, el("tr", {}, ["", "File", "Size", "Modified", ""].map((h) => el("th", {}, h)))),
            el("tbody", {}, rows)));
}

async function refreshProbe() {
    state.probe = null;
    renderRobot();
    state.probe = await api("/api/robot/probe");
    state.host = state.probe.best?.host || null;
    renderRobot();
    if (state.host) loadRobotLogs();
}

async function loadRobotLogs() {
    state.robotLogs = null;
    state.selected.clear();
    renderRobot();
    try {
        state.robotLogs = await api(`/api/robot/logs?host=${encodeURIComponent(state.host)}`);
    } catch (e) {
        state.robotLogs = { error: `Could not list logs on ${state.host}: ${e.message}` };
    }
    renderRobot();
}

/** Sync streams newline-delimited JSON so a multi-minute transfer shows progress. */
async function sync() {
    const files = [];
    for (const d of state.robotLogs.dirs) for (const f of d.files) if (state.selected.has(`${d.dir}/${f.name}`)) files.push(f);
    if (!files.length) return;

    state.busy = true;
    renderRobot();
    const progress = el("div", { class: "progress" });
    sections.robot.querySelector(".card").append(progress);

    const res = await fetch("/api/robot/sync", {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify({ host: state.host, files }),
    });
    const reader = res.body.getReader();
    const decoder = new TextDecoder();
    let buf = "";
    const lines = new Map();

    const paint = () => progress.replaceChildren(...[...lines.values()]);
    while (true) {
        const { done, value } = await reader.read();
        if (done) break;
        buf += decoder.decode(value, { stream: true });
        const parts = buf.split("\n");
        buf = parts.pop();
        for (const part of parts) {
            if (!part.trim()) continue;
            const ev = JSON.parse(part);
            if (ev.state === "complete") {
                lines.set("__done", el("div", { class: "prog-line ok" }, `Synced ${ev.count} log${ev.count === 1 ? "" : "s"}.`));
            } else if (ev.state === "failed") {
                lines.set("__done", el("div", { class: "prog-line bad" }, `Sync failed: ${ev.error}`));
            } else {
                const label = { start: "downloading", done: "downloaded", indexing: "indexing", indexed: "indexed", "index-failed": "index failed" }[ev.state] || ev.state;
                lines.set(ev.name, el("div", { class: `prog-line${ev.state === "index-failed" ? " bad" : ev.state === "indexed" ? " ok" : ""}` },
                    el("span", { class: "mono" }, ev.name), " — ", label,
                    ev.ms ? ` in ${(ev.ms / 1000).toFixed(1)}s` : "",
                    ev.error ? ` (${ev.error})` : ""));
            }
            paint();
        }
    }
    state.busy = false;
    state.selected.clear();
    await Promise.all([loadLocal(), loadRepo()]);
    renderRobot();
}

// ---------------------------------------------------------------- local logs

function summaryCells(s) {
    if (!s) return [el("td", { colspan: 6, style: "color:var(--tx-faint)" }, "not indexed")];
    const warn = (v, lim) => (v === null || v === undefined ? "" : v >= lim ? "bad" : "");
    return [
        el("td", { class: "num" }, fmtDuration(s.enabledSec)),
        el("td", { class: `num ${s.battery && s.battery.minVolts < 7 ? "warn-text" : ""}` }, s.battery ? `${s.battery.minVolts} V` : "—"),
        el("td", { class: "num" }, s.current ? `${s.current.peakAmps} A` : "—"),
        el("td", { class: "num" }, s.energyWh !== null ? `${s.energyWh} Wh` : "—"),
        el("td", { class: `num ${warn(s.loop?.overrunPct, 5) === "bad" ? "warn-text" : ""}` }, s.loop ? `${s.loop.overrunPct}%` : "—"),
        el("td", { class: `num ${warn(s.can?.maxUtilPct, 60) === "bad" ? "warn-text" : ""}` }, s.can ? `${s.can.maxUtilPct}%` : "—"),
    ];
}

function renderLocal() {
    const body = [];
    if (!state.localLogs.length) {
        body.push(el("div", { class: "empty" }, "No logs synced yet. Pull some off the robot above."));
    } else {
        body.push(
            el("div", { class: "table-wrap" },
                el("table", {},
                    el("thead", {}, el("tr", {},
                        ["File", "Size", "Enabled", "Min V", "Peak A", "Energy", "Overrun", "CAN max", "", ""].map((h) => el("th", {}, h)))),
                    el("tbody", {}, state.localLogs.map((l) =>
                        el("tr", {},
                            el("td", { class: "mono" }, l.name, l.summary?.note ? el("div", { class: "note-text" }, l.summary.note) : null),
                            el("td", { class: "num" }, fmtBytes(l.bytes)),
                            ...summaryCells(l.summary),
                            el("td", {},
                                el("button", { class: "mini", title: "Keep this specific log in git, not just its numbers", onclick: () => togglePin(l) },
                                    l.summary?.pinned ? "📌 pinned" : "pin")),
                            el("td", {},
                                el("a", { class: "btn mini", href: `/pages/power/?log=${encodeURIComponent(l.name)}` }, "Power"),
                                el("a", { class: "btn mini", href: `/pages/can/?log=${encodeURIComponent(l.name)}`, style: "margin-left:5px" }, "CAN")))))))
        );
    }
    sections.local.replaceChildren(
        el("h2", {}, "Synced logs"),
        el("div", { class: "card" }, ...body),
        el("div", { class: "notice" },
            el("strong", {}, "Overrun "), "is the share of loops longer than 20 ms. ",
            el("strong", {}, "CAN max "), "is peak CANivore utilization; CTRE wants this well under 60%.")
    );
}

async function togglePin(l) {
    if (!l.summary) await api(`/api/logs/${encodeURIComponent(l.name)}/index`, { method: "POST", body: "{}" });
    await api(`/api/logs/${encodeURIComponent(l.name)}/pin`, { method: "POST", body: JSON.stringify({ pinned: !l.summary?.pinned }) });
    await loadLocal();
    await loadRepo();
    renderLocal();
    renderRepo();
}

async function loadLocal() {
    const r = await api("/api/logs");
    state.localLogs = r.logs;
    state.logsDir = r.dir;
    state.logsDirExists = r.exists;
    renderLocal();
}

// ---------------------------------------------------------------- repo

function renderRepo() {
    const r = state.repo;
    const c = state.config;
    const body = [];
    if (!r) {
        body.push(el("div", {}, el("span", { class: "spin" }), " Checking…"));
    } else if (!r.isRepo) {
        body.push(
            el("div", { class: "notice warn" },
                el("strong", {}, "The logs repo is not cloned yet. "),
                "Synced logs will still land in ", el("code", {}, state.logsDir || r.path), ", but nothing can be committed until you clone it:"),
            el("pre", { class: "cmd" }, `git clone ${c?.logsRepo?.remote || ""} ${r.path}`)
        );
    } else {
        const pinned = state.localLogs.filter((l) => l.summary?.pinned).length;
        body.push(
            el("div", { class: "grid cols-4" },
                el("div", { class: "stat" }, el("div", { class: "label" }, "Branch"), el("div", { class: "value", style: "font-size:1.05rem" }, r.branch)),
                el("div", { class: "stat" }, el("div", { class: "label" }, "Uncommitted"), el("div", { class: "value" }, r.dirtyFiles)),
                el("div", { class: "stat" }, el("div", { class: "label" }, "Unpushed"), el("div", { class: "value" }, r.unpushed ?? "—")),
                el("div", { class: "stat" }, el("div", { class: "label" }, "Pinned logs"), el("div", { class: "value" }, pinned))),
            el("div", { class: "row", style: "margin-top:14px" },
                el("button", { class: "primary", onclick: publish }, "Commit & push manifest"),
                el("span", { style: "color:var(--tx-faint);font-size:0.8rem" },
                    `Commits the manifest${pinned ? ` and ${pinned} pinned log${pinned === 1 ? "" : "s"}` : ""}. Unpinned .wpilog files stay out of git.`)),
            el("div", { id: "publish-result" })
        );
    }
    sections.repo.replaceChildren(el("h2", {}, "Logs repo"), el("div", { class: "card" }, ...body));
}

async function publish() {
    const out = document.getElementById("publish-result");
    out.replaceChildren(el("div", { style: "margin-top:10px" }, el("span", { class: "spin" }), " Publishing…"));
    try {
        const r = await api("/api/logs/publish", { method: "POST", body: JSON.stringify({}) });
        out.replaceChildren(el("div", { class: `notice ${r.ok ? "" : "bad"}` },
            r.noChanges ? "Nothing to commit — the manifest is already up to date."
                : r.ok ? `Committed${r.pushed ? ` and pushed to ${r.branch}` : ""}.`
                : `Failed: ${r.error}`));
    } catch (e) {
        out.replaceChildren(el("div", { class: "notice bad" }, e.message));
    }
    await loadRepo();
    renderRepo();
}

async function loadRepo() {
    state.repo = await api("/api/logs/repo-status");
    renderRepo();
}

// ---------------------------------------------------------------- boot

async function boot() {
    state.config = await api("/api/config");
    renderRobot();
    renderLocal();
    renderRepo();
    await loadLocal();
    await loadRepo();
    await refreshProbe();
}
boot();
