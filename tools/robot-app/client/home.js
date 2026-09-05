import "./styles.css";
import { mountHeader, el, api } from "./lib/ui.js";

mountHeader();

const CARDS = [
    { href: "/pages/controls-pilot/", title: "Pilot controls", body: "Driver button map: the LT/RT launch chords, LB reorients, and the disabled-mode pit controls that live nowhere else." },
    { href: "/pages/controls-operator/", title: "Operator controls", body: "Hood and turret offset nudges, feed targeting, the feed override, and the turret zero button." },
    { href: "/pages/logs/", title: "Logs", body: "Find the robot, pull .wpilog files off it, and index them into the team logs repo." },
    { href: "/pages/power/", title: "Power", body: "Per-motor current against its configured limit, battery sag, energy per subsystem, and breaker trip simulation." },
    { href: "/pages/can/", title: "CAN Bus", body: "Bus utilization, error counters heading for bus-off, and motors that stopped answering mid-match." },
    { href: "/pages/swerve-align/", title: "Swerve Align", body: "Module alignment helper." },
];

document.getElementById("cards").append(
    ...CARDS.map((c) =>
        el("a", { href: c.href, class: "card", style: "color:inherit" },
            el("div", { style: "font-weight:650;margin-bottom:4px" }, c.title),
            el("div", { style: "color:var(--tx-dim);font-size:0.85rem" }, c.body))
    )
);

const probe = document.getElementById("probe");

async function showProbe() {
    try {
        const r = await api("/api/robot/probe");
        probe.replaceChildren(
            el("div", { class: "table-wrap" },
                el("table", {},
                    el("thead", {}, el("tr", {}, ["Address", "What it is", "Status", "Time"].map((h) => el("th", {}, h)))),
                    el("tbody", {}, r.candidates.map((c) =>
                        el("tr", {},
                            el("td", { class: "mono" }, c.host),
                            el("td", {}, c.label),
                            el("td", {}, el("span", { class: `tag ${c.up ? "ok" : ""}` }, c.up ? "reachable" : c.reason || "down")),
                            el("td", { class: "num" }, `${c.ms} ms`))))))
        );
        if (!r.best) {
            probe.append(el("div", { class: "notice warn" },
                el("strong", {}, "No robot found. "),
                "Check that the laptop is on the robot radio or plugged into the RIO by USB. Log sync needs a reachable robot; every other page works from logs already on disk."));
        }
    } catch (e) {
        probe.textContent = `Could not reach the app server: ${e.message}`;
    }
}

showProbe();
