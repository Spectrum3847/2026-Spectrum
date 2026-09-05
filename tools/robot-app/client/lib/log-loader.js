/** Shared log picker: choose from synced logs or drop a file in. Used by Power and CAN. */
import { el, api, fmtBytes } from "./ui.js";
import { parseWpilog } from "./wpilog.js";
import { LogModel } from "./log-model.js";

export async function loadProfile() {
    try {
        return await api("/data/robot-profile.json");
    } catch {
        return null;
    }
}

export function logPicker({ onLoad, current }) {
    const select = el("select", { id: "log-select", style: "min-width:320px" }, el("option", { value: "" }, "Loading…"));
    const file = el("input", { type: "file", accept: ".wpilog", style: "display:none" });
    const status = el("span", { style: "color:var(--tx-faint);font-size:0.8rem" });

    const box = el("div", { class: "row" },
        el("label", { style: "color:var(--tx-dim);font-size:0.84rem" }, "Log"),
        select,
        el("button", { onclick: () => file.click() }, "Open a file…"),
        file,
        status);

    async function loadByName(name) {
        status.textContent = "downloading…";
        const buf = await fetch(`/api/logs/file/${encodeURIComponent(name)}`).then((r) => {
            if (!r.ok) throw new Error(`${r.status} fetching ${name}`);
            return r.arrayBuffer();
        });
        await handle(buf, name);
    }

    async function handle(buf, name) {
        status.textContent = "parsing…";
        await new Promise((r) => setTimeout(r, 0)); // let the status paint before we block
        const t0 = performance.now();
        const log = parseWpilog(buf);
        status.textContent = `${fmtBytes(buf.byteLength)} · ${log.durationSec.toFixed(0)}s · parsed in ${(performance.now() - t0).toFixed(0)} ms`;
        onLoad(log, name);
    }

    file.addEventListener("change", async () => {
        const f = file.files?.[0];
        if (!f) return;
        await handle(await f.arrayBuffer(), f.name);
    });
    select.addEventListener("change", () => {
        if (select.value) {
            history.replaceState(null, "", `?log=${encodeURIComponent(select.value)}`);
            loadByName(select.value);
        }
    });

    let lastList = [];
    (async () => {
        try {
            const r = await api("/api/logs");
            lastList = r.logs;
            const wanted = current || new URLSearchParams(location.search).get("log");
            if (!r.logs.length) {
                select.replaceChildren(el("option", { value: "" }, "no synced logs — sync some on the Logs page"));
                status.textContent = "";
                return;
            }
            select.replaceChildren(
                el("option", { value: "" }, "choose a log…"),
                ...r.logs.map((l) => el("option", { value: l.name, selected: l.name === wanted }, `${l.name}  (${fmtBytes(l.bytes)})`))
            );
        } catch (e) {
            select.replaceChildren(el("option", { value: "" }, `server error: ${e.message}`));
            return;
        }

        // Loading and rendering is deliberately outside the catch above. A page that throws while
        // rendering is not a server error, and reporting it as one in the log dropdown sends
        // whoever is debugging it looking in the wrong place.
        try {
            const wanted = current || new URLSearchParams(location.search).get("log");
            if (wanted && lastList.some((l) => l.name === wanted)) await loadByName(wanted);
            else status.textContent = "";
        } catch (e) {
            status.textContent = "";
            box.append(el("div", { class: "notice bad", style: "flex-basis:100%" },
                el("strong", {}, "This page failed to render that log. "), e.message));
            throw e;
        }
    })();

    return box;
}

export { LogModel };
