import "../styles.css";
import "./controls.css";
import { mountHeader, el, api } from "./ui.js";
import { renderDiagram, renderBindingList, GEOMETRY } from "./controller-diagram.js";

export async function mountControlsPage(controllerId) {
    mountHeader();
    const root = document.getElementById("app");

    let data;
    try {
        data = await api("/data/controls.json");
    } catch (e) {
        root.replaceChildren(el("div", { class: "notice bad" }, `Could not load controls.json: ${e.message}`));
        return;
    }
    const c = data.controllers.find((x) => x.id === controllerId);
    if (!c) {
        root.replaceChildren(el("div", { class: "notice bad" }, `No controller "${controllerId}" in controls.json`));
        return;
    }

    document.title = `${c.name} controls — Spectrum`;
    let layerId = new URLSearchParams(location.search).get("layer") || c.layers[0].id;

    const diagramBox = el("div", { class: "diagram-box" });
    const listBox = el("div", { class: "list-box" });
    const tabs = el("div", { class: "layer-tabs" });

    function highlight(control) {
        for (const node of diagramBox.querySelectorAll("[data-control]")) {
            node.classList.toggle("hover", control !== null && node.dataset.control === control);
        }
        for (const node of listBox.querySelectorAll(".binding")) {
            node.classList.toggle("hover", control !== null && node.dataset.control === control);
        }
    }

    function draw() {
        tabs.replaceChildren(
            ...c.layers.map((l) =>
                el(
                    "button",
                    {
                        class: `layer-tab${l.id === layerId ? " active" : ""}${l.pit ? " pit" : ""}${l.sim ? " sim" : ""}`,
                        onclick: () => {
                            layerId = l.id;
                            history.replaceState(null, "", `?layer=${l.id}`);
                            draw();
                        },
                    },
                    l.name,
                    el("span", { class: "layer-count" }, l.bindings.length)
                )
            )
        );

        const layer = c.layers.find((l) => l.id === layerId);
        const svg = renderDiagram(c, layerId);
        for (const node of svg.querySelectorAll("[data-control]")) {
            node.addEventListener("mouseenter", () => highlight(node.dataset.control));
            node.addEventListener("mouseleave", () => highlight(null));
        }
        diagramBox.replaceChildren(
            svg,
            el("div", { class: "when" }, layer.when),
            el(
                "div",
                { class: "legend" },
                el("span", {}, el("i", { class: "sw bound" }), "bound in this layer"),
                el("span", {}, el("i", { class: "sw modifier" }), "the modifier"),
                el("span", {}, el("i", { class: "sw other" }), "used in another layer"),
                el("span", {}, el("i", { class: "sw unbound" }), "unbound")
            )
        );
        listBox.replaceChildren(renderBindingList(c, layerId, highlight));
    }

    const sticks = c.sticks.length
        ? el(
              "section",
              {},
              el("h2", {}, "Sticks"),
              el(
                  "div",
                  { class: "card table-wrap" },
                  el(
                      "table",
                      {},
                      el("thead", {}, el("tr", {}, ["Stick", "Axis", "Does", "Method"].map((h) => el("th", {}, h)))),
                      el(
                          "tbody",
                          {},
                          c.sticks.map((s) =>
                              el(
                                  "tr",
                                  {},
                                  el("td", {}, GEOMETRY[s.control]?.label === "L" ? "Left" : "Right"),
                                  el("td", {}, s.axis),
                                  el("td", {}, s.action),
                                  el("td", { class: "mono", style: "color:var(--tx-faint)" }, `${s.method} :${s.line}`)
                              )
                          )
                      )
                  )
              )
          )
        : null;

    const unbound = c.unbound?.length
        ? el(
              "section",
              {},
              el("h2", {}, "Unused"),
              el(
                  "div",
                  { class: "card" },
                  el("div", { style: "color:var(--tx-dim);font-size:0.86rem;margin-bottom:8px" }, "Declared on the gamepad but bound to nothing. Free real estate."),
                  el("div", { class: "row" }, c.unbound.map((u) => el("span", { class: "tag" }, GEOMETRY[u]?.label ?? u)))
              )
          )
        : null;

    // replaceChildren stringifies null, so filter before spreading -- the operator has no sticks
    // section and an empty one must vanish, not print "null".
    const parts = [
        el("h1", {}, `${c.name} controls`),
        el(
            "p",
            { class: "lede" },
            `USB port ${c.port}. Source of truth is `,
            el("code", {}, "data/controls.json"),
            ", checked against ",
            el("code", {}, "Robot.configureBindings()"),
            " by a Gradle drift check."
        ),
        ...(c.notes || []).map((n) => el("div", { class: "notice" }, n)),
        tabs,
        el("div", { class: "controls-split" }, diagramBox, listBox),
        sticks,
        unbound,
    ];
    root.replaceChildren(...parts.filter(Boolean));
    draw();
}
