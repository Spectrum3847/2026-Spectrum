/**
 * SVG Xbox-controller diagram plus the binding list beside it.
 *
 * Control geometry lives here; what each control DOES lives in data/controls.json. Hovering a
 * binding row lights up the matching control and vice versa, which is the whole point: a student
 * reading "LAUNCH_WITH_BRAKE" should be able to see instantly which finger does it.
 */
import { el } from "./ui.js";

const NS = "http://www.w3.org/2000/svg";

// x, y, and hit-shape for every control on a standard Xbox-style pad.
export const GEOMETRY = {
    LT: { x: 92, y: 26, w: 44, h: 30, rx: 14, label: "LT", kind: "trigger" },
    RT: { x: 384, y: 26, w: 44, h: 30, rx: 14, label: "RT", kind: "trigger" },
    LB: { x: 86, y: 66, w: 56, h: 20, rx: 10, label: "LB", kind: "bumper" },
    RB: { x: 378, y: 66, w: 56, h: 20, rx: 10, label: "RB", kind: "bumper" },
    leftStick: { cx: 130, cy: 152, r: 27, label: "L", kind: "stick" },
    rightStick: { cx: 320, cy: 210, r: 27, label: "R", kind: "stick" },
    lsClick: { cx: 130, cy: 152, r: 12, label: "L3", kind: "stick-click" },
    rsClick: { cx: 320, cy: 210, r: 12, label: "R3", kind: "stick-click" },
    dpadUp: { x: 190, y: 186, w: 22, h: 22, label: "▲", kind: "dpad" },
    dpadDown: { x: 190, y: 232, w: 22, h: 22, label: "▼", kind: "dpad" },
    dpadLeft: { x: 167, y: 209, w: 22, h: 22, label: "◀", kind: "dpad" },
    dpadRight: { x: 213, y: 209, w: 22, h: 22, label: "▶", kind: "dpad" },
    Y: { cx: 390, cy: 128, r: 16, label: "Y", kind: "face", color: "#e8c53a" },
    B: { cx: 418, cy: 156, r: 16, label: "B", kind: "face", color: "#e35d6a" },
    A: { cx: 390, cy: 184, r: 16, label: "A", kind: "face", color: "#3fbf7f" },
    X: { cx: 362, cy: 156, r: 16, label: "X", kind: "face", color: "#4f8ef7" },
    select: { cx: 232, cy: 140, r: 11, label: "⊟", kind: "small" },
    start: { cx: 288, cy: 140, r: 11, label: "≡", kind: "small" },
};

const BODY_PATH =
    "M 160 78 C 126 78 100 96 86 132 C 66 182 54 238 60 272 C 66 302 94 312 118 296 " +
    "C 144 278 170 264 202 260 L 318 260 C 350 264 376 278 402 296 C 426 312 454 302 460 272 " +
    "C 466 238 454 182 434 132 C 420 96 394 78 360 78 Z";

function svg(tag, attrs = {}, ...kids) {
    const n = document.createElementNS(NS, tag);
    for (const [k, v] of Object.entries(attrs)) if (v !== null && v !== undefined) n.setAttribute(k, String(v));
    for (const c of kids.flat()) if (c) n.append(c);
    return n;
}

function controlShape(id, g, state) {
    const cls = `ctl ctl-${state}`;
    const common = { class: cls, "data-control": id, tabindex: "0" };
    if (g.r !== undefined) return svg("circle", { ...common, cx: g.cx, cy: g.cy, r: g.r });
    return svg("rect", { ...common, x: g.x, y: g.y, width: g.w, height: g.h, rx: g.rx ?? 4 });
}

function controlLabel(id, g) {
    const x = g.cx ?? g.x + g.w / 2;
    const y = (g.cy ?? g.y + g.h / 2) + (g.kind === "dpad" ? 4 : 5);
    return svg("text", { class: "ctl-label", x, y, "text-anchor": "middle", "pointer-events": "none" }, document.createTextNode(g.label));
}

/**
 * @param {object} controller  one entry from controls.json
 * @param {string} layerId     which layer to highlight
 */
export function renderDiagram(controller, layerId) {
    const layer = controller.layers.find((l) => l.id === layerId) || controller.layers[0];
    const bound = new Set(layer.bindings.map((b) => b.control));
    const modifiers = new Set();
    // A layer named "LB held" means LB itself is the modifier; show it as engaged.
    if (layer.id === "lb-held") modifiers.add("LB");
    if (layer.id === "lt-held") modifiers.add("LT");
    const everBound = new Set(controller.layers.flatMap((l) => l.bindings.map((b) => b.control)));
    for (const s of controller.sticks) everBound.add(s.control);

    const shapes = [];
    const labels = [];
    for (const [id, g] of Object.entries(GEOMETRY)) {
        if (id === "lsClick" || id === "rsClick") continue; // drawn as the inner hub below
        const state = modifiers.has(id) ? "modifier" : bound.has(id) ? "bound" : everBound.has(id) ? "other" : "unbound";
        shapes.push(controlShape(id, g, state));
        labels.push(controlLabel(id, g));
    }
    // Stick hubs sit on top of the stick wells.
    for (const id of ["lsClick", "rsClick"]) {
        const g = GEOMETRY[id];
        shapes.push(svg("circle", { class: `ctl ctl-${bound.has(id) ? "bound" : everBound.has(id) ? "other" : "unbound"}`, "data-control": id, cx: g.cx, cy: g.cy, r: g.r }));
    }

    const root = svg(
        "svg",
        { viewBox: "0 0 520 320", class: "pad", role: "img", "aria-label": `${controller.name} controller diagram` },
        svg("path", { d: BODY_PATH, class: "pad-body" }),
        shapes,
        labels
    );
    return root;
}

export function renderBindingList(controller, layerId, onHover) {
    const layer = controller.layers.find((l) => l.id === layerId) || controller.layers[0];
    if (!layer.bindings.length) return el("div", { class: "empty" }, "No bindings in this layer.");

    const edgeText = { onTrue: "press", onFalse: "release", whileTrue: "hold", held: "hold" };

    return el(
        "div",
        { class: "binding-list" },
        layer.bindings.map((b) => {
            const g = GEOMETRY[b.control];
            const row = el(
                "div",
                {
                    class: "binding",
                    "data-control": b.control,
                    onmouseenter: () => onHover(b.control),
                    onmouseleave: () => onHover(null),
                },
                el("div", { class: "binding-key" }, g?.label ?? b.control, el("span", { class: "binding-edge" }, edgeText[b.edge] || b.edge)),
                el(
                    "div",
                    { class: "binding-body" },
                    el("div", { class: "binding-action" }, b.action, b.important ? el("span", { class: "tag warn", style: "margin-left:7px" }, "read this") : null),
                    b.detail ? el("div", { class: "binding-detail" }, b.detail) : null,
                    el(
                        "div",
                        { class: "binding-meta" },
                        b.state ? el("code", {}, b.state) : null,
                        b.conditional ? el("span", { class: "tag", title: "Behaviour depends on a Commands.either condition, not just the trigger" }, "conditional") : null,
                        b.special === "supplier" ? el("span", { class: "tag", title: "Passed as a live BooleanSupplier, not an edge binding" }, "live condition") : null,
                        el("span", { class: "binding-src" }, `Robot.java:${b.line}`)
                    )
                )
            );
            return row;
        })
    );
}
