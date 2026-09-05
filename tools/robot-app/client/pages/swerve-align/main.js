import "../../styles.css";
import { mountHeader, el } from "../../lib/ui.js";

/*
 * Placeholder for the swerve module alignment tool built in a separate session.
 *
 * To drop that tool in, replace this file and index.html with it. The page contract is documented
 * in tools/robot-app/README.md under "Adding a page"; the short version:
 *
 *   - Everything in this directory is one Vite entry point, discovered automatically. No build
 *     config to touch.
 *   - `import { mountHeader } from "../../lib/ui.js"` for the shared nav and robot-status pill.
 *   - `import "../../styles.css"` for the shell styling (cards, tables, stat tiles, tags).
 *   - The nav entry already exists in lib/ui.js.
 *   - Server endpoints: /api/robot/probe finds the RIO. For live module angles, talk NT4 from the
 *     browser at ws://<robot-host>:5810/nt/<client-id> -- no server round trip needed.
 *   - Module CAN ids and the current encoder offsets are in data/robot-profile.json under
 *     `swerve`, so the tool does not need to re-derive them.
 */

mountHeader();

document.getElementById("app").replaceChildren(
    el("h1", {}, "Swerve align"),
    el("p", { class: "lede" }, "Module alignment helper. Not wired up in this app yet."),
    el("div", { class: "notice warn" },
        el("strong", {}, "This page is a slot, not a tool. "),
        "The alignment app built in another session has not been pushed to this repository. ",
        "Drop it into ", el("code", {}, "tools/robot-app/client/pages/swerve-align/"), " and it will appear here — ",
        "the nav entry, shared styling and robot discovery are already in place. ",
        "See ", el("code", {}, "tools/robot-app/README.md"), " for the page contract."),
    el("h2", {}, "Why alignment matters on this robot"),
    el("div", { class: "card", style: "color:var(--tx-dim);font-size:0.87rem;line-height:1.7" },
        el("p", { style: "margin-top:0" },
            "The Aug 20 CANcoder offsets differ from the Aug 2 offsets by 175 to 186 degrees per module. ",
            "One of those two alignments leaves modules up to about 6 degrees off, which scrubs the wheels and drifts odometry."),
        el("p", {},
            "If you re-align to CTRE's template convention, revert the two drive inversion flags in ",
            el("code", {}, "SwerveConfig"), " ", el("strong", {}, "and"), " align with the bevels on the template's side. ",
            "Doing only one of the two is what made the robot drive backwards from every command."),
        el("p", { style: "margin-bottom:0" },
            "The manual procedure is in ", el("code", {}, "docs/tools/phoenix-tuner-x.md"),
            "; offsets live in ", el("code", {}, "src/main/java/frc/robot/configs/OM2026.java"), "."))
);
