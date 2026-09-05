#!/usr/bin/env node
/**
 * Verifies that the robot-app's hand-authored data files still describe the robot code.
 *
 *   node tools/robot-app/scripts/check-drift.mjs
 *
 * Two files claim to mirror the Java and will silently rot if nobody checks:
 *
 *   data/controls.json      every pilot.X / operator.X bound in Robot.configureBindings()
 *   data/robot-profile.json every motor's supply and stator current limit
 *
 * These are hand-written rather than generated on purpose -- the LT/RT bindings' real behaviour
 * lives inside Commands.either(...) conditions that no extractor can describe usefully -- so this
 * check is what keeps "hand-written" from meaning "wrong by March".
 *
 * This is NOT wired into the Gradle build. Robot code has to be free to move without anyone
 * stopping to update a web app first, so nothing here can fail a build or block a deploy. The app
 * calls runDriftCheck() instead and reports what it finds in its own header.
 *
 * Run as a CLI it still exits non-zero on drift, for anyone who deliberately wants that.
 */
import fs from "node:fs";
import { execFileSync } from "node:child_process";
import path from "node:path";
import { fileURLToPath } from "node:url";

const APP = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const REPO = path.resolve(APP, "../..");

const problems = [];
const notes = [];
const read = (rel) => fs.readFileSync(path.join(REPO, rel), "utf8");

// ---------------------------------------------------------------- controls

/** Body of a brace-balanced Java method, found by name. */
function methodBody(source, name) {
    const start = source.indexOf(`void ${name}(`);
    if (start === -1) return null;
    let i = source.indexOf("{", start);
    if (i === -1) return null;
    let depth = 0;
    for (let j = i; j < source.length; j++) {
        if (source[j] === "{") depth++;
        else if (source[j] === "}" && --depth === 0) return source.slice(i + 1, j);
    }
    return null;
}

function checkControls() {
    const controls = JSON.parse(fs.readFileSync(path.join(APP, "data/controls.json"), "utf8"));
    const robotJava = read(controls.sourceFile);
    const body = methodBody(robotJava, "configureBindings");
    if (!body) {
        problems.push(`Could not find configureBindings() in ${controls.sourceFile}. Was it renamed? Update controls.json's sourceMethod.`);
        return;
    }

    for (const controller of controls.controllers) {
        // Triggers the Java actually references, e.g. pilot.LT, operator.zeroTurretB.
        const re = new RegExp(`\\b${controller.id}\\.([A-Za-z_][A-Za-z0-9_]*)`, "g");
        const inJava = new Set();
        for (const m of body.matchAll(re)) inJava.add(`${controller.id}.${m[1]}`);

        // Triggers controls.json claims exist, ignoring simulation-only layers (they live in
        // configureSimBindings, a different method).
        const inJson = new Set();
        for (const layer of controller.layers) {
            if (layer.sim) continue;
            for (const b of layer.bindings) if (b.trigger) inJson.add(b.trigger);
        }

        // A documented trigger may be a chord like pilot.RT.and(pilot.LB); accept it if every
        // controller reference inside it appears in the Java.
        const refsOf = (expr) => [...expr.matchAll(new RegExp(`\\b${controller.id}\\.([A-Za-z_][A-Za-z0-9_]*)`, "g"))].map((m) => `${controller.id}.${m[1]}`);

        const documented = new Set();
        for (const t of inJson) for (const r of refsOf(t)) documented.add(r);

        for (const t of inJava) {
            // Config and lifecycle calls are not bindings.
            if (/\.(setAttached|getConfig|rumbleCommand|setDefaultCommand|config[A-Z])/.test(t)) continue;
            if (!documented.has(t)) {
                problems.push(
                    `${controls.sourceFile} binds ${t} but data/controls.json never mentions it. ` +
                        `Add it to the ${controller.name} layer it belongs to.`
                );
            }
        }
        for (const t of documented) {
            if (!inJava.has(t)) {
                problems.push(
                    `data/controls.json documents ${t} but ${controls.sourceMethod} does not use it. ` +
                        `Either the binding was removed from the Java, or the trigger name in controls.json is stale.`
                );
            }
        }

        // Line numbers are a convenience, not correctness -- warn, do not fail.
        const lines = robotJava.split("\n");
        for (const layer of controller.layers) {
            for (const b of layer.bindings) {
                if (!b.line || !b.trigger) continue;
                const short = b.trigger.split(".").slice(0, 2).join(".");
                const window = lines.slice(Math.max(0, b.line - 4), b.line + 3).join("\n");
                if (!window.includes(short)) {
                    notes.push(`controls.json line hint for ${b.trigger} (${controls.sourceFile}:${b.line}) looks stale.`);
                }
            }
        }
    }
}

// ---------------------------------------------------------------- current limits

/** Body of a named nested Java class. */
function classBody(source, name) {
    const re = new RegExp(`class\\s+${name}\\b`);
    const m = re.exec(source);
    if (!m) return null;
    let i = source.indexOf("{", m.index);
    if (i === -1) return null;
    let depth = 0;
    for (let j = i; j < source.length; j++) {
        if (source[j] === "{") depth++;
        else if (source[j] === "}" && --depth === 0) return source.slice(i + 1, j);
    }
    return null;
}

function fieldValue(body, field) {
    const m = new RegExp(`\\b${field}\\s*=\\s*(-?[0-9]+(?:\\.[0-9]+)?)`).exec(body);
    return m ? Number(m[1]) : null;
}

function checkLimits() {
    const profile = JSON.parse(fs.readFileSync(path.join(APP, "data/robot-profile.json"), "utf8"));
    const byKey = new Map(profile.motors.map((m) => [m.key, m]));
    for (const motor of profile.motors) {
        // RightConfig takes its limits from LeftConfig at construction time, so there is nothing
        // to read in its own class body -- check that the two profile entries agree instead.
        if (motor.inheritsLimitsFrom) {
            const parent = byKey.get(motor.inheritsLimitsFrom);
            if (!parent) {
                problems.push(`${motor.key} inheritsLimitsFrom "${motor.inheritsLimitsFrom}", which is not a motor in the profile.`);
            } else {
                for (const f of ["supplyAmps", "statorAmps"]) {
                    if (motor[f] !== parent[f]) {
                        problems.push(
                            `${motor.key} inherits its limits from ${parent.key} in Java, but the profile gives them different ${f} ` +
                                `(${motor[f]} vs ${parent[f]}).`
                        );
                    }
                }
            }
            continue;
        }
        if (!motor.javaFile || !motor.javaConfigClass) {
            notes.push(`robot-profile.json motor "${motor.key}" has no javaFile/javaConfigClass, so its limits are unchecked.`);
            continue;
        }
        let source;
        try {
            source = read(motor.javaFile);
        } catch {
            problems.push(`robot-profile.json points ${motor.key} at ${motor.javaFile}, which does not exist.`);
            continue;
        }
        const body = classBody(source, motor.javaConfigClass);
        if (!body) {
            problems.push(`Could not find class ${motor.javaConfigClass} in ${motor.javaFile} (for ${motor.key}).`);
            continue;
        }
        // Most configs name these fields identically; Turret calls them currentLimit and
        // torqueCurrentLimit, so the profile can name the pair explicitly.
        const fields = motor.javaLimitFields || { supplyAmps: "supplyCurrentLimit", statorAmps: "statorCurrentLimit" };
        for (const [profileKey, field] of Object.entries(fields)) {
            const java = fieldValue(body, field);
            if (java === null) {
                notes.push(`${motor.javaConfigClass} has no ${field} field; ${motor.key}.${profileKey} is unchecked.`);
                continue;
            }
            if (java !== motor[profileKey]) {
                problems.push(
                    `${motor.key}: ${motor.javaConfigClass}.${field} is ${java} A but robot-profile.json says ${profileKey} = ${motor[profileKey]} A. ` +
                        `Power-page limit lines and "% at limit" are wrong until these agree.`
                );
            }
        }
    }
}

// ---------------------------------------------------------------- elastic layout

/**
 * The Elastic dashboard layout binds NetworkTables keys that the code may no longer publish.
 * Dead widgets look like broken hardware to a student, which is worse than no widget.
 */
function checkElasticLayout() {
    const layoutPath = "src/main/deploy/elastic-layout.json";
    let layout;
    try {
        layout = read(layoutPath);
    } catch {
        return;
    }
    const topics = new Set([...layout.matchAll(/"topic"\s*:\s*"([^"]+)"/g)].map((m) => m[1]));
    if (!topics.size) return;

    // Everything the Java logs, as a set of key roots. Telemetry.log("Foo/Bar", ...) -> Foo/Bar
    const javaFiles = [];
    const walk = (dir) => {
        for (const e of fs.readdirSync(dir, { withFileTypes: true })) {
            const full = path.join(dir, e.name);
            if (e.isDirectory()) walk(full);
            else if (e.name.endsWith(".java")) javaFiles.push(full);
        }
    };
    walk(path.join(REPO, "src/main/java"));

    const logged = new Set();
    for (const f of javaFiles) {
        const src = fs.readFileSync(f, "utf8");
        for (const m of src.matchAll(/(?:Telemetry\.log|DogLog\.log|tunable)\s*\(\s*"([^"]+)"/g)) logged.add(m[1]);
        // Keys built as getName() + "/Suffix" -- record the suffix so we can match loosely.
        for (const m of src.matchAll(/logKey\s*\+\s*"([^"]+)"/g)) logged.add("*" + m[1]);
    }
    const suffixes = [...logged].filter((k) => k.startsWith("*")).map((k) => k.slice(1));

    for (const topic of topics) {
        if (!topic.startsWith("/Robot/")) continue;
        const key = topic.slice("/Robot/".length);
        if (logged.has(key)) continue;
        if (suffixes.some((s) => key.endsWith(s.replace(/^\//, "/")))) continue;
        notes.push(`elastic-layout.json binds ${topic}, which no Telemetry.log call produces. That widget is dead on the dashboard.`);
    }
}

// ---------------------------------------------------------------- pages are tracked

/**
 * Every page directory must actually be in git.
 *
 * The repo root ignores `logs/` for robot log files, which silently swallowed this app's Logs
 * page. It worked for everyone who had it on disk and 404'd for everyone who cloned. Nothing
 * pointed at the cause, so check it rather than trusting it.
 */
function checkPagesTracked() {
    const pagesDir = path.join(APP, "client", "pages");
    if (!fs.existsSync(pagesDir)) return;

    let tracked;
    try {
        tracked = new Set(
            execFileSync("git", ["ls-files", "--", "tools/robot-app/client/pages"], { cwd: REPO, encoding: "utf8" })
                .split("\n")
                .filter(Boolean)
        );
    } catch {
        notes.push("Could not run git ls-files, so page tracking is unchecked.");
        return;
    }

    for (const entry of fs.readdirSync(pagesDir, { withFileTypes: true })) {
        if (!entry.isDirectory()) continue;
        const files = fs.readdirSync(path.join(pagesDir, entry.name));
        for (const f of files) {
            const rel = `tools/robot-app/client/pages/${entry.name}/${f}`;
            if (!tracked.has(rel)) {
                problems.push(
                    `${rel} is not tracked by git. A fresh clone will not have it, Vite will not build that page, ` +
                        `and its nav tab will 404. Check \`git check-ignore -v ${rel}\` -- a .gitignore rule is probably eating it.`
                );
            }
        }
    }
}

// ---------------------------------------------------------------- staleness

/** Unix seconds of the last commit touching a path, or null if git cannot say. */
function lastCommitSeconds(rel) {
    try {
        const out = execFileSync("git", ["log", "-1", "--format=%ct", "--", rel], { cwd: REPO, encoding: "utf8" }).trim();
        return out ? Number(out) : null;
    } catch {
        return null;
    }
}

/**
 * Which data files the robot code has moved out from under.
 *
 * This is the softer half of the report, and the half that matters day to day. The checks above
 * only catch drift they know how to parse; a renamed state, a reworded binding or a whole new
 * mechanism slips straight past them. Commit times catch the rest: if Robot.java was committed
 * after controls.json was, the app is describing a robot that has since changed, whether or not
 * anything parseable disagrees yet.
 *
 * Commit times rather than mtimes, because a fresh clone stamps every file with the checkout time
 * and mtimes would say nothing.
 */
function checkStaleness() {
    const stale = [];
    const pairs = [];

    try {
        const controls = JSON.parse(fs.readFileSync(path.join(APP, "data/controls.json"), "utf8"));
        if (controls.sourceFile) {
            pairs.push(["data/controls.json", "tools/robot-app/data/controls.json", [controls.sourceFile]]);
        }
    } catch {
        // checkControls() already reported anything wrong with this file.
    }
    try {
        const profile = JSON.parse(fs.readFileSync(path.join(APP, "data/robot-profile.json"), "utf8"));
        const javaFiles = [...new Set(profile.motors.map((m) => m.javaFile).filter(Boolean))];
        if (javaFiles.length) {
            pairs.push(["data/robot-profile.json", "tools/robot-app/data/robot-profile.json", javaFiles]);
        }
    } catch {
        // checkLimits() already reported anything wrong with this file.
    }

    for (const [label, dataRel, sourceRels] of pairs) {
        const dataAt = lastCommitSeconds(dataRel);
        if (dataAt === null) continue;
        const newer = sourceRels
            .map((rel) => ({ rel, at: lastCommitSeconds(rel) }))
            .filter((x) => x.at !== null && x.at > dataAt)
            .sort((a, b) => b.at - a.at);
        if (!newer.length) continue;

        const days = Math.max(1, Math.round((newer[0].at - dataAt) / 86400));
        const others = newer.length - 1;
        stale.push({
            dataFile: label,
            sources: newer.map((x) => x.rel),
            daysBehind: days,
            message:
                `${label} has not been touched since ${newer[0].rel}` +
                `${others ? ` and ${others} other source file${others === 1 ? "" : "s"}` : ""} changed ` +
                `(about ${days} day${days === 1 ? "" : "s"} of robot code later). It may describe a robot that has since moved on.`,
        });
    }
    return stale;
}

// ---------------------------------------------------------------- run

/**
 * Runs every check and returns what it found. Never throws for drift, never exits.
 *
 * @returns {{problems: string[], notes: string[], stale: object[], checkedAt: string}}
 */
export function runDriftCheck() {
    problems.length = 0;
    notes.length = 0;
    checkControls();
    checkLimits();
    checkPagesTracked();
    checkElasticLayout();
    return {
        problems: [...problems],
        notes: [...notes],
        stale: checkStaleness(),
        checkedAt: new Date().toISOString(),
    };
}

// CLI only. Importers get runDriftCheck() and decide for themselves what drift is worth.
if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
    const r = runDriftCheck();
    const plural = (n, word) => `${n} ${word}${n === 1 ? "" : "s"}`;

    if (r.stale.length) {
        console.log("");
        console.log(`${plural(r.stale.length, "data file")} behind the robot code:`);
        for (const st of r.stale) console.log(`  - ${st.message}`);
    }
    if (r.notes.length) {
        console.log("");
        console.log(`${plural(r.notes.length, "warning")}:`);
        for (const n of r.notes) console.log(`  - ${n}`);
    }
    if (r.problems.length) {
        console.error("");
        console.error(`robot-app drift check found ${plural(r.problems.length, "problem")}:`);
        console.error("");
        for (const pr of r.problems) {
            console.error(`  x ${pr}`);
            console.error("");
        }
        console.error("This blocks nothing. Update the data files when it is convenient.");
        console.error("");
        process.exit(1);
    }
    console.log("");
    console.log(`robot-app drift check passed${r.notes.length || r.stale.length ? " (with warnings)" : ""}.`);
}
