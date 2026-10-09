import { defineConfig } from "vite";
import { resolve } from "node:path";
import { readdirSync, existsSync } from "node:fs";

const CLIENT = resolve(import.meta.dirname, "client");
const PAGES = resolve(CLIENT, "pages");

// Every directory under client/pages that has an index.html becomes its own entry point, so
// adding a page (the swerve-align tool, for instance) needs no build config changes.
function pageInputs() {
    const input = { main: resolve(CLIENT, "index.html") };
    if (!existsSync(PAGES)) return input;
    for (const name of readdirSync(PAGES, { withFileTypes: true })) {
        if (!name.isDirectory()) continue;
        const html = resolve(PAGES, name.name, "index.html");
        if (existsSync(html)) input[name.name] = html;
    }
    return input;
}

export default defineConfig({
    root: CLIENT,
    publicDir: resolve(import.meta.dirname, "data-public"),
    build: {
        // This app only ever runs in whatever browser is on the programming laptop, so there is no
        // reason to down-level. esnext keeps top-level await, which the pages use to load the
        // robot profile before rendering.
        target: "esnext",
        outDir: resolve(import.meta.dirname, "dist"),
        emptyOutDir: true,
        rollupOptions: { input: pageInputs() },
    },
    server: {
        port: 5173,
        proxy: {
            "/api": "http://localhost:5801",
            "/data": "http://localhost:5801",
        },
    },
});
