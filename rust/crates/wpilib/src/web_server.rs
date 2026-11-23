//! Stand-in for WPILib's `WebServer.start(port, directory)`: serves the deploy folder over
//! HTTP so Elastic can load its layout from the robot. Read-only, one thread, no keep-alive.

use crate::driver_station;
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::path::{Component, Path, PathBuf};
use std::thread;
use std::time::Duration;

const READ_TIMEOUT: Duration = Duration::from_secs(5);

/// Serves `root` on `port` from a background thread. A port that is already taken is
/// reported and skipped, so a second simulated robot still starts.
pub fn start(port: u16, root: PathBuf) {
    match TcpListener::bind(("0.0.0.0", port)) {
        Ok(listener) => serve_in_background(listener, root),
        Err(error) => driver_station::report_warning(&format!(
            "Could not start the web server on port {port}: {error}"
        )),
    }
}

fn serve_in_background(listener: TcpListener, root: PathBuf) {
    let spawned = thread::Builder::new()
        .name("web-server".to_owned())
        .spawn(move || {
            for stream in listener.incoming().flatten() {
                let _ = handle(stream, &root);
            }
        });
    if let Err(error) = spawned {
        driver_station::report_warning(&format!("Could not start the web server: {error}"));
    }
}

fn handle(mut stream: TcpStream, root: &Path) -> std::io::Result<()> {
    stream.set_read_timeout(Some(READ_TIMEOUT))?;
    stream.set_write_timeout(Some(READ_TIMEOUT))?;
    let mut request_line = String::new();
    BufReader::new(&stream).read_line(&mut request_line)?;
    let mut parts = request_line.split_whitespace();
    let (method, target) = (parts.next().unwrap_or(""), parts.next().unwrap_or(""));

    let response = respond(method, target, root);
    stream.write_all(&response.head())?;
    if method == "GET" {
        stream.write_all(&response.body)?;
    }
    Ok(())
}

struct Response {
    status: &'static str,
    content_type: &'static str,
    body: Vec<u8>,
}

impl Response {
    fn head(&self) -> Vec<u8> {
        format!(
            "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
            self.status,
            self.content_type,
            self.body.len()
        )
        .into_bytes()
    }

    fn text(status: &'static str) -> Self {
        Self {
            status,
            content_type: "text/plain",
            body: status.as_bytes().to_vec(),
        }
    }
}

fn respond(method: &str, target: &str, root: &Path) -> Response {
    if method != "GET" && method != "HEAD" {
        return Response::text("405 Method Not Allowed");
    }
    let Some(path) = resolve(root, target) else {
        return Response::text("404 Not Found");
    };
    match std::fs::read(&path) {
        Ok(body) => Response {
            status: "200 OK",
            content_type: content_type(&path),
            body,
        },
        Err(_) => Response::text("404 Not Found"),
    }
}

/// The file `target` names under `root`. `None` for anything that could leave `root`.
/// `/` and folders serve their `index.html`.
fn resolve(root: &Path, target: &str) -> Option<PathBuf> {
    let target = target.split(['?', '#']).next().unwrap_or("");
    let mut path = root.to_path_buf();
    for component in Path::new(target.trim_start_matches('/')).components() {
        match component {
            Component::Normal(part) => path.push(part),
            Component::CurDir => {}
            _ => return None,
        }
    }
    if path.is_dir() {
        path.push("index.html");
    }
    Some(path)
}

fn content_type(path: &Path) -> &'static str {
    match path.extension().and_then(|e| e.to_str()) {
        Some("html" | "htm") => "text/html",
        Some("json") => "application/json",
        Some("css") => "text/css",
        Some("js") => "text/javascript",
        Some("txt") => "text/plain",
        Some("png") => "image/png",
        Some("jpg" | "jpeg") => "image/jpeg",
        Some("svg") => "image/svg+xml",
        _ => "application/octet-stream",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;

    fn scratch_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("web-server-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn get(port: u16, request: &str) -> String {
        let mut stream = TcpStream::connect(("127.0.0.1", port)).unwrap();
        stream.write_all(request.as_bytes()).unwrap();
        let mut text = String::new();
        stream.read_to_string(&mut text).unwrap();
        text
    }

    #[test]
    fn serves_files_from_the_root_over_http() {
        let dir = scratch_dir("serve");
        std::fs::write(dir.join("elastic-layout.json"), "{\"tabs\":[]}").unwrap();
        std::fs::write(dir.join("index.html"), "home").unwrap();
        let listener = TcpListener::bind(("127.0.0.1", 0)).unwrap();
        let port = listener.local_addr().unwrap().port();
        serve_in_background(listener, dir.clone());

        let layout = get(port, "GET /elastic-layout.json HTTP/1.1\r\n\r\n");
        assert!(layout.starts_with("HTTP/1.1 200 OK"));
        assert!(layout.contains("Content-Type: application/json"));
        assert!(layout.ends_with("{\"tabs\":[]}"));

        assert!(get(port, "GET / HTTP/1.1\r\n\r\n").ends_with("home"));
        assert!(get(port, "GET /missing HTTP/1.1\r\n\r\n").starts_with("HTTP/1.1 404"));
        assert!(get(port, "POST / HTTP/1.1\r\n\r\n").starts_with("HTTP/1.1 405"));

        let head = get(port, "HEAD /index.html HTTP/1.1\r\n\r\n");
        assert!(head.contains("Content-Length: 4"));
        assert!(!head.ends_with("home"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn paths_cannot_leave_the_root() {
        let root = Path::new("/srv/deploy");
        assert_eq!(resolve(root, "/../secret"), None);
        assert_eq!(resolve(root, "/a/../../secret"), None);
        assert_eq!(
            resolve(root, "/a/b.json?x=1"),
            Some(root.join("a").join("b.json"))
        );
    }
}
