//! `cargo xtask <task>`: build, deploy and simulation tasks for Linux, Windows and macOS.

mod deploy;
mod fetch;
mod platform;
mod run;

use run::SimOptions;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

const USAGE: &str = "usage: cargo xtask <task>

  gate                       format check, clippy with warnings as errors, all tests
  fetch-natives [platform]   download WPILib and CTRE libraries into native/<platform>
                             (default: this machine; linuxathena is the roboRIO)
  build-roborio [cargo args] cross-compile the robot program for the roboRIO
  deploy [host]              copy the program and libraries to a roboRIO
                             (default 10.38.47.2; USB is 172.22.11.2)
  sim [name] [-p package] [--ds] [--no-gui]
                             run the robot, or an example, with the native libraries
                             on the library path. Opens the Sim GUI unless --no-gui.
                             --ds also loads the driver station socket.";

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if let Err(e) = dispatch(&args) {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn dispatch(args: &[String]) -> Result<()> {
    let rest: Vec<&str> = args.iter().skip(1).map(String::as_str).collect();
    match args.first().map(String::as_str) {
        Some("fetch-natives") => match rest.first() {
            Some(platform) => fetch::fetch_natives(platform),
            None => fetch::fetch_natives(platform::host_platform()?),
        },
        Some("gate") => run::gate(),
        Some("build-roborio") => run::build_roborio(&args[1..]),
        Some("deploy") => deploy::deploy(rest.first().copied().unwrap_or("10.38.47.2")),
        Some("sim") => {
            let (name, package, options) = parse_sim(&rest)?;
            run::sim(name, package, options)
        }
        _ => {
            println!("{USAGE}");
            Ok(())
        }
    }
}

type SimArgs<'a> = (Option<&'a str>, Option<&'a str>, SimOptions);

fn parse_sim<'a>(args: &[&'a str]) -> Result<SimArgs<'a>> {
    let (mut name, mut package) = (None, None);
    let mut options = SimOptions::default();
    let mut it = args.iter().copied();
    while let Some(arg) = it.next() {
        match arg {
            "-p" | "--package" => package = Some(it.next().ok_or("-p needs a package name")?),
            "--ds" => options.ds = true,
            "--no-gui" => options.gui = false,
            _ if name.is_none() => name = Some(arg),
            _ => return Err(format!("unexpected argument {arg}").into()),
        }
    }
    Ok((name, package, options))
}

#[cfg(test)]
pub fn scratch_dir(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("xtask-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_arguments() {
        let default = SimOptions::default();
        assert_eq!(parse_sim(&[]).unwrap(), (None, None, default));
        assert_eq!(
            parse_sim(&["hal_smoke"]).unwrap(),
            (Some("hal_smoke"), None, default)
        );
        assert_eq!(
            parse_sim(&["pathplanner_telemetry", "-p", "pathplanner"]).unwrap(),
            (Some("pathplanner_telemetry"), Some("pathplanner"), default)
        );
        let (_, _, options) = parse_sim(&["--ds", "--no-gui"]).unwrap();
        assert_eq!(
            options,
            SimOptions {
                gui: false,
                ds: true
            }
        );
        assert!(parse_sim(&["a", "b"]).is_err());
        assert!(parse_sim(&["-p"]).is_err());
    }
}
