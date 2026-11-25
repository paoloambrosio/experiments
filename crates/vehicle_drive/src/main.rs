mod config;

use std::{
    io::{self, BufRead, IsTerminal, Write},
    path::{Path, PathBuf},
    process::ExitCode,
};

use clap::{Arg, Command, value_parser};
use vehicle_assets::ror::{AssetKind, discover_entries};

use crate::config::Config;

fn main() -> ExitCode {
    let args = Command::new("vehicle_drive")
        .about("Select terrain and vehicle definitions from Rigs of Rods ZIP archives")
        .arg(
            Arg::new("config")
                .long("config")
                .value_name("PATH")
                .value_parser(value_parser!(PathBuf))
                .default_value("config.toml")
                .help("TOML file with terrain and vehicle archive paths"),
        )
        .get_matches();
    let path = args
        .get_one::<PathBuf>("config")
        .expect("config has a default");

    let stdin = io::stdin();
    let stderr = io::stderr();
    let mut input = (stdin.is_terminal() && stderr.is_terminal()).then(|| stdin.lock());
    let mut diagnostics = stderr.lock();
    match run(
        path,
        input.as_mut(),
        &mut io::stdout().lock(),
        &mut diagnostics,
    ) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            let _ = writeln!(diagnostics, "{error}");
            ExitCode::FAILURE
        }
    }
}

fn run(
    path: &Path,
    mut input: Option<&mut impl BufRead>,
    output: &mut impl Write,
    diagnostics: &mut impl Write,
) -> Result<(), String> {
    let config = Config::read(path)?;
    let assets = [
        (
            AssetKind::Terrain,
            "terrain",
            &config.assets.terrain,
            config.assets.terrain_entry.as_deref(),
        ),
        (
            AssetKind::Vehicle,
            "vehicle",
            &config.assets.vehicle,
            config.assets.vehicle_entry.as_deref(),
        ),
    ];
    let mut selections = Vec::with_capacity(assets.len());

    for (kind, field, archive, requested) in assets {
        let entries = discover_entries(archive, kind)
            .map_err(|error| format!("[assets].{field}: {error}"))?;
        let selected = select_entry(
            &entries,
            requested,
            field,
            archive,
            input.as_deref_mut(),
            diagnostics,
        )?;
        selections.push((field, archive, selected.to_owned()));
    }

    for (field, archive, selected) in selections {
        writeln!(output, "{field}: {} -> {selected:?}", archive.display())
            .map_err(|error| format!("cannot write selection summary: {error}"))?;
    }
    Ok(())
}

fn select_entry<'a>(
    entries: &'a [String],
    requested: Option<&str>,
    field: &str,
    archive: &Path,
    input: Option<&mut impl BufRead>,
    diagnostics: &mut impl Write,
) -> Result<&'a str, String> {
    if let Some(requested) = requested {
        return entries.iter().find(|entry| entry.as_str() == requested).map(String::as_str)
            .ok_or_else(|| {
                let available = if entries.is_empty() {
                    "no eligible definitions".into()
                } else {
                    format!("available entries: {entries:?}")
                };
                format!("[assets].{field}_entry {requested:?} is not an eligible entry in {} ({available})", archive.display())
            });
    }

    match entries {
        [] => {
            return Err(format!(
                "[assets].{field}: no eligible {field} definitions in {}",
                archive.display()
            ));
        }
        [only] => return Ok(only),
        _ => {}
    }

    let Some(input) = input else {
        return Err(format!(
            "[assets].{field}: multiple definitions in {}. Set {field}_entry in [assets] to one of {entries:?}",
            archive.display()
        ));
    };

    choose_from_menu(entries, field, archive, input, diagnostics)
        .map_err(|error| {
            format!(
                "[assets].{field}: cannot select an entry in {}: {error}",
                archive.display()
            )
        })?
        .ok_or_else(|| {
            format!(
                "[assets].{field}: input closed. Set {field}_entry in [assets] to select directly"
            )
        })
}

fn choose_from_menu<'a>(
    entries: &'a [String],
    field: &str,
    archive: &Path,
    input: &mut impl BufRead,
    diagnostics: &mut impl Write,
) -> io::Result<Option<&'a str>> {
    writeln!(diagnostics, "Select {field} from {}", archive.display())?;
    for (index, entry) in entries.iter().enumerate() {
        writeln!(diagnostics, "  {}. {entry:?}", index + 1)?;
    }

    let mut line = String::new();
    loop {
        write!(diagnostics, "Enter a number (1-{}): ", entries.len())?;
        diagnostics.flush()?;
        line.clear();
        if input.read_line(&mut line)? == 0 {
            return Ok(None);
        }
        if let Ok(number) = line.trim().parse::<usize>()
            && (1..=entries.len()).contains(&number)
        {
            return Ok(Some(&entries[number - 1]));
        }
        writeln!(diagnostics, "Choose one of the listed numbers.")?;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn menu_retries_and_returns_the_same_entry_as_explicit_config() {
        let entries = ["accord.truck".to_owned(), "accordb.truck".to_owned()];
        let archive = Path::new("accord23.zip");
        let mut diagnostics = Vec::new();

        let interactive = select_entry(
            &entries,
            None,
            "vehicle",
            archive,
            Some(&mut &b"0\ncar\n3\n1\n"[..]),
            &mut diagnostics,
        )
        .unwrap();
        let explicit = select_entry(
            &entries,
            Some("accord.truck"),
            "vehicle",
            archive,
            None::<&mut &[u8]>,
            &mut Vec::new(),
        )
        .unwrap();
        assert_eq!(interactive, explicit);
        let menu = String::from_utf8(diagnostics).unwrap();
        assert!(menu.contains("1. \"accord.truck\""));
        assert!(menu.contains("2. \"accordb.truck\""));
        assert_eq!(menu.matches("Choose one of the listed numbers.").count(), 3);
    }

    #[test]
    fn closed_input_exits_with_the_config_key() {
        let entries = ["accord.truck".to_owned(), "accordb.truck".to_owned()];
        let error = select_entry(
            &entries,
            None,
            "vehicle",
            Path::new("accord23.zip"),
            Some(&mut &b""[..]),
            &mut Vec::new(),
        )
        .unwrap_err();
        assert!(error.contains("input closed"));
        assert!(error.contains("vehicle_entry"));
    }
}
