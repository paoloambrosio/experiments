//! Discovery of Rigs of Rods definitions in ZIP archives.

use std::{
    fmt,
    fs::File,
    path::{Path, PathBuf},
};

use zip::{ZipArchive, result::ZipError};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AssetKind {
    Terrain,
    Vehicle,
}

impl AssetKind {
    fn extension(self) -> &'static str {
        match self {
            Self::Terrain => "terrn2",
            Self::Vehicle => "truck",
        }
    }
}

/// An error opening a ZIP archive or reading its directory.
#[derive(Debug)]
pub struct ArchiveError {
    path: PathBuf,
    source: ZipError,
}

impl fmt::Display for ArchiveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "cannot read ZIP archive {}: {}",
            self.path.display(),
            self.source
        )
    }
}

impl std::error::Error for ArchiveError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.source)
    }
}

/// Returns sorted, full entry names for the requested definition type.
///
/// Extensions match case-insensitively. Names retain their spelling and path
/// within the ZIP, so consumers can use them to select an exact entry.
pub fn discover_entries(path: &Path, kind: AssetKind) -> Result<Vec<String>, ArchiveError> {
    read_directory(path, kind).map_err(|source| ArchiveError {
        path: path.to_owned(),
        source,
    })
}

fn read_directory(path: &Path, kind: AssetKind) -> Result<Vec<String>, ZipError> {
    let archive = ZipArchive::new(File::open(path)?)?;
    let mut entries = Vec::new();
    for name in archive.file_names() {
        let name = name?;
        if let Some((_, extension)) = name.rsplit_once('.')
            && extension.eq_ignore_ascii_case(kind.extension())
        {
            entries.push(name.into_owned());
        }
    }
    entries.sort_unstable();
    Ok(entries)
}
