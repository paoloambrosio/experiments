use std::{
    fmt,
    fs::File,
    io::{Cursor, Read, Seek},
    path::{Path, PathBuf},
};

use zip::{ZipArchive, result::ZipError};

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct ImportWarning {
    pub entry: String,
    pub message: String,
}

#[derive(Debug)]
pub struct ImportError {
    archive: PathBuf,
    entry: String,
    detail: String,
}

impl fmt::Display for ImportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} :: {}: {}",
            self.archive.display(),
            self.entry,
            self.detail
        )
    }
}

impl std::error::Error for ImportError {}

pub(super) type Import<T> = Result<T, ImportError>;

pub(super) fn warn(warnings: &mut Vec<ImportWarning>, entry: &str, message: impl Into<String>) {
    warnings.push(ImportWarning {
        entry: entry.to_owned(),
        message: message.into(),
    });
}

trait ArchiveReader: Read + Seek {}
impl<T: Read + Seek> ArchiveReader for T {}

pub(super) struct Archive {
    path: PathBuf,
    zip: ZipArchive<Box<dyn ArchiveReader>>,
    names: Vec<String>,
}

impl Archive {
    pub(super) fn select(path: &Path, entry: &str) -> Import<(Self, String)> {
        Self::select_reader(path, entry, || Ok(File::open(path)?))
    }

    pub(super) fn select_embedded(
        path: &Path,
        bytes: &'static [u8],
        entry: &str,
    ) -> Import<(Self, String)> {
        Self::select_reader(path, entry, || Ok(Cursor::new(bytes)))
    }

    fn select_reader<R: ArchiveReader + 'static>(
        path: &Path,
        entry: &str,
        reader: impl FnOnce() -> Result<R, ZipError>,
    ) -> Import<(Self, String)> {
        let archive = reader()
            .and_then(|reader| Self::open(path, Box::new(reader)))
            .map_err(|error| ImportError {
                archive: path.to_owned(),
                entry: entry.to_owned(),
                detail: error.to_string(),
            })?;
        let resolved = archive
            .resolve("", entry)
            .map_err(|error| archive.error(entry, error))?;
        Ok((archive, resolved))
    }

    fn open(path: &Path, reader: Box<dyn ArchiveReader>) -> Result<Self, ZipError> {
        let zip = ZipArchive::new(reader)?;
        let names = zip
            .file_names()
            .map(|entry| entry.map(|name| name.into_owned()))
            .collect::<Result<_, _>>()?;
        Ok(Self {
            zip,
            names,
            path: path.to_owned(),
        })
    }

    pub(super) fn names(&self) -> &[String] {
        &self.names
    }

    pub(super) fn error(&self, entry: &str, detail: impl Into<String>) -> ImportError {
        ImportError {
            archive: self.path.clone(),
            entry: entry.to_owned(),
            detail: detail.into(),
        }
    }

    pub(super) fn resolve(&self, from: &str, reference: &str) -> Result<String, String> {
        let reference = reference.replace('\\', "/");
        let parent = from.rsplit_once('/').map_or("", |(parent, _)| parent);
        if reference.starts_with('/') || reference.contains(':') {
            return Err(format!("invalid archive reference {reference:?}"));
        }

        let relative = normalize(parent, &reference)
            .ok_or_else(|| format!("invalid archive reference {reference:?}"))?;
        let root = normalize("", &reference).filter(|root| *root != relative);
        for candidate in std::iter::once(relative.as_str()).chain(root.as_deref()) {
            if self.names.iter().any(|name| name == candidate) {
                return Ok(candidate.to_owned());
            }
            let mut matches = self
                .names
                .iter()
                .filter(|name| name.eq_ignore_ascii_case(candidate));
            if let Some(found) = matches.next() {
                if matches.next().is_some() {
                    return Err(format!("ambiguous archive reference {reference:?}"));
                }
                return Ok(found.clone());
            }
        }
        Err(format!("missing archive reference {reference:?}"))
    }

    pub(super) fn reference(&self, from: &str, reference: &str) -> Import<String> {
        self.resolve(from, reference)
            .map_err(|error| self.error(from, error))
    }

    pub(super) fn read(&mut self, entry: &str) -> Import<Vec<u8>> {
        let path = &self.path;
        let error = |detail| ImportError {
            archive: path.clone(),
            entry: entry.to_owned(),
            detail,
        };
        let mut source = self
            .zip
            .by_name(entry)
            .map_err(|err| error(err.to_string()))?;
        let mut bytes = Vec::new();
        source
            .read_to_end(&mut bytes)
            .map_err(|err| error(err.to_string()))?;
        Ok(bytes)
    }

    pub(super) fn text(&mut self, entry: &str) -> Import<String> {
        String::from_utf8(self.read(entry)?).map_err(|error| self.error(entry, error.to_string()))
    }
}

fn normalize(parent: &str, reference: &str) -> Option<String> {
    let mut parts = Vec::new();
    for part in parent.split('/').chain(reference.split('/')) {
        match part {
            "" | "." => {}
            ".." => {
                parts.pop()?;
            }
            value => parts.push(value),
        }
    }
    (!parts.is_empty()).then(|| parts.join("/"))
}
