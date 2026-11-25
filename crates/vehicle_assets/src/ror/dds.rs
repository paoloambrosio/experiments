//! Uncompressed legacy DDS pixels used by RoR textures.

use image::RgbaImage;

pub(super) fn decode_rgb(bytes: &[u8]) -> Result<Option<RgbaImage>, String> {
    if !bytes.starts_with(b"DDS ") {
        return Ok(None);
    }
    let header = bytes.get(..128).ok_or("truncated DDS header")?;
    let field = |offset: usize| u32::from_le_bytes(header[offset..offset + 4].try_into().unwrap());
    let pixel_flags = field(80);
    if pixel_flags & 0x4 != 0 {
        return Ok(None); // FourCC surfaces use the image crate's DDS decoder.
    }
    const RGB: u32 = 0x40;
    const ALPHA: u32 = 0x1;
    let bits = field(88);
    let has_alpha = pixel_flags & ALPHA != 0;
    if field(4) != 124
        || field(76) != 32
        || pixel_flags & RGB == 0
        || !matches!(bits, 24 | 32)
        || [field(92), field(96), field(100)] != [0x00ff0000, 0x0000ff00, 0x000000ff]
        || (has_alpha && (bits != 32 || field(104) != 0xff000000))
    {
        return Err("unsupported uncompressed DDS pixel format".into());
    }

    let (width, height) = (field(16), field(12));
    if width == 0 || height == 0 {
        return Err("DDS image has zero width or height".into());
    }
    let bpp = (bits / 8) as usize;
    let row_bytes = (width as usize)
        .checked_mul(bpp)
        .ok_or("DDS row size overflow")?;
    let stride = if field(8) & 0x8 != 0 {
        (field(20) as usize).max(row_bytes)
    } else {
        row_bytes
    };
    let size = stride
        .checked_mul(height as usize)
        .ok_or("DDS pixel size overflow")?;
    let data = bytes[128..].get(..size).ok_or("truncated DDS pixel data")?;
    let capacity = (width as usize)
        .checked_mul(height as usize)
        .and_then(|pixels| pixels.checked_mul(4))
        .ok_or("DDS image size overflow")?;
    let mut rgba = Vec::with_capacity(capacity);
    for row in data.chunks_exact(stride) {
        for pixel in row[..row_bytes].chunks_exact(bpp) {
            let alpha = if has_alpha { pixel[3] } else { 255 };
            rgba.extend([pixel[2], pixel[1], pixel[0], alpha]);
        }
    }
    let image = RgbaImage::from_raw(width, height, rgba).ok_or("invalid DDS pixel dimensions")?;
    Ok(Some(image))
}
