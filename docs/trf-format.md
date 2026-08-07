# The `.trf` transform file format

The first pass (`vidstabdetect`) writes a `.trf` file; the second pass
(`vidstabtransform`) reads it. This document describes what is in it.

It is derived from `src/serialize.c` and `src/transformtype.h`, which remain the
authoritative reference. The current format version is **1**
(`LIBVIDSTAB_FILE_FORMAT_VERSION` in `src/serialize.h`).

**What the file is not:** it does not contain the final camera path. It contains
the *raw per-field measurements* of the first pass. The global transform of each
frame — the thing that actually moves the picture — is computed in the second
pass from these measurements, and depends on the second-pass options
(`smoothing`, `optalgo`, `maxshift`, …). See
[Getting the global transforms](#getting-the-global-transforms) below if that is
what you are after.

## Two encodings

There are two encodings of the same data, and reading auto-detects which one is
in front of it: if the file starts with the three bytes `TRF` it is binary,
otherwise it is parsed as text (`vsGuessSerializationMode`). You therefore never
have to tell the reader which one you have.

Which one you *get* depends on the caller:

- **Through ffmpeg you always get the binary encoding.** `vidstabdetect` exposes
  no option to select the encoding, and the library default is binary
  (`src/motiondetect.c:117`).
- The text encoding is only reachable from a library caller that sets
  `md->serializationMode = ASCII_SERIALIZATION_MODE` before `vsMotionDetectInit`.

Older releases defaulted to text, so `.trf` files found in old bug reports and
tutorials are usually text. Both are still read.

> **Always open the file in binary mode** — `fopen(path, "rb")` / `"wb"`. On
> Windows a text-mode handle stops reading at the first `0x1A` byte, which
> occurs in practically every binary `.trf`, and silently truncates it. This
> applies to the text encoding too, so that a file written on Windows stays
> readable elsewhere. See the contract comment at the top of `src/serialize.h`.

## The data model

Both encodings carry the same thing: for every frame, a list of **local
motions**. One local motion is one measurement field that was tracked
successfully from the previous frame to this one.

```c
typedef struct _localmotion {
    Vec   v;         // int16 x, y  — measured displacement, in pixels
    Field f;         // int16 x, y, size — where the field was, in pixels
    double contrast; // local contrast of the field
    double match;    // mean abs. pixel difference of the best match
} LocalMotion;
```

- `v.x`, `v.y` — how far this patch of image moved, **in pixels**, from the
  previous frame to this one. In tripod mode (`tripod=N`) it is measured against
  the reference frame N instead.
- `f.x`, `f.y` — the **centre** of the measurement field, in pixels, in the
  coordinate system of the frame that was analysed. Origin is top-left, x right,
  y down.
- `f.size` — edge length of the (square) field, in pixels.
- `contrast` — local contrast of the field. Fields below `mincontrast` are
  discarded and never reach the file.
- `match` — quality of the match: `minerror / (size * size)`, i.e. the mean
  absolute pixel difference of the winning position. **Lower is better.** A
  value of `-1` marks a field whose match was rejected; such fields are filtered
  out before writing.

Because every quantity above is in pixels of the analysed frame, a `.trf` is
tied to the resolution it was produced at. This is why detecting at one
resolution and transforming at another does not work without scaling `v.x`,
`v.y`, `f.x`, `f.y` and `f.size` yourself.

Frame numbers are **1-based**. Frame 1 normally carries an empty list, since
there is no previous frame to measure against. A reader tolerates gaps in the
frame numbering (it logs an informational message) but rejects indices outside
`1 … 4194304` and lists longer than `1048576` entries as corrupt.

## Binary encoding

All multi-byte values are **little-endian on disk**, on every host: the writer
byte-swaps on big-endian machines (`writeInt16`/`writeInt32`/`writeDouble`), and
the reader swaps back. Doubles are IEEE-754 binary64.

Header, once at the top of the file:

| Bytes | Type | Meaning |
|---|---|---|
| 3 | `char[3]` | the literal `TRF` |
| var | decimal text | format version, written with `%hhu` — for version 1 this is the single character `1`, i.e. byte `0x31`, *not* the byte `0x01` |
| 4 | `int32` | `accuracy` |
| 4 | `int32` | `shakiness` |
| 4 | `int32` | `stepsize` |
| 8 | `double` | `mincontrast` |

Then one record per frame, repeated until end of file:

| Bytes | Type | Meaning |
|---|---|---|
| 4 | `int32` | frame number (1-based) |
| 4 | `int32` | number of local motions that follow, *n* |
| 26·*n* | — | *n* local motion records |

Each local motion record is 26 bytes, with no padding:

| Bytes | Type | Field |
|---|---|---|
| 2 | `int16` | `v.x` |
| 2 | `int16` | `v.y` |
| 2 | `int16` | `f.x` |
| 2 | `int16` | `f.y` |
| 2 | `int16` | `f.size` |
| 8 | `double` | `contrast` |
| 8 | `double` | `match` |

Note that the header stores the detection settings but *not* the frame size, so
the resolution a file belongs to is not recorded anywhere in it.

## Text encoding

Line-oriented and ASCII. A line beginning with `#` is a comment and is ignored
on reading; leading whitespace, blank lines and CRLF line endings are tolerated.

```
VID.STAB 1
#      accuracy = 15
#     shakiness = 5
#      stepsize = 6
#   mincontrast = 0.250000
Frame 1 (List 2 [(LM -2 1 493 479 112 0.687232 1.848852),(LM 0 -8 787 199 112 0.665356 2.596540)])
```

- The first line is `VID.STAB ` followed by the format version.
- The four `#` lines record the detection settings. They are comments: the
  reader skips them and does not parse the values back.
- Each following line is `Frame <n> (List <len> [<lm>,<lm>,...])`, where `<len>`
  is the number of local motions and must match the number of entries.
- Each `<lm>` is `(LM <v.x> <v.y> <f.x> <f.y> <f.size> <contrast> <match>)`, in
  that order — the same seven fields as the binary record. The two doubles are
  written with `%lf`, so six decimals.

A frame with no usable measurements is written as `Frame <n> (List 0 [])`.

## Getting the global transforms

If what you want is one displacement per frame rather than the per-field
measurements, run the second pass with `debug=1`:

```shell
ffmpeg -i input.mp4 -vf vidstabtransform=debug=1 -f null -
```

This writes `global_motions.trf` into the current directory, in the deprecated
five/six-column format, one data line per frame — each followed by a comment
line carrying diagnostics, which a reader ignores:

```
0 0 0 0 0 1
# no fields
0 0.216345 -4.491032 0.021718 1.540513 0
#					 17.486730 2
0 0.295468 -4.358188 0.019088 0.794120 0
#					 19.948448 2
```

There is no header line. The first frame has no measurements, so it is written
as all zeros with `extra = 1`.

The same format is accepted as *input* to `vidstabtransform`, which is the
supported way to feed in a camera path you computed yourself: the reader tries
the current format first and falls back to this one
(`vsReadOldTransforms` in `src/serialize.c`).

Columns are `time x y alpha zoom extra`, separated by whitespace:

- `time` — **ignored on reading.** Frames are taken in file order, not by this
  value. The writer always emits `0`.
- `x`, `y` — translation in pixels.
- `alpha` — rotation around the frame centre, in **radians**.
- `zoom` — zoom in percent; `0` means no zoom. This column is optional: a line
  with five columns is read as `time x y alpha extra` with `zoom = 0`.
- `extra` — unused, write `0`.

Whether these are absolute (relative to the first frame) or per-frame increments
is not stored in the file — it is decided by the `relative` option of
`vidstabtransform`.
