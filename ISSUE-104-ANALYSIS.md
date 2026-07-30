# Issue #104 "Windows Builds: Cannot parse localmotion!" - diagnosis

**Short version:** FFmpeg's vidstab filters open the `.trf` transform file with
`fopen(..., "w")` / `fopen(..., "r")` - i.e. in **text mode** - but since
vid.stab 1.1.x the default transform file format is **binary**. On Windows,
MSVCRT/UCRT stop reading a text-mode stream at the first `0x1A` (Ctrl-Z) byte,
which it interprets as end-of-file. A binary `.trf` contains such a byte roughly
every 700 bytes, so the file is silently truncated a few frames in and the
second pass fails with a flood of `Cannot parse localmotion!`. On Linux/macOS
text and binary mode are identical, which is why the very same command works
there.

The fix belongs in **FFmpeg** (two characters); vid.stab can only improve the
diagnostics and document the contract, which this branch does.

## 1. Why FFmpeg users always get the binary format

`libavfilter/vf_vidstabdetect.c` contains no reference to `serializationMode`.
Its private context is allocated zeroed by FFmpeg, so `md->serializationMode == 0`
when `vsMotionDetectInit()` runs, and `src/motiondetect.c` then does:

```c
if(md->serializationMode != ASCII_SERIALIZATION_MODE &&
   md->serializationMode != BINARY_SERIALIZATION_MODE)
  md->serializationMode = BINARY_SERIALIZATION_MODE;
```

So **every** FFmpeg user gets the binary format and there is no filter option to
choose ASCII (which is why the `fileformat=1` workaround mentioned in the thread
only works for callers other than FFmpeg's own filters; FFmpeg's `vidstabdetect`
has no such option).

Side note (separate contract/UB nit): `vsMotionDetectInit()` *reads* the public
field `md->serializationMode` before it is initialised. The caller is nowhere
told that it must set this field before calling init, and for a caller that does
not zero the struct this is indeterminate. The intended contract ("set
`md.serializationMode` before `vsMotionDetectInit`") should be documented, or the
field should move into `VSMotionDetectConfig` where all other user-settable
values live.

## 2. The two FFmpeg lines that cause the bug

```
libavfilter/vf_vidstabdetect.c:145    sd->f = fopen(sd->result, "w");
libavfilter/vf_vidstabtransform.c:211 f = fopen(tc->input, "r");
```

No `b` in the mode string. On POSIX that is harmless. On Windows, per
Microsoft's `fopen` documentation, a text-mode stream

* expands `0x0A` to `0x0D 0x0A` when writing and collapses `0x0D 0x0A` to `0x0A`
  when reading, and
* **treats `0x1A` (Ctrl-Z) as end-of-file when reading**.

Note that the CRLF translation alone is *not* the problem: write-then-read is a
symmetric round trip, so CRLF cannot corrupt the data (a frequent misdiagnosis).
The lossy, asymmetric part is the Ctrl-Z-as-EOF rule.

Whether a MinGW/MSYS2 build actually ends up in text mode also depends on the
CRT and on the startup default for `_fmode`, which plausibly explains why some
Windows builds in the thread reproduce the bug and others do not.

## 3. Empirical proof (on Linux, by emulating MSVCRT text-mode semantics)

Driver: generate 300 frames x 100 local motions with realistic values (field
grid coordinates, small motion vectors, `contrast`/`match` in [0,1]), write with
`vsPrepareFile`/`vsWriteToFile` in binary mode, read back with
`vsReadLocalMotionsFile`.

```
=== 1. write binary .trf (300 frames x 100 local motions) ===
  file size 782424 bytes; 0x1A count = 1167 (first at offset 2524);
                          0x0A count = 3976; 0x0D count = 1139
=== 2. plain binary read-back (Linux, fopen "rb") ===
  -> vsReadLocalMotionsFile = VS_OK, frames restored = 300      # fine
=== 3. emulate MSVCRT TEXT-mode READ: truncate at first 0x1A (Ctrl-Z = EOF) ===
  truncated to 2524 of 782424 bytes (0.32%)
  -> vsReadLocalMotionsFile = VS_OK, frames restored = 1
     [with the code as of master: "Cannot parse localmotion!" x 5, then
      nothing else, and stabilisation continues with garbage]
=== 4. emulate text-mode WRITE (LF->CRLF) then BINARY read ===
  expanded 782424 -> 786400 bytes
  -> "Cannot parse localmotion!" x 16,641,545   # does NOT match the reports
=== 5. emulate FULL MSVCRT round trip (text write + text read) ===
  disk bytes 786400, text-read yields 2524 logical bytes (of 782424 written)
  -> byte-identical to the stage-3 file; frames restored = 1
```

Conclusions:

* **Stage 3/5 reproduce the reported failure exactly**: N repetitions of
  `Cannot parse localmotion!` and then *no further error message*, followed by
  the transform pass running on almost no data ("not enough transforms found,
  use last transformation!", nonsense final zoom). The absence of a follow-up
  message is a distinguishing detail: the old `vsRestoreLocalmotionsBinary()`
  appended a `null_localmotion()` for every failed record, so the
  `len != vs_vector_size` check never fired, and the next `readInt32(frameNum)`
  hit EOF and ended the loop quietly. N equals the number of local motions left
  in the frame where truncation happened - matching the arbitrary counts in the
  reports (132, 236, 256, ...).
* **Stage 5 is byte-identical to stage 3**, confirming that CRLF translation is a
  perfect involution and that Ctrl-Z truncation is the whole mechanism.
* **Stage 4** (mixed toolchain: text-mode write, binary read - what you get when
  a `.trf` is moved between systems) produces millions of messages, i.e. it is a
  real corruption too, but it is *not* what the reporters saw.

### How likely is a `0x1A` byte? Effectively certain.

In the 782,424-byte file above there are **1167** `0x1A` bytes (0.149% of all
bytes), and the first one appears at offset 2524, i.e. inside the *second*
frame. Provenance:

```
total = 1167   in frameNum/len int32s = 2
               in the five int16 fields = 0
               in the contrast/match doubles = 1165
```

The `double` mantissas are effectively random bytes, so with 16 double-bytes per
local motion, `P(no 0x1A anywhere)` for any file with more than a couple of
hundred local motions is indistinguishable from zero. Every real video is
affected, and truncation happens within the first frames - consistent with
"output identical to input".

## 4. Where the fix belongs

### FFmpeg (the actual bug) - one character per line

```diff
--- a/libavfilter/vf_vidstabdetect.c
+++ b/libavfilter/vf_vidstabdetect.c
-    sd->f = fopen(sd->result, "w");
+    sd->f = fopen(sd->result, "wb");
--- a/libavfilter/vf_vidstabtransform.c
+++ b/libavfilter/vf_vidstabtransform.c
-    f = fopen(tc->input, "r");
+    f = fopen(tc->input, "rb");
```

(Even better inside libavfilter: `avpriv_fopen_utf8(path, "wb"/"rb")`, which
additionally fixes non-ASCII paths on Windows.)

This is safe for the ASCII format as well - with the vid.stab change in §5.3,
legacy ASCII `.trf` files with CRLF line endings still parse from a binary
handle.

### vid.stab cannot fix this unilaterally

The library only ever receives a `FILE*`; it cannot change the mode of a stream
it did not open (`_setmode()` on a stdio stream is Windows-specific and would
require the fd, and vid.stab is not the owner of the handle). What vid.stab
*can* do is documented below.

## 5. Changes made in vid.stab on this branch

1. **`src/serialize.h`**: documented the contract - all `FILE*` passed to the
   serialization API must be opened in **binary** mode (`"wb"`/`"rb"`) - with a
   short explanation of the Ctrl-Z and CRLF failure modes. *Recommended: yes.*
   This is the one thing that would have prevented the bug.
2. **`src/serialize.c` - honest, non-repeating errors.** The binary reader now
   distinguishes truncation from a malformed record and reports it once per
   frame instead of once per record:

   ```
   Cannot parse localmotion: unexpected end of file. The transform file is
   truncated or corrupt. Make sure it is written and read through a file handle
   opened in *binary* mode (fopen(..., "wb") / fopen(..., "rb")).
   Cannot parse the given number of localmotions (got 95 of 100)!
   ```

   It also stops filling the list with `null_localmotion()` placeholders after a
   failed read (those placeholders were fed into the stabilisation as if they
   were measurements) and rejects an implausible list length
   (`> 1<<20`) instead of allocating it and emitting millions of errors. In
   stage 4 above this reduces 16.6 million log lines to a single actionable one.
   *Recommended: yes* - purely diagnostic, no format change.
3. **`src/serialize.c` - ASCII reader tolerates `\r`/`\t`** between records, so
   that ASCII `.trf` files written through a Windows text-mode handle remain
   readable once callers switch to `"rb"`. *Recommended: yes* - required for the
   FFmpeg patch above to be fully backward compatible.
4. **Test** `tests/test_serialize_robust.c` (`./tests --testSRO`, also part of
   `--all`): binary round trip; truncation at the first `0x1A` must not invent
   localmotions; corrupt list length must be rejected; ASCII file with CRLF must
   parse. Suite: **15/15 units pass** (14/14 before, +1 new unit).

### Not changed: the default serialization mode

Should `serializationMode == 0` still default to BINARY? Arguments:

* *For switching the default to ASCII:* the reader auto-detects the format
  (`vsGuessSerializationMode`), so ASCII output is fully backward/forward
  compatible, and it is immune to text-mode handles. Flipping this default would
  fix every FFmpeg build in the wild without an FFmpeg release - the pragmatic
  escape hatch if the FFmpeg patch takes a long time to reach users.
* *Against:* it silently reverts the ~40% file-size win the binary format was
  introduced for, and it papers over a caller bug that is trivially fixable.

Recommendation: keep BINARY as the default, land the FFmpeg two-character fix,
and expose `fileformat` as a `vidstabdetect` filter option in FFmpeg so users
have a documented workaround. If the maintainers prefer a zero-coordination fix,
flipping the default to ASCII is defensible and safe - it is a one-line change in
`vsMotionDetectInit()`.

## 6. Loose ends / honest caveats

* This mechanism explains the **Windows** reports (the subject of the issue) and
  is proven by emulation, not by running on Windows (no Windows host available
  here). The one prediction to verify on Windows: with the two-character FFmpeg
  patch the second pass succeeds; without it, `transforms.trf` written by
  `vidstabdetect` is larger on Windows than on Linux by exactly the number of
  `0x0A` bytes in the payload (~0.5%), which is itself a cheap smoking-gun test.
* It does **not** explain the macOS and Linux-static-build comments in the same
  thread (`fopen` has no text mode there). Those are a different (or a stale
  `.trf`) problem; note that one reporter later retracted, and that a genuine
  cross-platform asymmetry did exist before June 2022: the reader used
  `fscanf(f, "TRF%hhu\n", &version)` while the writer wrote no newline, so the
  trailing whitespace directive ate one payload byte whenever the low byte of
  `accuracy` was whitespace. Measured on the header layout:

  ```
  accuracy = 1..8, 14, 15  -> stream position 4 (correct)
  accuracy = 9,10,11,12,13 -> stream position 5  <== STREAM SHIFTED, file unreadable
  ```

  That affected every platform and matches "works with accuracy=10, breaks
  everything downstream"; it was fixed by commit 1fe9f83 (June 2022). Anyone
  still reporting the error on a non-Windows platform should confirm they are
  running vid.stab with that fix.
* `vsGuessSerializationMode()` uses `ftell`/`fgetc`x3/`fseek`. In text mode this
  happens to be safe because it is only ever called at offset 0 and seeks back
  to a value returned by `ftell` (the only thing MSVCRT guarantees in text
  mode). No change needed.
* `vsReadFileVersionBinary()` compares `fscanf(...)` against
  `LIBVIDSTAB_FILE_FORMAT_VERSION`, i.e. it compares a conversion *count* with a
  version *number*. It works only because both happen to be 1; it should be
  `!= 1` (or the version should be checked separately). Cosmetic today, a trap
  for format version 2. Left untouched deliberately.
