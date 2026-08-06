# Relicensing record

vid.stab was published under the GNU General Public License, version 2 or
later, up to and including release v1.1.2. It is now published under the GNU
Lesser General Public License, version 2.1 or later.

The reason is that vid.stab is a library. Under the GPL every application
linking it had to be GPL too, which in practice meant FFmpeg could only build
the `vidstabdetect` and `vidstabtransform` filters under `--enable-gpl`, and
most distributed FFmpeg builds therefore shipped without video stabilization.
The LGPL removes that barrier while keeping the reciprocity that matters:
modifications to vid.stab itself still have to be published under the LGPL.

## Permission from the copyright holders

The change was announced in
[issue #165](https://github.com/georgmartius/vid.stab/issues/165) and every
contributor with code still in `src/` was asked to agree to it. Each of the
following granted permission in that issue, in these words:

> I am a copyright holder of contributions to vid.stab. I hereby grant
> permission to relicense my contributions to vid.stab under the GNU Lesser
> General Public License, version 2.1 or (at your option) any later version.
> This permission is in addition to, and does not revoke, the GPL-2.0-or-later
> license under which I originally contributed.

| Contributor | Contribution | Statement |
| --- | --- | --- |
| Ernest Wilkerson ([@gabilan](https://github.com/gabilan)) | binary transform file serialization | [comment](https://github.com/georgmartius/vid.stab/issues/165#issuecomment-5172580219) |
| Alexey Osipov ([@lion-simba](https://github.com/lion-simba)) | spiral search and SSE optimizations | [comment](https://github.com/georgmartius/vid.stab/issues/165#issuecomment-5174846607) |
| Clément Bœsch ([@ubitux](https://github.com/ubitux)) | transform and interpolation cleanups | [comment](https://github.com/georgmartius/vid.stab/issues/165#issuecomment-5175324868) |

Georg Martius, the author of the great majority of the library, agreed to the
change by making it.

Contributions from anyone not listed here were single-line build and
portability fixes below the threshold of copyrightability, or have since been
replaced. Issue #165 invited anyone who saw it differently to say so.

## What this does not change

Copies of vid.stab obtained under the GPL stay available under the GPL: the
permissions above are additional and revoke nothing. Anyone who prefers the
GPL terms may continue to rely on them.
