# Parameter meanings under the L1 optimal camera path

Notes for a later patch to ffmpeg's `doc/filters.texi` (section
`vidstabtransform`) and, where noted, to `libavfilter/vf_vidstabtransform.c`.

vid.stab 1.3 makes `optalgo=opt` (`VSOptimalL1`, the L1 optimal camera path of
Grundmann et al., CVPR 2011) actually run instead of silently falling back to
the gaussian filter. It deliberately introduces **no new filter options**: the
optimization reads its two inputs off options that already exist. What changes
is therefore not the option list but what three of the options *mean* when
`optalgo=opt` is selected.

The filter code needs no change for this — `VSTransformConfig` has no new
fields, and the existing `AVOption` table already fills `smoothing`, `zoom`,
`optzoom` and `zoomspeed`. Only the documentation is out of date. (One
optional code change is noted under "Defaults" below.)

## How the algorithm uses them

The L1 optimization computes a *crop window*, smaller than the frame, and moves
it along an optimized path made of static, linear and parabolic segments, under
the hard constraint that the window never leaves the frame. So it needs exactly
two things from the user: how much smaller than the frame that window may be
(the zoom budget), and over what timescale the path should stay rigid.

## `smoothing`

Currently documented as the number of frames (value\*2 + 1) used for lowpass
filtering the camera movements — accurate for `optalgo=gauss` and `avg`, and
meaningless as written for `optalgo=opt`, which does not use a sliding window at
all. Since `optalgo=opt` is the default (see "Defaults" below), the documented
meaning is wrong for the default configuration.

Proposed addition:

> With @code{optalgo=opt} there is no filter window; @var{smoothing} is read as
> the timescale, in frames, over which the camera path should stay rigid. It
> sets the relative weights of the first, second and third derivative of the
> path in the optimization: a small value produces short static holds with quick
> transitions between them, a large value long sweeping dolly- or crane-like
> moves. The default of 15 corresponds to the weights of the original paper.

Worth knowing when advising users: the effect is real and monotone but modest.
On a synthetic 200-frame shaky pan at 640x480 with the default 15% budget,
going from `smoothing=3` to `smoothing=60` lowered the L1 norm of the
stabilized path's acceleration by ~30% and of its jerk by ~40%. The zoom budget
below is by far the stronger knob; users chasing a smoother result should reach
for `zoom`/`optzoom` first.

## `zoom` and `optzoom`

Currently: `zoom` is a percentage to zoom in (negative zooms out), `optzoom`
selects whether an optimal zoom is determined (0 = none, 1 = static, 2 =
adaptive), `zoomspeed` bounds the per-frame zoom of the adaptive mode.

Under `optalgo=opt` these together supply the zoom budget, i.e. how much smaller
than the frame the crop window is allowed to be. The budget is spent *inside*
the optimization rather than applied on top of it, which is the part users need
to be told:

> With @code{optalgo=opt}, @var{zoom} and @var{optzoom} give the optimization
> the zoom it may spend on stabilization, rather than being applied to the
> result afterwards. @code{optzoom=0} restricts it to exactly @var{zoom}
> percent; with @code{optzoom=1} or @code{2} it uses @var{zoom} if one was
> given and 15% otherwise. The larger the budget, the more freedom the
> optimization has and the smoother the resulting path. @code{optzoom=2} and
> @var{zoomspeed} have no effect here: the crop window has to be fixed before
> the optimization runs, so @code{optzoom=2} behaves like @code{optzoom=1}.
>
> @code{optzoom=0} together with @code{zoom=0} leaves the optimization no room
> to move at all; in that case it declines and the gaussian filter is used
> instead, with a message in the log.

A negative `zoom` (zoom out) is not meaningful as a budget and is treated as 0.

## Defaults — note that this changes the out-of-the-box behaviour

`vf_vidstabtransform.c` already defaults to `optalgo=opt` (`{.i64 =
VSOptimalL1}`, marked "from version 1.0 on"), which until now silently fell back
to the gaussian filter inside the library. Once ffmpeg is built against
vid.stab 1.3, **every user who does not pass `optalgo` explicitly gets the L1
optimal path instead of the gaussian filter.** The option table needs no change
for that, but the patch should say so plainly, and it is the main thing
reviewers will want to know.

The remaining defaults — `smoothing=15`, `zoom=0`, `optzoom=1` — give a 15% zoom
budget and the paper's weights, i.e. exactly the library's own defaults. No
change needed.

## Tripod mode

`tripod=1` sets `relative=0` and `smoothing=0`. The L1 optimization requires
relative transforms (it builds the camera path as the running composition of the
frame-pair transforms), so it declines and the gaussian filter runs instead,
with a message in the log. Tripod mode therefore behaves exactly as before, and
the documentation of `tripod` needs no change — but it is worth a sentence in
the `optalgo` description:

> @code{optalgo=opt} requires relative transforms and at least 4 frames; where
> it cannot be applied — virtual tripod mode, for instance, which implies
> @code{relative=0} — the gaussian filter is used instead and a message is
> written to the log.

## Optional code change

`vf_vidstabtransform.c` fills `VSTransformConfig` field by field and never calls
`vsTransformGetDefaultConfig()`, so any field it does not know about arrives as
zero. That is safe today — the L1 path adds no fields, and the fields it does
read are all set by the filter — but calling `vsTransformGetDefaultConfig()`
first and overriding from the option table would make the filter robust against
future library fields. Worth mentioning in the patch as a follow-up, not a
requirement.
