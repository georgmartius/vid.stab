# Field of view, and why wide-angle footage wobbles

Written 2026-08-09, from the discussion in #139 ("Accounting of Field of View
for 3D deshaking"). This is the background note; `fov-model.md` describes the
model that was built from it.

## The symptom

On wide-angle footage the stabilizer holds the centre of the frame steady but
the edges and corners appear to swim, breathing in and out in time with the
shake. Reporters variously call it "inner wobbling", "magic mushroom", or
rubbery corners. It gets worse the more aggressive the smoothing is, and it is
at its worst in tripod mode, because that is where the demanded corrections are
largest.

It is easy to blame the lens. Mostly that is wrong.

## Three distinct causes, worth keeping apart

**1. Barrel / fisheye distortion.** The lens does not project rectilinearly:
straight lines in the world are not straight in the image. This is a property
of the glass, it is measurable, and it is correctable — ffmpeg's
`lenscorrection` and `lensfun` filters do it. `VSTransform` carries an unused
`barrel` field for doing it internally one day.

**2. Rectilinear projection non-linearity.** This is the one that surprises
people: it happens with a **perfect, distortion-free** lens. It is not an
optical defect at all, it is the geometry of perspective projection, and no
amount of lens correction removes it. This is the subject of #139 and of
`fov-model.md`.

**3. Rolling shutter.** The sensor does not expose all rows at the same
instant, so camera motion during readout shears and wobbles the frame
independently of everything above. `VSTransform` has an unused `rshutter`
field. Different problem, different fix.

The three are separable and they superimpose. If correcting distortion did not
help, cause 2 is the likely culprit — it scales with field of view rather than
with lens quality.

## Cause 2 in detail

Take an ideal pinhole camera with focal length `f` in pixels. A yaw of θ about
the optical centre maps an image point at horizontal offset `x` (measured from
the principal point) to

    x' = f · tan( atan(x/f) + θ )

Differentiating at θ = 0, the displacement for a small yaw is

    Δx ≈ f · θ · (1 + x²/f²)

So the shift is not constant across the frame — it grows with the square of the
field angle. Concretely, at a 90° horizontal field of view the frame edge sits
at `x = f`, and a point there moves **twice as far** as a point at the centre
for the same pan.

vid.stab currently fits a similarity transform: `x`, `y`, `alpha`, `zoom` —
four parameters, one uniform translation. Compensating a pan can therefore get
the centre right and undercorrect the edges, or the reverse, but not both. The
residual is proportional to the shake amplitude, so it pulses with the shake.
That is the observed wobble.

This is a **model-order error**. The estimator is not noisy or badly tuned; it
is fitting four parameters to a motion that genuinely has more structure than
four parameters can hold. At narrow fields of view the `x²/f²` term vanishes
and the similarity model is very nearly exact, which is why the problem is
invisible on long lenses and only shows up as the lens gets wider.

## The fix needs no depth

The natural repair is to model the frame-to-frame motion as a **rotation of the
camera about its optical centre**, which induces the homography

    H = K R K⁻¹

where `K = [[f,0,cx],[0,f,cy],[0,0,1]]` is the intrinsic matrix and `R` is a
3×3 rotation. This is three rotational degrees of freedom — yaw, pitch, roll —
instead of two translations plus a roll.

The important property is that **no depth term appears in `H`**. Under pure
rotation about the optical centre, near and far scene points move identically:
there is no parallax. Depth and structure-from-motion only become necessary
once the camera *translates*.

This is worth stating plainly because the request in #139 is phrased as "3D
deshaking", which reads as though it needs scene geometry. It does not. "3D"
there is the panorama-stitching sense of the word — three rotational degrees of
freedom, exactly what Hugin, PTGui and panotools mean by it — not 3D
reconstruction.

The distinction matters for scope:

| | needs | vid.stab |
| --- | --- | --- |
| Rotational (3-DoF) stabilization | one scalar: the field of view | tractable, planned |
| Parallax-aware ("true 3D") stabilization | per-pixel depth or SfM | not planned |

The second is what Deshaker3D and the VirtualDub2 3D filters attempt, and it is
presumably why they are described as unsatisfying: reconstructing depth from
handheld video well enough to reproject through it is a much harder problem
than removing the projection non-linearity.

## Where the parameter comes from

`f` is the only unknown, and it follows from the horizontal field of view:

    f = (width/2) / tan(fov_h / 2)

The field of view that matters is that of the **stored frame, after any lens
correction** — not the lens specification. Cropped, anamorphic, or
already-defished footage all differ from the nominal number, which is a
realistic way for the value to be wrong. And a badly wrong `fov` is worse than
none: the model is only a gain when the parameter is roughly right.

## What to do today

Until the rotational model exists:

- Apply `lenscorrection` or `lensfun` **before** `vidstabdetect` to remove
  cause 1. It must be applied identically in both passes — both passes have to
  see the same frames (see the README).
- Reduce the smoothing window, which reduces the size of the corrections and so
  the size of the residual from cause 2.
- Crop in. A centre crop of a wide frame is a narrower effective field of view,
  and the similarity model fits it better.
