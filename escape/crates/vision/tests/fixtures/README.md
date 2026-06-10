# Vision fixtures

Downsampled (640×360, 16:9) frames + per-column free-space labels, used to TDD
the floor segmenter. See `escape/SPEC.md` §4.2.

These are authored interactively: the human draws the boundary on each photo
with `escape/tools/vision_label/label_ui.py` (see that dir's README). The tool
populates `images/` (JPEG fixtures) and `labels/` (JSON) from `photos/`. The set
is therefore generated out-of-band, not committed by hand — `cargo test -p
vision` validates whatever exists.

## Label schema (`labels/*.json`)

Per-column `free_frac` (fraction of image height that is floor, from the bottom,
left column first) — exactly the `vision::FreeSpace` contract. Plus sample
floor/obstacle points, the timestamp-overlay `ignore_rect`, a per-image
`tolerance`, and a `holdout` flag (held out of algorithm tuning).

## Robot self-visibility

The camera will be positioned so the **robot is not in frame** at test time, so
fixtures should be chosen/curated to match (frames showing the robot's own
bumper/casters are avoided). No self-mask is needed; if a stray robot part does
appear it is simply labeled as an obstacle.
