"""
Tests for move_planner.plan_move using real PiCar camera fixture images.

Test categories
---------------
TestPlanMoveStructure    — structural invariants that must hold for every frame,
                           regardless of scene content or calibration constants.
TestMaxDistanceParameter — the max_distance cap and minimum-distance floor.
TestDeterminism          — identical input must produce identical output.
TestSceneBehavior        — looser behavioral assertions grounded in visual
                           inspection of specific fixture frames.  These may
                           need revisiting if HORIZON_FRAC / DISTANCE_K are
                           re-calibrated.
TestFixtureSnapshots     — one test per fixture image recording the exact output
                           of plan_move; acts as a regression suite and helps
                           developers understand what the algorithm does for
                           real scenes.

Fixtures overview (from visual inspection + texture algorithm)
--------------------------------------------------------------
frame_0001  camera pressed into a plywood corner; smooth walls fill all zones → reverse
frame_0003  plywood bench fills centre and right; carpet visible on left → left
frame_0004  smooth plywood fills centre and right; narrow carpet strip optically
            at 1.24 m on left, but zone-consistency check (ratio 2.1×) flags
            it as a dead-end → reverse
frame_0007  open room, carpet floor; slight obstruction on right → left
frame_0009  plywood box fills right zone; carpet clear on left and centre → left
frame_0010  large box at < 0.1 m; doorway gap on left is optically clear but
            physically unreachable (zone-consistency check → reverse)
frame_0011  same dead-end scene as 0010, slightly further back → reverse
frame_0012  same dead-end scene as 0010, slightly further back → reverse
frame_0013  same dead-end scene as 0010, slightly further back → reverse
frame_0008_door  lit doorway (glass partition + room beyond) in centre zone;
                 detect_door() → 'straight'; plan_move() (no target) → 'left' (tie)
frame_0014  all zones clear (carpet visible at horizon in all three) → left
"""

import base64
import pathlib

import pytest

import move_planner as planner
from move_planner import (
    MoveAction,
    REVERSE_DISTANCE_M,
    REVERSE_THRESHOLD,
    SAFETY_FACTOR,
    detect_door,
)

FIXTURES = pathlib.Path(__file__).parent / "fixtures"
ALL_FRAMES = sorted(FIXTURES.glob("frame_*.jpg"))

assert ALL_FRAMES, f"No fixture images found in {FIXTURES}"


def _b64(path: pathlib.Path) -> str:
    return base64.b64encode(path.read_bytes()).decode()


# ── Structural invariants (all 14 frames) ─────────────────────────────────────

@pytest.mark.parametrize("frame_path", ALL_FRAMES, ids=lambda p: p.name)
class TestPlanMoveStructure:
    """
    Properties guaranteed by the algorithm regardless of scene or calibration.
    Every one of these tests must pass on any valid camera image.
    """

    def test_returns_move_action(self, frame_path):
        result = planner.plan_move(_b64(frame_path))
        assert isinstance(result, MoveAction)

    def test_direction_is_valid(self, frame_path):
        result = planner.plan_move(_b64(frame_path))
        assert result.direction in ("left", "straight", "right", "reverse", "reverse+left", "reverse+right")

    def test_distance_within_default_bounds(self, frame_path):
        result = planner.plan_move(_b64(frame_path))
        assert 0.1 <= result.distance_metres <= 5.0

    def test_clearances_are_non_negative(self, frame_path):
        result = planner.plan_move(_b64(frame_path))
        assert result.left_clearance_m >= 0.0
        assert result.center_clearance_m >= 0.0
        assert result.right_clearance_m >= 0.0

    def test_clearances_do_not_exceed_max_distance(self, frame_path):
        max_d = 5.0
        result = planner.plan_move(_b64(frame_path), max_distance=max_d)
        assert result.left_clearance_m <= max_d
        assert result.center_clearance_m <= max_d
        assert result.right_clearance_m <= max_d

    def test_chosen_direction_has_highest_clearance(self, frame_path):
        """
        For forward moves the chosen direction must have the most clearance.
        For reverse moves every zone is below REVERSE_THRESHOLD so no forward
        direction is meaningful; the invariant becomes that each clearance is
        indeed below the threshold.
        """
        result = planner.plan_move(_b64(frame_path))
        if result.direction in ("reverse", "reverse+left", "reverse+right"):
            assert result.left_clearance_m   < REVERSE_THRESHOLD
            assert result.center_clearance_m < REVERSE_THRESHOLD
            assert result.right_clearance_m  < REVERSE_THRESHOLD
        else:
            clearances = {
                "left":     result.left_clearance_m,
                "straight": result.center_clearance_m,
                "right":    result.right_clearance_m,
            }
            assert result.direction == max(clearances, key=clearances.get)

    def test_distance_is_safety_factor_times_best_clearance(self, frame_path):
        """
        Forward:  distance = round(min(max(best * SAFETY_FACTOR, 0.1), max_d), 2)
        Reverse:  distance = min(REVERSE_DISTANCE_M, max_d)
        """
        max_d = 5.0
        result = planner.plan_move(_b64(frame_path), max_distance=max_d)
        if result.direction in ("reverse", "reverse+left", "reverse+right"):
            expected = min(REVERSE_DISTANCE_M, max_d)
            assert result.distance_metres == pytest.approx(expected, abs=0.005)
        else:
            best = max(
                result.left_clearance_m,
                result.center_clearance_m,
                result.right_clearance_m,
            )
            expected = round(min(max(best * SAFETY_FACTOR, 0.1), max_d), 2)
            assert result.distance_metres == pytest.approx(expected, abs=0.005)

    def test_reasoning_is_non_empty_string(self, frame_path):
        result = planner.plan_move(_b64(frame_path))
        assert isinstance(result.reasoning, str)
        assert result.reasoning.strip()


# ── max_distance parameter behaviour ──────────────────────────────────────────

class TestMaxDistanceParameter:
    # frame_0007 is an open room — likely to produce non-trivial distances.
    FRAME = FIXTURES / "frame_0007.jpg"

    @pytest.mark.parametrize("cap", [0.1, 0.5, 1.0, 2.5, 5.0])
    def test_distance_never_exceeds_cap(self, cap):
        result = planner.plan_move(_b64(self.FRAME), max_distance=cap)
        assert result.distance_metres <= cap + 0.005  # tolerance for float rounding

    @pytest.mark.parametrize("cap", [0.1, 0.5, 1.0, 2.5, 5.0])
    def test_clearances_never_exceed_cap(self, cap):
        result = planner.plan_move(_b64(self.FRAME), max_distance=cap)
        assert result.left_clearance_m <= cap
        assert result.center_clearance_m <= cap
        assert result.right_clearance_m <= cap

    def test_distance_is_exactly_cap_when_cap_equals_minimum(self):
        """
        When max_distance == 0.1 the formula collapses:
        min(max(x * 0.7, 0.1), 0.1) == 0.1 for all x ≥ 0.
        """
        result = planner.plan_move(_b64(self.FRAME), max_distance=0.1)
        assert result.distance_metres == pytest.approx(0.1, abs=0.005)

    def test_larger_cap_gives_same_or_greater_distance(self):
        """
        Raising the cap can only increase or preserve the output distance,
        never decrease it.
        """
        b64 = _b64(self.FRAME)
        tight = planner.plan_move(b64, max_distance=0.5)
        loose = planner.plan_move(b64, max_distance=5.0)
        assert loose.distance_metres >= tight.distance_metres - 0.005


# ── Determinism ───────────────────────────────────────────────────────────────

@pytest.mark.parametrize("frame_path", ALL_FRAMES, ids=lambda p: p.name)
def test_deterministic(frame_path):
    """plan_move is a pure function — identical bytes in must give identical output."""
    b64 = _b64(frame_path)
    a = planner.plan_move(b64)
    b = planner.plan_move(b64)
    assert a.direction == b.direction
    assert a.distance_metres == b.distance_metres
    assert a.left_clearance_m == b.left_clearance_m
    assert a.center_clearance_m == b.center_clearance_m
    assert a.right_clearance_m == b.right_clearance_m


# ── Scene-specific behavioral tests ───────────────────────────────────────────

class TestSceneBehavior:
    """
    Behavioral assertions grounded in visual inspection of specific fixture
    frames and verified against actual algorithm output.

    The texture model classifies rough carpet as floor and smooth plywood as
    obstacle.  Obstacle faces appear in the upper ROI (far side of obstacle);
    carpet appears below the obstacle's bottom edge.
    """

    def test_plywood_dead_end_returns_reverse(self):
        """
        frame_0004: smooth plywood fills centre and right (~0.63 m and
        ~0.59 m); only a narrow carpet strip is visible on the left (~1.24 m
        optical, ratio 2.1× vs min(S,R)).  The zone-consistency check flags
        this as a dead-end and clamps left to 0.1 m.  The left zone is more
        open than the right in the original probes, so the obstacle is on the
        right → direction 'reverse+right'.
        """
        result = planner.plan_move(_b64(FIXTURES / "frame_0004.jpg"))
        assert result.direction in ("reverse", "reverse+left", "reverse+right"), (
            f"Expected a reverse variant for frame_0004 (dead-end plywood), "
            f"got '{result.direction}' "
            f"(L={result.left_clearance_m:.2f}m "
            f"S={result.center_clearance_m:.2f}m "
            f"R={result.right_clearance_m:.2f}m)"
        )
        assert result.left_clearance_m   < REVERSE_THRESHOLD
        assert result.center_clearance_m < REVERSE_THRESHOLD
        assert result.right_clearance_m  < REVERSE_THRESHOLD

    def test_open_zone_higher_clearance_than_obstructed_zone(self):
        """
        The left zone of frame_0007 (open room, carpet to horizon) must report
        more clearance than the left zone of frame_0004 (carpet visible but
        only at moderate depth).
        """
        open_room = planner.plan_move(_b64(FIXTURES / "frame_0007.jpg"))
        near_wall = planner.plan_move(_b64(FIXTURES / "frame_0004.jpg"))
        assert open_room.left_clearance_m > near_wall.left_clearance_m, (
            f"open room left={open_room.left_clearance_m:.2f}m should exceed "
            f"obstructed left={near_wall.left_clearance_m:.2f}m"
        )

    def test_bench_blocks_center_right_left_zone_clear(self):
        """
        frame_0003: plywood bench fills the centre and right zones; carpet
        is visible on the left.  Left clearance must exceed both centre and
        right, and direction must be 'left' (away from the bench).
        """
        result = planner.plan_move(_b64(FIXTURES / "frame_0003.jpg"))
        assert result.left_clearance_m > result.center_clearance_m, (
            f"left={result.left_clearance_m:.2f}m should exceed "
            f"center={result.center_clearance_m:.2f}m for frame_0003"
        )
        assert result.left_clearance_m > result.right_clearance_m, (
            f"left={result.left_clearance_m:.2f}m should exceed "
            f"right={result.right_clearance_m:.2f}m for frame_0003"
        )
        assert result.direction == "left", (
            f"Expected 'left' for frame_0003 (bench on right), "
            f"got '{result.direction}'"
        )

    def test_chair_legs_block_left_and_centre(self):
        """
        frame_0006: black metal chair legs fill left and centre zones.
        Dark-row detection must cap those clearances below the right zone,
        and direction must be 'right' (away from the chairs).
        """
        result = planner.plan_move(_b64(FIXTURES / "frame_0006.jpg"))
        assert result.left_clearance_m < result.right_clearance_m, (
            f"left={result.left_clearance_m:.2f}m should be less than "
            f"right={result.right_clearance_m:.2f}m for frame_0006 (chairs on left)"
        )
        assert result.center_clearance_m < result.right_clearance_m, (
            f"center={result.center_clearance_m:.2f}m should be less than "
            f"right={result.right_clearance_m:.2f}m for frame_0006 (chairs reach centre)"
        )
        assert result.direction == "right", (
            f"Expected 'right' for frame_0006 (chair legs block left/centre), "
            f"got '{result.direction}'"
        )

    def test_box_on_right_lower_clearance(self):
        """
        frame_0009: plywood box fills the right zone (no carpet detected →
        small clearance); carpet is clear on left and centre.  Right clearance
        must be less than left clearance.
        """
        result = planner.plan_move(_b64(FIXTURES / "frame_0009.jpg"))
        assert result.right_clearance_m < result.left_clearance_m, (
            f"right={result.right_clearance_m:.2f}m should be less than "
            f"left={result.left_clearance_m:.2f}m for frame_0009 (box on right)"
        )

    @pytest.mark.parametrize("frame_name", [
        "frame_0004.jpg",
        "frame_0010.jpg", "frame_0011.jpg", "frame_0012.jpg", "frame_0013.jpg",
    ])
    def test_box_fills_all_zones_returns_reverse(self, frame_name):
        """
        frame_0004: plywood fills centre/right; narrow carpet strip at 1.24 m
        on the left — zone-consistency ratio 2.1× exceeds ZONE_JUMP_RATIO.
        frames 0010–0013: large box at < 0.1 m blocks centre and right; a
        doorway gap gives optical clearance of 3–6 m on the left, but it is
        physically unreachable (ratio 5–10×).
        In all cases all probes fall below REVERSE_THRESHOLD → 'reverse'.
        """
        result = planner.plan_move(_b64(FIXTURES / frame_name))
        assert result.direction in ("reverse", "reverse+left", "reverse+right"), (
            f"Expected a reverse variant for {frame_name} (box dead-end), "
            f"got '{result.direction}' "
            f"(L={result.left_clearance_m:.2f}m "
            f"S={result.center_clearance_m:.2f}m "
            f"R={result.right_clearance_m:.2f}m)"
        )
        assert result.left_clearance_m   < REVERSE_THRESHOLD
        assert result.center_clearance_m < REVERSE_THRESHOLD
        assert result.right_clearance_m  < REVERSE_THRESHOLD

    def test_open_room_recommends_non_trivial_distance(self):
        """frame_0007 has a clear open room; distance must exceed the 0.1 m floor."""
        result = planner.plan_move(_b64(FIXTURES / "frame_0007.jpg"))
        assert result.distance_metres > 0.1, (
            f"Expected distance > 0.1 m for open room, got {result.distance_metres} m"
        )

    def test_all_clearances_reported_in_reasoning(self):
        """The reasoning string must contain L=, S=, and R= for every frame."""
        for frame_path in ALL_FRAMES:
            result = planner.plan_move(_b64(frame_path))
            for marker in ("L=", "S=", "R="):
                assert marker in result.reasoning, (
                    f"{frame_path.name}: missing '{marker}' in reasoning: "
                    f"'{result.reasoning}'"
                )


# ── Door detection ────────────────────────────────────────────────────────────

class TestDoorDetection:
    """
    Tests for detect_door() and plan_move() with target_zone.

    The calibration fixture is frame_0008_door.jpg: a lit room visible through
    a glass partition at ~4 m gives a column-brightness prominence of ~45,
    well above the DOOR_PROMINENCE threshold of 40.  All other frames in the
    suite have prominence ≤ 30 and must return None.
    """

    DOOR_FRAME = FIXTURES / "frame_0008_door.jpg"

    def test_detects_door_in_straight_zone(self):
        """frame_0008_door: lit glass partition in the centre → 'straight'."""
        result = detect_door(_b64(self.DOOR_FRAME))
        assert result == "straight", (
            f"Expected detect_door to return 'straight', got {result!r}"
        )

    @pytest.mark.parametrize(
        "frame_path",
        [f for f in ALL_FRAMES if f.name != "frame_0008_door.jpg"],
        ids=lambda p: p.name,
    )
    def test_no_false_positives(self, frame_path):
        """detect_door must return None for every frame that has no doorway."""
        result = detect_door(_b64(frame_path))
        assert result is None, (
            f"False positive: detect_door returned {result!r} for {frame_path.name}"
        )

    def test_plan_move_with_target_zone_steers_toward_door(self):
        """
        With target_zone='straight' the planner overrides the left/straight
        tie and returns direction='straight'.  Distance is the same 3.5 m
        because both zones have 5.0 m clearance.
        """
        result = planner.plan_move(_b64(self.DOOR_FRAME), target_zone="straight")
        assert result.direction == "straight", (
            f"Expected 'straight' with target_zone='straight', "
            f"got '{result.direction}'"
        )
        assert result.distance_metres == pytest.approx(3.5, abs=0.005)

    def test_plan_move_without_target_zone_leaves_direction_unchanged(self):
        """Without target_zone, standard left/straight tie-breaking wins ('left')."""
        result = planner.plan_move(_b64(self.DOOR_FRAME))
        assert result.direction == "left"

    def test_target_zone_ignored_when_zone_is_blocked(self):
        """
        If the requested target_zone has less than TARGET_ZONE_MIN_CLEARANCE,
        the planner falls back to best-clearance selection.  Use frame_0001
        (all zones blocked) with target_zone='straight': the planner must
        return 'reverse', not 'straight'.
        """
        result = planner.plan_move(
            _b64(FIXTURES / "frame_0001.jpg"), target_zone="straight"
        )
        assert result.direction == "reverse", (
            f"Expected 'reverse' (blocked scene), got '{result.direction}'"
        )


# ── Per-fixture snapshot tests ────────────────────────────────────────────────

class TestFixtureSnapshots:
    """
    Regression tests: one test per fixture image recording the exact output
    of plan_move.  These make it easy to see how algorithm changes affect
    specific real-world scenes and help developers build intuition for what
    the planner does with each image.

    If you intentionally change the algorithm constants (HORIZON_FRAC,
    DISTANCE_K, FLOOR_TEXTURE_THRESHOLD, etc.) update the expected values by
    re-running:

        python3 -c "
        import base64, pathlib, move_planner as p
        for f in sorted(pathlib.Path('tests/fixtures').glob('frame_*.jpg')):
            r = p.plan_move(base64.b64encode(f.read_bytes()).decode())
            print(f.name, r.direction, r.distance_metres,
                  r.left_clearance_m, r.center_clearance_m, r.right_clearance_m)
        "
    """

    def test_frame_0001(self):
        """
        Camera pressed into the junction of two plywood walls (a corner).
        Smooth plywood fills all three zones — all probe clearances are below
        REVERSE_THRESHOLD → direction 'reverse', distance REVERSE_DISTANCE_M.
        Left zone has no detectable floor; centre and right have tiny carpet
        peaks at y_norm≈0.85 (~0.69 m) but those remain below the threshold.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0001.jpg"))
        assert r.direction          == "reverse"
        assert r.distance_metres    == pytest.approx(REVERSE_DISTANCE_M, abs=0.005)
        assert r.left_clearance_m   == pytest.approx(0.1, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(0.6947465155460254, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(0.6884200940962208, abs=1e-6)

    def test_frame_0002(self):
        """
        Left and centre zones have carpet visible from the horizon → 5.0 m.
        Right zone is partially blocked by the bench/furniture at ~1.87 m.
        Left wins the tie with centre (first dict key) → direction 'left'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0002.jpg"))
        assert r.direction          == "left"
        assert r.distance_metres    == 3.5
        assert r.left_clearance_m   == pytest.approx(5.0, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(5.0, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(1.8674351585014404, abs=1e-6)

    def test_frame_0003(self):
        """
        Plywood bench fills the centre and right zones (smooth → low texture).
        Carpet is visible on the left extending to the horizon → 5.0 m.
        Centre obstruction detected at ~1.59 m; right blocked immediately
        (~0.58 m).  Planner correctly avoids the bench → direction 'left'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0003.jpg"))
        assert r.direction          == "left"
        assert r.distance_metres    == 3.5
        assert r.left_clearance_m   == pytest.approx(5.0, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(1.5949367088607593, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(0.5831083686849209, abs=1e-6)

    def test_frame_0004(self):
        """
        Smooth plywood fills the centre and right zones (~0.63 m and ~0.59 m).
        A narrow carpet strip is optically visible on the left at ~1.24 m, but
        the zone-consistency check (L 2.1× min(S,R) > ZONE_JUMP_RATIO=1.5)
        recognises this as a dead-end and clamps L to 0.1 m.  All probes are
        below REVERSE_THRESHOLD.  Original L probe (1.24 m) > R probe (0.59 m)
        by more than REVERSE_STEER_MARGIN and exceeds REVERSE_STEER_MIN_CLEARANCE,
        so the obstacle is on the right → direction 'reverse+right'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0004.jpg"))
        assert r.direction          == "reverse+right"
        assert r.distance_metres    == pytest.approx(REVERSE_DISTANCE_M, abs=0.005)
        assert r.left_clearance_m   == pytest.approx(0.1, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(0.6322832450515751, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(0.5861222380152473, abs=1e-6)

    def test_frame_0005(self):
        """
        Carpet visible from the horizon in all three zones → all return
        5.0 m.  Left wins the three-way tie → direction 'left'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0005.jpg"))
        assert r.direction          == "left"
        assert r.distance_metres    == 3.5
        assert r.left_clearance_m   == pytest.approx(5.0, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(5.0, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(5.0, abs=1e-6)

    def test_frame_0006(self):
        """
        Black metal chair legs fill the left and centre zones.  The darkness
        layer detects dense dark rows at y_norm≈0.53 in both zones, capping
        their clearance at ~1.99 m and ~1.91 m respectively.  The right zone
        has no dark obstacle rows → 5.0 m → direction 'right'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0006.jpg"))
        assert r.direction          == "right"
        assert r.distance_metres    == 3.5
        assert r.left_clearance_m   == pytest.approx(1.990346643264589,  abs=1e-6)
        assert r.center_clearance_m == pytest.approx(1.910699241786015,  abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(5.0, abs=1e-6)

    def test_frame_0007(self):
        """
        Open room.  Left and centre have carpet from the horizon → 5.0 m.
        Right zone partially obstructed (carpet visible but further in at
        ~3.14 m).  Left wins the left/centre tie → direction 'left'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0007.jpg"))
        assert r.direction          == "left"
        assert r.distance_metres    == 3.5
        assert r.left_clearance_m   == pytest.approx(5.0, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(5.0, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(3.141274238227146, abs=1e-6)

    def test_frame_0008(self):
        """
        Left and centre show carpet from the horizon → 5.0 m.  Right zone
        obstructed at ~3.37 m (bench on right).  Left wins the left/centre
        tie → direction 'left'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0008.jpg"))
        assert r.direction          == "left"
        assert r.distance_metres    == 3.5
        assert r.left_clearance_m   == pytest.approx(5.0, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(5.0, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(3.3749999999999987, abs=1e-6)

    def test_frame_0008_door(self):
        """
        Lit doorway (glass partition + room beyond) in the centre zone at ~4 m.
        Centre probe clearance is 24 m (floor visible through glass); both left
        and centre are capped at 5.0 m.  Without target_zone, left wins the tie.
        detect_door() returns 'straight'; plan_move(target_zone='straight') steers
        the car through the door at 3.5 m.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0008_door.jpg"))
        assert r.direction          == "left"
        assert r.distance_metres    == 3.5
        assert r.left_clearance_m   == pytest.approx(5.0, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(5.0, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(4.6570841889117025, abs=1e-6)

    def test_frame_0009(self):
        """
        Plywood box on the right — smooth face → no carpet detected until
        very close (~1.34 m).  Left and centre have carpet from the horizon
        → 5.0 m.  Left wins the left/centre tie → direction 'left'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0009.jpg"))
        assert r.direction          == "left"
        assert r.distance_metres    == 3.5
        assert r.left_clearance_m   == pytest.approx(5.0, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(5.0, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(1.3443983402489625, abs=1e-6)

    def test_frame_0010(self):
        """
        Large plywood box directly ahead fills centre and right (~0.69 m and
        ~0.59 m).  Carpet visible through a doorway gap on the left optically
        reads as ~5.9 m, but the zone-consistency check recognises it as an
        unreachable optical gap (L >> S ≈ R) and clamps it to 0.1 m.  All
        probes are below REVERSE_THRESHOLD.  Original L probe (5.9 m) >> R probe
        (0.59 m), so the obstacle is on the right → direction 'reverse+right'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0010.jpg"))
        assert r.direction          == "reverse+right"
        assert r.distance_metres    == pytest.approx(REVERSE_DISTANCE_M, abs=0.005)
        assert r.left_clearance_m   == pytest.approx(0.1, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(0.6884200940962208, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(0.5868805796351404, abs=1e-6)

    def test_frame_0011(self):
        """
        Same scene as frame_0010, slightly further back.  Doorway gap on the
        left optically shows ~3.36 m, but the zone-consistency check clamps it
        to 0.1 m.  Original L probe >> R probe → obstacle on right →
        direction 'reverse+right'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0011.jpg"))
        assert r.direction          == "reverse+right"
        assert r.distance_metres    == pytest.approx(REVERSE_DISTANCE_M, abs=0.005)
        assert r.left_clearance_m   == pytest.approx(0.1, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(0.6376159685127916, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(0.587260486794407, abs=1e-6)

    def test_frame_0012(self):
        """
        Same scene as frame_0010/0011, slightly further back.  Doorway gap
        optically ~3.21 m; consistency check clamps to 0.1 m.  Original L probe
        >> R probe → obstacle on right → direction 'reverse+right'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0012.jpg"))
        assert r.direction          == "reverse+right"
        assert r.distance_metres    == pytest.approx(REVERSE_DISTANCE_M, abs=0.005)
        assert r.left_clearance_m   == pytest.approx(0.1, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(0.6349384098544232, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(0.5861222380152473, abs=1e-6)

    def test_frame_0013(self):
        """
        Same scene as frame_0010–0012, slightly further back.  Doorway gap
        optically ~3.11 m; consistency check clamps to 0.1 m.  Original L probe
        >> R probe → obstacle on right → direction 'reverse+right'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0013.jpg"))
        assert r.direction          == "reverse+right"
        assert r.distance_metres    == pytest.approx(REVERSE_DISTANCE_M, abs=0.005)
        assert r.left_clearance_m   == pytest.approx(0.1, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(0.6296501943364796, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(0.5846114189972934, abs=1e-6)

    def test_frame_0014(self):
        """
        Carpet visible at the horizon in all three zones → all return 5.0 m.
        Left wins the three-way tie → direction 'left'.
        """
        r = planner.plan_move(_b64(FIXTURES / "frame_0014.jpg"))
        assert r.direction          == "left"
        assert r.distance_metres    == 3.5
        assert r.left_clearance_m   == pytest.approx(5.0, abs=1e-6)
        assert r.center_clearance_m == pytest.approx(5.0, abs=1e-6)
        assert r.right_clearance_m  == pytest.approx(5.0, abs=1e-6)
