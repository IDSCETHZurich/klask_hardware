"""Sprite asset extraction script.

Processes captured Klask board images to extract individual sprite PNGs
(with transparent backgrounds) and offset metadata for each object
(left peg, right peg, ball).
"""

import json
import logging
import sys
import threading
from dataclasses import dataclass, field, asdict
from enum import Enum
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


class ObjectType(Enum):
    """Types of objects we want to extract sprites for."""

    LEFT_PEG = "left_peg"
    RIGHT_PEG = "right_peg"
    BALL = "ball"


@dataclass
class BoardConfig:
    """Physical dimensions of the board."""

    width_m: float = 0.42
    height_m: float = 0.32


@dataclass
class BallSegConfig:
    """Parameters for ball segmentation in HSV color space."""

    h_range: Tuple[int, int] = (15, 40)
    s_min: int = 80
    v_min: int = 120


@dataclass
class PegSegConfig:
    """Parameters for peg segmentation."""

    v_max: int = 60
    bg_diff_threshold: int = 25


@dataclass
class MorphConfig:
    """Parameters for morphological operations."""

    kernel_size: int = 3
    open_iterations: int = 1
    close_iterations: int = 2


@dataclass
class ComponentConfig:
    """Parameters for filtering connected components."""

    min_area_px: int = 30
    max_area_px: int = 800


@dataclass
class SegmentationConfig:
    """Configuration for the segmentation pipeline."""

    rough_crop_radius_px: int = 40
    ball: BallSegConfig = field(default_factory=BallSegConfig)
    peg: PegSegConfig = field(default_factory=PegSegConfig)
    morphology: MorphConfig = field(default_factory=MorphConfig)
    component: ComponentConfig = field(default_factory=ComponentConfig)
    sprite_padding_px: int = 3
    alpha_blur_sigma: float = 0.0


class DebugMode(Enum):
    """How to present debug images."""

    OFF = "off"  # no debug output
    SAVE = "save"  # save to disk
    SHOW = "show"  # display in OpenCV window one by one


@dataclass
class ProcessConfig:
    """Which object types to process."""

    peg: bool = True
    ball: bool = True
    background: bool = True


@dataclass
class Config:
    """Overall configuration for sprite processing, loaded from YAML."""

    data_dir: Path = Path(".")
    output_dir: Path = Path("./sprite_output")
    board: BoardConfig = field(default_factory=BoardConfig)
    seg: SegmentationConfig = field(default_factory=SegmentationConfig)
    process: ProcessConfig = field(default_factory=ProcessConfig)
    debug_mode: DebugMode = DebugMode.OFF


@dataclass
class SpriteResult:
    """Result of processing a single sprite, for manifest reporting."""

    object_type: str
    source_image: str
    position_m: Tuple[float, float]
    position_px: Tuple[int, int]
    sprite_path: Optional[str] = None
    sprite_size_px: Optional[Tuple[int, int]] = None
    offset_px: Optional[Tuple[float, float]] = None
    success: bool = False
    failure_reason: Optional[str] = None


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------


def load_config(config_path: Path) -> Config:
    """Load configuration from a YAML file."""
    with open(config_path) as f:
        raw = yaml.safe_load(f)

    config_dir = config_path.parent

    # Resolve paths relative to config file
    paths = raw.get("paths", {})
    data_dir = Path(paths.get("data_dir", "."))
    output_dir = Path(paths.get("output_dir", "./sprite_output"))
    if not data_dir.is_absolute():
        data_dir = (config_dir / data_dir).resolve()
    if not output_dir.is_absolute():
        output_dir = (config_dir / output_dir).resolve()

    # Board
    brd = raw.get("board", {})
    board = BoardConfig(
        width_m=brd.get("width_m", 0.42),
        height_m=brd.get("height_m", 0.32),
    )

    # Segmentation
    s = raw.get("segmentation", {})
    ball_raw = s.get("ball", {})
    peg_raw = s.get("peg", {})
    morph_raw = s.get("morphology", {})
    comp_raw = s.get("component", {})

    ball_h = ball_raw.get("h_range", [15, 40])
    seg = SegmentationConfig(
        rough_crop_radius_px=s.get("rough_crop_radius_px", 40),
        ball=BallSegConfig(
            h_range=(ball_h[0], ball_h[1]),
            s_min=ball_raw.get("s_min", 80),
            v_min=ball_raw.get("v_min", 120),
        ),
        peg=PegSegConfig(
            v_max=peg_raw.get("v_max", 60),
            bg_diff_threshold=peg_raw.get("bg_diff_threshold", 25),
        ),
        morphology=MorphConfig(
            kernel_size=morph_raw.get("kernel_size", 3),
            open_iterations=morph_raw.get("open_iterations", 1),
            close_iterations=morph_raw.get("close_iterations", 2),
        ),
        component=ComponentConfig(
            min_area_px=comp_raw.get("min_area_px", 30),
            max_area_px=comp_raw.get("max_area_px", 800),
        ),
        sprite_padding_px=s.get("sprite_padding_px", 3),
        alpha_blur_sigma=s.get("alpha_blur_sigma", 0.0),
    )

    proc_raw = raw.get("process", {})
    process = ProcessConfig(
        peg=proc_raw.get("peg", True),
        ball=proc_raw.get("ball", True),
        background=proc_raw.get("background", True),
    )

    debug = raw.get("debug", {})
    debug_mode_str = debug.get("mode", "off")
    try:
        debug_mode = DebugMode(debug_mode_str)
    except ValueError:
        log.warning("Unknown debug mode '%s', falling back to 'off'", debug_mode_str)
        debug_mode = DebugMode.OFF

    return Config(
        data_dir=data_dir,
        output_dir=output_dir,
        board=board,
        seg=seg,
        process=process,
        debug_mode=debug_mode,
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def load_labels(data_dir: Path) -> dict:
    """Load labels.json from the data directory."""
    labels_path = data_dir / "labels.json"
    with open(labels_path) as f:
        return json.load(f)


def meters_to_pixels(xy_m: Tuple[float, float], board: BoardConfig, image_shape: Tuple[int, ...]) -> Tuple[int, int]:
    """Convert (x_m, y_m) in meters to (px_x, px_y) in pixels."""
    img_h, img_w = image_shape[:2]
    px = int(round(xy_m[0] * img_w / board.width_m))
    py = int(round(xy_m[1] * img_h / board.height_m))
    return (px, py)


def compute_median_background(data_dir: Path, labels: dict) -> np.ndarray:
    """Compute a median background from all background samples."""
    bg_samples = [s for s in labels["samples"] if s["image_relpath"].startswith("background/")]
    if not bg_samples:
        raise RuntimeError("No background samples found in labels.json")

    frames = []
    for s in bg_samples:
        img_path = data_dir / s["image_relpath"]
        img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"Failed to load background image: {img_path}")
        frames.append(img.astype(np.float32))

    median = np.median(np.stack(frames, axis=0), axis=0).astype(np.uint8)
    log.info("Computed median background from %d frames", len(frames))
    return median


def rough_crop(image: np.ndarray, center_px: Tuple[int, int], radius: int) -> Tuple[np.ndarray, int, int]:
    """Extract a square region centered on center_px, clamped to image bounds.

    Returns (cropped_image, crop_origin_x, crop_origin_y).
    """
    h, w = image.shape[:2]
    cx, cy = center_px
    x0 = max(0, cx - radius)
    y0 = max(0, cy - radius)
    x1 = min(w, cx + radius)
    y1 = min(h, cy + radius)
    return image[y0:y1, x0:x1].copy(), x0, y0


def segment_ball(
    crop_bgr: np.ndarray,
    bg_diff: np.ndarray,
    cfg: SegmentationConfig,
) -> np.ndarray:
    """Create a binary mask for the ball (yellow object)."""
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    h_lo, h_hi = cfg.ball.h_range
    mask_hsv = cv2.inRange(
        hsv,
        (h_lo, cfg.ball.s_min, cfg.ball.v_min),
        (h_hi, 255, 255),
    )

    # Refine with background diff — reject static yellow-ish artifacts
    diff_gray = cv2.cvtColor(bg_diff, cv2.COLOR_BGR2GRAY)
    _, diff_mask = cv2.threshold(diff_gray, 30, 255, cv2.THRESH_BINARY)

    # TODO: it looks like the diff_mask is actually hurting more than helping.

    return mask_hsv  # cv2.bitwise_and(mask_hsv, diff_mask)


def segment_peg(
    crop_bgr: np.ndarray,
    bg_diff: np.ndarray,
    cfg: SegmentationConfig,
) -> np.ndarray:
    """Create a binary mask for a peg (dark object)."""
    # Background subtraction: pixels that changed significantly
    diff_gray = cv2.cvtColor(bg_diff, cv2.COLOR_BGR2GRAY)
    _, diff_mask = cv2.threshold(diff_gray, cfg.peg.bg_diff_threshold, 255, cv2.THRESH_BINARY)

    # Darkness filter on original crop
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    dark_mask = cv2.inRange(hsv, (0, 0, 0), (179, 255, cfg.peg.v_max))

    return cv2.bitwise_and(diff_mask, dark_mask)


def morphological_cleanup(mask: np.ndarray, cfg: MorphConfig) -> np.ndarray:
    """Apply morphological open then close to clean up the mask."""
    ks = cfg.kernel_size
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ks, ks))
    cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=cfg.open_iterations)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=cfg.close_iterations)
    return cleaned


def select_best_component(
    mask: np.ndarray,
    expected_center_in_crop: Tuple[int, int],
    cfg: ComponentConfig,
) -> np.ndarray:
    """Pick the connected component closest to expected_center_in_crop, filtered by area range.

    Returns a clean binary mask.
    """
    num_labels, labels_img, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

    best_label = -1
    best_dist = float("inf")
    ex, ey = expected_center_in_crop

    for label in range(1, num_labels):  # skip background (0)
        area = stats[label, cv2.CC_STAT_AREA]
        if area < cfg.min_area_px or area > cfg.max_area_px:
            continue
        cx, cy = centroids[label]
        dist = (cx - ex) ** 2 + (cy - ey) ** 2
        if dist < best_dist:
            best_dist = dist
            best_label = label

    if best_label < 0:
        return np.zeros_like(mask)

    return ((labels_img == best_label) * 255).astype(np.uint8)


def apply_alpha_blur(mask: np.ndarray, sigma: float) -> np.ndarray:
    """Optionally blur the mask for anti-aliased alpha edges."""
    if sigma <= 0:
        return mask
    blurred = cv2.GaussianBlur(mask.astype(np.float32), (0, 0), sigma)
    return np.clip(blurred, 0, 255).astype(np.uint8)


def create_rgba(crop_bgr: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Merge BGR crop with mask as alpha channel → RGBA image."""
    b, g, r = cv2.split(crop_bgr)
    return cv2.merge([b, g, r, mask])


def tight_crop_sprite(rgba: np.ndarray, mask: np.ndarray, padding: int) -> Tuple[Optional[np.ndarray], int, int]:
    """Crop RGBA to the bounding box of non-zero alpha, with padding.

    Returns (cropped_rgba, tight_origin_x, tight_origin_y) or
    (None, 0, 0) if the mask is empty.
    """
    coords = cv2.findNonZero(mask)
    if coords is None:
        return None, 0, 0

    x, y, w, h = cv2.boundingRect(coords)
    rh, rw = rgba.shape[:2]

    x0 = max(0, x - padding)
    y0 = max(0, y - padding)
    x1 = min(rw, x + w + padding)
    y1 = min(rh, y + h + padding)

    return rgba[y0:y1, x0:x1].copy(), x0, y0


def save_debug_images(debug_dir: Path, stages: dict) -> None:
    """Save a dict of {filename: image} to the debug directory."""
    debug_dir.mkdir(parents=True, exist_ok=True)
    for name, img in stages.items():
        path = debug_dir / name
        cv2.imwrite(str(path), img)


TILE_HEIGHT = 150  # target height for each tile in the combined debug image
CHECKER_SIZE = 8  # checkerboard square size in pixels


def _make_checkerboard(shape: Tuple[int, int]) -> np.ndarray:
    """Create a grey checkerboard background (H x W x 3, float32 0-255)."""
    h, w = shape
    rows = np.arange(h) // CHECKER_SIZE
    cols = np.arange(w) // CHECKER_SIZE
    checker = ((rows[:, None] + cols[None, :]) % 2).astype(np.float32)
    light, dark = 200.0, 140.0
    checker = np.where(checker, light, dark)
    return np.stack([checker, checker, checker], axis=-1)


def show_debug_images(title_prefix: str, stages: dict) -> None:
    """Tile all debug stages into one image and display it in a single window."""
    tiles = []
    for name, img in stages.items():
        tile = img.copy()

        # Normalise to 3-channel BGR for display
        if tile.ndim == 2:
            tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
        elif tile.shape[2] == 4:
            # Composite against checkerboard so transparency is visible
            bgr = tile[:, :, :3].astype(np.float32)
            alpha = tile[:, :, 3:4].astype(np.float32) / 255.0
            checker = _make_checkerboard(tile.shape[:2])
            tile = (bgr * alpha + checker * (1.0 - alpha)).astype(np.uint8)

        # Resize to target height, keep aspect ratio
        h, w = tile.shape[:2]
        scale = TILE_HEIGHT / h
        tile = cv2.resize(tile, (max(1, int(w * scale)), TILE_HEIGHT), interpolation=cv2.INTER_NEAREST)

        # Add stage label
        label = name.replace(".png", "")
        cv2.putText(tile, label, (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

        tiles.append(tile)

    combined = np.hstack(tiles)
    cv2.imshow(title_prefix, combined)

    # Run input() in a thread so the main thread can keep driving the Qt event loop
    entered = threading.Event()
    threading.Thread(
        target=lambda: (input("Press Enter to continue to next sample..."), entered.set()), daemon=True
    ).start()
    while not entered.is_set():
        cv2.waitKey(100)

    cv2.destroyAllWindows()
    cv2.waitKey(1)


# ---------------------------------------------------------------------------
# Per-object processing
# ---------------------------------------------------------------------------


def process_single_object(
    image: np.ndarray,
    median_bg: np.ndarray,
    center_m: Tuple[float, float],
    object_type: ObjectType,
    sample_idx: int,
    source_image: str,
    cfg: Config,
) -> SpriteResult:
    """Run the full extraction pipeline for one object instance."""
    board = cfg.board
    seg = cfg.seg
    center_px = meters_to_pixels(center_m, board, image.shape)

    result = SpriteResult(
        object_type=object_type.value,
        source_image=source_image,
        position_m=center_m,
        position_px=center_px,
    )

    # Determine output paths
    type_dir = "peg" if object_type in (ObjectType.LEFT_PEG, ObjectType.RIGHT_PEG) else "ball"
    name = f"{object_type.value}_{sample_idx:03d}"
    sprite_dir = cfg.output_dir / "sprites" / type_dir
    sprite_dir.mkdir(parents=True, exist_ok=True)
    sprite_path = sprite_dir / f"{name}.png"
    meta_path = sprite_dir / f"{name}.json"

    debug_stages = {}

    # Step 1: Rough crop
    radius = seg.rough_crop_radius_px
    crop, crop_ox, crop_oy = rough_crop(image, center_px, radius)
    bg_crop, _, _ = rough_crop(median_bg, center_px, radius)
    debug_stages["01_rough_crop.png"] = crop

    # Expected center within the crop
    local_cx = center_px[0] - crop_ox
    local_cy = center_px[1] - crop_oy

    # Step 2: Background subtraction
    bg_diff = cv2.absdiff(crop, bg_crop)
    debug_stages["02_bg_subtracted.png"] = bg_diff

    # Step 3: HSV segmentation
    hsv_crop = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    debug_stages["03_hsv_h.png"] = hsv_crop[:, :, 0]
    debug_stages["03_hsv_s.png"] = hsv_crop[:, :, 1]
    debug_stages["03_hsv_v.png"] = hsv_crop[:, :, 2]

    if object_type == ObjectType.BALL:
        raw_mask = segment_ball(crop, bg_diff, seg)
    else:
        raw_mask = segment_peg(crop, bg_diff, seg)
    debug_stages["04_binary_mask.png"] = raw_mask

    # Step 4: Morphological cleanup
    morphed = morphological_cleanup(raw_mask, seg.morphology)
    debug_stages["05_morphed_mask.png"] = morphed

    # Step 5: Connected component selection
    component_mask = select_best_component(morphed, (local_cx, local_cy), seg.component)
    debug_stages["06_component_mask.png"] = component_mask

    if cv2.countNonZero(component_mask) == 0:
        result.failure_reason = "No valid component found after filtering"
        log.warning("FAILED %s: %s", name, result.failure_reason)
        if cfg.debug_mode == DebugMode.SAVE:
            save_debug_images(cfg.output_dir / "debug" / type_dir / name, debug_stages)
        elif cfg.debug_mode == DebugMode.SHOW:
            show_debug_images(name, debug_stages)
        return result

    # Step 6: Apply alpha (with optional blur)
    alpha = apply_alpha_blur(component_mask, seg.alpha_blur_sigma)
    rgba = create_rgba(crop, alpha)
    debug_stages["07_masked_rgba.png"] = rgba

    # Step 7: Tight crop + offset
    sprite, tight_ox, tight_oy = tight_crop_sprite(rgba, component_mask, seg.sprite_padding_px)
    if sprite is None:
        result.failure_reason = "Tight crop produced empty sprite"
        log.warning("FAILED %s: %s", name, result.failure_reason)
        if cfg.debug_mode == DebugMode.SAVE:
            save_debug_images(cfg.output_dir / "debug" / type_dir / name, debug_stages)
        elif cfg.debug_mode == DebugMode.SHOW:
            show_debug_images(name, debug_stages)
        return result

    debug_stages["08_final_sprite.png"] = sprite

    # Offset: from sprite top-left to object center
    offset_x = float(local_cx - tight_ox)
    offset_y = float(local_cy - tight_oy)

    # Step 8: Save
    cv2.imwrite(str(sprite_path), sprite)

    sh, sw = sprite.shape[:2]
    meta = {
        "object_type": object_type.value,
        "source_image": source_image,
        "position_m": list(center_m),
        "position_px": list(center_px),
        "sprite_size_px": [sw, sh],
        "offset_px": [round(offset_x, 2), round(offset_y, 2)],
    }
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)

    result.sprite_path = str(sprite_path.relative_to(cfg.output_dir))
    result.sprite_size_px = (sw, sh)
    result.offset_px = (round(offset_x, 2), round(offset_y, 2))
    result.success = True

    log.info("OK  %s  size=%dx%d  offset=(%.1f, %.1f)", name, sw, sh, offset_x, offset_y)

    if cfg.debug_mode == DebugMode.SAVE:
        save_debug_images(cfg.output_dir / "debug" / type_dir / name, debug_stages)
    elif cfg.debug_mode == DebugMode.SHOW:
        show_debug_images(name, debug_stages)

    return result


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------


def process_all(cfg: Config) -> List[SpriteResult]:
    """Process all samples and return a list of SpriteResults."""
    labels = load_labels(cfg.data_dir)
    median_bg = compute_median_background(cfg.data_dir, labels)

    if cfg.process.background:
        bg_dir = cfg.output_dir / "background"
        bg_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(bg_dir / "median_background.png"), median_bg)
        log.info("Saved median background to %s", bg_dir / "median_background.png")

    results: List[SpriteResult] = []
    peg_idx = 0
    ball_idx = 0

    # Cache loaded images to avoid re-reading the same file
    image_cache: dict = {}

    for sample in labels["samples"]:
        relpath = sample["image_relpath"]
        img_path = cfg.data_dir / relpath

        # Load image (with caching)
        if str(img_path) not in image_cache:
            img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
            if img is None:
                log.error("Failed to load image: %s", img_path)
                continue
            image_cache[str(img_path)] = img
        image = image_cache[str(img_path)]

        # Grid samples: extract both pegs
        if cfg.process.peg:
            if "left_peg_xy" in sample:
                pos = tuple(sample["left_peg_xy"])
                r = process_single_object(image, median_bg, pos, ObjectType.LEFT_PEG, peg_idx, relpath, cfg)
                results.append(r)
                peg_idx += 1

            if "right_peg_xy" in sample:
                pos = tuple(sample["right_peg_xy"])
                r = process_single_object(image, median_bg, pos, ObjectType.RIGHT_PEG, peg_idx, relpath, cfg)
                results.append(r)
                peg_idx += 1

        # Ball samples
        if cfg.process.ball and "ball_xy" in sample:
            pos = tuple(sample["ball_xy"])
            r = process_single_object(image, median_bg, pos, ObjectType.BALL, ball_idx, relpath, cfg)
            results.append(r)
            ball_idx += 1

    return results


def write_manifest(cfg: Config, results: List[SpriteResult]) -> None:
    """Write manifest.json summarizing all generated sprites."""
    success_count = sum(1 for r in results if r.success)
    fail_count = len(results) - success_count

    # Per-type stats
    type_stats = {}
    for r in results:
        t = r.object_type
        if t not in type_stats:
            type_stats[t] = {"success": 0, "fail": 0}
        if r.success:
            type_stats[t]["success"] += 1
        else:
            type_stats[t]["fail"] += 1

    manifest = {
        "summary": {
            "total": len(results),
            "success": success_count,
            "failed": fail_count,
            "by_type": type_stats,
        },
        "config": {
            "board": asdict(cfg.board),
            "segmentation": {
                "rough_crop_radius_px": cfg.seg.rough_crop_radius_px,
                "ball": asdict(cfg.seg.ball),
                "peg": asdict(cfg.seg.peg),
                "morphology": asdict(cfg.seg.morphology),
                "component": asdict(cfg.seg.component),
                "sprite_padding_px": cfg.seg.sprite_padding_px,
                "alpha_blur_sigma": cfg.seg.alpha_blur_sigma,
            },
        },
        "sprites": [],
    }

    for r in results:
        entry = {
            "object_type": r.object_type,
            "source_image": r.source_image,
            "position_m": list(r.position_m),
            "position_px": list(r.position_px),
            "success": r.success,
        }
        if r.success:
            entry["sprite_path"] = r.sprite_path
            entry["sprite_size_px"] = list(r.sprite_size_px)
            entry["offset_px"] = list(r.offset_px)
        else:
            entry["failure_reason"] = r.failure_reason
        manifest["sprites"].append(entry)

    manifest_path = cfg.output_dir / "manifest.json"
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
    log.info("Wrote manifest to %s", manifest_path)


def main():
    """Main entry point."""
    default_config = Path(__file__).resolve().parent.parent / "config" / "sprite_processing.yaml"

    if len(sys.argv) > 2:
        print(f"Usage: {sys.argv[0]} [config.yaml]", file=sys.stderr)
        sys.exit(1)

    config_path = Path(sys.argv[1]).resolve() if len(sys.argv) == 2 else default_config
    if not config_path.exists():
        print(f"Config file not found: {config_path}", file=sys.stderr)
        sys.exit(1)

    log.info("Loading config from %s", config_path)
    cfg = load_config(config_path)
    log.info("Data dir:   %s", cfg.data_dir)
    log.info("Output dir: %s", cfg.output_dir)
    log.info("Debug:      %s", cfg.debug_mode.value)

    cfg.output_dir.mkdir(parents=True, exist_ok=True)

    results = process_all(cfg)
    write_manifest(cfg, results)

    success = sum(1 for r in results if r.success)
    total = len(results)
    log.info("Done: %d/%d sprites extracted successfully", success, total)

    if success < total:
        log.warning(
            "%d sprites failed — check debug images and manifest.json",
            total - success,
        )


if __name__ == "__main__":
    main()
