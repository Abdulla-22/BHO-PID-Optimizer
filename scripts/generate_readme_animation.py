"""Render the README walkthrough using Pillow; no project runtime is required.

Run from any directory: python scripts/generate_readme_animation.py
Install the rendering dependency with: python -m pip install Pillow
The diagrams and response curves are explanatory illustrations, not test data.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import random

from PIL import Image, ImageColor, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "docs" / "assets"
WIDTH, HEIGHT = 1120, 704
SCALE = 2
FRAMES_PER_STAGE = 24
FRAME_MS = 140
BG = "#0b1220"
PANEL = "#121e30"
EDGE = "#28394e"
WHITE = "#edf4fc"
MUTED = "#a6b8cd"
CYAN = "#54dfcf"
ORANGE = "#ffbe76"
GRID = "#223247"
STAGES = (
    ("01", "Generate", "Sample candidate PI, PD or PID gains within the chosen bounds."),
    ("02", "Evaluate", "Simulate each controller; the lowest-cost candidate becomes the black hole."),
    ("03", "Attract", "Move other candidates toward the best gains, then evaluate them again."),
    ("04", "Explore", "Reinitialize absorbed stars to explore new gains; repeat the search."),
    ("05", "Validate", "Compare responses with Ziegler-Nichols; test motor gains on the prototype."),
)


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    """Use a local system font without redistributing a font file."""
    filename = "segoeuib.ttf" if bold else "segoeui.ttf"
    windows_fonts = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
    candidates = (
        windows_fonts / filename,
        Path("/usr/share/fonts/truetype/dejavu")
        / ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"),
        Path("/System/Library/Fonts/Supplemental/Arial Bold.ttf" if bold
             else "/System/Library/Fonts/Supplemental/Arial.ttf"),
    )
    for candidate in candidates:
        if candidate.is_file():
            return ImageFont.truetype(str(candidate), size * SCALE)
    return ImageFont.truetype("DejaVuSans.ttf", size * SCALE)


FONTS = {(size, bold): font(size, bold) for size, bold in (
    (13, False), (14, False), (15, False), (16, False), (17, False),
    (18, False), (18, True), (20, True), (32, True),
)}


def ease(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


class Canvas:
    def __init__(self) -> None:
        self.image = Image.new("RGB", (WIDTH * SCALE, HEIGHT * SCALE), BG)
        self.draw = ImageDraw.Draw(self.image)

    def text(self, xy, value, size=16, color=WHITE, bold=False, anchor=None):
        self.draw.text(tuple(v * SCALE for v in xy), value,
                       font=FONTS[size, bold], fill=color, anchor=anchor)

    def line(self, points, color=EDGE, width=1):
        self.draw.line([(round(x * SCALE), round(y * SCALE)) for x, y in points],
                       fill=color, width=width * SCALE, joint="curve")

    def box(self, bounds, fill=PANEL, outline=EDGE, radius=12):
        self.draw.rounded_rectangle(tuple(round(v * SCALE) for v in bounds),
                                    radius=radius * SCALE, fill=fill, outline=outline,
                                    width=SCALE)

    def dot(self, xy, radius=4, fill=CYAN, outline=None):
        x, y = xy
        self.draw.ellipse(((x-radius)*SCALE, (y-radius)*SCALE,
                           (x+radius)*SCALE, (y+radius)*SCALE),
                          fill=fill, outline=outline, width=SCALE)

    def arrow(self, points, color=MUTED, width=2):
        self.line(points, color, width)
        x0, y0 = points[-2]
        x1, y1 = points[-1]
        angle = math.atan2(y1-y0, x1-x0)
        self.line([(x1-7*math.cos(angle-.5), y1-7*math.sin(angle-.5)),
                   (x1, y1),
                   (x1-7*math.cos(angle+.5), y1-7*math.sin(angle+.5))], color, width)


rng = random.Random(22)
STARS = [(rng.uniform(.06, .94), rng.uniform(.08, .92)) for _ in range(23)]
BH = STARS[8]
RECYCLED = {2, 7, 12, 17}
NEW_STARS = {i: (rng.uniform(.08, .92), rng.uniform(.08, .92)) for i in RECYCLED}


def position(point, target, fraction):
    return tuple(a + (b-a) * fraction for a, b in zip(point, target))


def gain_plot(canvas: Canvas, stage: int, phase: float) -> None:
    canvas.box((36, 202, 532, 492))
    canvas.text((58, 222), "Gain-space search", 20, bold=True)
    canvas.text((58, 253), "Each star is one candidate controller", 15, MUTED)
    left, right, top, bottom = 91, 495, 293, 425
    for i in range(5):
        x = left + (right-left)*i/4
        y = top + (bottom-top)*i/4
        canvas.line([(x, top), (x, bottom)], GRID)
        canvas.line([(left, y), (right, y)], GRID)
    canvas.text((65, 352), "Ki", 15, MUTED)
    canvas.text((488, 433), "Kp", 15, MUTED)

    def pixel(point):
        return left+(right-left)*point[0], bottom-(bottom-top)*point[1]

    movement = .7 * ease(phase) if stage == 2 else (.7 if stage >= 3 else 0)
    if stage >= 1:
        center = pixel(BH)
        canvas.dot(center, 23, fill=None, outline=ORANGE)
        canvas.dot(center, 29, fill=None, outline=EDGE)
        canvas.text((center[0]+34, center[1]-10), "best", 14, ORANGE)
    for i, star in enumerate(STARS):
        if stage == 0 and i > int(phase * len(STARS)):
            continue
        if i == 8 and stage >= 1:
            continue
        point = position(star, BH, movement)
        color = CYAN
        if stage >= 2 and i % 3 == 0:
            canvas.line([pixel(star), pixel(point)], "#305955")
        if stage >= 3 and i in RECYCLED:
            point = position(point, NEW_STARS[i], ease(phase)) if stage == 3 else NEW_STARS[i]
            color = ORANGE
        canvas.dot(pixel(point), 4, color)
    if stage >= 1:
        canvas.dot(pixel(BH), 8, BG, ORANGE)
        canvas.dot(pixel(BH), 3, ORANGE)
    canvas.dot((62, 468), 4, CYAN)
    canvas.text((74, 457), "candidate", 14, MUTED)
    canvas.dot((184, 468), 4, ORANGE)
    canvas.text((196, 457), "best / reinitialized", 14, MUTED)
    canvas.text((383, 457), "2D projection", 14, MUTED)


def initial_response(t):
    return 1 - math.exp(-5*t) * math.cos(15*t)


def tuned_response(t):
    return 1 - math.exp(-10*t)


def response_plot(canvas: Canvas, stage: int, phase: float) -> None:
    canvas.box((552, 202, 1084, 492))
    canvas.text((574, 222), "Closed-loop response", 20, bold=True)
    canvas.text((574, 253), "Score error, overshoot and rise time", 15, MUTED)
    left, right, top, bottom = 607, 1053, 293, 425
    max_y = 1.6

    def pixel(t, value):
        return left+t*(right-left), bottom-value/max_y*(bottom-top)

    for i in range(5):
        canvas.line([(left+(right-left)*i/4, top), (left+(right-left)*i/4, bottom)], GRID)
        canvas.line([(left, top+(bottom-top)*i/4), (right, top+(bottom-top)*i/4)], GRID)
    canvas.text((574, 350), "y(t)", 14, MUTED)
    canvas.text((1015, 433), "Time", 14, MUTED)
    for x in range(left, right, 13):
        canvas.line([(x, pixel(0, 1)[1]), (min(x+6, right), pixel(0, 1)[1])], MUTED)
    before = [pixel(i/200, initial_response(i/200)) for i in range(201)]
    canvas.line(before, "#6b7b90", 2)
    improvement = (.58 * ease(phase) if stage == 2 else
                   .58 + .42 * ease(phase) if stage == 3 else
                   1.0 if stage == 4 else 0.0)
    visible = int(200 * ease(min(1, phase*1.8))) if stage == 0 else 200
    current = [pixel(i/200, (1-improvement)*initial_response(i/200)
                     + improvement*tuned_response(i/200)) for i in range(visible+1)]
    if len(current) > 1:
        canvas.line(current, CYAN, 3)
        canvas.dot(current[min(int(phase*200), visible)], 4, CYAN)
    canvas.line([(580, 468), (600, 468)], "#6b7b90", 2)
    canvas.text((607, 457), "initial", 14, MUTED)
    canvas.line([(680, 468), (700, 468)], CYAN, 3)
    canvas.text((707, 457), "current", 14, MUTED)
    canvas.line([(798, 468), (818, 468)], MUTED)
    canvas.text((825, 457), "reference", 14, MUTED)


def loop_diagram(canvas: Canvas, stage: int, phase: float) -> None:
    canvas.box((36, 510, 1084, 611))
    canvas.text((58, 528), "CONTROL LOOP", 13, MUTED)
    canvas.text((62, 558), "Reference", 18)
    for bounds, label in (((275, 540, 443, 582), "PI / PD / PID"),
                          ((532, 540, 758, 582), "Selected plant")):
        canvas.box(bounds, BG, CYAN if stage == 4 else EDGE, radius=8)
        canvas.text(((bounds[0]+bounds[2])/2, 560), label, 18, anchor="mm")
    canvas.text((864, 549), "Output", 18)
    canvas.dot((222, 561), 12, BG, MUTED)
    canvas.text((222, 559), "+", 18, MUTED, anchor="mm")
    canvas.arrow([(159, 561), (209, 561)])
    canvas.arrow([(235, 561), (275, 561)])
    canvas.arrow([(443, 561), (532, 561)])
    canvas.arrow([(758, 561), (856, 561)])
    canvas.arrow([(818, 561), (818, 598), (222, 598), (222, 575)])
    canvas.line([(230, 581), (237, 581)], MUTED)
    canvas.box((433, 585, 640, 608), PANEL, PANEL, radius=0)
    canvas.text((443, 586), "encoder / simulated feedback", 13, MUTED)
    canvas.text((928, 549), "y(t)", 18, CYAN)
    for x0, x1 in ((159, 207), (237, 273), (447, 528), (762, 851)):
        canvas.dot((x0+(x1-x0)*phase, 561), 3, CYAN)


def render(stage: int, phase: float) -> Image.Image:
    canvas = Canvas()
    canvas.text((36, 22), "BLACK HOLE OPTIMIZATION", 13, CYAN)
    canvas.text((36, 46), "Better gains. Better control.", 32, bold=True)
    canvas.text((36, 92), "Ball-and-beam  /  Cruise control  /  DC motor  /  Custom transfer function", 17, MUTED)
    canvas.box((886, 32, 1084, 72), PANEL, EDGE, radius=20)
    canvas.text((985, 51), "MATLAB + Python", 18, CYAN, anchor="mm")
    for i, (number, label, _) in enumerate(STAGES):
        x = 36 + i * 212
        active = i == stage
        canvas.box((x, 131, x+200, 181), "#1e393d" if active else PANEL,
                   CYAN if active else EDGE, radius=9)
        canvas.text((x+15, 145), number, 18, CYAN if active else MUTED)
        canvas.text((x+52, 144), label, 18, WHITE if active else MUTED, bold=True)
        canvas.line([(x+12, 180), (x+12+176*(phase if active else 1 if i < stage else 0), 180)],
                    CYAN if i <= stage else EDGE, 2)
    gain_plot(canvas, stage, phase)
    response_plot(canvas, stage, phase)
    loop_diagram(canvas, stage, phase)
    canvas.text((36, 631), STAGES[stage][2], 17)
    canvas.text((36, 669), "Conceptual walkthrough: schematic gain space and illustrative responses; not measured results.", 14, MUTED)
    return canvas.image.resize((WIDTH, HEIGHT), Image.Resampling.LANCZOS)


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    frames = [render(stage, frame/(FRAMES_PER_STAGE-1))
              for stage in range(len(STAGES)) for frame in range(FRAMES_PER_STAGE)]
    frames[-1].save(OUTPUT / "bho-workflow-static.png", optimize=True)
    # One palette for every frame prevents changing colors and keeps the GIF small.
    palette = frames[-1].quantize(colors=96)
    # Reserve brand colors: sparse orange markers can be lost by median-cut alone.
    palette_values = palette.getpalette()[:96*3]
    for color in (BG, PANEL, EDGE, WHITE, MUTED, CYAN, ORANGE, GRID):
        palette_values.extend(ImageColor.getrgb(color))
    palette_values.extend(list(ImageColor.getrgb(BG)) * ((768-len(palette_values))//3))
    palette.putpalette(palette_values)
    indexed = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames]
    durations = [FRAME_MS] * len(indexed)
    durations[-1] = 2200
    indexed[0].save(OUTPUT / "bho-workflow.gif", save_all=True,
                    append_images=indexed[1:], duration=durations,
                    loop=0, optimize=True, disposal=1)
    print(f"Rendered {len(frames)} frames ({WIDTH} x {HEIGHT}) into {OUTPUT}")
    print(f"GIF size: {(OUTPUT / 'bho-workflow.gif').stat().st_size / 1024:.0f} KiB")


if __name__ == "__main__":
    main()
