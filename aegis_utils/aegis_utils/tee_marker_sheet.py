"""
Writes the A4 sheet with the tee top face and its two ArUco markers, as a 1:1 SVG.

Print it at 100 % scale (no "fit to page"), check the 100 mm scale bar with a ruler, cut along
the outline and glue it on the tee top with the crossbar edge on the crossbar.

    python3 tee_marker_sheet.py tee_markers.svg
"""

import sys

import cv2
import numpy as np

from aegis_utils.tee_tracker import ARUCO_DICT, MARKER_SIZE, MARKERS, TEE_OUTLINE

PAGE_MM = (210.0, 297.0)
# tee origin (crossbar centre) on the page; the page y axis points down, the tee +y up
ORIGIN_MM = (105.0, 80.0)


def marker_bits(marker_id: int) -> np.ndarray:
    """[6, 6] bool, True = black: the 4x4 code with its one-cell black border."""
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, ARUCO_DICT))
    if hasattr(aruco, "generateImageMarker"):
        img = aruco.generateImageMarker(dictionary, marker_id, 6)
    else:
        img = aruco.drawMarker(dictionary, marker_id, 6)
    return img < 128


def to_page(xy_m) -> tuple[float, float]:
    return ORIGIN_MM[0] + 1e3 * xy_m[0], ORIGIN_MM[1] - 1e3 * xy_m[1]


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "tee_markers.svg"
    w, h = PAGE_MM
    parts = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}mm" height="{h}mm" '
            f'viewBox="0 0 {w} {h}">'
        ),
        f'<rect width="{w}" height="{h}" fill="white"/>',
    ]
    outline = " ".join("{:.2f},{:.2f}".format(*to_page(p)) for p in TEE_OUTLINE)
    parts.append(
        f'<polygon points="{outline}" fill="none" stroke="#888" stroke-width="0.3"/>'
    )
    cell = 1e3 * MARKER_SIZE / 6
    for marker_id, centre in MARKERS.items():
        cx, cy = to_page(centre)
        x0, y0 = cx - 3 * cell, cy - 3 * cell
        for r, row in enumerate(marker_bits(marker_id)):
            for c, black in enumerate(row):
                if black:
                    parts.append(
                        f'<rect x="{x0 + c * cell:.3f}" y="{y0 + r * cell:.3f}" '
                        f'width="{cell + 0.02:.3f}" height="{cell + 0.02:.3f}" fill="black"/>'
                    )
        parts.append(
            f'<text x="{cx + 3 * cell + 3:.1f}" y="{cy:.1f}" font-size="3" '
            f'fill="#888">id {marker_id}</text>'
        )
    # scale check and labels below the tee
    y_bar = 280.0
    parts += [
        f'<line x1="55" y1="{y_bar}" x2="155" y2="{y_bar}" stroke="black" stroke-width="0.4"/>',
        f'<line x1="55" y1="{y_bar - 3}" x2="55" y2="{y_bar + 3}" stroke="black" stroke-width="0.4"/>',
        f'<line x1="155" y1="{y_bar - 3}" x2="155" y2="{y_bar + 3}" stroke="black" stroke-width="0.4"/>',
        f'<text x="80" y="{y_bar - 4}" font-size="4">100 mm at 100 % scale</text>',
        (
            f'<text x="20" y="20" font-size="4">push_t tee top, {ARUCO_DICT}, markers '
            f"{MARKER_SIZE * 1e3:.0f} mm, cut along the grey outline</text>"
        ),
    ]
    parts.append("</svg>")
    with open(out, "w") as f:
        f.write("\n".join(parts))
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
