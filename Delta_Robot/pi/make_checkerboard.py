"""
make_checkerboard.py -- writes a printable checkerboard as an SVG at EXACT physical size.

    python make_checkerboard.py                     # 9x6 inner corners, 25 mm squares, US Letter
    python make_checkerboard.py --paper a4

Print it from a web browser (open the .svg, Print, set Scale = 100% / "Actual size", NOT
"fit to page"), then MEASURE one square with a ruler or calipers. The measured size is
what you pass to calibrate_stereo.py as --square-mm (printers are rarely exact).

Pattern = (cols x rows) INNER CORNERS, i.e. (cols+1) x (rows+1) squares. 9x6 corners is
asymmetric (one side odd, one even), which lets OpenCV tell the board's orientation apart
-- both eyes always label the corners in the same order.
"""
import argparse

PAPER_MM = {"letter": (279.4, 215.9), "a4": (297.0, 210.0)}   # landscape (width, height)


def checkerboard_svg(cols: int, rows: int, square_mm: float, paper: str) -> str:
    pw, ph = PAPER_MM[paper]
    nx, ny = cols + 1, rows + 1                       # number of squares
    bw, bh = nx * square_mm, ny * square_mm
    if bw > pw - 10 or bh > ph - 20:
        raise SystemExit(f"Board {bw:.0f}x{bh:.0f} mm does not fit on {paper} "
                         f"({pw:.0f}x{ph:.0f} mm). Use a smaller --square-mm.")
    x0, y0 = (pw - bw) / 2.0, (ph - bh) / 2.0 - 4.0     # slightly up, room for the caption
    rects = []
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:                       # black squares
                rects.append(f'<rect x="{x0 + i*square_mm:.3f}" y="{y0 + j*square_mm:.3f}" '
                             f'width="{square_mm:.3f}" height="{square_mm:.3f}"/>')
    caption = (f"{cols}x{rows} inner corners, {square_mm:g} mm squares (nominal). "
               f"Print at 100% scale, then measure a square.")
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{pw}mm" height="{ph}mm" '
            f'viewBox="0 0 {pw} {ph}">\n<rect width="{pw}" height="{ph}" fill="white"/>\n'
            f'<g fill="black">\n' + "\n".join(rects) + "\n</g>\n"
            f'<text x="{pw/2}" y="{y0 + bh + 9:.1f}" font-size="4" text-anchor="middle" '
            f'font-family="sans-serif">{caption}</text>\n</svg>\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=9, help="inner corners along the long side")
    ap.add_argument("--rows", type=int, default=6, help="inner corners along the short side")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--paper", choices=PAPER_MM, default="letter")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = a.out or f"checkerboard_{a.cols}x{a.rows}_{a.square_mm:g}mm_{a.paper}.svg"
    with open(out, "w") as f:
        f.write(checkerboard_svg(a.cols, a.rows, a.square_mm, a.paper))
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
