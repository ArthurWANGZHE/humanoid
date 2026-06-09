#!/usr/bin/env python3
"""
Compose two A3-landscape print sheets from the architecture diagram PNGs.

Sheet 1 (architecture):  01, 02, 03a, 03b, 03c, 04
Sheet 2 (network):       net_01, net_02, net_03, net_04

Output: figure/real_robot/print_sheet_1_architecture.png
        figure/real_robot/print_sheet_2_networks.png
        (also PDF versions via matplotlib)
"""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


# ── A3 landscape at 300 dpi ──────────────────────────────────────────────────
DPI        = 300
A3_W_MM    = 420
A3_H_MM    = 297
MARGIN_MM  = 8      # outer margin
GAP_MM     = 6      # gap between cells
LABEL_MM   = 5      # height reserved for caption below each image

def mm(v: float) -> int:
    """Convert mm to pixels at DPI."""
    return round(v / 25.4 * DPI)

A3_W  = mm(A3_W_MM)
A3_H  = mm(A3_H_MM)
MAR   = mm(MARGIN_MM)
GAP   = mm(GAP_MM)
LABEL = mm(LABEL_MM)

BG    = (255, 255, 255)
RULE  = (180, 180, 180)   # thin separator line colour
TEXT  = (60,  60,  60)


def load(path: Path) -> Image.Image:
    img = Image.open(path).convert("RGB")
    return img


def fit(img: Image.Image, max_w: int, max_h: int) -> Image.Image:
    """Scale img to fit within (max_w, max_h) preserving aspect ratio."""
    iw, ih = img.size
    scale = min(max_w / iw, max_h / ih)
    nw, nh = round(iw * scale), round(ih * scale)
    return img.resize((nw, nh), Image.LANCZOS)


def paste_centered(canvas: Image.Image, img: Image.Image,
                   cell_x: int, cell_y: int,
                   cell_w: int, cell_h: int) -> None:
    """Paste img centred inside the cell rectangle."""
    ox = cell_x + (cell_w - img.width)  // 2
    oy = cell_y + (cell_h - img.height) // 2
    canvas.paste(img, (ox, oy))


def draw_label(draw: ImageDraw.ImageDraw,
               text: str,
               cell_x: int, cell_y: int,
               cell_w: int, cell_h: int,
               font) -> None:
    """Draw a small caption centred below the cell."""
    lx = cell_x + cell_w // 2
    ly = cell_y + cell_h - LABEL // 2
    draw.text((lx, ly), text, fill=TEXT, font=font, anchor="mm")


def draw_separator(draw: ImageDraw.ImageDraw,
                   x0: int, y0: int, x1: int, y1: int) -> None:
    draw.line([(x0, y0), (x1, y1)], fill=RULE, width=2)


def make_sheet(
    canvas_w: int,
    canvas_h: int,
    rows: list[list[tuple[Image.Image, str]]],
    title: str,
    font_title,
    font_label,
) -> Image.Image:
    """
    rows: list of rows; each row is a list of (image, caption) pairs.
    Images in the same row share the same row height (tallest image sets it).
    Columns within a row are distributed proportionally by image width.
    """
    canvas = Image.new("RGB", (canvas_w, canvas_h), BG)
    draw   = ImageDraw.Draw(canvas)

    # Title at top
    title_h = mm(10)
    draw.text((canvas_w // 2, MAR + title_h // 2), title,
              fill=(0, 0, 0), font=font_title, anchor="mm")

    # Available area below title
    avail_w = canvas_w - 2 * MAR
    avail_h = canvas_h - 2 * MAR - title_h - GAP

    n_rows   = len(rows)
    row_gaps = (n_rows - 1) * GAP
    row_h    = (avail_h - row_gaps) // n_rows

    cur_y = MAR + title_h + GAP

    for row in rows:
        n_cols   = len(row)
        col_gaps = (n_cols - 1) * GAP
        col_w    = (avail_w - col_gaps) // n_cols

        cell_h = row_h - LABEL   # leave room for label

        cur_x = MAR
        for img, caption in row:
            fitted = fit(img, col_w, cell_h)
            paste_centered(canvas, fitted, cur_x, cur_y, col_w, cell_h)
            draw_label(draw, caption, cur_x, cur_y, col_w, row_h, font_label)
            # thin border around cell
            draw.rectangle(
                [cur_x, cur_y, cur_x + col_w - 1, cur_y + cell_h - 1],
                outline=RULE, width=1,
            )
            cur_x += col_w + GAP

        cur_y += row_h + GAP

    return canvas


def try_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        "msyh.ttc", "msyhbd.ttc",          # Microsoft YaHei
        "simhei.ttf",                        # SimHei
        "Arial.ttf", "arial.ttf",
        "DejaVuSans.ttf",
    ]
    for name in candidates:
        try:
            return ImageFont.truetype(name, size)
        except (IOError, OSError):
            pass
    return ImageFont.load_default()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path("figure/architecture"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("figure/real_robot"),
    )
    args = parser.parse_args()

    src = args.input_dir
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    font_title = try_font(52)
    font_label = try_font(32)

    # ── Sheet 1: Architecture diagrams ───────────────────────────────────────
    # Row 1: 01  02
    # Row 2: 03a 03b
    # Row 3: 03c 04
    sheet1_rows = [
        [
            (load(src / "01_isaac_sim_rl_architecture.png"),
             "01  Isaac Sim 强化学习架构"),
            (load(src / "02_src_keyboard_diffusion_servo_architecture.png"),
             "02  控制部署架构"),
        ],
        [
            (load(src / "03a_collection.png"),
             "03a  示教采集流程"),
            (load(src / "03b_data_processing.png"),
             "03b  数据处理流程"),
        ],
        [
            (load(src / "03c_training_deployment.png"),
             "03c  训练与部署流程"),
            (load(src / "04_pushcube_architecture.png"),
             "04  PushCube 架构"),
        ],
    ]

    sheet1 = make_sheet(
        A3_W, A3_H,
        sheet1_rows,
        "架构总览",
        font_title, font_label,
    )
    p1 = out / "print_sheet_1_architecture.png"
    sheet1.save(p1, dpi=(DPI, DPI))
    print(f"Saved: {p1}")

    # ── Sheet 2: Network diagrams ─────────────────────────────────────────────
    # Row 1: net_01  net_02
    # Row 2: net_03  net_04
    sheet2_rows = [
        [
            (load(src / "net_01_diffusion_policy_network.png"),
             "net_01  DiffusionPolicy 网络结构"),
            (load(src / "net_02_ddpm_training_inference.png"),
             "net_02  DDPM 训练 & 推理流程"),
        ],
        [
            (load(src / "net_03_ppo_mlppolicy.png"),
             "net_03  PPO MlpPolicy"),
            (load(src / "net_04_bc_mlp_comparison.png"),
             "net_04  BC MLP 网络对比"),
        ],
    ]

    sheet2 = make_sheet(
        A3_W, A3_H,
        sheet2_rows,
        "网络结构总览",
        font_title, font_label,
    )
    p2 = out / "print_sheet_2_networks.png"
    sheet2.save(p2, dpi=(DPI, DPI))
    print(f"Saved: {p2}")

    print(f"\nBoth sheets: {A3_W}x{A3_H} px  ({A3_W_MM}x{A3_H_MM} mm)  @{DPI} dpi")
    print("Ready to print on A3 landscape.")


if __name__ == "__main__":
    main()
