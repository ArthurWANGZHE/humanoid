#!/usr/bin/env python3
"""
把四张流程图拼成一张 A3 横向打印页
布局：四列并排，每列一张，等比缩放到相同高度
输出：figure/real_robot/print_sheet_flowcharts.png  (300dpi, A3横向)
"""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

# ── A3 横向 300dpi ────────────────────────────────────────────────────────────
DPI      = 300
A3_W_MM  = 420
A3_H_MM  = 297
MAR_MM   = 8    # 外边距
GAP_MM   = 5    # 列间距
HEAD_MM  = 10   # 顶部标题区高度
FOOT_MM  = 7    # 底部图注区高度

def mm(v):
    return round(v / 25.4 * DPI)

A3_W  = mm(A3_W_MM)   # 4961
A3_H  = mm(A3_H_MM)   # 3508
MAR   = mm(MAR_MM)
GAP   = mm(GAP_MM)
HEAD  = mm(HEAD_MM)
FOOT  = mm(FOOT_MM)

SRC = Path("figure/flowcharts")
OUT = Path("figure/real_robot")

FILES = [
    ("flow_01_ddpm_inference.png",      "flow_01  DDPM 推理流程"),
    ("flow_02_imitation_pipeline.png",  "flow_02  模仿学习完整 Pipeline"),
    ("flow_03_deploy_control_loop.png", "flow_03  真机部署控制循环"),
    ("flow_04_data_pipeline.png",       "flow_04  数据采集与处理流程"),
]

def try_font(size):
    for name in ["msyh.ttc", "msyhbd.ttc", "simhei.ttf",
                 "Arial.ttf", "arial.ttf", "DejaVuSans.ttf"]:
        try:
            return ImageFont.truetype(name, size)
        except (IOError, OSError):
            pass
    return ImageFont.load_default()

def main():
    imgs = [Image.open(SRC / fn).convert("RGB") for fn, _ in FILES]
    labels = [lbl for _, lbl in FILES]

    n = len(imgs)

    # 可用区域（去掉边距、标题、图注）
    avail_w = A3_W - 2 * MAR - (n - 1) * GAP
    avail_h = A3_H - 2 * MAR - HEAD - FOOT

    col_w = avail_w // n   # 每列宽度

    # 每张图等比缩放到 col_w 宽，取最大高度作为统一行高
    scaled = []
    for img in imgs:
        iw, ih = img.size
        scale = col_w / iw
        nw = col_w
        nh = round(ih * scale)
        scaled.append(img.resize((nw, nh), Image.LANCZOS))

    row_h = min(avail_h, max(s.height for s in scaled))

    # 如果最高的图超出可用高度，再按高度缩一次
    if max(s.height for s in scaled) > avail_h:
        scale2 = avail_h / max(s.height for s in scaled)
        rescaled = []
        for s in scaled:
            nw = round(s.width * scale2)
            nh = round(s.height * scale2)
            rescaled.append(s.resize((nw, nh), Image.LANCZOS))
        scaled = rescaled
        row_h = max(s.height for s in scaled)

    # ── 画布 ─────────────────────────────────────────────────────────────────
    canvas = Image.new("RGB", (A3_W, A3_H), (255, 255, 255))
    draw   = ImageDraw.Draw(canvas)

    font_title = try_font(52)
    font_label = try_font(30)

    # 总标题
    draw.text((A3_W // 2, MAR + HEAD // 2),
              "程序流程图",
              fill=(0, 0, 0), font=font_title, anchor="mm")

    # 细分隔线（标题下方）
    ty = MAR + HEAD
    draw.line([(MAR, ty), (A3_W - MAR, ty)], fill=(180, 180, 180), width=2)

    # 粘贴每列图片 + 图注
    x = MAR
    for img, label in zip(scaled, labels):
        # 图片居中粘贴在列内
        ox = x + (col_w - img.width) // 2
        oy = ty + (avail_h - img.height) // 2
        canvas.paste(img, (ox, oy))

        # 细边框
        draw.rectangle(
            [ox, oy, ox + img.width - 1, oy + img.height - 1],
            outline=(200, 200, 200), width=1,
        )

        # 图注（底部）
        lx = x + col_w // 2
        ly = A3_H - MAR - FOOT // 2
        draw.text((lx, ly), label,
                  fill=(60, 60, 60), font=font_label, anchor="mm")

        x += col_w + GAP

    # 底部分隔线
    by = A3_H - MAR - FOOT
    draw.line([(MAR, by), (A3_W - MAR, by)], fill=(180, 180, 180), width=2)

    out_path = OUT / "print_sheet_flowcharts.png"
    canvas.save(out_path, dpi=(DPI, DPI))
    print(f"saved: {out_path}")
    print(f"size:  {A3_W}x{A3_H}px  ({A3_W_MM}x{A3_H_MM}mm)  @{DPI}dpi")

if __name__ == "__main__":
    main()
