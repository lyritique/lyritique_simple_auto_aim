#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成可 1:1 打印的棋盘格标定板 SVG（只用 Python 标准库，不装任何包）。

用法：
    python3 tools/make_chessboard.py --cols 12 --rows 9 --square 25 --out chessboard.svg

注意 --cols / --rows 是「格子数」，比标定程序里的「内角点数」各多 1：
    12 x 9 个格子  ->  11 x 8 个内角点  ->  camera_calibration --cols 11 --rows 8
这是最容易搞混的地方，标定时填错会一张都检测不到。

打印要点：
  1. 用 SVG 的 mm 尺寸原样打印，缩放必须选 100% / 实际大小，别选「适应页面」；
  2. 打印完拿尺子实测 10 个格子的总长再除以 10，把实测值填进 --square；
     打印机缩放差 2% 就会让 fx/fy 整体偏 2%，比角点误差影响大得多；
  3. 板子要贴在硬纸板上保持平整，四周留白边（--margin）别裁掉，
     OpenCV 的 findChessboardCorners 靠白边判断棋盘外轮廓。
"""

import argparse


def build_svg(cols: int, rows: int, square: float, margin: float) -> str:
    width = cols * square + 2 * margin
    height = rows * square + 2 * margin

    out = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<svg xmlns="http://www.w3.org/2000/svg" '
        'width="{:.3f}mm" height="{:.3f}mm" viewBox="0 0 {:.3f} {:.3f}">'.format(
            width, height, width, height),
        '  <!-- 棋盘格标定板：{} x {} 格，每格 {:.3f} mm -->'.format(cols, rows, square),
        '  <rect x="0" y="0" width="{:.3f}" height="{:.3f}" fill="#ffffff"/>'.format(width, height),
    ]

    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 1:
                continue
            x = margin + c * square
            y = margin + r * square
            out.append(
                '  <rect x="{:.3f}" y="{:.3f}" width="{:.3f}" height="{:.3f}" fill="#000000"/>'.format(
                    x, y, square, square))

    # 外框细线：方便打印后裁切，同时不干扰角点检测
    out.append('  <rect x="{:.3f}" y="{:.3f}" width="{:.3f}" height="{:.3f}" '
               'fill="none" stroke="#888888" stroke-width="0.2"/>'.format(
                   margin, margin, cols * square, rows * square))
    out.append('</svg>')
    return '\n'.join(out) + '\n'


def main() -> None:
    ap = argparse.ArgumentParser(description="生成可打印的棋盘格标定板 SVG")
    ap.add_argument("--cols", type=int, default=12, help="横向格子数，默认 12")
    ap.add_argument("--rows", type=int, default=9, help="纵向格子数，默认 9")
    ap.add_argument("--square", type=float, default=25.0, help="方格边长 mm，默认 25")
    ap.add_argument("--margin", type=float, default=15.0, help="四周白边 mm，默认 15")
    ap.add_argument("--out", default="chessboard.svg", help="输出 SVG 路径")
    args = ap.parse_args()

    if args.cols < 3 or args.rows < 3 or args.square <= 0 or args.margin < 0:
        ap.error("格子数至少 3，边长必须为正，白边不能为负")

    svg = build_svg(args.cols, args.rows, args.square, args.margin)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(svg)

    inner_cols = args.cols - 1
    inner_rows = args.rows - 1
    physical_w = args.cols * args.square
    physical_h = args.rows * args.square
    print("已生成: {}".format(args.out))
    print("  棋盘尺寸    : {:.1f} x {:.1f} mm（含白边 {:.1f} x {:.1f} mm）".format(
        physical_w + 2 * args.margin, physical_h + 2 * args.margin,
        physical_w, physical_h))
    print("  内角点数    : {} x {}  ->  --cols {} --rows {}".format(
        inner_cols, inner_rows, inner_cols, inner_rows))
    print("  打印后请实测方格边长，再用 --square <实测值> 传给标定程序。")


if __name__ == "__main__":
    main()
