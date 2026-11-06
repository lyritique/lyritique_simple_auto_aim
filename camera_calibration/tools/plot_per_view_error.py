#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 output/per_view_error.csv 画成 SVG 柱状图（只用 Python 标准库）。

fixed 版里的 tools/plot_states.py 要 numpy + matplotlib，这个脚本零依赖，
专门给「每张标定图的重投影误差」用：一眼看出哪几张图拖后腿。

用法：
    python3 tools/plot_per_view_error.py output/per_view_error.csv output/error.svg
"""

import argparse
import csv
import html
import math


def nice_upper(value: float) -> float:
    """把纵轴上限取成 0.5 的整数倍，好读"""
    if value <= 0:
        return 1.0
    step = 0.5 if value <= 5 else 1.0 if value <= 20 else 5.0
    return math.ceil(value / step) * step


def render(rows, path: str) -> None:
    width, height = 960, 460
    left, right, top, bottom = 70, 25, 60, 95
    plot_w = width - left - right
    plot_h = height - top - bottom

    values = [max(r["first"], 0.0) for r in rows]
    y_max = nice_upper(max(values + [1.0]) * 1.15)
    n = max(len(rows), 1)
    slot = plot_w / n
    bar_w = min(slot * 0.66, 34.0)

    out = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" '
        'viewBox="0 0 {} {}">'.format(width, height, width, height),
        '  <rect width="{}" height="{}" fill="#ffffff"/>'.format(width, height),
        '  <style>text {{ font-family: "Microsoft YaHei", "Noto Sans CJK SC", sans-serif; }}</style>',
        '  <text x="{}" y="30" font-size="18" fill="#1a1a1a">'
        '逐视图重投影误差（越低越好）</text>'.format(left),
        '  <text x="{}" y="49" font-size="12" fill="#666666">'
        '绿色 = 参与标定　灰色 = 被剔除　虚线 = 1.0 px 参考线</text>'.format(left),
    ]

    # 纵轴网格 + 刻度
    ticks = 5
    for i in range(ticks + 1):
        v = y_max * i / ticks
        y = top + plot_h - plot_h * i / ticks
        out.append('  <line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" '
                   'stroke="#e6e6e6" stroke-width="1"/>'.format(left, y, left + plot_w, y))
        out.append('  <text x="{:.1f}" y="{:.1f}" font-size="11" fill="#555555" '
                   'text-anchor="end">{:.2f}</text>'.format(left - 8, y + 4, v))
    out.append('  <text x="16" y="{}" font-size="12" fill="#555555" '
               'transform="rotate(-90 16 {})" text-anchor="middle">RMS / px</text>'.format(
                   top + plot_h / 2, top + plot_h / 2))

    # 1.0 px 参考线
    y_ref = top + plot_h - plot_h * min(1.0 / y_max, 1.0)
    out.append('  <line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" '
               'stroke="#e06030" stroke-width="1.5" stroke-dasharray="6 4"/>'.format(
                   left, y_ref, left + plot_w, y_ref))
    out.append('  <text x="{:.1f}" y="{:.1f}" font-size="11" fill="#e06030">1.0 px</text>'.format(
        left + plot_w - 48, y_ref - 6))

    # 柱
    for i, r in enumerate(rows):
        value = max(r["first"], 0.0)
        bar_h = plot_h * min(value / y_max, 1.0) if y_max > 0 else 0.0
        x = left + slot * i + (slot - bar_w) / 2
        y = top + plot_h - bar_h
        color = "#1f9d6b" if r["used"] else "#b8b8b8"
        out.append('  <rect x="{:.2f}" y="{:.2f}" width="{:.2f}" height="{:.2f}" '
                   'fill="{}" rx="2"><title>{}: {:.4f} px{}</title></rect>'.format(
                       x, y, bar_w, bar_h, color,
                       html.escape(r["label"]), value,
                       "" if r["used"] else "（已剔除）"))
        if bar_h > 14:
            out.append('  <text x="{:.2f}" y="{:.2f}" font-size="10" fill="#333333" '
                       'text-anchor="middle">{:.2f}</text>'.format(
                           x + bar_w / 2, y - 4, value))
        if not r["used"]:
            out.append('  <text x="{:.2f}" y="{:.2f}" font-size="13" fill="#c0392b" '
                       'text-anchor="middle">x</text>'.format(
                           x + bar_w / 2, top + plot_h + 14))
        # 底部标签：太密时隔几个显示一个
        if n <= 20 or i % max(1, n // 20) == 0:
            label = r["label"] if len(r["label"]) <= 9 else r["label"][:8] + "…"
            out.append('  <text x="{:.2f}" y="{:.2f}" font-size="10" fill="#555555" '
                       'transform="rotate(-40 {:.2f} {:.2f})" text-anchor="end">{}</text>'.format(
                           x + bar_w / 2, top + plot_h + 28,
                           x + bar_w / 2, top + plot_h + 28,
                           html.escape(label)))

    out.append('  <line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" '
               'stroke="#888888" stroke-width="1"/>'.format(
                   left, top + plot_h, left + plot_w, top + plot_h))
    out.append('</svg>')

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")


def main() -> None:
    ap = argparse.ArgumentParser(description="逐视图误差柱状图（零依赖 SVG）")
    ap.add_argument("csv_path", nargs="?", default="output/per_view_error.csv")
    ap.add_argument("svg_path", nargs="?", default="output/per_view_error.svg")
    args = ap.parse_args()

    rows = []
    with open(args.csv_path, newline="", encoding="utf-8-sig") as f:
        for item in csv.DictReader(f):
            def num(key):
                try:
                    return float(item.get(key, ""))
                except ValueError:
                    return -1.0
            rows.append({
                "label": item.get("label", "?"),
                "used": item.get("used", "0").strip() == "1",
                "first": num("rms_first_pass"),
            })

    if not rows:
        raise SystemExit("CSV 里没有数据: {}".format(args.csv_path))

    render(rows, args.svg_path)

    used = [r["first"] for r in rows if r["used"] and r["first"] >= 0]
    rejected = [r["label"] for r in rows if not r["used"]]
    print("已生成: {}".format(args.svg_path))
    if used:
        print("  参与标定: {} 张，第一轮 RMS 平均 {:.4f} px / 最大 {:.4f} px".format(
            len(used), sum(used) / len(used), max(used)))
    if rejected:
        print("  被剔除  : {} 张 -> {}".format(len(rejected), ", ".join(rejected)))


if __name__ == "__main__":
    main()
