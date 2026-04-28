#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
预警判级时序图生成脚本（支持两种 CSV）

用法示例:
    python scripts/plot_warning_timeline.py \
      --input data/gt_塔吊标准节_综合预警_评估.csv \
      --output data/gt_warning_timeline.png --no-show

    python scripts/plot_warning_timeline.py \
      --input data/prediction_塔吊标准节_综合预警_评估.csv \
      --output data/pred_warning_timeline.png --no-show
"""

import argparse
import csv
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.font_manager import findfont

# ============================================================
# 配置：中文字体与样式
# ============================================================
chinese_font_candidates = [
    "PingFang SC", "PingFang TC", "Heiti SC", "Heiti TC", "STHeiti",
    "SimHei", "Microsoft YaHei", "WenQuanYi Micro Hei",
]

used_font = None
for f in chinese_font_candidates:
    try:
        findfont(f, fallback_to_default=False)
        used_font = f
        break
    except Exception:
        continue

if used_font:
    plt.rcParams.update({
        "font.family": used_font,
        "font.size": 11,
        "axes.unicode_minus": False,
    })
    print(f"[信息] 使用字体: {used_font}")
else:
    print("[警告] 未找到中文字体，中文可能显示为方框。")
    plt.rcParams["font.size"] = 11

# ============================================================
# 等级与颜色定义
# ============================================================
LEVELS = ["无危险", "提示", "报警", "紧急"]
LEVEL_TO_CODE = {lv: i for i, lv in enumerate(LEVELS)}
LEVEL_COLORS = ["#2ecc71", "#f1c40f", "#e67e22", "#e7442c"]


def normalize_level(level_text: str) -> str:
    text = (level_text or "").strip()
    aliases = {
        "安全": "无危险",
        "warning": "报警",
        "alert": "提示",
        "danger": "紧急",
        "none": "无危险",
    }
    return aliases.get(text, text)


def load_csv_timeline(csv_path: Path):
    with csv_path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fields = set(reader.fieldnames or [])

    if not rows:
        raise ValueError(f"CSV 为空: {csv_path}")

    if {"start_time_sec", "end_time_sec", "level"}.issubset(fields):
        # GT 区间格式
        segments = []
        for r in rows:
            lv = normalize_level(r["level"])
            if lv not in LEVEL_TO_CODE:
                continue
            st = float(r["start_time_sec"])
            et = float(r["end_time_sec"])
            if et < st:
                st, et = et, st
            segments.append((st, et, LEVEL_TO_CODE[lv]))

        if not segments:
            raise ValueError(f"没有可用区间数据: {csv_path}")

        segments.sort(key=lambda x: x[0])
        t = []
        y = []
        for st, et, lv_code in segments:
            if not t:
                t.extend([st, et])
                y.extend([lv_code, lv_code])
            else:
                if st > t[-1]:
                    t.append(st)
                    y.append(y[-1])
                t.append(st)
                y.append(lv_code)
                t.append(et)
                y.append(lv_code)

        source_type = "GT区间"
        return np.array(t, dtype=float), np.array(y, dtype=int), source_type

    if {"timestamp_sec", "level"}.issubset(fields):
        # Prediction 点序列格式
        points = []
        for r in rows:
            lv = normalize_level(r["level"])
            if lv not in LEVEL_TO_CODE:
                continue
            ts = float(r["timestamp_sec"])
            points.append((ts, LEVEL_TO_CODE[lv]))

        if not points:
            raise ValueError(f"没有可用点序列数据: {csv_path}")

        points.sort(key=lambda x: x[0])

        # 轻度去重：同一时刻保留最后一个等级
        dedup = {}
        for ts, lv_code in points:
            dedup[ts] = lv_code
        ts_sorted = sorted(dedup.keys())

        t = np.array(ts_sorted, dtype=float)
        y = np.array([dedup[ts] for ts in ts_sorted], dtype=int)
        source_type = "Prediction点序列"
        return t, y, source_type

    raise ValueError(
        "无法识别的 CSV 字段，期望其一:\n"
        "1) start_time_sec,end_time_sec,level\n"
        "2) timestamp_sec,level"
    )


def find_first_trigger(levels, target_level):
    idx = np.where(levels >= target_level)[0]
    if len(idx) > 0:
        return idx[0]
    return None


def plot_level_timeline(t, level, output_path: Path, title: str, source_type: str, show: bool):
    fig, ax = plt.subplots(figsize=(14, 4.8))

    for i, color in enumerate(LEVEL_COLORS):
        ax.axhspan(i - 0.35, i + 0.35, alpha=0.1, color=color)

    ax.step(t, level, where="post", color="#2c3e50", linewidth=2.0, label="预警等级")

    for lv_code in [1, 2, 3]:
        idx = find_first_trigger(level, lv_code)
        if idx is not None:
            trig_t = t[idx]
            ax.axvline(x=trig_t, color=LEVEL_COLORS[lv_code], linestyle="--", linewidth=1.2, alpha=0.8)
            ax.text(
                trig_t,
                lv_code + 0.15,
                f"首次达到{LEVELS[lv_code]}\n{trig_t:.2f}s",
                color=LEVEL_COLORS[lv_code],
                ha="center",
                va="bottom",
                fontsize=9,
                bbox=dict(boxstyle="round,pad=0.2", facecolor="white", edgecolor=LEVEL_COLORS[lv_code], alpha=0.85),
            )

    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("预警等级")
    ax.set_yticks([0, 1, 2, 3])
    ax.set_yticklabels(LEVELS)
    ax.set_ylim(-0.5, 3.8)
    ax.set_xlim(float(np.min(t)), float(np.max(t)))
    ax.grid(True, alpha=0.3, axis="x")
    ax.legend(loc="upper right")

    fig.suptitle(f"{title} - 预警等级时序图（{source_type}）", fontsize=14, fontweight="bold", y=0.98)
    fig.subplots_adjust(left=0.07, right=0.98, top=0.86, bottom=0.15)

    plt.savefig(output_path, dpi=300, bbox_inches="tight", facecolor="white")
    print(f"[完成] 图表已保存: {output_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="生成预警等级时序图（支持 GT 区间 / Prediction 点序列 CSV）")
    parser.add_argument("--input", required=True, help="输入 CSV 路径")
    parser.add_argument("--output", required=True, help="输出图片路径 (png/pdf)")
    parser.add_argument("--title", default="动态风险升级与预警判级", help="图标题前缀")
    parser.add_argument("--no-show", action="store_true", help="不显示窗口，仅保存")
    args = parser.parse_args()

    csv_path = Path(args.input)
    output_path = Path(args.output)

    t, level, source_type = load_csv_timeline(csv_path)
    plot_level_timeline(t, level, output_path, args.title, source_type, show=not args.no_show)


if __name__ == "__main__":
    main()
