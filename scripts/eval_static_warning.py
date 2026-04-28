#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
静态障碍物距离阈值预警效果测试 — 指标自动统计脚本
用法:
    python eval_static_warning.py --gt ground_truth.csv --pred prediction.csv
输出:
    控制台表格 + LaTeX 表格代码（可直接贴入论文 D4 表）
"""

import argparse
import csv
import sys
import bisect
from collections import defaultdict
from pathlib import Path

# 等级定义（有序）
LEVELS = ["无危险", "提示", "报警", "紧急"]
LEVEL2IDX = {lv: i for i, lv in enumerate(LEVELS)}
IDX2LEVEL = {i: lv for i, lv in enumerate(LEVELS)}


def read_ground_truth(path):
    """
    读取真值文件。支持三种格式自动识别：
    格式A（片段式帧+时间，推荐用于人工标注）:
        video_name,start_frame,end_frame,level,start_time_sec,end_time_sec
    格式B（片段式仅时间）:
        video_name,start_time_sec,end_time_sec,level
    格式C（逐帧式）:
        video_name,frame_id,level
    返回: (gt_by_frame, gt_intervals)
        gt_by_frame: dict[(video, frame)] -> level
        gt_intervals: dict[video] -> [(start_time, end_time, level), ...]
    """
    rows = []
    with open(path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        sys.exit("[错误] 真值文件为空")

    fields = set(rows[0].keys())
    gt_by_frame = {}
    gt_intervals = {}

    has_frame_seg = {"start_frame", "end_frame"} <= fields
    has_time_seg = {"start_time_sec", "end_time_sec"} <= fields
    has_frame_row = {"frame_id"} <= fields or {"frame"} <= fields

    # 格式 A: 含帧区间（可选同时含时间区间）
    if has_frame_seg:
        for r in rows:
            vid = r.get("video_name", r.get("video", "default"))
            start = int(r["start_frame"])
            end = int(r["end_frame"])
            lv = r["level"].strip()
            if lv not in LEVEL2IDX:
                sys.exit(f"[错误] 真值文件出现未知等级: '{lv}'。合法等级: {LEVELS}")
            for fr in range(start, end + 1):
                gt_by_frame[(vid, fr)] = lv

            if has_time_seg:
                st = float(r["start_time_sec"])
                et = float(r["end_time_sec"])
                gt_intervals.setdefault(vid, []).append((st, et, lv))

        print(f"[信息] 真值文件按片段解析: 共 {len(rows)} 个片段, {len(gt_by_frame)} 帧")
        if has_time_seg:
            print(f"         同时包含时间信息，可用于时间对齐")

    # 格式 B: 仅时间区间
    elif has_time_seg:
        for r in rows:
            vid = r.get("video_name", r.get("video", "default"))
            st = float(r["start_time_sec"])
            et = float(r["end_time_sec"])
            lv = r["level"].strip()
            if lv not in LEVEL2IDX:
                sys.exit(f"[错误] 真值文件出现未知等级: '{lv}'。合法等级: {LEVELS}")
            gt_intervals.setdefault(vid, []).append((st, et, lv))
        print(f"[信息] 真值文件按时间片段解析: 共 {len(rows)} 个片段")

    # 格式 C: 逐帧
    elif has_frame_row:
        frame_key = "frame_id" if "frame_id" in fields else "frame"
        for r in rows:
            vid = r.get("video_name", r.get("video", "default"))
            fr = int(r[frame_key])
            lv = r["level"].strip()
            if lv not in LEVEL2IDX:
                sys.exit(f"[错误] 真值文件出现未知等级: '{lv}'。合法等级: {LEVELS}")
            gt_by_frame[(vid, fr)] = lv
        print(f"[信息] 真值文件按逐帧解析: 共 {len(gt_by_frame)} 帧")
    else:
        sys.exit(f"[错误] 无法识别真值文件格式。合法列名需包含: start_frame/end_frame 或 start_time_sec/end_time_sec 或 frame_id")

    return gt_by_frame, gt_intervals


def read_prediction(path):
    """
    读取系统预测文件。支持两种格式：
    帧格式:
        video_name,frame_id,level
    时间格式（推荐用于与录屏视频对齐）:
        video_name,timestamp_sec,level
    返回: (pred_by_frame, pred_by_time, mode)
        mode: "frame" 或 "time"
    """
    pred_by_frame = {}
    pred_by_time = {}

    with open(path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        sys.exit("[错误] 预测文件为空")

    fields = set(rows[0].keys())
    has_time = {"timestamp_sec"} <= fields
    has_frame = {"frame_id"} <= fields or {"frame"} <= fields

    if has_time:
        for r in rows:
            vid = r.get("video_name", r.get("video", "default"))
            ts = float(r["timestamp_sec"])
            lv = r["level"].strip()
            if lv not in LEVEL2IDX:
                sys.exit(f"[错误] 预测文件出现未知等级: '{lv}'。合法等级: {LEVELS}")
            pred_by_time[(vid, ts)] = lv
        print(f"[信息] 预测文件按时间戳解析: 共 {len(pred_by_time)} 个时间点")
        return pred_by_frame, pred_by_time, "time"

    elif has_frame:
        frame_key = "frame_id" if "frame_id" in fields else "frame"
        for r in rows:
            vid = r.get("video_name", r.get("video", "default"))
            fr = int(r[frame_key])
            lv = r["level"].strip()
            if lv not in LEVEL2IDX:
                sys.exit(f"[错误] 预测文件出现未知等级: '{lv}'。合法等级: {LEVELS}")
            pred_by_frame[(vid, fr)] = lv
        print(f"[信息] 预测文件按帧解析: 共 {len(pred_by_frame)} 帧")
        return pred_by_frame, pred_by_time, "frame"

    else:
        sys.exit("[错误] 预测文件需包含 frame_id 或 timestamp_sec 列")


def resolve_level_by_time(intervals, timestamp):
    """
    在时间区间列表中查找给定时间戳对应的等级。
    intervals: [(start_time, end_time, level), ...] 已按 start_time 排序
    返回: level 字符串，若不在任何区间则返回 None
    """
    starts = [i[0] for i in intervals]
    idx = bisect.bisect_right(starts, timestamp) - 1
    if idx >= 0:
        start, end, level = intervals[idx]
        if start <= timestamp <= end:
            return level
    return None


def build_time_windows(gt_intervals, ignore_head_sec, ignore_tail_sec):
    """
    基于真值区间构建每个视频的有效统计时间窗。
    返回: dict[video] -> (start_ts, end_ts)
    """
    windows = {}
    for vid, intervals in gt_intervals.items():
        if not intervals:
            continue
        starts = [item[0] for item in intervals]
        ends = [item[1] for item in intervals]
        min_ts = min(starts)
        max_ts = max(ends)
        start_ts = min_ts + ignore_head_sec
        end_ts = max_ts - ignore_tail_sec
        if start_ts > end_ts:
            sys.exit(
                f"[错误] video_name='{vid}' 的有效时间窗为空。"
                f"请减小 --ignore-head-sec / --ignore-tail-sec（当前总窗口长度={max_ts - min_ts:.3f}s）"
            )
        windows[vid] = (start_ts, end_ts)
    return windows


def compute_metrics(
    gt_by_frame,
    gt_intervals,
    pred_by_frame,
    pred_by_time,
    mode,
    ignore_head_sec=0.0,
    ignore_tail_sec=0.0,
):
    """
    计算全部指标。返回 dict。
    支持两种对齐模式：
      - frame: 按 (video_name, frame_id) 交集对齐
      - time:  按时间戳对齐，对预测的每个时间点查找真值区间
    """
    # ---------- 按模式对齐 ----------
    if mode == "frame":
        common = sorted(set(gt_by_frame.keys()) & set(pred_by_frame.keys()))
        if not common:
            sys.exit("[错误] 真值与预测无共同帧，请检查 video_name 和 frame_id 是否对齐")
        n_total = len(common)
        y_true = [gt_by_frame[k] for k in common]
        y_pred = [pred_by_frame[k] for k in common]
        print(f"[信息] 帧对齐: 共 {n_total} 个共同帧")

    else:  # time mode
        if not gt_intervals:
            sys.exit("[错误] 时间对齐模式需要真值文件包含 start_time_sec/end_time_sec 列。\n"
                     "        请使用新版 annotate_video.py 重新标注，或手动添加时间列。")

        time_windows = build_time_windows(gt_intervals, ignore_head_sec, ignore_tail_sec)

        y_true = []
        y_pred = []
        common = []

        for (vid, ts), pl in sorted(pred_by_time.items()):
            if vid not in gt_intervals:
                continue
            window = time_windows.get(vid)
            if window is None:
                continue
            if ts < window[0] or ts > window[1]:
                continue
            intervals = sorted(gt_intervals[vid], key=lambda x: x[0])
            lv = resolve_level_by_time(intervals, ts)
            if lv is not None:
                common.append((vid, ts))
                y_true.append(lv)
                y_pred.append(pl)

        n_total = len(common)
        if n_total == 0:
            sys.exit("[错误] 真值与预测无共同时间点，请检查 video_name 和 timestamp_sec 是否对齐")
        print(f"[信息] 时间对齐: 共 {n_total} 个共同时间点")

    # ---------- 1. 混淆矩阵 ----------
    cm = defaultdict(lambda: defaultdict(int))
    for yt, yp in zip(y_true, y_pred):
        cm[yt][yp] += 1

    # ---------- 2. 等级精确率 / 召回率 / F1 ----------
    precisions = {}
    recalls = {}
    f1s = {}
    for lv in LEVELS:
        tp = cm[lv][lv]
        fp = sum(cm[other][lv] for other in LEVELS if other != lv)
        fn = sum(cm[lv][other] for other in LEVELS if other != lv)
        prec = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        rec = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        f1 = 2 * prec * rec / (prec + rec) if (prec + rec) > 0 else 0.0
        precisions[lv] = prec
        recalls[lv] = rec
        f1s[lv] = f1

    # ---------- 3. 分级准确率 ----------
    correct = sum(1 for yt, yp in zip(y_true, y_pred) if yt == yp)
    acc = correct / n_total

    # ---------- 4. 误报率 FAR (预测 > 真值) ----------
    far_count = sum(
        1 for yt, yp in zip(y_true, y_pred)
        if LEVEL2IDX[yp] > LEVEL2IDX[yt]
    )
    far = far_count / n_total

    # ---------- 5. 漏报率 MDR (预测 < 真值) ----------
    mdr_count = sum(
        1 for yt, yp in zip(y_true, y_pred)
        if LEVEL2IDX[yp] < LEVEL2IDX[yt]
    )
    mdr = mdr_count / n_total

    # ---------- 6. MAE ----------
    mae = sum(abs(LEVEL2IDX[yp] - LEVEL2IDX[yt]) for yt, yp in zip(y_true, y_pred)) / n_total

    return {
        "n_total": n_total,
        "cm": cm,
        "precisions": precisions,
        "recalls": recalls,
        "f1s": f1s,
        "acc": acc,
        "far": far,
        "mdr": mdr,
        "mae": mae,
    }


def print_console(metrics):
    print("\n" + "=" * 60)
    print("静态障碍物距离阈值预警效果测试结果")
    print("=" * 60)

    print(f"\n总帧数: {metrics['n_total']}")

    # 混淆矩阵
    print("\n【混淆矩阵】")
    label = "真值\\预测"
    header = f"{label:>8}"
    for lv in LEVELS:
        header += f"{lv:>8}"
    print(header)
    print("-" * (8 + 8 * len(LEVELS)))
    for yt in LEVELS:
        row = f"{yt:>8}"
        for yp in LEVELS:
            row += f"{metrics['cm'][yt][yp]:>8}"
        print(row)

    # 各等级 P/R/F1
    print("\n【各等级精确率 / 召回率 / F1】")
    print(f"{'等级':>8} {'精确率':>10} {'召回率':>10} {'F1分数':>10}")
    print("-" * 42)
    for lv in LEVELS:
        print(f"{lv:>8} {metrics['precisions'][lv]:>10.4f} {metrics['recalls'][lv]:>10.4f} {metrics['f1s'][lv]:>10.4f}")

    # 综合指标
    print("\n【综合指标】")
    print(f"  分级准确率 (Acc) : {metrics['acc']:.4f}")
    print(f"  误报率   (FAR)   : {metrics['far']:.4f}")
    print(f"  漏报率   (MDR)   : {metrics['mdr']:.4f}")
    print(f"  平均绝对误差 MAE : {metrics['mae']:.4f}")


def print_latex(metrics):
    """输出可直接贴入论文的 LaTeX 代码"""
    print("\n" + "=" * 60)
    print("LaTeX 表格代码（可直接贴入论文 D4 表）")
    print("=" * 60)

    # D4a: 分级有效性表
    latex = r"""
% 表 D4a：静态阈值判级有效性统计表
\begin{table}[htbp]
\centering
\caption{D4：静态阈值判级有效性统计表}
\label{tab:ch5_d4_static_warning}
\begin{tabular}{p{3.2cm}p{2.6cm}p{2.6cm}p{2.6cm}}
\hline
指标项 & 提示级(3.0m) & 报警级(2.0m) & 紧急级(1.0m) \\
\hline
精确率  & """ + f"{metrics['precisions']['提示']:.4f}" + r""" & """ + f"{metrics['precisions']['报警']:.4f}" + r""" & """ + f"{metrics['precisions']['紧急']:.4f}" + r""" \\
召回率  & """ + f"{metrics['recalls']['提示']:.4f}" + r""" & """ + f"{metrics['recalls']['报警']:.4f}" + r""" & """ + f"{metrics['recalls']['紧急']:.4f}" + r""" \\
F1分数  & """ + f"{metrics['f1s']['提示']:.4f}" + r""" & """ + f"{metrics['f1s']['报警']:.4f}" + r""" & """ + f"{metrics['f1s']['紧急']:.4f}" + r""" \\
\hline
分级准确率 & \multicolumn{3}{c}{""" + f"{metrics['acc']:.4f}" + r"""} \\
误报率（过度预警率） & \multicolumn{3}{c}{""" + f"{metrics['far']:.4f}" + r"""} \\
漏报率（预警不足率） & \multicolumn{3}{c}{""" + f"{metrics['mdr']:.4f}" + r"""} \\
平均绝对误差 MAE & \multicolumn{3}{c}{""" + f"{metrics['mae']:.4f}" + r"""} \\
\hline
\end{tabular}
\end{table}
"""
    print(latex)

    # D4b: 混淆矩阵表
    latex_cm = r"""
% 表 D4b：等级混淆矩阵
\begin{table}[htbp]
\centering
\caption{D4b：静态预警等级混淆矩阵}
\label{tab:ch5_d4b_confusion_matrix}
\begin{tabular}{c|cccc}
\hline
真值\textbackslash 预测 & 无危险 & 提示 & 报警 & 紧急 \\
\hline
"""
    for yt in LEVELS:
        row = f"{yt}"
        for yp in LEVELS:
            row += f" & {metrics['cm'][yt][yp]}"
        row += r" \\" + "\n"
        latex_cm += row
    latex_cm += r"""\hline
\end{tabular}
\end{table}
"""
    print(latex_cm)


def main():
    parser = argparse.ArgumentParser(description="静态障碍物距离阈值预警效果测试指标统计")
    parser.add_argument("--gt", required=True, help="真值 CSV 文件路径")
    parser.add_argument("--pred", required=True, help="系统预测 CSV 文件路径")
    parser.add_argument("--time-offset", type=float, default=0.0,
                        help="时间偏移量（秒）。视频比 ROS 晚开始时设正值。"
                             "例如：视频从 rosbag 第 12s 开始，则设 --time-offset 12")
    parser.add_argument("--time-scale", type=float, default=1.0,
                        help="时间缩放系数。录屏时 ROS 以非原速播放需设置。"
                             "例如：0.5 倍速播放则设 --time-scale 0.5")
    parser.add_argument("--ignore-head-sec", type=float, default=0.0,
                        help="时间对齐模式下，忽略每个视频开头多少秒（默认 0）")
    parser.add_argument("--ignore-tail-sec", type=float, default=0.0,
                        help="时间对齐模式下，忽略每个视频结尾多少秒（默认 0）")
    args = parser.parse_args()

    if args.ignore_head_sec < 0 or args.ignore_tail_sec < 0:
        sys.exit("[错误] --ignore-head-sec 和 --ignore-tail-sec 不能为负数")

    gt_by_frame, gt_intervals = read_ground_truth(args.gt)
    pred_by_frame, pred_by_time, mode = read_prediction(args.pred)

    # 应用时间变换（仅在时间对齐模式下）
    # 关系：rosbag_time = offset + video_time * scale
    # 因此将真值区间（video_time）映射到 rosbag 时间轴
    if mode == "time" and (args.time_offset != 0.0 or args.time_scale != 1.0):
        offset = args.time_offset
        scale = args.time_scale
        adjusted_intervals = {}
        for vid, intervals in gt_intervals.items():
            adjusted = []
            for start, end, level in intervals:
                adjusted.append((offset + start * scale, offset + end * scale, level))
            adjusted_intervals[vid] = adjusted
        gt_intervals = adjusted_intervals
        print(f"[信息] 已应用时间变换: rosbag_time = {offset:.3f} + video_time * {scale:.3f}")

    metrics = compute_metrics(
        gt_by_frame,
        gt_intervals,
        pred_by_frame,
        pred_by_time,
        mode,
        ignore_head_sec=args.ignore_head_sec,
        ignore_tail_sec=args.ignore_tail_sec,
    )
    print_console(metrics)
    print_latex(metrics)


if __name__ == "__main__":
    main()
