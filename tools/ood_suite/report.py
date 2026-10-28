"""Regenerate auditable Markdown and JSON summaries from saved OOD trials."""

import argparse
import collections
import json
import math
from pathlib import Path

from .metrics import wilson_interval


PAPER_OBJECTS = ("box", "espressomachine", "ketchup", "laptop", "microwave", "mixer", "notebook", "waffleiron")
ERROR_FIELDS = (
    "paper_position_error_m", "paper_orientation_error_rad", "paper_articulation_error_rad",
    "public_position_error_m", "public_orientation_error_rad", "public_articulation_error_rad",
    "public_position_error_clipped_m", "public_orientation_error_clipped_rad", "public_articulation_error_clipped_rad",
    "lift_max_height_m", "lift_final_height_m", "lift_target_error_m",
)


def _load_rows(path):
    if not path.exists():
        return []
    rows = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if line.strip():
            try:
                row = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError("Invalid JSON at %s:%d: %s" % (path, number, error))
            if not isinstance(row, dict):
                raise ValueError("Each episodes.jsonl row must be an object")
            rows.append(row)
    return rows


def _is_success(row, metric):
    return bool(row.get("valid", True) and row.get(metric, False))


def _reachable(row):
    for key in ("target_reachable", "joint_target_reachable", "articulation_reachable", "target_within_joint_limits", "joint_limit_feasible"):
        if key in row:
            return bool(row[key])
    return None


def _aggregate(planned, completed):
    results = [completed[item["episode_id"]] for item in planned if item["episode_id"] in completed]
    valid = [row for row in results if row.get("valid", True)]
    count = len(planned)
    summary = {
        "planned": count,
        "completed": len(results),
        "missing": count - len(results),
        "invalid": len(results) - len(valid),
        "valid": len(valid),
        "unreachable_targets": sum(_reachable(completed.get(item["episode_id"], item)) is False for item in planned),
        "reachability_unknown": sum(_reachable(completed.get(item["episode_id"], item)) is None for item in planned),
        "provisional": len(results) < count,
    }
    for metric in ("paper_success", "public_success", "ood_goal_success", "lift_target_success"):
        successes = sum(_is_success(row, metric) for row in results)
        summary[metric + "_count"] = successes
        summary[metric + "_rate"] = successes / count if count else None
        # Unobserved trials are a provisional lower bound, not observed failures
        # from which a binomial confidence interval can legitimately be computed.
        summary[metric + "_wilson95"] = (wilson_interval(successes, count)
                                         if len(results) == count else None)
    for field in ERROR_FIELDS:
        values = [float(row[field]) for row in valid
                  if row.get(field) is not None and math.isfinite(float(row[field]))]
        summary[field + "_mean"] = sum(values) / len(values) if values else None
        summary[field + "_n"] = len(values)
    return summary


def _fmt(value, digits=4):
    return "—" if value is None else ("%.*f" % (digits, value))


def _profile_description(profile, field, default):
    value = profile.get(field)
    return default if value is None else json.dumps(value, ensure_ascii=False)


def _success_cell(summary, prefix):
    rate = summary[prefix + "_rate"]
    if rate is None:
        return "—"
    interval = summary[prefix + "_wilson95"]
    if interval is None:
        return "%d/%d (%.1f%%; 暫定下限・CI未算出)" % (
            summary[prefix + "_count"], summary["planned"], 100 * rate)
    low, high = interval
    return "%d/%d (%.1f%%; %.1f–%.1f%%)" % (
        summary[prefix + "_count"], summary["planned"], 100 * rate, 100 * low, 100 * high)


def _mean_cells(summary, public=False, clipped=False):
    prefix = "public" if public else "paper"
    suffix = "_clipped" if clipped else ""
    return " | ".join(_fmt(summary["%s_%s_error%s_%s_mean" % (prefix, name, suffix, unit)])
                      for name, unit in (("position", "m"), ("orientation", "rad"), ("articulation", "rad")))


def _paper_baselines(plan):
    explicit = plan.get("paper_baselines")
    if isinstance(explicit, dict) and "dynamic_task_table3" in explicit:
        return explicit
    path = Path(__file__).resolve().parents[2] / "configs" / "ood_paper_baselines.json"
    return json.loads(path.read_text(encoding="utf-8"))


def _paired_comparisons(planned, completed, profiles, id_profile):
    by_profile = collections.defaultdict(dict)
    for item in planned:
        key = (item["object"], item.get("sample_index", item.get("sample", 0)))
        by_profile[item["profile"]][key] = item
    pairs = []
    for profile in profiles:
        if profile == id_profile:
            continue
        for metric in ("paper_success", "public_success"):
            counts = {"both_success": 0, "id_only_success": 0, "ood_only_success": 0, "both_failure": 0, "missing_pairs": 0}
            keys = set(by_profile[id_profile]) | set(by_profile[profile])
            for key in sorted(keys):
                left = by_profile[id_profile].get(key)
                right = by_profile[profile].get(key)
                if not left or not right or left["episode_id"] not in completed or right["episode_id"] not in completed:
                    counts["missing_pairs"] += 1
                    continue
                left_success = _is_success(completed[left["episode_id"]], metric)
                right_success = _is_success(completed[right["episode_id"]], metric)
                name = "both_success" if left_success and right_success else "id_only_success" if left_success else "ood_only_success" if right_success else "both_failure"
                counts[name] += 1
            n = sum(counts[key] for key in ("both_success", "id_only_success", "ood_only_success", "both_failure"))
            pairs.append(dict(profile=profile, metric=metric, paired_trials=n,
                              success_difference_pp=100.0 * (counts["ood_only_success"] - counts["id_only_success"]) / n if n else None,
                              **counts))
    return pairs


def generate_report(results_dir):
    """Read plan.json + episodes.jsonl; write report.md and summary.json.

    Planned trials always determine success denominators. Invalid trials count
    as failures, while uncompleted plans are explicit provisional non-successes.
    Error means use valid finite measurements only and retain their sample counts.
    No actual simulation is performed by this function.
    """
    directory = Path(results_dir).resolve()
    plan = json.loads((directory / "plan.json").read_text(encoding="utf-8"))
    planned = plan.get("episodes", plan.get("trials", []))
    if not planned:
        raise ValueError("plan.json must contain a non-empty episodes list")
    for item in planned:
        for field in ("episode_id", "object", "profile"):
            if field not in item:
                raise ValueError("Each planned episode requires %s" % field)
    ids = [item["episode_id"] for item in planned]
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate episode_id in plan.json")
    rows = _load_rows(directory / "episodes.jsonl")
    completed = {}
    for row in rows:
        identifier = row.get("episode_id")
        if identifier not in ids:
            raise ValueError("Result episode_id is absent from plan: %r" % identifier)
        if identifier in completed:
            raise ValueError("Duplicate episode result: %s" % identifier)
        expected = next(item for item in planned if item["episode_id"] == identifier)
        if any(row.get(key) != expected[key] for key in ("object", "profile")):
            raise ValueError("Result object/profile differs from plan for %s" % identifier)
        completed[identifier] = row
    baseline = _paper_baselines(plan)
    paper_objects = tuple(baseline.get("paper_objects", PAPER_OBJECTS))
    objects = list(dict.fromkeys(item["object"] for item in planned))
    profiles = list(dict.fromkeys(item["profile"] for item in planned))
    groups = []
    aggregates = []
    for profile in profiles:
        for obj in objects:
            subset = [item for item in planned if item["object"] == obj and item["profile"] == profile]
            if subset:
                groups.append(dict(object=obj, profile=profile, **_aggregate(subset, completed)))
        for scope, subset in (
            ("all_objects", [item for item in planned if item["profile"] == profile]),
            ("paper_objects", [item for item in planned if item["profile"] == profile and item["object"] in paper_objects]),
        ):
            if subset:
                summary = _aggregate(subset, completed)
                per_object = [group for group in groups if group["profile"] == profile and (scope == "all_objects" or group["object"] in paper_objects)]
                summary["object_count"] = len(per_object)
                for metric in ("paper_success", "public_success"):
                    summary[metric + "_macro_rate"] = sum(group[metric + "_rate"] for group in per_object) / len(per_object)
                aggregates.append(dict(profile=profile, scope=scope, **summary))

    id_profile = next((name for name in ("id", "in_distribution", "id_control", "published") if name in profiles), None)
    paired = _paired_comparisons(planned, completed, profiles, id_profile) if id_profile else []
    controller_changes = sorted({change for row in rows for change in row.get("controller_changes", [])})
    summary = {
        "schema_version": 1, "planned": len(planned), "completed": len(completed),
        "missing": len(planned) - len(completed), "objects": objects, "profiles": profiles,
        "groups": groups, "aggregates": aggregates, "paired_comparisons": paired,
        "controller_changes": controller_changes,
        "baseline_source": baseline["source"], "published_baselines": baseline["dynamic_task_table3"],
    }
    lines = [
        "# ArtiGrasp OOD 目標状態の評価結果", "",
        "進捗: **%d / %d 試行完了**。" % (len(completed), len(planned)), "",
        "成功率の分母は全計画試行です。実行異常は失敗として含め、未実行は未達として明示します。未実行が残る表は暫定の下限値で、信頼区間は未算出です。全試行完了後にWilson法の95%区間を算出します。", "",
        "## 評価条件", "",
        "分割: `%s`、seed: `%s`。条件間で同じ参照・初期状態を対応させ、目標を変えて評価します。持ち上げ量は移送途中の目標で、最終配置位置のz成分は机上です。" % (plan.get("split", plan.get("config", {}).get("split", "不明")), plan.get("seed", plan.get("config", {}).get("seed", "不明"))), "",
        "| 条件 | 持ち上げ量 (m) | 水平移動距離 (m) | 関節角度 |",
        "| --- | --- | --- | --- |",
    ]
    for name in profiles:
        configuration = plan.get("config", {}).get("profiles", {}).get(name, {})
        angle_description = _profile_description(configuration, "articulation_deg", "公開設定（初期角度を参照）")
        if "articulation_deg" in configuration:
            angle_description += " deg"
        elif "articulation_rad" in configuration:
            angle_description = _profile_description(configuration, "articulation_rad", "") + " rad"
        lines.append("| %s | %s | %s | %s |" % (name,
            _profile_description(configuration, "lift_m", "公開設定: 通常0.15 / ketchup 0.30"),
            _profile_description(configuration, "translation_distance_m", "公開設定: x,y各[-0.10,-0.05]"), angle_description))
    lines.extend(["",
        "## 指標の区別", "",
        "- **論文 Suc.T**: 最後の `env.step` 後の物理状態で PE < 0.05 m、SO(3)測地距離 AE < 0.2 rad、AAE < 0.5 rad を同時に満たす割合。誤差は無加工です。",
        "- **公開実装の判定互換**: `92fb307` の compose/ketchup の成功・誤差の計算式を同じロールアウトへ適用します。到達・姿勢・開閉を別々の時刻に達成しても成功を保持し、最後に公開コードのguardを適用します。元コードの1ステップ遅れと、物理姿勢の代わりに目標quaternionを読む添字も保持します。制御側の補正がある場合、元スクリプトと同一の軌跡を再現したという意味ではありません。",
        "- **OOD目標達成**: 論文Suc.Tに加え、持ち上げ区間で指定上昇量との差が0.05 m未満になることを要求する本実験独自の補助指標です。最終目標は机上への再配置なので、論文Suc.Tだけでは途中の大きな持ち上げ目標の達成を判定できません。",
        "- 公開実装のPE/AEは移送終了付近、AAEは最後のステップの直前の状態です。公開実装の表示に合わせたクリップ値（PE 0.1 m、AE 0.3 rad、AAE 1.0 rad上限）を無加工値と分けます。",
        "- 誤差平均は正常かつ有限の測定値だけを使います。測定数は別表に表示します。異常試行を誤差0として埋めません。", "",
        "180°など関節範囲外の要求目標も勝手に丸めず評価します。範囲外件数を併記し、到達可能な目標への汎化とは区別してください。", "",
        "## 物体別の成功率", "",
        "| 条件 | 物体 | 完了/計画 | 異常 | 未実行 | 関節範囲外 | 論文 Suc.T (95% CI) | 公開実装互換 (95% CI) |",
        "| --- | --- | ---: | ---: | ---: | ---: | --- | --- |",
    ])
    if "ketchup_forward_requested_articulation" in controller_changes:
        location = lines.index("180°など関節範囲外の要求目標も勝手に丸めず評価します。範囲外件数を併記し、到達可能な目標への汎化とは区別してください。")
        lines[location:location] = [
            "**ketchupの目標角度入力の補正**: 公開スクリプトは`set_obj_goal(angle)`で表示・位置用の目標を変えても、ポリシーの関節目標`final_obj_angle_`を更新せず0 radを維持します。本実験では開閉準備への遷移時に要求角度を`set_goals`にも渡します。元の段階・制御切替は保持しますが、要求目標を実際にポリシーへ渡すための補正により軌跡は公開スクリプトと異なり得ます。公開互換の数値は、この補正後の同じ軌跡に元の判定式を適用した結果です。", "",
        ]
    for group in groups:
        lines.append("| %s | %s | %d/%d | %d | %d | %d | %s | %s |" % (
            group["profile"], group["object"], group["completed"], group["planned"], group["invalid"], group["missing"], group["unreachable_targets"],
            _success_cell(group, "paper_success"), _success_cell(group, "public_success")))

    lines.extend(["", "## 論文 Table 3 との比較", "",
        "[ArtiGrasp 論文・Table 3 / §6.1 / Supplement B.4](%s)。公開値は8物体・約7500試行の集約で、物体別のTable 3値は掲載されていません。物体・条件ごとに同数のサンプルを評価する本実験とは重みや目標分布が異なります。差は参考比較であり、再現誤差とは解釈しません。" % baseline["source"]["url"], "",
        "| 結果 | 対象 | n | 論文 Suc.T | PE (m) | AE (rad) | AAE (rad) |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: |"])
    for row in baseline["dynamic_task_table3"]:
        lines.append("| 論文 %s | 論文8物体 | 約7500 | %.2f | %.2f | %.2f | %.2f |" % (
            row["model"], row["task_success"], row["position_error_m"], row["orientation_error_rad"], row["articulation_error_rad"]))
    for aggregate in aggregates:
        scope = "全%d物体" % aggregate["object_count"] if aggregate["scope"] == "all_objects" else "論文対象の%d物体" % aggregate["object_count"]
        lines.append("| 実験 %s | %s | %d | %s | %s |" % (aggregate["profile"], scope, aggregate["planned"], _fmt(aggregate["paper_success_rate"]), _mean_cells(aggregate)))
    lines.extend(["", "### 集約方法", "",
        "| 条件 | 対象 | 論文・試行平均 (95% CI) | 論文・物体平均 | 公開互換・試行平均 (95% CI) | 公開互換・物体平均 |",
        "| --- | --- | --- | ---: | --- | ---: |"])
    for group in aggregates:
        lines.append("| %s | %s | %s | %s | %s | %s |" % (group["profile"], group["scope"], _success_cell(group, "paper_success"),
            _fmt(group["paper_success_macro_rate"]), _success_cell(group, "public_success"), _fmt(group["public_success_macro_rate"])))
    lines.extend(["", "## 持ち上げ目標と複合目標の補助評価", "",
        "持ち上げ達成は区間 `[grasp_steps, grasp_steps + transit_steps]` の物理高さから初期高さを引いた上昇量で判定します。目標誤差は区間内での最小絶対誤差です。これらはTable 3の論文指標と区別して解釈してください。", "",
        "| 条件 | 物体 | 持ち上げ目標達成 (95% CI) | OOD目標達成 (95% CI) | 最大上昇量 (m) | 持ち上げ区間終了の上昇量 (m) | 目標との最小誤差 (m) |",
        "| --- | --- | --- | --- | ---: | ---: | ---: |"])
    for group in groups:
        lines.append("| %s | %s | %s | %s | %s | %s | %s |" % (group["profile"], group["object"],
            _success_cell(group, "lift_target_success"), _success_cell(group, "ood_goal_success"), _fmt(group["lift_max_height_m_mean"]),
            _fmt(group["lift_final_height_m_mean"]), _fmt(group["lift_target_error_m_mean"])))
    lines.extend(["", "## 誤差の物体別集計", "",
        "nはPE/AE/AAEの各有限測定数です。論文の誤差は同一の最終時刻、公開互換は元実装の各測定時刻です。", "",
        "| 条件 | 物体 | 指標 | n (PE/AE/AAE) | PE (m) | AE (rad) | AAE (rad) |",
        "| --- | --- | --- | --- | ---: | ---: | ---: |"])
    for group in groups:
        for public, clipped, label in ((False, False, "論文・無加工"), (True, False, "公開互換・無加工"), (True, True, "公開互換・表示クリップ")):
            prefix, suffix = ("public" if public else "paper"), ("_clipped" if clipped else "")
            counts = "/".join(str(group["%s_%s_error%s_%s_n" % (prefix, name, suffix, unit)]) for name, unit in (("position", "m"), ("orientation", "rad"), ("articulation", "rad")))
            lines.append("| %s | %s | %s | %s | %s |" % (group["profile"], group["object"], label, counts, _mean_cells(group, public, clipped)))

    if paired:
        lines.extend(["", "## 同じ参照ペアによる ID → OOD の変化", "",
            "同じ物体・sample_indexを対応させた比較です。両条件の実行が完了したペアを示し、異常完了は失敗に含めます。実行前に同じsample_indexが同じ参照/初期状態へ対応していることはplan.jsonで確認できます。", "",
            "| OOD条件 | 指標 | 完了ペア | 両方成功 | IDのみ成功 | OODのみ成功 | 両方失敗 | 未完ペア | 成功率差 (pp) |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"])
        for pair in paired:
            lines.append("| %s | %s | %d | %d | %d | %d | %d | %d | %s |" % (pair["profile"], pair["metric"], pair["paired_trials"], pair["both_success"], pair["id_only_success"], pair["ood_only_success"], pair["both_failure"], pair["missing_pairs"], _fmt(pair["success_difference_pp"], 1)))

    lines.extend(["", "## 定性的な動画", "",
        "成功/失敗は指定した指標に従います。各条件・物体で最大5件ずつを保存します。0成功の条件で成功例を生成することはできないため、不足をそのまま記録します。"])
    indexes = sorted((directory / "videos").glob("*/index.json"))
    if (directory / "videos" / "index.json").exists():
        indexes.append(directory / "videos" / "index.json")
    if indexes:
        for index in indexes:
            md_path = index.with_suffix(".md")
            relative = (md_path if md_path.exists() else index).relative_to(directory).as_posix()
            lines.append("- [%s の動画一覧](%s)" % (index.parent.name, relative))
    else:
        lines.append("動画インデックスはまだありません。定量評価後にrecordコマンドを実行してください。")
    lines.extend(["", "| 条件 | 物体 | 正常な成功候補 | 正常な失敗候補 | 成功不足 (5件基準) | 失敗不足 (5件基準) |",
        "| --- | --- | ---: | ---: | ---: | ---: |"])
    for group in groups:
        valid = [row for row in completed.values() if row["object"] == group["object"] and row["profile"] == group["profile"] and row.get("valid", True)]
        successes = sum(_is_success(row, "paper_success") for row in valid)
        failures = len(valid) - successes
        lines.append("| %s | %s | %d | %d | %d | %d |" % (group["profile"], group["object"], successes, failures, max(0, 5 - successes), max(0, 5 - failures)))
    lines.extend(["", "上表の候補と不足は論文指標を基準にしています。実際の録画件数・再生検証・公開互換指標での選別は各動画インデックスを参照してください。", "",
        "## 再現用ファイル", "",
        "- [計画と固定seed](plan.json)", "- [試行ごとの全結果](episodes.jsonl)", "- [機械可読集計](summary.json)", "",
        "計画は条件ごとに分けており、物体数や関節上限が異なる対象を黙って落としません。未実行・異常・範囲外目標の有無を確認してから結果を使用してください。", ""])
    (directory / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (directory / "report.md").write_text("\n".join(lines), encoding="utf-8")
    return {"report": str(directory / "report.md"), "summary": str(directory / "summary.json")}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_dir")
    args = parser.parse_args(argv)
    paths = generate_report(args.results_dir)
    print(paths["report"])


if __name__ == "__main__":
    main()
