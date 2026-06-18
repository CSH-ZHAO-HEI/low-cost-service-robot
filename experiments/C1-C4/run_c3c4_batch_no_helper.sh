#!/bin/bash
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
# ──────────────────────────────────────────────────────────────────────────────
# Batch runner — NO-HELPER VERSION (跑 C3 + C4 × E1, E2, E3 各 3 次)
#
# ⚠️ 這個腳本 ONLY 跑 --no-helper-prompt 版本：
#    BASE_PROMPT 中的 run_c1_* … run_c4_* helper 會被移除，LLM 必須從
#    primitive 自行組合。失敗會觸發 Judge → VLM → AdjustLLM 微調流程，
#    產生追加的微調評估資料。
#
# 對比版本（用 helper 的 baseline）請另寫腳本，不要混在這裡。
#
# 輸出 CSV 全部帶 -no-helper 後綴（runner 自動加），跟 helper 版的
# C-E1.csv / C-E2.csv 完全分開，不會覆蓋。
#
# Usage:
#   bash C1-C4/run_c3c4_batch_no_helper.sh
# ──────────────────────────────────────────────────────────────────────────────

set -uo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
OUTPUTS_DIR="${PROJECT_ROOT}/C1-C4/outputs"
PIPELINE="${PROJECT_ROOT}/C1-C4/run_e1e2e3_composite_pipeline.py"
TS=$(date +%Y%m%d_%H%M%S)
BACKUP_DIR="${OUTPUTS_DIR}/_backup_no_helper_${TS}"

echo "════════════════════════════════════════════════════════════════════════"
echo "BATCH MODE: --no-helper-prompt (primitive composition, replan-rich)"
echo "Tasks:     C3, C4"
echo "Order:     E1 → E2 → E3"
echo "Run 數：   E1 = 6 (C3 ×3 + C4 ×3)，E2 = 6 (C3 ×3 + C4 ×3) → E1+E2 = 12"
echo "           E3 = 36 (C3 ×3 + C4 ×3) × 2 models × 3 temps（靜態，快）"
echo "Adjust:    C3 / C4 max_replan_times = 3（每個 block 獨立 3 次配額，用光跳下一塊）"
echo "Output:    C-E1-no-helper.csv, C-E2-no-helper.csv, C-E3-no-helper.csv"
echo "Backup:    ${BACKUP_DIR}"
echo "════════════════════════════════════════════════════════════════════════"

# ── 0. 備份既有 no-helper CSV（不動 helper 版本的 C-E1.csv 等）─────────
mkdir -p "${BACKUP_DIR}"
echo "[batch] backing up existing -no-helper CSVs and adjust evidence"
cp "${OUTPUTS_DIR}"/C-E*-no-helper*.csv "${BACKUP_DIR}/" 2>/dev/null || true
cp -r "${OUTPUTS_DIR}/judge_events_per_run" "${BACKUP_DIR}/" 2>/dev/null || true
cp -r "${OUTPUTS_DIR}/run_evidence" "${BACKUP_DIR}/" 2>/dev/null || true

# ── 1. 清乾淨舊的 vlm_image_writer ─────────────────────────────────────
pkill -9 -f vlm_image_writer 2>/dev/null || true
sleep 1

# ── 2. 跑 pipeline（強制帶 --no-helper-prompt）──────────────────────────
# 預估時間：~45-90 min（每個 task 約 2-5 min × 3 repeats × 2 exp 跑 Gazebo
# + E3 靜態 ~10 min）
echo "[batch] launching pipeline (estimated 45-90 minutes)"

# C3 跑 adjust=3（每個 block 獨立 3 次，紅藍黃各 3 次）
echo "[batch] ===== C3 with adjust_policy=3 per block (E1+E2+E3) ====="
python3 "${PIPELINE}" \
    --tasks C3 \
    --order e1 e2 e3 \
    --e1-repeats 3 \
    --e2-repeats 3 \
    --e3-repeats 3 \
    --e1-adjust-policy 3 \
    --e2-adjust-policies 3 \
    --e3-models flash pro \
    --e3-temperatures 0.0 0.4 0.8 \
    --start-writer \
    --continue-on-error \
    --no-helper-prompt
RC_C3=$?

# C4 跑 adjust=3（patrol + 單個 block，相對簡單）
echo "[batch] ===== C4 with adjust_policy=3 (E1+E2+E3) ====="
python3 "${PIPELINE}" \
    --tasks C4 \
    --order e1 e2 e3 \
    --e1-repeats 3 \
    --e2-repeats 3 \
    --e3-repeats 3 \
    --e1-adjust-policy 3 \
    --e2-adjust-policies 3 \
    --e3-models flash pro \
    --e3-temperatures 0.0 0.4 0.8 \
    --start-writer \
    --continue-on-error \
    --no-helper-prompt
RC_C4=$?

# 任一 fail 就標記 fail；都 OK 才 0
if [[ $RC_C3 -ne 0 || $RC_C4 -ne 0 ]]; then
    PIPELINE_RC=1
else
    PIPELINE_RC=0
fi
echo "[batch] C3 rc=${RC_C3}, C4 rc=${RC_C4}, combined rc=${PIPELINE_RC}"

# ── 3. cleanup ─────────────────────────────────────────────────────────
pkill -9 -f vlm_image_writer 2>/dev/null || true

# ── 4. 把這次 batch 結果複製到專屬資料夾，CSV 檔名也帶 batch_id ─────
# 結構：
#   outputs/batch_no_helper_<TS>/
#     ├── C-E1-no-helper__batch_<TS>.csv
#     ├── C-E1-no-helper-appendix__batch_<TS>.csv
#     ├── C-E2-no-helper__batch_<TS>.csv
#     ├── ... (其他 CSV)
#     ├── judge_events_per_run/   (本批次完整 jsonl 拷貝)
#     └── run_evidence/           (本批次完整 evidence 拷貝)
#
# 這樣每次跑 batch 都是獨立快照，不會跟下一次 batch 互相覆蓋。
# outputs 下的「最新版」(C-E1-no-helper.csv 等沒帶 batch_id) 也保留
# 給 quick check 用，不刪。
BATCH_DIR="${OUTPUTS_DIR}/batch_no_helper_${TS}"
mkdir -p "${BATCH_DIR}"
echo "[batch] copying this batch's CSVs to ${BATCH_DIR} with batch_id suffix"
for f in C-E1-no-helper.csv C-E1-no-helper-appendix.csv \
         C-E2-no-helper.csv C-E2-no-helper-appendix.csv \
         C-E3-no-helper.csv C-E3-no-helper-appendix.csv; do
    src="${OUTPUTS_DIR}/${f}"
    if [[ -f "${src}" ]]; then
        # 把 .csv 之前的部分 + __batch_<TS> + .csv
        base="${f%.csv}"
        dst="${BATCH_DIR}/${base}__batch_${TS}.csv"
        cp "${src}" "${dst}"
    fi
done
# 微調事件原始資料也拷貝
cp -r "${OUTPUTS_DIR}/judge_events_per_run" "${BATCH_DIR}/" 2>/dev/null || true
cp -r "${OUTPUTS_DIR}/run_evidence"         "${BATCH_DIR}/" 2>/dev/null || true

# ── 5. summary ───────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════════════════════"
echo "[batch] DONE (pipeline rc=${PIPELINE_RC})  — NO-HELPER VERSION"
echo "[batch] batch_id = ${TS}"
echo "════════════════════════════════════════════════════════════════════════"
echo ""
echo "本批次 CSV 快照（檔名含 __batch_${TS}）："
for f in "${BATCH_DIR}"/*.csv; do
    if [[ -f "${f}" ]]; then
        n=$(($(wc -l < "${f}") - 1))
        echo "  ✓ ${f#${OUTPUTS_DIR}/}  rows=${n}"
    fi
done

echo ""
echo "Latest 版本（可被下次 batch 覆蓋）："
for f in C-E1-no-helper.csv C-E2-no-helper.csv C-E3-no-helper.csv; do
    p="${OUTPUTS_DIR}/${f}"
    if [[ -f "${p}" ]]; then
        n=$(($(wc -l < "${p}") - 1))
        echo "  ${f}  rows=${n}"
    fi
done

echo ""
echo "微調事件資料："
N_JSONL=$(ls -1 "${BATCH_DIR}/judge_events_per_run/"*.jsonl 2>/dev/null | wc -l)
N_EVID=$(ls -1 "${BATCH_DIR}/run_evidence/"*.json 2>/dev/null | wc -l)
echo "  本批次 judge_events_per_run/  共 ${N_JSONL} 個 jsonl"
echo "  本批次 run_evidence/          共 ${N_EVID} 個 json"

echo ""
echo "對照：helper baseline 的 C-E1.csv / C-E2.csv 等未被觸碰"
echo "上次跑的備份：${BACKUP_DIR}"
echo "本次 batch 完整快照：${BATCH_DIR}"

exit ${PIPELINE_RC}
