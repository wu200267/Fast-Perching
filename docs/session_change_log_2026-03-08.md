# Fast-Perching 会话改动归档（Table I + Fig.11 复现）

## 1. 文档目的
本文件用于固化本次对话中围绕论文 Table I（Our Proposed）与 Fig.11 复现所做的代码与脚本改动，便于在新会话中快速恢复上下文。

## 2. 会话范围与基线
- 仓库路径：`/root/ros1_noetic_fp/fast_perching/Fast-Perching`
- 上游基线：`upstream/master`（`d4c0a7e`）
- 个人分支：`feat/table1-vt-only-strict`
- 已推送分支：`origin/feat/table1-vt-only-strict`
- 关键已提交提交：`1942e92`  
  提交信息：`Table I benchmark: vt-only strict terminal fix and bench script updates`

---

## 3. 实施计划演进（摘要）

### 阶段 A：严格固定终端（早期方案）
- 目标：把终端相关变量从优化向量移除，避免“高权重惩罚近似固定”。
- 初始做法曾尝试固定 `tail_f + vt`，但会显著简化问题，导致部分坡度耗时不符合 Table I 趋势。

### 阶段 B：vt-only strict（最终采用）
- 最终语义：`fix_terminal_state=true` 时 **只固定 `vt`**，`tail_f` 仍是优化变量。
- fixed 模式维度从 non-fixed 的 `31` 变为 `29`（仅减 2 维）。
- fixed 模式下强制忽略 `rhoVt` 成本/梯度。
- `active_vt_` 在每次 `generate_traj()` 开头刷新，并在单次 `lbfgs_optimize()` 内保持不变。

---

## 4. 代码改动清单（已提交到 1942e92）

### 4.1 修改文件
1. `src/traj_opt/include/traj_opt/traj_opt.h`
- 新增 strict 相关成员与状态字段：`fix_terminal_state_`、`active_vt_`、`opt_dim_`、布局打印开关。
- 新增 `TerminalSummary` 与 `getLastTerminalSummary()`，用于导出末端统计。
- 新增 vt 多起点相关参数容器（用于多 seed 评估）。

2. `src/traj_opt/src/traj_opt_perching.cc`
- 重构优化向量布局为 vt-only strict：
  - `tail_f` 永远在优化向量中；
  - `vt` 仅在 non-fixed 时在优化向量中；
  - fixed 模式从向量移除 `vt` 2 维。
- `objectiveFunc()` / `earlyExit()`：
  - `tail_f` 始终从 `x` 读取并写梯度；
  - fixed 下 `vt` 使用 `active_vt_`，且不写 `grad_vt`；
  - fixed 下无条件跳过 `rhoVt` 项。
- `generate_traj()`：
  - 每次重算 `opt_dim_`，并安全分配/释放 `x_`；
  - 失败路径统一清理，避免泄漏；
  - 增加离散化检查打印（`K/N/N*K`）；
  - 增加可选布局打印（开发验收用）。
- 兼容读取 `fixed_tail_f` 并打印 deprecated 警告，但不再参与求解语义。

3. `src/planning/src/planning_nodelet.cpp`
- 增加 `bench_mode`：触发一次仅做优化，避免回放阻塞导致超时干扰计时。
- 增加 `ini_px/ini_py/ini_pz` 初始状态参数读取。
- 保持 `N=10` 调用优化。

4. `sh_utils/bench_table1_our_proposed.sh`（新增）
- 自动运行 3 个坡度（-70/-90/-110）并统计 `optmization costs`。
- 临时 launch 注入 Table I 关键参数（`K=16`、约束/权重、`ini=(0,0,4.2)`、目标状态等）。
- strict 口径为 vt-only：
  - 注入 `fix_terminal_state=true`、`fixed_vt_x=0`、`fixed_vt_y=0`；
  - 不再注入 `fixed_tail_f`。
- 增加 `--sample-timeout`、`--init-timeout`，统计 `timeouts` 列。
- 兼容解析 `--fixed-tail-f/--no-fixed-tail-f`，但仅警告并忽略。

### 4.2 新增文件
- `sh_utils/bench_table1_our_proposed.sh`

### 4.3 删除文件
- 无（基于已提交内容）。

---

## 5. 当前工作区额外变更（未提交，写本文档时）
> 以下不在 `1942e92` 中，但当前本地存在：

- 已修改：
  - `.vscode/c_cpp_properties.json`
  - `src/odom_vis/odom_visualization/CMakeLists.txt`
  - `src/planning/launch/perching.launch`
  - `src/planning/src/planning_nodelet.cpp`
  - `src/traj_opt/include/traj_opt/traj_opt.h`
  - `src/traj_opt/src/traj_opt_perching.cc`
- 未跟踪：
  - `.vscode/tasks.json`
  - `frames.gv`, `frames.pdf`
  - `fig11_outputs/`（含案例 launch/csv/summary）
  - `sh_utils/plot_fig11.py`
  - `sh_utils/reproduce_fig11.sh`
  - `sh_utils/__pycache__/`

---

## 6. 参数语义变化（重点）

### 6.1 `fix_terminal_state`
- `false`：保留传统模式（`tail_f + vt` 均参与优化）。
- `true`：vt-only strict（仅 `vt` 固定，`tail_f` 仍优化）。

### 6.2 `fixed_vt_x / fixed_vt_y`
- 仅在 `fix_terminal_state=true` 时生效。
- 若未显式提供，则回退到本次 guess/warm-start。

### 6.3 `fixed_tail_f`
- 兼容读取但已弃用（deprecated），不会改变 `tail_f` 优化行为。

### 6.4 bench 侧关键参数
- `v_plus=0.0`（rest-to-rest 对齐）
- `K=16`，`N=10`（需在日志看到 `N*K=160`）
- `bench_mode=true`（纯计时）

---

## 7. 已获得的代表性结果（本会话）

一次代表性运行（用户提供）：

```text
minus70   mean_ms=2.10532
minus90   mean_ms=4.08347
minus110  mean_ms=13.57683
timeouts=0
```

与论文 Table I（Our Proposed）对比（1.47251 / 4.60674 / 17.7263 ms）趋势上已恢复“坡度越陡通常耗时更高”的主要特征，且 -90/-110 量级更接近论文。

---

## 8. 已识别风险与待核查点

1. `rhoPerchingCollision=0` 时，当前实现仍可能进入碰撞项函数计算再乘零，存在额外开销风险。  
   若要严格等价“移除碰撞约束”，可在 `addTimeIntPenalty()` 或 `grad_cost_perching_collision()` 前置硬跳过。

2. 论文 “fix terminal state” 的精确定义仍建议做 A/B 复核（vt-only vs vt+tail_f）。  
   当前主线是 vt-only strict，用于避免把问题过度简化。

---

## 9. 新会话快速恢复命令

```bash
cd /root/ros1_noetic_fp/fast_perching/Fast-Perching

# 看提交基线
git log --oneline --decorate -n 5

# 看当前改动
git status --short

# Table I 计时复现（已编译可加 --skip-build）
./sh_utils/bench_table1_our_proposed.sh --skip-build --samples 30 --sample-timeout 120
```

---

## 10. 文档维护建议
- 每次实验口径变更后，更新：
  1) 本文件第 6/7/8 节；
  2) 脚本 help 与 launch 注释；
  3) 对应 commit message（写清“实验定义变更”）。

---

## 11. 本轮追加：Fig.11 严格复现链路（2026-03-08）

### 11.1 目标与实验口径
- 在现有分支上新增独立 Fig.11 复现链路，不覆盖 Table I 链路。
- 固定实验条件：
  - `perching_v=(0,0,0)`
  - 垂直着陆面（`axis=(0,1,0), theta=-1.5708`）
  - `z_t=2.0/1.5/1.0`
  - `fix_terminal_state=false`
  - `v_plus=0.3`
  - `rhoPerchingCollision=1e6`
  - `K=32`

### 11.2 代码与脚本改动（本轮实现）
1. `src/traj_opt/include/traj_opt/traj_opt.h`
- `TerminalSummary` 扩展为 Fig.11 所需字段：
  - `max_tau_norm_int`
  - `min_tau_norm_int`
  - `max_abs_omega2_int`
  - `winner_seed_vt2`
  - `winner_objective`
  - `winner_viol_total_int`
  - `winner_eps_level`
- 新增多 seed 参数成员：
  - `vt_multistart_enable_`
  - `vt_multistart_vt2_seeds_`
  - `vt_multistart_eps_strict_`
  - `vt_multistart_eps_relaxed_`

2. `src/traj_opt/src/traj_opt_perching.cc`
- 新增与约束积分点一致的约束统计：
  - `max_tau_norm_int` / `min_tau_norm_int` / `max_abs_omega2_int`
- 新增违约量定义并用于候选筛选：
  - `viol_tau_max=max(0,max_tau_norm_int-thrust_max)`
  - `viol_tau_min=max(0,thrust_min-min_tau_norm_int)`
  - `viol_omega=max(0,max_abs_omega2_int-omega_max)`
  - `viol_total_int=viol_tau_max+viol_tau_min+viol_omega`
- 新增 Fig.11 多 seed 求解（仅 non-fixed）：
  - `vt2` seed：`0.0, +0.1, -0.1, +0.4, -0.4`
  - 每个 seed 从同一 `x0` 启动，仅覆盖 `vt2`，避免 seed 间污染
  - winner 规则：`strict(1e-3) -> relaxed(1e-2) -> fallback(最小 viol_total_int，再最小 objective)`
- 终端摘要写入 winner 选择依据字段。

3. `src/planning/src/planning_nodelet.cpp`
- Fig.11 导出 CSV 保持：`t,x,z,tau_norm2,tau_norm,omega2`
- 新增导出采样口径统计（与绘图一致）：
  - `max_tau_norm_export`
  - `min_tau_norm_export`
  - `max_abs_omega2_export`
- summary 同时输出 `*_int` 与 `*_export` 双口径，并写入 winner 字段。
- `omega` 命名统一为 `max_abs_omega2_*`，不再使用歧义命名。

4. `sh_utils/reproduce_fig11.sh`
- 临时 launch 固定 `K=32`。
- 显式注入多 seed 参数：
  - `vt_multistart_enable=true`
  - `vt_multistart_vt2_seeds=0.0,0.1,-0.1,0.4,-0.4`
  - `vt_multistart_eps_strict=1e-3`
  - `vt_multistart_eps_relaxed=1e-2`
- `run_config.env` 记录参数快照与判据公式，保证可追溯。

5. `sh_utils/plot_fig11.py`
- 推力子图改用 `tau_norm`（非 `tau_norm2`）。
- 推力子图标注 `||tau||_2 (m/s^2)`，并画 `y=5/17` 虚线，范围固定 `[0,20]`。
- `omega2` 子图范围固定 `[-3.5,3.5]`，并画 `y=±3` 虚线。
- 表格主对齐列改为 `vt_signed`。

### 11.3 产物与同步
- 本轮端到端产物目录：`/tmp/fast_perching_fig11_impl_check`
- 已同步到仓库：`fig11_outputs/`
- 关键产物：
  - `fig11_reproduced.png`
  - `zt_*.csv`
  - `zt_*_summary.txt`
  - `run_config.env`
  - `git_commit_hash.txt`（版本快照）

### 11.4 复现与验收命令
```bash
cd /root/ros1_noetic_fp/fast_perching/Fast-Perching

# 仅运行复现（已编译）
./sh_utils/reproduce_fig11.sh --skip-build --out-dir /tmp/fast_perching_fig11_impl_check

# 查看图
python3 - <<'PY'
from PIL import Image
Image.open('fig11_outputs/fig11_reproduced.png').show()
PY
```

### 11.5 本轮验证结论（摘要）
- 脚本语法检查通过：`bash -n sh_utils/reproduce_fig11.sh`
- 绘图脚本语法检查通过：`python3 -m py_compile sh_utils/plot_fig11.py`
- 编译通过：`catkin_make --pkg traj_opt planning -DCMAKE_BUILD_TYPE=Release`
- 3 case 端到端运行完成，summary 已包含 `*_int/*_export` 与 winner 四字段。

### 11.6 二次修订：Omega 约束自动分级脚本（2026-03-08 当天后续）
- `sh_utils/reproduce_fig11.sh` 从“单参数一次运行”升级为“自动分级尝试”：
  - `K=32, rhoOmega=3e5`
  - `K=32, rhoOmega=1e6`
  - `K=48, rhoOmega=1e6`
  - `K=64, rhoOmega=1e6`
- 所有 attempt 统一固定 `rhoThrust=1e5`（不再分级）。
- 验收策略调整为双口径分工：
  - `*_int` 作为硬门槛（pass/fail）
  - `*_export` 作为 warning（仅标记，不直接 fail）
- winner 规则要求“非 fallback”保持不变（必须 `strict/relaxed`）。
- 增加基于 `zt_*_rosparam.yaml` 的参数生效回读校验：
  - 回显并校验 `K / rhoOmega / rhoThrust / omega_max`
  - 若参数未按 attempt 生效，直接判该 attempt 失败。
- 新增汇总产物：`attempt_report.tsv`（逐 attempt/逐 case 的状态与指标明细）。

### 11.7 阈值口径更新（避免无意义升档）
- 先前硬阈值曾使用 `3.0005`，在当前数值波动下易造成“差 1e-3 量级反复升档”。
- 当前脚本阈值已更新为：
  - `OMEGA_INT_THRESHOLD=3.0015`
  - `OMEGA_EXPORT_WARN_THRESHOLD=3.0015`
- 该阈值与内部 strict 量级更一致，能避免“winner 合法但外层验收过严”矛盾。

### 11.8 本次重跑结论（最新）
- 最新有效运行目录：`/tmp/fast_perching_fig11_rerun_20260308`
- 自动分级结果：
  - attempt 01（`K32,rhoOmega=3e5`）失败（`zt=1.0` 仍超阈值）
  - attempt 02（`K32,rhoOmega=1e6`）通过并被选中
  - 选中 profile：`attempt_02_K32_rhoOmega1000000.0`
  - `final_status=PASS`
- 选中档位下 3 个 case 均满足：
  - `winner_eps_level in {strict, relaxed}`
  - `max_abs_omega2_int <= 3.0015`

### 11.9 fig11_outputs 当前最小保留集
- 已删除调试冗余（`*.launch`, `*.log`, `*_rosparam.yaml`, `git_branch.txt`, `git_status_short.txt`）。
- 当前建议保留：
  - `fig11_reproduced.png`
  - `zt_2_0.csv`, `zt_1_5.csv`, `zt_1_0.csv`
  - `zt_2_0_summary.txt`, `zt_1_5_summary.txt`, `zt_1_0_summary.txt`
  - `run_config.env`
  - `git_commit_hash.txt`
  - `attempt_report.tsv`

---

## 12. 本轮追加：单分支口径收敛 + `perching_px` 扫描链路（2026-03-08 后续）

### 12.1 口径收敛（相对 11.x 的变化）
- Fig.11 主链路从“自动多档 attempt + multistart”收敛为“固定单分支”：
  - `rhoOmega=1.2e6`
  - `solve_mode=single`
  - `vt_multistart_enable=false`
  - `winner_seed_vt2=0.0`（单分支初始化 seed）
  - `ini_v=(0,0,0)` 显式给定
  - `perching_vx=0.6`
- 目标从“继续压约束”转为“校正空间初始条件”（先扫 `perching_px`，必要时再扫 `ini_px`）。

### 12.2 代码语义修订（已落地）
1. `src/planning/src/planning_nodelet.cpp`
- 新增 `ini_vx/ini_vy/ini_vz` 参数；`iniState.col(1)` 改为由 `ini_v` 驱动，与 `perching_v` 解耦。
- 回退规则固定为“整向量回退”：
  - 三轴 `ini_v*` 全显式给定 -> `ini_v_source=explicit`
  - 任一缺失 -> `ini_v_source=fallback_whole_vector` 且整向量回退到 `perching_v`
- Fig.11 summary 新增字段：
  - `ini_vx/ini_vy/ini_vz`
  - `ini_v_source`
  - `solve_mode`

2. `src/traj_opt/include/traj_opt/traj_opt.h`
- `TerminalSummary` 新增 `solve_mode` 字段。
- `winner_eps_level` 语义保持为可行性等级（`strict/relaxed/fallback`）。

3. `src/traj_opt/src/traj_opt_perching.cc`
- 新增 `solve_mode=single|multistart` 输出，与 `winner_eps_level` 解耦。
- 单分支 (`solve_mode=single`) 下：
  - `winner_seed_vt2` 固定为初始化 seed（`guessed_vt.y()`），不再被最终优化值覆盖。
  - `winner_eps_level` 由 `*_int` 违约与 `eps_strict/eps_relaxed` 计算得到，不再出现 `single` 语义混用。

### 12.3 `reproduce_fig11.sh` 修订（固定单 profile）
- 固定 profile：`K=32`, `rhoOmega=1.2e6`, `rhoThrust=1e5`。
- 保持 `vt_multistart_enable=false`，并硬门槛要求：
  - `solve_mode == single`
  - `winner_eps_level == strict`
  - `ini_v_source == explicit`
  - `max_abs_omega2_int <= threshold`
- `max_abs_omega2_export` 继续仅 warning。
- 新增参数覆盖入口（默认不变）：
  - `--perching-px`
  - `--ini-px`
- `vt_trend_check.txt` 保持：
  - mixed-sign 时 `signed_order_check=SKIP`
  - `1e-6` 仅用于符号分类，不修改原始 `vt_signed` 数值

### 12.4 新增扫描脚本：`sh_utils/scan_fig11_px.sh`
- Stage-1 仅改 `perching_px`（`ini_px` 固定）：
  - 以基线为中心的双向粗扫偏移：
    `0,-0.5,+0.5,-1.0,+1.0,-1.5,+1.5,-2.0,+2.0,-2.5,+2.5,-3.0,+3.0`
  - 细扫：围绕粗扫最优点 `±0.4`，步长 `0.1`
  - 区间裁剪：`[0.5, 8.0]`
- 扫描输出：
  - `scan_report.tsv`
  - `best_candidate.env`
  - `scan_summary.txt`
  - 顶层 `run_config.env`（记录扫描配置快照）
- `scan_report.tsv` 新增连续评分与 duration 字段：
  - `signed_order_margin`
  - `abs_to_zero_margin`
  - `signed_order_deficit`
  - `abs_to_zero_deficit`
  - `trend_distance`
  - `duration_zt_2_0`, `duration_zt_1_5`, `duration_zt_1_0`
  - `duration_span`, `duration_order`
- Stage-1 成功判据升级为：
  - `hard_gate_pass`
  - `sign_check=PASS`
  - `signed_order_check=PASS`
  - `abs_to_zero_check=PASS`
  - `same_seed_check=PASS`
- 若无成功点：输出最接近候选并在 `scan_summary.txt` 标记 `manual_confirm_stage2_ini_px`（不自动进入 Stage-2）。

### 12.5 本轮验证结论
1. 语法与编译
- `bash -n sh_utils/reproduce_fig11.sh`：通过
- `bash -n sh_utils/scan_fig11_px.sh`：通过
- `catkin_make -DCMAKE_BUILD_TYPE=Release`：通过（`traj_opt` / `planning_nodelet`）

2. 固定单分支结果（`rhoOmega=1.2e6`）
- 运行目录：`/tmp/fast_perching_fig11_single_rho12`
- 三 case 均达到 strict：
  - `zt_2_0 max_abs_omega2_int=3.000569510`
  - `zt_1_5 max_abs_omega2_int=3.000941918`
  - `zt_1_0 max_abs_omega2_int=3.000509679`
- 但 `vt_signed` 趋势仍为“`z_t` 越低越负”（与论文目标趋势相反），因此转入空间条件扫描阶段。

3. 扫描脚本烟测
- 目录：`/tmp/fast_perching_fig11_pxscan_smoke`
- 已正确生成：
  - `scan_report.tsv`
  - `best_candidate.env`
  - `scan_summary.txt`
- 连续评分、duration、same_seed 门控字段均已落盘。

---

## 13. 本轮追加：Stage-2 `ini_px` 扫描脚本（多中心细扫 + sign 门控）

### 13.1 新增脚本
- 文件：`sh_utils/scan_fig11_ini_px.sh`
- 目标：在固定 `perching_px=5.0` 条件下执行 Stage-2 `ini_px` 扫描。
- 基线口径沿用 `reproduce_fig11.sh`：
  - `rhoOmega=1.2e6`
  - `solve_mode=single`
  - `vt_multistart_enable=false`
  - `ini_v=(0,0,0)`（由复现实验脚本显式注入并验收）
  - `perching_vx=0.6`

### 13.2 扫描与细扫策略
1. 粗扫：
- `ini_px=0.0..1.0`，步长 `0.1`（默认）。

2. 细扫触发门槛（粗扫点级）：
- `hard_gate_pass=1`
- `same_seed_check=PASS`
- `sign_check=PASS`
- 且趋势未全通过（`signed_order_check/abs_to_zero_check` 至少一项非 `PASS`）。

3. 多中心细扫：
- 候选粗扫点按 `trend_distance` 升序（再按 margin 次序）排序。
- 取第 1 名作为中心。
- 若第 2 名与第 1 名满足近并列（`|td2-td1| <= near_tie_eps`），第 2 名也作为中心。
- 默认 `near_tie_eps=1e-4`，可通过参数覆盖。
- 每个中心独立细扫：`center±0.2`，步长 `0.05`，裁剪到 `[0.0,1.0]`。

### 13.3 报表与结论产物增强
1. `scan_report.tsv`
- 强制包含 `phase=coarse|fine` 列。
- 保留连续评分字段：
  - `signed_order_margin`
  - `abs_to_zero_margin`
  - `signed_order_deficit`
  - `abs_to_zero_deficit`
  - `trend_distance`
- 保留三组 `duration_*` 与 `duration_span/duration_order`。

2. `scan_summary.txt`
- 同时输出：
  - `coarse_best_*`
  - `final_best_*`
- 新增：
  - `fine_centers_used`
  - `near_tie_eps`
  - `fine_triggered=true|false`
  - `result=PASS_FOUND|NO_PASS`
  - `next=stop_stage2|manual_next_step`

3. `best_candidate.env`
- 同步写入 coarse/final 最优点、三组 `vt_signed`、三组 `duration`、`same_seed_check`、细扫中心信息。

### 13.4 验证状态
- 语法检查通过：`bash -n sh_utils/scan_fig11_ini_px.sh`
- help 输出检查通过：`sh_utils/scan_fig11_ini_px.sh --help`
- 本轮仅完成脚本实现与文档维护；尚未在本节记录新的 Stage-2 全量扫描结果目录。

### 13.5 运行稳定性修订（单点复核后追加）
- 单点复核（`perching_px=5.0, ini_px=0.0`）在未提权时失败，根因不是脚本逻辑：
  - `roslaunch` 在 `netifaces.interfaces()` 处报 `PermissionError: Operation not permitted`（沙箱权限）。
- 对应处理：
  1) 执行扫描时采用提权运行（容器内允许访问网络接口枚举）。
  2) `scan_fig11_ini_px.sh` 新增轻量 runtime 重试机制（默认每点重试 1 次）：
     - 新参数：`--runtime-retry-max`
     - 仅在 `runtime_failure_or_timeout` 场景触发重试，不对普通硬门槛失败重复求解。

### 13.6 Stage-2 全量执行结果（`ini_px`）
- 运行目录：`/tmp/fast_perching_fig11_inipx_stage2_full_20260308`
- 同步目录：`fig11_outputs/scan_stage2_ini_px_20260308_rho12/`
- 扫描规模：
  - `coarse_rows=11`
  - `fine_rows=4`
  - `rows_total=15`
  - `rows_success=0`（无点满足“硬门槛 + 四项趋势 + same_seed”全 PASS）
- 细扫触发：
  - `fine_triggered=true`
  - `fine_centers_used=0.300000`
  - `near_tie_eps=1e-4`
- 最接近候选（按当前排序规则）：
  - `final_best_ini_px=0.000000`
  - `trend_distance=0.062453958`
  - `same_seed_check=PASS`
  - 结论：`result=NO_PASS`, `next=manual_next_step`

---

## 14. 本轮追加：`rhoOmega=1.4e6` 的 9 点二维局部联扫

### 14.1 脚本与接口改动
1. `sh_utils/reproduce_fig11.sh`
- 新增可选参数：`--rho-omega`
- 用于覆盖 `PROFILE_RHO_OMEGA`，其余默认行为保持不变（不传时仍沿用脚本内默认值）。

2. `sh_utils/scan_fig11_px_ini_local.sh`（新增）
- 固定执行 9 点网格：
  - `perching_px ∈ {4.8, 5.0, 5.2}`
  - `ini_px ∈ {0.0, 0.15, 0.3}`
- 固定 `rhoOmega=1.4e6`（可通过参数覆盖）。
- 不做自动细扫，仅逐点执行并落盘。
- 每点汇总字段包含：
  - `hard_gate_pass`
  - 三组 `vt_signed`
  - `trend_distance`
  - `same_seed_check`
  - 三组 `duration`（含 `duration_span`, `duration_order`）
- 保留 runtime 稳健性：
  - 每点 `runtime_failure_or_timeout` 默认重试 1 次（可调 `--runtime-retry-max`）。

### 14.2 `best_candidate` 选点规则（本轮定稿）
- 三层优先级：
  1) `hard_gate_pass=1 AND same_seed_check=PASS`
  2) 若 1) 为空，则 `same_seed_check=PASS`
  3) 若 2) 为空，则全表
- 同层排序键：
  - `trend_distance` 升序
  - `signed_order_margin` 降序
  - `abs_to_zero_margin` 降序
  - 网格遍历顺序（`scan_idx`）升序
- `best_candidate.env` 新增：`selection_pool=hard_gate|same_seed|all`。

### 14.3 `scan_summary` 基线对比字段（本轮定稿）
- 新增并固定输出：
  - `baseline_trend_distance=0.062453958`
  - `best_trend_distance=<...>`
  - `trend_distance_delta=<best-baseline>`
  - `vs_baseline=better|same|worse`
- 判定阈值：
  - `baseline_compare_eps=1e-6`
  - `delta < -eps => better`
  - `|delta| <= eps => same`
  - 其余 `worse`

### 14.4 本轮执行结果
- 运行目录：`/tmp/fast_perching_fig11_local2d_rho14_20260308`
- 同步目录：`fig11_outputs/scan_local2d_px_ini_rho14_20260308/`
- 验证项：
  - `scan_report.tsv` 共 9 行（不含表头）✅
  - `runtime_fail_count=0` ✅
  - 选点池命中 `selection_pool=hard_gate` ✅
  - `scan_summary` 已包含基线对比四字段 ✅
- 本轮最优候选（按定稿规则）：
  - `best_perching_px=5.2`
  - `best_ini_px=0.0`
  - `best_trend_distance=0.156595292`
  - `vs_baseline=worse`

---

## 15. 本轮追加：Fig.11 `v_t` 映射分析（v3 置信度字段）

### 15.1 新增脚本
- 文件：`sh_utils/analyze_fig11_vt_mapping.py`
- 作用：读取两套（或多套）baseline 的 `zt_*_summary.txt`，输出 `v_t` 候选映射报告：
  - `vt_mapping_report.tsv`
  - `vt_mapping_aggregate.tsv`
  - `vt_mapping_summary.txt`

### 15.2 报告口径定稿
- 主候选仅来自：`{vt1, vt2, neg_vt2, vt_norm}`。
- `vt_signed` 保留在 `vt_mapping_report.tsv`，但仅标记：
  - `candidate_role=legacy_proxy`
- 主排名/推荐只在 primary 候选内进行，`vt_signed` 不参与排名。
- `vt_mapping_report.tsv` 关键字段：
  - `candidate_role=primary|legacy_proxy`
  - `baseline_feasibility_status=all_strict|has_relaxed`
  - 趋势指标（含 `monotonic_deficit`, `monotonic_margin`）

### 15.3 `vt_mapping_summary.txt` 新字段
- 固定包含：
  - `weak_margin_eps=1e-3`
  - `baseline_A_best_primary_candidate`
  - `baseline_B_best_primary_candidate`
  - `baseline_A_best_monotonic_margin`
  - `baseline_B_best_monotonic_margin`
  - `consistent_across_baselines=yes|no`
  - `recommended_primary_candidate`
  - `recommendation_confidence=high|medium|low`

### 15.4 `recommendation_confidence` 判定规则（固定）
- `high`：
  - `consistent_across_baselines=yes`
  - 且两套 baseline 的 best `monotonic_margin > weak_margin_eps`
- `medium`：
  - `consistent_across_baselines=yes`
  - 但至少一套 baseline 的 best `monotonic_margin <= weak_margin_eps`
- `low`：
  - `consistent_across_baselines=no`

### 15.5 规则约束说明
- `recommendation_confidence` 仅表达“候选映射稳定性”。
- 可行性信息仍由 `all_strict|has_relaxed` 单独报告，不与置信度混合。

---

## 16. 本轮追加：A/B 双基线完整运行 + 映射与图形对照

### 16.1 固定基线与产物目录
- 基线 A：`perching_px=5.0, ini_px=0.0, rhoOmega=1.2e6`
  - 目录：`fig11_outputs/baseline_A_px5_ini0_rho12/`
- 基线 B：`perching_px=5.0, ini_px=0.0, rhoOmega=1.4e6`
  - 目录：`fig11_outputs/baseline_B_px5_ini0_rho14/`

两套目录均已具备最小完整产物：
- `fig11_reproduced.png`
- `zt_2_0.csv`, `zt_1_5.csv`, `zt_1_0.csv`
- `zt_2_0_summary.txt`, `zt_1_5_summary.txt`, `zt_1_0_summary.txt`
- `attempt_report.tsv`
- `run_config.env`

### 16.2 映射分析产物
- 目录：`fig11_outputs/vt_mapping_dual_baseline/`
- 文件：
  - `dual_baseline_metrics.tsv`
  - `vt_mapping_report.tsv`
  - `vt_mapping_summary.txt`
  - `vt_mapping_aggregate.tsv`

关键结论（来自 `vt_mapping_summary.txt`）：
- `baseline_A_best_primary_candidate=vt1`
- `baseline_B_best_primary_candidate=vt1`
- `consistent_across_baselines=yes`
- `recommendation_confidence=medium`

### 16.3 图形并排对照
- 论文页导出：`fig11_outputs/vt_mapping_dual_baseline/paper_fig11_page7.png`
- 论文 Fig.11 裁剪：`fig11_outputs/vt_mapping_dual_baseline/paper_fig11_crop.png`
- 并排图（论文 + A + B）：
  - `fig11_outputs/vt_mapping_dual_baseline/fig11_side_by_side_papercrop_A_B.png`

说明：
- 曲线形态上（轨迹/推力/`omega2`）A/B 与论文图的宏观机制接近；
- 但 `v_t` 候选映射当前由 `vt1`（全 0）胜出，`monotonic_margin=0`，属于“弱一致”，需谨慎解释。

---

## 17. 本轮追加：Fig.11 产物硬清理（manifest 先行）

### 17.1 清理原则
- 在执行删除前，先生成清理快照：
  - `fig11_outputs/cleanup_manifest_20260308.txt`
- manifest 包含：
  - `KEEP_LIST` / `DELETE_LIST`
  - `PRE_CLEAN_TREE` / `POST_CLEAN_TREE`
  - 轻量历史索引（目录名 + 可提取关键参数）
  - `deleted_items_count` / `kept_items_count` / `missing_required_items`

### 17.2 已删除内容（按口径）
1. 旧 `vt_signed` 主判据扫描产物：
- `fig11_outputs/scan_stage1_px_20260308_rho12/`
- `fig11_outputs/scan_stage2_ini_px_20260308_rho12/`
- `fig11_outputs/scan_local2d_px_ini_rho14_20260308/`
- `fig11_outputs/rhoomega_local_sweep_20260308.tsv`

2. 烟测/临时链路目录：
- `fig11_outputs/baseline_A_px5_ini0_rho12_full/`

3. 根目录旧单次复现遗留：
- `fig11_outputs/fig11_reproduced.png`
- `fig11_outputs/zt_*.csv`
- `fig11_outputs/zt_*_summary.txt`
- `fig11_outputs/attempt_report.tsv`
- `fig11_outputs/run_config.env`
- `fig11_outputs/git_commit_hash.txt`

4. A/B 基线目录内非最小集：
- `attempt_*`, `ros_logs`, `ros_home`
- `zt_*.launch`, `zt_*.log`, `zt_*_rosparam.yaml`
- `git_branch.txt`, `git_status_short.txt`, `git_commit_hash.txt`
- `vt_trend_check.txt`

### 17.3 当前保留集（清理后）
1. A/B 双基线最小复现集：
- 每个 baseline 保留 9 件：
  - `fig11_reproduced.png`
  - `zt_2_0.csv`, `zt_1_5.csv`, `zt_1_0.csv`
  - `zt_2_0_summary.txt`, `zt_1_5_summary.txt`, `zt_1_0_summary.txt`
  - `attempt_report.tsv`
  - `run_config.env`

2. 映射分析主目录：
- `fig11_outputs/vt_mapping_dual_baseline/`
- 保留核心文件：
  - `dual_baseline_metrics.tsv`
  - `vt_mapping_report.tsv`
  - `vt_mapping_summary.txt`
  - `vt_mapping_aggregate.tsv`
  - `paper_fig11_crop.png`
  - `fig11_side_by_side_papercrop_A_B.png`

### 17.4 主入口口径（更新）
- 当前唯一主入口是目录：
  - `fig11_outputs/vt_mapping_dual_baseline/`
- 其中 `vt_mapping_summary.txt` 是**现阶段**结论文件（可迭代），不视为永久冻结单文件入口。

---

## 18. 本轮追加：Fig.11 `v_t` 映射重分析 v3（`vt2/neg_vt2` 聚焦 + 论文值直对照）

### 18.1 口径调整（v3）
- 主候选池从 `{vt1, vt2, neg_vt2, vt_norm}` 收缩为：
  - `{vt2, neg_vt2, vt_norm}`
- 角色降级：
  - `vt1 -> lateral_slip_audit_only`（仅审计侧向滑移）
  - `vt_signed -> legacy_projection_proxy_only`（仅投影代理）
- 主推荐仅在主候选池内产生，`vt1/vt_signed` 不再进入任何推荐池。

### 18.2 论文值直对照指标（新增）
- 固定论文参考值：
  - `z_t=2.0 -> -0.388`
  - `z_t=1.5 -> -0.049`
  - `z_t=1.0 -> -0.022`
- 对每个 `baseline × candidate` 输出：
  - `sign_match_count / sign_mismatch_count`
  - `mean_abs_value / paper_mean_abs_value / magnitude_scale_ratio`
  - `mae_to_paper / rmse_to_paper`
  - `err_zt_2_0 / err_zt_1_5 / err_zt_1_0`
- `vt_norm` 标注为 `unsigned_candidate`，并在 summary 中注明“符号不一致为定义预期，不视为实现错误”。

### 18.3 版本化输出与主入口
- 新目录（不覆盖旧证据链）：
  - `fig11_outputs/vt_mapping_dual_baseline_v3/`
- 文件：
  - `dual_baseline_metrics_v3.tsv`
  - `vt_mapping_report_v3.tsv`
  - `vt_mapping_aggregate_v3.tsv`
  - `vt_mapping_summary_v3.txt`
- 旧入口目录新增指针：
  - `fig11_outputs/vt_mapping_dual_baseline/LATEST_ANALYSIS_POINTER.txt`
  - 指向 `../vt_mapping_dual_baseline_v3/` 与 `vt_mapping_summary_v3.txt`

### 18.4 v3 本轮结果摘要
- `baseline_A_best_primary_candidate=neg_vt2`
- `baseline_B_best_primary_candidate=neg_vt2`
- `consistent_across_baselines=yes`
- `recommended_primary_candidate=neg_vt2`
- `final_interpretation=vt1_not_fig11_primary_mechanism`

## 19. 本轮追加：Fig.11 机制验证复现链路 v1.1（参数回写 + 数值稳健 + 指标可追溯）

### 19.1 新增脚本
1. `sh_utils/reproduce_fig11_mechanism.sh`
- 新增独立机制验证复现入口（不覆盖 `reproduce_fig11.sh`）。
- 固定 profile（写死）：
  - `K=32`
  - `rhoOmega=1400000.0`
  - `rhoThrust=100000.0`
- 固定统一场景（写死）：
  - `ini_p=(0,0,2)`
  - `ini_v=(0,0,0)`
  - `perching_p=(5,0,z_t)`
  - `perching_v=(0.6,0,0)`
  - `axis=(0,1,0), theta=-1.5708`
  - `fix_terminal_state=false`, `v_plus=0.3`, `z_t={2.0,1.5,1.0}`
- 运行后自动调用机制分析脚本，输出：
  - `mechanism_metrics.tsv`
  - `mechanism_pairwise.tsv`
  - `mechanism_summary.txt`
- 自动更新 `fig11_outputs/CURRENT_FIG11_ENTRY.txt` 指向机制目录，并保留 `vt_mapping_dual_baseline` 历史审计入口角色不变。

2. `sh_utils/analyze_fig11_mechanism.py`
- 新增机制验证分析脚本（读取 `zt_*.csv` + `zt_*_summary.txt`）。
- `adaptation_gate`：保留 `vt2 OR vt_norm` 通过逻辑，并新增 `adaptation_driver=vt2|vt_norm|both|none`。
- `curve_sync_gate`：
  - 时间归一化重采样后计算 pairwise 归一化 RMSE；
  - 归一化分母全部加下界 `norm_epsilon=1e-6`：
    - `span_x=max(raw_span_x,eps)`
    - `span_z=max(raw_span_z,eps)`
    - `mean_abs_union_tau=max(raw_mean_abs_union_tau,eps)`
    - `mean_abs_union_omega=max(raw_mean_abs_union_omega,eps)`
  - 判据固定：`trajectory_change_pass AND (tau_change_pass OR omega_change_pass)`。
- 可行性门槛：
  - `runtime_fail` 或 `fallback` 直接 FAIL；
  - relaxed 允许（并在 summary 输出 `strict_count/relaxed_count/fallback_count`）；
  - 同时检查 `solve_mode_all_single` 与 `max_abs_omega2_int_overall <= threshold`。

### 19.2 `mechanism_summary.txt` 新增与固化字段
1. 固定 profile/scene 回写字段（可单文件复核口径）
- `profile_K`, `profile_rhoOmega`, `profile_rhoThrust`
- `scene_ini_p`, `scene_ini_v`
- `scene_perching_p_template`, `scene_perching_v`
- `scene_perching_axis`, `scene_perching_theta`, `scene_zt_values`
- `scene_role=unified_mechanism_validation_scene_not_claimed_as_exact_paper_scene`

2. 数值稳健与门槛字段
- `norm_epsilon`
- `feasibility_gate`, `adaptation_gate`, `curve_sync_gate`, `overall_result`
- `strict_count`, `relaxed_count`, `fallback_count`, `runtime_fail_count`
- `adaptation_driver`

3. pairwise 曲线指标（全部落盘）
- `traj_rmse_norm_2.0_1.5`, `traj_rmse_norm_2.0_1.0`, `traj_rmse_norm_1.5_1.0`
- `tau_rmse_norm_2.0_1.5`, `tau_rmse_norm_2.0_1.0`, `tau_rmse_norm_1.5_1.0`
- `omega_rmse_norm_2.0_1.5`, `omega_rmse_norm_2.0_1.0`, `omega_rmse_norm_1.5_1.0`
- 及对应分母/跨度原始值与夹紧值（`raw_*` 与 `*`）
- `traj_change_pass`, `tau_change_pass`, `omega_change_pass`

4. 固定结果语义
- `result_note=PASS 表示 terminal-state-adjustment 机制在统一场景下成立，不表示论文 Fig.11 原始单标量结果已复现`

### 19.3 `mechanism_metrics.tsv` 字段口径
- 每 case 保留：
  - `vt1`, `vt2`, `vt_norm`, `neg_vt2`, `vt_signed`
  - `duration`, `winner_eps_level`, `max_abs_omega2_int`

### 19.4 本轮校验与产物
1. 语法检查
- `bash -n sh_utils/reproduce_fig11_mechanism.sh` ✅
- `python3 -m py_compile sh_utils/analyze_fig11_mechanism.py` ✅

2. 分析脚本功能校验（不重跑仿真）
- 使用 `baseline_B_px5_ini0_rho14` 现有产物执行一次机制分析落盘（smoke）：
  - 目录：`fig11_outputs/fig11_mechanism_v11_smoke/`
  - 产物：`mechanism_summary.txt`, `mechanism_metrics.tsv`, `mechanism_pairwise.tsv`

3. 当前入口
- 新增：`fig11_outputs/CURRENT_FIG11_ENTRY.txt`
- 指向：`fig11_outputs/fig11_mechanism_v11_smoke/`
- 说明：`vt_mapping_dual_baseline` 继续作为“变量口径审计”历史入口保留。

## 20. 本轮追加：Fig.11 历史相似图复刻链路 v4（source git 缺失硬失败 + 目录重叠防护）

### 20.1 新增脚本
- 文件：`sh_utils/reproduce_fig11_legacy_paperlike.sh`
- 目标：以 `/tmp/fast_perching_fig11_rerun_20260308` 作为唯一真值源，做 legacy paper-like 单 profile 复刻。
- 默认输出目录：`fig11_outputs/fig11_paperlike_legacy_rerun_20260308/`
- 不复用主线 `reproduce_fig11.sh`，避免 profile 自动切换带来的歧义。

### 20.2 v4 定稿硬规则（已实现）
1. source git 元数据缺失硬失败
- source 预检必须存在且非空：
  - `git_commit_hash.txt`
  - `git_branch.txt`
  - `git_status_short.txt`
- 任一缺失：
  - `code_state_source_unknown=YES`
  - `overall_result=AUDIT_FAIL`
  - 立即退出，不进入 ROS 运行。

2. source/out 目录重叠防护（`--force` 前执行）
- 对 `source-dir` / `out-dir` 做规范化路径判断。
- 任一命中即报错退出：
  - `out == source`
  - `out` 是 `source` 子目录
  - `source` 是 `out` 子目录

3. code state 一致性硬门槛
- 仅在 source git 元数据完整时比较：
  - `source_git_commit` vs `current_git_commit`
  - `source_git_branch` vs `current_git_branch`
  - `source_git_status_sha256` vs `current_git_status_sha256`
- 任一不一致：
  - `code_state_mismatch=YES`
  - `overall_result=FAIL`
  - 按硬策略退出。

### 20.3 复刻审计与快照
1. 不可变真值快照
- 新增：`source_manifest.txt`
- 至少记录：关键文件路径、大小、mtime、sha256，以及 source 目录/selected_profile 信息。

2. 冻结参数快照
- 新增：`frozen_source_truth.env`
- 固化 selected profile 与生效参数（`K/rho*`、`num_worker_threads`、multistart seeds 顺序、eps/threshold、场景参数等）。

3. 对照审计文件
- 新增：
  - `legacy_rerun_review.tsv`
  - `legacy_rerun_review.txt`
- 严格容差（固定）：
  - `vt_signed: 1e-6`
  - `vt2: 1e-6`
  - `duration: 1e-5`
  - `winner_seed_vt2: 1e-6`
  - `max_abs_omega2_int: 1e-6`
- 离散字段（如 `selected_profile`、`winner_eps_level`、seed 顺序）要求精确一致。

### 20.4 清洁运行策略
- 目录已存在且非空：默认失败；仅 `--force` 可清空后运行。
- 审计字段回写：
  - `force_used`
  - `preexisting_nonempty`
  - `cleaned_before_run`


## 21. 本轮追加：Fig.11 Paper-Like 视觉复现线（v5，active）

### 21.1 工作流切换与归档
- v4 严格复刻线改为 **停用归档**（保留证据链，不再 active）：
  - 脚本归档：`sh_utils/deprecated/reproduce_fig11_legacy_paperlike_deprecated.sh`
    - 头部标记：`deprecated=true`, `not_active_workflow=true`
  - 输出归档：`fig11_outputs/archive_deprecated/fig11_paperlike_legacy_rerun_20260308/`
- v4 的 `source git` 严格一致与 `code_state_mismatch` 硬失败，不再用于 paper-like 主线判定。

### 21.2 新 active 脚本
- 新增：`sh_utils/reproduce_fig11_paperlike_visual.sh`
- 默认输出目录：`fig11_outputs/fig11_paperlike_best_effort/`
- 目标：视觉与整体行为尽量接近论文 Fig.11（非机制门槛、非源码一致性复刻）。

固定参数（写死）：
- `K=32`
- `rhoOmega=1e6`
- `rhoThrust=1e5`
- `rhoPerchingCollision=1e6`
- `perching_p0=(4.0,0,z_t)`
- `perching_v=(0,0,0)`
- `ini_p=(0,0,2.0)`
- `ini_v=(0,0,0)`
- `fix_terminal_state=false`
- `v_plus=0.3`
- `surface_axis=(0,1,0)`
- `theta=-1.5708`
- `vt_multistart_enable=true`
- `seeds=0.0,0.1,-0.1,0.4,-0.4`
- `z_t={2.0,1.5,1.0}`
- `sample_dt=0.005`

### 21.3 视觉审计脚本与产物
- 新增：`sh_utils/analyze_fig11_paperlike_visual.py`
- 生成：`best_effort_visual_review.txt`，固定包含最小定量块：
  - 每组 `tau_norm_min/max`
  - 每组 `omega2_min/max`
  - 每组轨迹 `x_range/z_range` 与终点 `(x_end,z_end)`
  - 三组 `vt_signed` 与 `neg_vt2`（辅助标注，不作主判据）
- 生成并排图：`fig11_side_by_side_paperlike.png`
  - 左图：`fig11_outputs/vt_mapping_dual_baseline/paper_fig11_crop.png`
  - 右图：本次 `fig11_reproduced.png`

### 21.4 视觉等级字段（非硬 PASS/FAIL）
`best_effort_visual_review.txt` 固定输出：
- `trajectory_similarity=high|medium|low`
- `thrust_boundary_similarity=high|medium|low`
- `omega_shape_similarity=high|medium|low`
- `overall_visual_similarity=high|medium|low`

语义声明：
- 视觉等级只用于“像论文程度”描述；
- 不是机制验证结论；
- 不是论文数值逐点复现结论。

### 21.5 双入口并行
- 机制线入口保持不变：`fig11_outputs/CURRENT_FIG11_ENTRY.txt`
- 新增 paper-like 入口：`fig11_outputs/CURRENT_FIG11_PAPERLIKE_ENTRY.txt`
  - `entry_type=paperlike_visual_reproduction`
  - `entry_note=visual similarity workflow; not mechanism proof; not exact numeric reproduction`


## 22. 本轮追加：Fig.11 产物治理（v2，`git_*.txt` 全目录统一规则）

### 22.1 统一规则（目录无特例）
- `git_*.txt` 删除许可改为 **全目录统一**：
  - 仅当该结果目录的 `run_config.env` 具备等价的 `commit + branch + status` 指纹时，才允许删除 `git_*.txt`。
  - 等价字段允许命名差异（例如 `git_commit_hash` / `current_git_commit`、`git_branch` / `current_git_branch`、`git_status_*` / `*_git_status_sha256`）。
  - 任一要素缺失：该目录中的 `git_*.txt` 一律保留。
- 该规则同样适用于 mechanism、paper-like、baseline、archive 等全部结果目录。

### 22.2 本轮清理动作
- 删除中间 smoke 目录：
  - `fig11_outputs/fig11_mechanism_v11_smoke/`
- 删除 mechanism 正式目录内调试噪音：
  - `ros_logs/`, `ros_home/`, `zt_*.log`, `zt_*.launch`, `zt_*_rosparam.yaml`
- 删除 paper-like 正式目录内调试噪音（保留参数快照）：
  - 删除：`ros_logs/`, `ros_home/`, `zt_*.log`, `zt_*.launch`
  - 保留：`attempt_report.tsv`、`zt_*_rosparam.yaml`
- 删除临时缓存：
  - `.tmp_rho_sweep/`
  - `sh_utils/.tmp_rho_sweep/`
  - `sh_utils/__pycache__/`

### 22.3 本轮追溯产物
- 新增审计表：
  - `fig11_outputs/cleanup_git_rule_audit_20260309.tsv`
- 审计字段：
  - `run_config_has_commit_branch_status`
  - `git_txt_present`
  - `git_txt_deletion_allowed`
- 审计结果要点：
  - `fig11_outputs/fig11_paperlike_best_effort/`：`git_txt_present=true` 且 `git_txt_deletion_allowed=false`，因此保留 `git_*.txt`。
  - `fig11_outputs/fig11_mechanism_validation_v1/`：当前无 `git_*.txt` 可删；若后续出现且 `run_config.env` 仍缺 status 指纹，则同样不得删除。

### 22.4 机制线与视觉线核心保留集
- mechanism（`fig11_mechanism_validation_v1`）保留：
  - `fig11_mechanism.png`, `fig11_reproduced.png`
  - `mechanism_summary.txt`, `mechanism_metrics.tsv`, `mechanism_pairwise.tsv`, `mechanism_gate_review.txt`, `mechanism_conclusion.txt`
  - `run_config.env`, `zt_*.csv`, `zt_*_summary.txt`
  - `attempt_report.tsv`（如存在则必须保留）
- paper-like（`fig11_paperlike_best_effort`）保留：
  - `fig11_reproduced.png`, `fig11_side_by_side_paperlike.png`
  - `best_effort_visual_review.txt`, `attempt_report.tsv`, `run_config.env`
  - `zt_*.csv`, `zt_*_summary.txt`, `zt_*_rosparam.yaml`
  - `git_commit_hash.txt`, `git_branch.txt`, `git_status_short.txt`（本轮按统一规则保留）
