# Handoff — Seesaw report work (state-space + validation)

Written for a fresh agent picking up the report. Read this top to bottom once, then
skim the three memory files referenced at the end. Everything here is verified as of
branch `lab-tracking-analysis` @ commit `8ea9e82`.

---

## 0. Mission

**Correction 2026-06-07:** the old LQR hardware-loader / `controller_lqr.mat` notes in this
handoff are stale and should not be used as deployed-design evidence. For LQR/LQI weights,
state ordering, and synthesis, use `scripts/control/lqr_design.m`: state order
`[x_c, theta, dot{x_c}, dot{theta}, xi]`, Bryson limits `0.05 m`, `1 deg`, `0.1 m/s`,
`4 deg*s`, `3 V`, and continuous/discrete `lqr(A5,B5,Q5,R5)` / `dlqr(A5d,B5d,Q5,R5)`.

Finish the lab report at `docs/reports/synced_report/Thesis.tex`. The report documents a
Quanser IP02 + Seesaw-E project: model the cart, model the coupled cart-seesaw, then
design/validate four controllers (cascaded frequency-domain PID, state-space pole
placement, LQR, and estimators) plus a lift-up maneuver.

Two people contributed to the .tex:
- **Teammate** wrote the *design derivations* (intro, setups, modeling, cart PID design,
  angle cascade, PP analytical method, LQR/Kalman theory) — on Overleaf.
- **Us (Claude + user Karim)** wrote the *hardware validation* and reconciled the
  *deployed* controller numbers/figures. All our work is local git only — NOT yet on Overleaf.

User's standing guidance:
- **State-space (PP/LQR/observers) = full latitude to overhaul.**
- **PID / frequency-domain design = light touch** (teammate's; user not confident reviewing it).
- **All hardware validation = ours, edit freely.**
- Be rigorous and honest; the user catches over-claims (see §5 the Kalman episode).

---

## 1. Repo / branch state

- Branch `lab-tracking-analysis`, 5+ commits ahead of `main`, **never pushed**, never merged.
- The report lives in `docs/reports/synced_report/` (Thesis.tex + Images/ + bibliography.bib).
  This folder is the teammate's Overleaf export; **the report PNGs are gitignored** — when you
  add a figure the report needs, `git add -f` it (precedent set for the result figures).
- LaTeX build artifacts (.aux/.log/.pdf/...) are untracked noise — never commit them.
- Our report commits: `a4bf81f` (reconcile gains), `92ff23f` (adopt teammate base + port),
  `3c2ff95` (PP overhaul), `f9aa6f2` (free-run tests + first Kalman claim), `8ea9e82`
  (walk back Kalman to "inconclusive").

---

## 2. CRITICAL gotchas (these will bite you)

1. **Two different state orderings AND two different plants.** Verified by eigenvector +
   integrator-row analysis:
   - Pole placement / observer (`tuned_seesaw.mat` A_sw, cart damping −14.35): state =
     `[x_c, θ, ẋ_c, θ̇]`.
   - LQR (`controller_lqr.mat` A_sw, cart damping −36.55, different B): state =
     `[x_c, ẋ_c, θ, θ̇, ∫θ]`.
   - **Never tabulate gain vectors as bare ordered rows** — always label per physical variable.
2. **Deployed ≠ simulated numbers.** The teammate's PP/cart derivations use *simulated /
   pre-deployment* values. The numbers that actually ran are in `data/*.mat` and were verified
   to reproduce from the live scripts. User's policy: **present sim and deployed side by side**
   for the PID (light touch); for PP we overhauled fully onto deployed.
3. **`kf.mat` lesson — don't validate an estimator by position correlation.** A directly-measured
   state's estimate correlating ~1.0 with its measurement is trivially true for ANY estimator and
   proves nothing. Use velocity-channel internal consistency (is the rate channel = d/dt of the
   position estimate?) and innovation whiteness. See §5.
4. **Mojibake in the teammate's .tex.** Pasted plan arrows became corrupt glyphs (`U+F0E0`,
   and earlier `U+00EF U+0192 U+00A0`). The Edit tool can't match these. To replace text
   spanning them, use PowerShell regex with `RegexOptions::Singleline`, read/write as UTF-8
   no-BOM. Smart quotes (`" " '`) remain — cosmetic, compile-safe under utf8 inputenc.
5. **MATLAB `ss`/`bandwidth`/`margin` got shadowed** by a lingering workspace variable `ss`
   earlier — `clear all` fixes it; this also breaks `seesaw_params` (fails at an `ss()` call).
6. **MATLAB toolboxes:** Control + System ID present; **no Signal Processing / Statistics**
   toolbox. Use base-MATLAB DSP (the helpers in `analyze_tracking.m`) and `corrcoef` not `corr`.
7. **MATLAB MCP server occasionally disconnects** — re-`ToolSearch` `mcp__matlab__evaluate_matlab_code`.
8. **`pole_placement_design.m` has an IC state-ordering bug** (spawned as a separate task):
   its IC test vector perturbs cart velocity not tilt. Gains are unaffected; only its
   IC-response figures were wrong — we regenerated them correctly.

---

## 3. Source-of-truth numbers (deployed)

State orderings as in §2.
- **Cart inner PID:** Kp 301.4, Ki 335.9, Kd 27.14, N 100 (teammate's *simulated* is 149.3/217.8/19.6).
- **Outer PD:** Kp −0.863, Kd −0.0786, N 25.  **Outer PID:** Kp −0.581, Ki −0.134, Kd −0.185, N 25.
- **PP:** `K₄=[258.3, −196.8, 24.9, −69.7]`; poles `−5.5±4.125j, −8.25, −6.6`; integral pole −0.5 →
  `K₅=[276.5, −231.7, 26.9, −77.1, −58.9]`. Loop PM 58.4°, ω_gc 12.9 rad/s. OL poles `{+2.74, −1.48±1.30j, −14.13}`.
- **Luenberger observer:** k_obs=5, poles `{−27.4, −27.6, −41.2, −33.1}`, L saved in `data/observer.mat`.
- **LQR (LQI):** `K=[191.9, 14.5, −275.5, −92.8, −63.2]` (on `[x_c, ẋ_c, θ, θ̇, ∫θ]`),
  Q=diag([1000,0,5000,0,2000]), R=0.5; margins PM 63.7°, ω_gc 10.8 rad/s.
- **Measured tracking (common protocol):** step DC gain ~1.0; ss error PID 0.06° / PP 0.01° / LQR 0.02°;
  swept-sine RMS 0.52 / 0.33 / 0.42°; free-run rocking (dedicated) PD 3.5° / PID 0.69° / PP 0.40° / LQR 0.39°;
  V RMS 1.56 / 0.56 / 0.58; cart p-p 11.6 / 7.5 / 5.0 cm. Staircase: clean to 10°, 1.5–2° return hysteresis.
- **Bandwidth is unreliable** (0.2° swept-sine sits below the 0.3–0.7° limit cycle): report it only as a
  lower bound; the resonant peak / damping ranking are the trustworthy frequency-domain results.

---

## 4. Data / file map

- Deployed gains: `data/{pole_placement,observer,controller_lqr,controller_inner_pid,controller_outer_pid,controller_outer_pd,tuned_seesaw,tuned_cart}.mat`.
- Design scripts: `scripts/control/{pole_placement_design,lqr_design}.m`, `scripts/config/seesaw_params.m`.
- Tracking analysis: `validation/scripts/analyze_tracking.m` + `load_lab_capture.m` →
  `validation/data/tracking_results.mat`; staircase `validation/scripts/analyze_staircase.m`.
- Lab captures (root): 5 tracking runs `PID_TEST_FULL/PPDDTEST/LQRDDTEST/PID/PID_STEP.mat`;
  staircase `LQRDD_STAIRCASE.mat`; named tests `pd_stable/pid_stable/pp_stable/lqr_stable_dist/kf.mat`;
  ~30 undecoded `data_26-May-2026_*.mat`. Channel layouts decoded in the memory file.
- Old superseded report: `docs/reports/Thesis.tex` (mined its Luenberger derivation; otherwise stale).

## 7. Working conventions

- Verify LaTeX after edits: count `\begin{}`/`\end{}` for equation/figure/bmatrix/table/itemize/align,
  and grep for dangling `\ref`/`\label` after removing blocks.
- Reproduce/verify numbers in MATLAB from the `data/*.mat` and scripts before writing them — don't
  trust either report draft's numbers.
- Commit per coherent change; force-add report figures; end commit messages with the Co-Authored-By trailer.

## 8. Memory files (auto-loaded; richer detail)
- `seesaw_deployed_gains` — deployed gains + the state-ordering trap.
- `lab_data_may26_formats` — capture channel layouts, named-test inventory, the KF-inconclusive correction.
- `project_seesaw_modeling` — overall project structure.
