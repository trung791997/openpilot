# V-ASM fine-tuning on the owner's routes

github.com/prabhaavp/vasm-op published its training recipe on 2026-10-16 (`SELF_TRAIN.md`, `prepare_dataset.py`,
`train_pipeline.py`, still no `.pt` checkpoint). Read statically: its crop (mask, letterbox to 352) matches the daemon's
`_run_inference`; `train_pipeline.py` does not run as published (`DEVICE`, `BATCH_SIZE`, `NUM_WORKERS`, `TRAIN_STRIDE`,
`INFERENCE_BATCH_SIZE`, `Tee` undefined); and its alarm sweep uses `alpha = 0.05 / smooth_sec` on frames sampled every
4, 6 or 30 video frames, so its `smooth_sec` is not the daemon's (`alpha = dt / smooth_sec`). Use our val_sweep.py. The shipped model's ONNX metadata says it is an Ultralytics YOLO26n-cls classifier: input `images`
[1,3,352,352] RGB/255, output `output0` [1,3] softmax over {0 no car, 1 car, 2 distant or rear}.

1. `convert.py`: ONNX to PyTorch (onnx2torch). It rewrites opset-18 `Split(num_outputs)` to explicit sizes, sets
   opset 17, and frees the batch dim baked into the `Reshape` constants. It then compares the converted model with
   cv2.dnn on the daemon's exact input blobs. The output is already softmax, so train on log-probs.
   Result on 96 crops from 00000321--67540f4492 s42: max abs diff 1.97e-06, class and ≥0.8 agreement 100 %.

Run it the same way as `tools/vasm_replay` (scratch dir mounted at /work), with
`pip install av opencv-python-headless onnx onnx2torch torch --extra-index-url https://download.pytorch.org/whl/cpu`.

2. `extract_crops.py`: crops for labelling and the benchmark. It samples both sides at 4 Hz and saves the daemon's
   exact model input (its own `_run_inference` blob) as a 352×352 PNG. Every PNG is checked to reload bit-identical to
   that blob. `crops/manifest.csv` holds the shipped model's three probabilities, radar on that car side, the real
   D-118 pass-hold state, speed, blinker, crop brightness, a proposed label, and why the row needs review.
   Run `tools/vasm_replay/rlog_extract.py` first for the `ex_*.json` files. It is resumable per segment.
   **Test-split crops must all be labelled by hand.** Model proposals there would bias the benchmark toward the shipped
   model. Proposals are only a shortcut for training crops.
   Result on 00000321 s30/31/41/42 + 00000323 s3/4: 2,582 crops (164 MB). `p_car` matches the earlier
   `conf_replay` run within 5e-05. 1,272 were flagged for review: 1,091 uncertain (<0.98), 174 where the pass hold says
   occupied but the camera says clear, 14 where radar is within 8 m but the camera says clear.

3. `labeler.py`: a local labelling page, stdlib only, on 127.0.0.1. Start it with
   `python3 tools/vasm_finetune/labeler.py`, then open `http://127.0.0.1:8765/?split=test`.
   - **Keys:** 0 no car, 1 adjacent lane, 2 two or more lanes over, t truck part, x unsure, arrow keys to move.
   - **Class rules (owner, 2026-10-13):** 1 = the vehicle covers the lane line and fills most of the crop, or any part of
     an adjacent-lane vehicle is in view (a bit of its front, a truck hitch). 2 = two or more lanes over, judged by
     distance: the vehicle sits beyond the lane line; with no lane lines, small and far = 2, big and close = 1. Truck
     trailers, containers and the rest of the cab are the vehicle, never 0. `t` saves 1 with `"truck": true`: the
     current model handles these crops badly, so training oversamples them.
   - **Context:** each crop is shown with the same side 0.25 s and 0.5 s before and after it, plus radar and the pass-hold state.
   - **Blind test labelling:** test crops are labelled blind (the model's probabilities and proposal are hidden). `?split=train`
     shows only the flagged crops, with the model's view.
   - **Saving:** labels go to `<crops>/labels.json` after every key press. Reopening resumes at the first unlabelled crop.
   - **Interval mode** (`/interval`, the vasm-op workflow): one segment side as a filmstrip. A key sets the pen class
     and paints each crop the cursor moves onto (space plays at 4 Hz real time, `[` `]` speed, Esc lifts the pen);
     Enter saves the side in one write. Owner labels are locked and never repainted; agent labels can be. Saved crops
     carry `"how": "interval"`, and crops next to a class change also `"edge": true`; `?split=edges` lists those
     (blind) until each is re-keyed one by one. Headless Chromium on a 30-crop scratch copy, 2026-10-16: painted
     runs, truck tag, repaint, edge flags and the blind edges page as expected; owner labels byte-identical after.
   - **Verified with headless Chromium:** five rapid key presses land five labels, on five crops, in order. The first version lost
     fast presses to a save race. A path-traversal request returns 404.

4. `benchmark.py`: the frozen benchmark, scored on the hand-labelled test crops only.
   - **Model:** it runs any ONNX model through cv2.dnn on the saved PNGs, the same runtime the daemon uses. For the shipped
     model it first checks the results reproduce the manifest.
   - **Frame level:** precision, recall, F0.5, false positives and false negatives at the on threshold, plus the 3-class
     confusion. Subsets: night, pass hold, radar within 8 m, and each route. A threshold sweep is included.
   - **Event level:** the daemon's 1 s alternating sampling with the 0.3 s follow-up and on/off hysteresis. It reports
     missed car events (at least 0.5 s long), onset latency p50/p90, and false-occupied s/h.
   - **Compare:** `--compare base.json cand.json` refuses runs made on different test sets (both hash files are checked).
     The gate is: missed ≤, false-occupied ≤, and latency p50 ≤ base + 0.125 s. Comparing runs at different `--on/--off`
     is how a threshold-only change is judged.
   - **Self-test:** the model's own proposals were used as stand-in labels, which proves the code only, not the model.
     The manifest was reproduced to 5e-05, the confusion came out diagonal, and the gate passes against itself.

5. `train.py`: fine-tunes the converted model on labelled train crops only (test is never read) and exports ONNX.
   - **Sampling:** classes are balanced. A crop tagged `"truck": true` in labels.json, or listed in `<crops>/truck_tags.json`,
     is drawn `--truck-weight` (default 4) times as often, because the shipped model handles truck parts badly.
   - **Validation:** whole (route, segment) groups, 10 % by hash. The best epoch is chosen by class-1 F0.5 at `--on`.
   - **Smoke test (2026-10-13, docker oprad-test:py312):** 2 epochs on 64 crops ran. The exported ONNX loads in cv2.dnn and
     matches the torch model to 4e-10. This proves the pipeline only, not any accuracy.
6. `pick.py`: picks one (soup, on, off) from `select_queue.py`'s qualifying candidates. It uses the held-out road route
   (`cropsroad`, route 329) and never the test split. The rule is the owner's from 2026-10-14, and `--rule highest` is
   run12's. It was moved here from the scratch `diag/run11_pick.py` with the logic unchanged.
7. `pipeline.py`: the run12 recipe as one resumable command, like upstream's `train_pipeline.py`:
   `check, train (3 seeds), soup, select, pick, baseline, test, gate, package`. Run it from the scratch root:
   `PYTHONPATH=/tmp/oppath venv/bin/python <repo>/tools/vasm_finetune/pipeline.py --crops crops12 --run run13`.
   - **Resuming:** every stage is skipped when its output exists, so after a crash you re-run the same command.
   - **Data binding:** a run is tied to the sha256 of manifest, labels, hard negatives and queue. If any of them change,
     the run refuses to continue and asks for a new `--run`.
   - **Test once:** an existing `test.json` is never overwritten.
   - **Gate:** only a passing `benchmark.py --compare` produces `deliver/vasm.onnx` and `deploy.json`.
   - **Inspect:** `--dry-run` prints the commands; `--status` prints the stages and the labelling backlog.
   - **What stays manual:** crop extraction, labelling and deploying to the car.
8. `autolabel.py` + `autolabel_check.py`: automatic labels for unlabelled train crops, kept out of training until an
   independent check passes. v1 (one run12 soup, averaged) failed it on route 329: class 1 precision 0.70, Wilson 95%
   lower 0.59, all 23 errors owner class 2, so v1 labels are not used. v2 adds `--agree` (per-model validation
   thresholds, label only where differently trained models agree) and `--classes`. `autolabel_check.py` first
   recomputes the report's validation result from the labeller's crops and refuses if it differs (so the check
   judges exactly what was labelled), then scores the same decision on an owner-labelled route outside the
   labeller's crops; every emitted class needs a Wilson lower bound >= 0.98. `pipeline.py` refuses a recipe with
   `auto-labels` unless `<labels>.check.json` passed for that file's sha256, and binds both hashes to the run.
   Verified 2026-10-16: the check reproduces the scratch alcheck numbers for v1 exactly and fails it; v2 without
   `--agree` reproduces v1's 147,672 labels byte-identically; `test_autolabel.py` 10 passed (run with
   `--noconftest -c <empty ini>`, the repo conftest needs the aarch64 binaries).
