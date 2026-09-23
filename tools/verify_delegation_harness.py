#!/usr/bin/env python3
"""
Standalone Verification Script for agy-flash, agy-pro, and agy-worktree Delegation Harness.
Validates Git path resolution, multi-worktree handling, gate evaluation, cryptographic receipts,
Safeguard 2 / Opus verification, mutation detection, allowlist boundary enforcement, and clean teardown.
"""

import os
import sys
import json
import shutil
import tempfile
import subprocess
from pathlib import Path

AGY_FLASH_BIN = "/Users/peternguyen/.local/bin/agy-flash"
AGY_PRO_BIN = "/Users/peternguyen/.local/bin/agy-pro"
AGY_WORKTREE_BIN = "/Users/peternguyen/.local/bin/agy-worktree"
CLAUDE_AUDIT_BIN = "/Users/peternguyen/.local/bin/claude-audit"
OPUS_AUDIT_BIN = "/Users/peternguyen/.local/bin/opus-audit"

def log(msg):
    print(f"[TEST] {msg}")

def run_cmd(cmd, cwd=None, env=None):
    res = subprocess.run(cmd, cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return res.returncode, res.stdout.strip(), res.stderr.strip()

def test_help_invocations():
    log("Testing --help invocations from main repo and nested worktrees...")
    main_repo = "/Users/peternguyen/openpilot-radar"
    claude_worktree = "/Users/peternguyen/openpilot-radar/.claude/worktrees/repo-state-assessment-6969b6"

    # 1. agy-flash --help from main repo
    rc, out, err = run_cmd([AGY_FLASH_BIN, "--help"], cwd=main_repo)
    assert rc == 0, f"agy-flash --help in main repo failed ({rc}): {err}"
    assert "Hardened agy-flash" in out

    # 2. agy-pro --help from main repo
    rc, out, err = run_cmd([AGY_PRO_BIN, "--help"], cwd=main_repo)
    assert rc == 0, f"agy-pro --help in main repo failed ({rc}): {err}"
    assert "Hardened agy-flash" in out

    # 3. agy-worktree --help
    rc, out, err = run_cmd([AGY_WORKTREE_BIN, "--help"], cwd=main_repo)
    assert rc == 0, f"agy-worktree --help failed ({rc}): {err}"

    # 4. claude-audit and opus-audit --help from main repo
    rc, out, err = run_cmd([CLAUDE_AUDIT_BIN, "--help"], cwd=main_repo)
    assert rc == 0, f"claude-audit --help failed ({rc}): {err}"
    rc, out, err = run_cmd([OPUS_AUDIT_BIN, "--help"], cwd=main_repo)
    assert rc == 0, f"opus-audit --help failed ({rc}): {err}"

    # 5. Help from nested Claude worktree (guarantee test executes)
    if Path(claude_worktree).exists():
        wt_target = claude_worktree
        created_temp = False
    else:
        # Create a transient nested worktree to ensure test coverage
        wt_target = str(Path(main_repo) / ".claude" / "worktrees" / "tmp-help-test")
        run_cmd(["git", "worktree", "add", "-B", "claude/tmp-help", wt_target, "HEAD"], cwd=main_repo)
        created_temp = True

    try:
        rc, out, err = run_cmd([AGY_FLASH_BIN, "--help"], cwd=wt_target)
        assert rc == 0, f"agy-flash --help in claude worktree failed ({rc}): {err}"
        rc, out, err = run_cmd([AGY_PRO_BIN, "--help"], cwd=wt_target)
        assert rc == 0, f"agy-pro --help in claude worktree failed ({rc}): {err}"
        rc, out, err = run_cmd([AGY_WORKTREE_BIN, "--help"], cwd=wt_target)
        assert rc == 0, f"agy-worktree --help in claude worktree failed ({rc}): {err}"
        rc, out, err = run_cmd([CLAUDE_AUDIT_BIN, "--help"], cwd=wt_target)
        assert rc == 0, f"claude-audit --help in claude worktree failed ({rc}): {err}"
        rc, out, err = run_cmd([OPUS_AUDIT_BIN, "--help"], cwd=wt_target)
        assert rc == 0, f"opus-audit --help in claude worktree failed ({rc}): {err}"
    finally:
        if created_temp:
            run_cmd(["git", "worktree", "remove", "--force", wt_target], cwd=main_repo)
            run_git_prune = run_cmd(["git", "worktree", "prune"], cwd=main_repo)
            run_cmd(["git", "branch", "-D", "claude/tmp-help"], cwd=main_repo)

    log("✅ --help tests passed successfully.")

def test_git_path_resolutions():
    log("Testing Git path resolutions across all worktree topologies...")
    dirs_to_check = [
        "/Users/peternguyen/openpilot-radar",
        "/Users/peternguyen/openpilot-radar/.worktrees/d057-rebase-verify",
        "/Users/peternguyen/openpilot-radar/.claude/worktrees/repo-state-assessment-6969b6"
    ]

    for d in dirs_to_check:
        p = Path(d)
        if not p.exists():
            continue
        # Verify git rev-parse --git-common-dir resolves to a directory, not a file
        rc, cdir, _ = run_cmd(["git", "rev-parse", "--git-common-dir"], cwd=d)
        assert rc == 0
        c_path = Path(cdir) if Path(cdir).is_absolute() else (p / cdir).resolve()
        assert c_path.is_dir(), f"Expected common dir to be directory: {c_path}"
        assert c_path.name == ".git"

        # Verify git rev-parse --git-path info/exclude resolves properly
        rc, edir, _ = run_cmd(["git", "rev-parse", "--git-path", "info/exclude"], cwd=d)
        assert rc == 0
        e_path = Path(edir) if Path(edir).is_absolute() else (p / edir).resolve()
        assert e_path.name == "exclude"

        # Verify .git is handled correctly (file in linked worktree, dir in main repo)
        dot_git = p / ".git"
        if d == "/Users/peternguyen/openpilot-radar":
            assert dot_git.is_dir(), f"Expected main repo .git to be directory"
        else:
            assert dot_git.is_file(), f"Expected linked worktree .git to be file"

    log("✅ Git path resolutions verified.")

def test_end_to_end_isolated_worktree_lifecycle():
    log("Testing end-to-end worktree delegation, gates, receipts, and teardown in sandbox...")
    sandbox = Path("/Users/peternguyen/openpilot-radar/.agents/teamwork_preview_reviewer_r1/e2e_sandbox")
    if sandbox.exists():
        shutil.rmtree(sandbox)
    sandbox.mkdir(parents=True)

    try:
        # Create git repo
        run_cmd(["git", "init"], cwd=str(sandbox))
        run_cmd(["git", "config", "user.email", "test@test.local"], cwd=str(sandbox))
        run_cmd(["git", "config", "user.name", "Tester"], cwd=str(sandbox))
        readme = sandbox / "README.md"
        readme.write_text("# Test Repo\n")
        run_cmd(["git", "add", "README.md"], cwd=str(sandbox))
        run_cmd(["git", "commit", "-m", "Initial commit"], cwd=str(sandbox))

        # Create simulated Claude nested worktree: .claude/worktrees/nested-claude-test
        claude_wt = sandbox / ".claude" / "worktrees" / "nested-claude-test"
        claude_wt.parent.mkdir(parents=True, exist_ok=True)
        rc, _, err = run_cmd(["git", "worktree", "add", "-B", "claude/test-session", str(claude_wt), "HEAD"], cwd=str(sandbox))
        assert rc == 0, f"Failed to create simulated Claude worktree: {err}"

        # Create mock agy worker binary that writes a working file during execution
        mock_agy = sandbox / "mock_agy.sh"
        mock_agy.write_text("""#!/usr/bin/env bash
if [ "$1" = "models" ]; then
    echo "gemini-3.8-flash-high"
    echo "gemini-3.1-pro-high"
    exit 0
fi
# Simulate worker making legitimate code changes inside the worktree
echo "worker output" > worker_generated.txt
exit 0
""")
        mock_agy.chmod(0o755)

        test_env = os.environ.copy()
        test_env["AGY_BIN"] = str(mock_agy)
        test_env["AGY_FLASH_BIN"] = AGY_FLASH_BIN

        # --- Test 1: agy-flash dispatch with --verify "true" inside nested Claude worktree ---
        log("Running agy-flash -w test-validation --verify 'true' 'echo test' from nested worktree...")
        rc, out, err = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-validation", "--verify", "true", "echo test"],
            cwd=str(claude_wt),
            env=test_env
        )
        assert rc == 0, f"agy-flash invocation failed ({rc}):\nSTDOUT: {out}\nSTDERR: {err}"
        assert "[PASS]" in out
        assert "Verification receipt issued" in out

        # Verify worktree location: MUST be at main_root/.worktrees/test-validation
        expected_wt = sandbox / ".worktrees" / "test-validation"
        assert expected_wt.exists(), f"Worktree was not created at {expected_wt}"
        assert (expected_wt / ".git").is_file()

        # Verify receipt
        receipt_path = expected_wt / ".agy-receipt.json"
        assert receipt_path.exists(), "Receipt file does not exist"
        receipt = json.loads(receipt_path.read_text())
        assert receipt["overall_result"] == "SUCCESS"
        assert receipt["milestone"] == "test-validation"
        assert len(receipt["gates"]) == 1
        assert receipt["gates"][0]["status"] == "PASS"
        assert receipt["gates"][0]["exit_code"] == 0

        # --- Test 2: agy-pro dispatch with identical flag semantics ---
        log("Running agy-pro -w test-pro-validation --verify 'true' 'echo test-pro' from nested worktree...")
        rc, out, err = run_cmd(
            [AGY_PRO_BIN, "-w", "test-pro-validation", "--verify", "true", "echo test-pro"],
            cwd=str(claude_wt),
            env=test_env
        )
        assert rc == 0, f"agy-pro invocation failed ({rc}):\nSTDOUT: {out}\nSTDERR: {err}"
        pro_wt = sandbox / ".worktrees" / "test-pro-validation"
        assert pro_wt.exists()
        pro_receipt = json.loads((pro_wt / ".agy-receipt.json").read_text())
        assert pro_receipt["worker_model"] == "gemini-3.1-pro-high"
        assert pro_receipt["overall_result"] == "SUCCESS"

        # --- Test 3: Verification gate failure handling ---
        log("Testing failing gate --verify 'false'...")
        rc, out, err = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-failing-gate", "--verify", "false", "echo test-fail"],
            cwd=str(claude_wt),
            env=test_env
        )
        assert rc == 12, f"Expected exit code 12 for failing gate, got {rc}"
        assert "[FAIL]" in out
        assert "FAILED_GATES" in out

        # --- Test 4: agy-worktree status ---
        log("Testing agy-worktree status from inside nested worktree...")
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "status", "test-validation"], cwd=str(claude_wt))
        assert rc == 0
        assert "TECHNICALLY_VERIFIED" in out

        # --- Test 5: Safeguard 2 & fallback-accept validations ---
        log("Testing agy-worktree fallback-accept enforcements...")
        # 5a: Missing .opus-report.json fails with exit code 5 (OPUS_IDENTITY_UNVERIFIED)
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "fallback-accept", "test-validation", "--risk-level", "1"], cwd=str(claude_wt))
        assert rc == 5, f"Expected exit 5 for missing opus report, got {rc}: {out}"
        assert "OPUS_IDENTITY_UNVERIFIED" in out

        # 5b: Opus report with unverified reviewer model fails with exit code 5
        opus_file = expected_wt / ".opus-report.json"
        opus_data = {
            "verdict": "APPROVED",
            "reviewer_model": "Imitation Model",
            "reviewed_tree_hash": receipt["verified_tree_hash"]
        }
        opus_file.write_text(json.dumps(opus_data))
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "fallback-accept", "test-validation", "--risk-level", "1"], cwd=str(claude_wt))
        assert rc == 5, f"Expected exit 5 for unverified reviewer model, got {rc}"
        assert "REVIEWER UNVERIFIED" in out

        # 5c: Opus report with CORRECTION_REQUIRED fails with exit code 1
        opus_data["reviewer_model"] = "Claude Opus 4.6 Thinking"
        opus_data["verdict"] = "CORRECTION_REQUIRED"
        opus_file.write_text(json.dumps(opus_data))
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "fallback-accept", "test-validation", "--risk-level", "1"], cwd=str(claude_wt))
        assert rc == 1, f"Expected exit 1 for unapproved verdict, got {rc}"
        assert "LOGIC REVIEW INCOMPLETE" in out

        # 5d: Level 3 high risk blocked without --allow-high-risk
        opus_data["verdict"] = "APPROVED"
        opus_file.write_text(json.dumps(opus_data))
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "fallback-accept", "test-validation", "--risk-level", "3"], cwd=str(claude_wt))
        assert rc == 1, f"Expected exit 1 for Level 3 without explicit human approval, got {rc}"
        assert "LEVEL 3 HIGH RISK BLOCKED" in out

        # 5e: Level 3 with --allow-high-risk succeeds
        rc, out, _ = run_cmd([
            AGY_WORKTREE_BIN, "fallback-accept", "test-validation",
            "--risk-level", "3", "--allow-high-risk", "--justification", "human signed off"
        ], cwd=str(claude_wt))
        assert rc == 0, f"Expected fallback-accept Level 3 with --allow-high-risk to succeed, got {rc}: {out}"
        assert "FALLBACK_ACCEPTED" in out

        # --- Test 6: Mutation Detection (Anti-tampering) ---
        log("Testing post-receipt mutation detection...")
        tamper_file = expected_wt / "tamper_hack.py"
        tamper_file.write_text("evil()\n")
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "accept", "test-validation"], cwd=str(claude_wt))
        assert rc == 2, f"Expected exit code 2 (MUTATION DETECTED) on accept after tampering, got {rc}: {out}"
        assert "MUTATION DETECTED" in out

        # Revert tampering
        tamper_file.unlink()

        # --- Test 7: Claude accept ---
        log("Testing agy-worktree accept from inside nested worktree...")
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "accept", "test-validation"], cwd=str(claude_wt))
        assert rc == 0, f"accept failed ({rc}): {out}"
        assert "CLAUDE_ACCEPTED" in out

        # --- Test 8: Verified Merge into Caller Worktree ---
        log("Testing agy-worktree merge into nested worktree branch...")
        rc, out, err = run_cmd([AGY_WORKTREE_BIN, "merge", "test-validation"], cwd=str(claude_wt))
        assert rc == 0, f"agy-worktree merge failed ({rc}): {err}\n{out}"
        assert "FINALIZED" in out

        # Verify merged file exists in claude_wt
        assert (claude_wt / "worker_generated.txt").exists(), "Merged file should exist in caller worktree"
        assert not expected_wt.exists(), "Merged worktree should be cleaned up"

        # Verify rollback log exists in common dir without NotADirectoryError
        rollback_log = sandbox / ".git" / "agy-rollback-log.json"
        assert rollback_log.exists(), "Rollback log should be written to common .git"
        log_entries = json.loads(rollback_log.read_text())
        assert len(log_entries) == 1
        assert log_entries[0]["milestone"] == "test-validation"

        # --- Test 9: Stale Worktree Pruning Recovery ---
        log("Testing recovery when worktree directory was deleted without git worktree remove...")
        # Create worktree, rm -rf directory, then recreate via agy-flash
        stale_wt = sandbox / ".worktrees" / "test-stale-prune"
        run_cmd(["git", "worktree", "add", "-B", "worker/test-stale-prune", str(stale_wt), "HEAD"], cwd=str(sandbox))
        shutil.rmtree(stale_wt)
        # agy-flash must prune stale registrations and succeed
        rc, out, err = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-stale-prune", "--verify", "true", "echo recover"],
            cwd=str(claude_wt),
            env=test_env
        )
        assert rc == 0, f"Failed to recover from stale worktree: {err}\n{out}"
        run_cmd([AGY_WORKTREE_BIN, "abort", "test-stale-prune"], cwd=str(claude_wt))

        # --- Test 10: Clean Abort Teardown ---
        log("Testing agy-worktree abort on remaining worker worktrees...")
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "abort", "test-pro-validation"], cwd=str(claude_wt))
        assert rc == 0
        assert not pro_wt.exists()

        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "abort", "test-failing-gate"], cwd=str(claude_wt))
        assert rc == 0

        # Verify no dirty locks or lingering worker branches
        rc, branch_out, _ = run_cmd(["git", "branch"], cwd=str(sandbox))
        assert "worker/test-validation" not in branch_out
        assert "worker/test-pro-validation" not in branch_out
        assert "worker/test-failing-gate" not in branch_out
        assert "worker/test-stale-prune" not in branch_out

        log("✅ Full isolated worktree lifecycle passed without errors!")
    finally:
        shutil.rmtree(sandbox, ignore_errors=True)

def test_fault_tolerance_and_edge_cases():
    log("Testing fault tolerance, edge cases, and graceful degradation...")
    sandbox = Path("/Users/peternguyen/openpilot-radar/.agents/teamwork_preview_reviewer_r1/e2e_edge_cases")
    if sandbox.exists():
        shutil.rmtree(sandbox)
    sandbox.mkdir(parents=True)

    try:
        run_cmd(["git", "init"], cwd=str(sandbox))
        run_cmd(["git", "config", "user.email", "edge@test.local"], cwd=str(sandbox))
        run_cmd(["git", "config", "user.name", "EdgeTester"], cwd=str(sandbox))
        readme = sandbox / "README.md"
        readme.write_text("# Edge Cases\n")
        run_cmd(["git", "add", "README.md"], cwd=str(sandbox))
        run_cmd(["git", "commit", "-m", "Initial commit"], cwd=str(sandbox))

        # Nested worktree
        nested_wt = sandbox / ".claude" / "worktrees" / "nested-edge"
        nested_wt.parent.mkdir(parents=True, exist_ok=True)
        run_cmd(["git", "worktree", "add", "-B", "claude/edge", str(nested_wt), "HEAD"], cwd=str(sandbox))

        mock_agy = sandbox / "mock_agy.sh"
        mock_agy.write_text("#!/usr/bin/env bash\necho '[MOCK AGY] ok'\nexit 0\n")
        mock_agy.chmod(0o755)

        test_env = os.environ.copy()
        test_env["AGY_BIN"] = str(mock_agy)
        test_env["AGY_FLASH_BIN"] = AGY_FLASH_BIN

        # Edge Case 1: Direct write enforcement in unisolated main repo
        log("Testing isolation rejection in main repo checkout...")
        rc, out, _ = run_cmd([AGY_FLASH_BIN, "direct write attempt"], cwd=str(sandbox), env=test_env)
        assert rc == 13, f"Expected code 13 for unisolated direct checkout write, got {rc}"
        assert "FAILED_ISOLATION_REQUIRED" in out

        # Edge Case 2: Direct write allowed inside linked worktree (since already isolated)
        log("Testing direct write permission inside already isolated worktree...")
        rc, out, _ = run_cmd([AGY_FLASH_BIN, "working directly"], cwd=str(nested_wt), env=test_env)
        assert rc == 0, f"Expected code 0 inside linked worktree without -w, got {rc}:\n{out}"

        # Edge Case 3: Fault tolerance when info/exclude is read-only
        log("Testing graceful degradation when info/exclude is read-only...")
        exclude_file = sandbox / ".git" / "info" / "exclude"
        exclude_file.parent.mkdir(parents=True, exist_ok=True)
        exclude_file.write_text("")
        exclude_file.chmod(0o444)
        rc, out, _ = run_cmd([AGY_FLASH_BIN, "-w", "test-readonly-exclude", "--verify", "true", "echo test"], cwd=str(nested_wt), env=test_env)
        assert rc == 0, f"Expected agy-flash to degrade gracefully when info/exclude is read-only, got {rc}"
        exclude_file.chmod(0o644)
        run_cmd([AGY_WORKTREE_BIN, "abort", "test-readonly-exclude"], cwd=str(nested_wt))

        # Edge Case 4: Baseline drift detection in agy-worktree merge
        log("Testing baseline drift detection in agy-worktree merge...")
        rc, out, _ = run_cmd([AGY_FLASH_BIN, "-w", "test-drift", "--verify", "true", "echo drift"], cwd=str(nested_wt), env=test_env)
        assert rc == 0
        run_cmd([AGY_WORKTREE_BIN, "accept", "test-drift"], cwd=str(nested_wt))
        # Advance nested worktree HEAD commit
        dummy = nested_wt / "advance.txt"
        dummy.write_text("advance\n")
        run_cmd(["git", "add", "advance.txt"], cwd=str(nested_wt))
        run_cmd(["git", "commit", "-m", "advance branch"], cwd=str(nested_wt))
        # Attempt merge: should detect baseline drift
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "merge", "test-drift"], cwd=str(nested_wt))
        assert rc == 3, f"Expected exit code 3 for baseline drift, got {rc}"
        assert "BASELINE DRIFT DETECTED" in out
        run_cmd([AGY_WORKTREE_BIN, "abort", "test-drift"], cwd=str(nested_wt))

        # Edge Case 5: Path allowlist prefix boundary enforcement
        log("Testing path allowlist prefix boundary enforcement...")
        # A worker touching 'allowed_dir/sub.txt' should PASS when allowed_paths='allowed_dir'
        # A worker touching 'allowed_dir_backdoor.txt' should FAIL when allowed_paths='allowed_dir'
        mock_leaker = sandbox / "mock_leaker.sh"
        mock_leaker.write_text("""#!/usr/bin/env bash
mkdir -p allowed_dir
echo "safe" > allowed_dir/safe.txt
echo "unsafe" > allowed_dir_backdoor.txt
exit 0
""")
        mock_leaker.chmod(0o755)
        leaker_env = test_env.copy()
        leaker_env["AGY_BIN"] = str(mock_leaker)

        rc, out, _ = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-leak-gate", "-a", "allowed_dir", "--verify", "true", "leak test"],
            cwd=str(nested_wt),
            env=leaker_env
        )
        assert rc == 10, f"Expected exit code 10 (FAILED_PATH_VIOLATION) for prefix leakage, got {rc}: {out}"
        assert "Path Allowlist Breached" in out
        assert "allowed_dir_backdoor.txt" in out
        run_cmd([AGY_WORKTREE_BIN, "abort", "test-leak-gate"], cwd=str(nested_wt))

        # Edge Case 6: Worker committing all changes inside worktree
        log("Testing agy-worktree merge when worker commits all changes...")
        mock_committer = sandbox / "mock_committer.sh"
        mock_committer.write_text("""#!/usr/bin/env bash
echo "committed file" > committed.txt
git add committed.txt
git commit -m "worker committed"
exit 0
""")
        mock_committer.chmod(0o755)
        committer_env = test_env.copy()
        committer_env["AGY_BIN"] = str(mock_committer)

        rc, out, err = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-committed-merge", "--verify", "true", "committed task"],
            cwd=str(nested_wt),
            env=committer_env
        )
        assert rc == 0, f"agy-flash dispatch failed: {err}\n{out}"
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "accept", "test-committed-merge"], cwd=str(nested_wt))
        assert rc == 0
        rc, out, err = run_cmd([AGY_WORKTREE_BIN, "merge", "test-committed-merge"], cwd=str(nested_wt))
        assert rc == 0, f"Expected merge of committed worker to succeed, got {rc}: {err}\n{out}"
        assert "FINALIZED" in out
        assert (nested_wt / "committed.txt").exists()

        # Edge Case 7: Multi-attempt workflow (--attempt 2) on existing worktree
        log("Testing multi-attempt workflow (--attempt 2) baseline stability...")
        rc, out, err = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-multi-attempt", "--attempt", "1", "--verify", "true", "attempt 1"],
            cwd=str(nested_wt),
            env=committer_env
        )
        assert rc == 0, f"Attempt 1 failed: {err}\n{out}"
        # Second attempt should retain caller baseline commit
        rc, out, err = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-multi-attempt", "--attempt", "2", "--verify", "true", "attempt 2"],
            cwd=str(nested_wt),
            env=committer_env
        )
        assert rc == 0, f"Attempt 2 failed: {err}\n{out}"
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "accept", "test-multi-attempt"], cwd=str(nested_wt))
        assert rc == 0
        rc, out, err = run_cmd([AGY_WORKTREE_BIN, "merge", "test-multi-attempt"], cwd=str(nested_wt))
        assert rc == 0, f"Expected multi-attempt merge to succeed without false drift, got {rc}: {err}\n{out}"
        assert "FINALIZED" in out

        # Edge Case 8: Trailing slash and .worktrees/ prefix in milestone names
        log("Testing milestone name sanitization (trailing slashes & prefixes)...")
        rc, out, err = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-slash/", "--verify", "true", "slash test"],
            cwd=str(nested_wt),
            env=test_env
        )
        assert rc == 0, f"Failed on milestone with trailing slash: {err}\n{out}"
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "status", ".worktrees/test-slash"], cwd=str(nested_wt))
        assert rc == 0
        run_cmd([AGY_WORKTREE_BIN, "abort", ".worktrees/test-slash/"], cwd=str(nested_wt))

        # Edge Case 9: Path allowlist with leading ./
        log("Testing path allowlist with leading ./...")
        mock_dotslash = sandbox / "mock_dotslash.sh"
        mock_dotslash.write_text("""#!/usr/bin/env bash
mkdir -p subfolder
echo "ok" > subfolder/test.txt
exit 0
""")
        mock_dotslash.chmod(0o755)
        dotslash_env = test_env.copy()
        dotslash_env["AGY_BIN"] = str(mock_dotslash)
        rc, out, _ = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-dotslash", "-a", "./subfolder", "--verify", "true", "dotslash test"],
            cwd=str(nested_wt),
            env=dotslash_env
        )
        assert rc == 0, f"Expected leading ./ in allowlist to match, got {rc}: {out}"
        run_cmd([AGY_WORKTREE_BIN, "abort", "test-dotslash"], cwd=str(nested_wt))

        # Edge Case 10: Agent scaffolding (.agents/ and metadata files) does not alter tree hash
        log("Testing agent scaffolding metadata does not mutate tree hash...")
        rc, out, err = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-agent-meta", "--verify", "true", "meta test"],
            cwd=str(nested_wt),
            env=test_env
        )
        assert rc == 0
        wt_dir = sandbox / ".worktrees" / "test-agent-meta"
        # Create agent notes inside worktree
        agent_dir = wt_dir / ".agents" / "worker"
        agent_dir.mkdir(parents=True, exist_ok=True)
        (agent_dir / "progress.md").write_text("# Progress\n")
        (wt_dir / "ORIGINAL_REQUEST.md").write_text("# Request\n")
        # Accept should NOT report mutation
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "accept", "test-agent-meta"], cwd=str(nested_wt))
        assert rc == 0, f"Internal metadata falsely triggered mutation: {out}"
        run_cmd([AGY_WORKTREE_BIN, "abort", "test-agent-meta"], cwd=str(nested_wt))

        # Edge Case 11: claude-audit -w in nested worktree
        log("Testing claude-audit -w from nested worktree...")
        rc, out, err = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-audit-wt", "--verify", "true", "audit test"],
            cwd=str(nested_wt),
            env=test_env
        )
        assert rc == 0
        rc, out, err = run_cmd([CLAUDE_AUDIT_BIN, "-w", "test-audit-wt", "--simulate", "CLAUDE_AUDIT_PASSED"], cwd=str(nested_wt))
        assert rc == 0, f"claude-audit -w failed: {err}\n{out}"
        run_cmd([AGY_WORKTREE_BIN, "abort", "test-audit-wt"], cwd=str(nested_wt))

        # Edge Case 12: In-tree directory collision (e.g. milestone named 'tools' matching repo dir)
        log("Testing in-tree directory collision (milestone named 'tools')...")
        intree_tools = sandbox / "tools"
        intree_tools.mkdir(exist_ok=True)
        (intree_tools / "build.py").write_text("# in-tree tool\n")
        run_cmd(["git", "add", "tools/build.py"], cwd=str(sandbox))
        run_cmd(["git", "commit", "-m", "add in-tree tools"], cwd=str(sandbox))
        # Dispatch milestone named 'tools'
        rc, out, err = run_cmd(
            [AGY_FLASH_BIN, "-w", "tools", "--verify", "true", "tools milestone"],
            cwd=str(nested_wt),
            env=committer_env
        )
        assert rc == 0, f"agy-flash -w tools failed: {err}\n{out}"
        tools_wt = sandbox / ".worktrees" / "tools"
        assert tools_wt.exists(), f"Expected worktree at {tools_wt}, not {intree_tools}"
        # Status, accept, merge should all target .worktrees/tools, not ./tools
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "status", "tools"], cwd=str(nested_wt))
        assert rc == 0
        assert str(tools_wt) in out
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "accept", "tools"], cwd=str(nested_wt))
        assert rc == 0
        rc, out, err = run_cmd([AGY_WORKTREE_BIN, "merge", "tools"], cwd=str(nested_wt))
        assert rc == 0, f"Failed to merge milestone 'tools': {err}\n{out}"
        assert (nested_wt / "committed.txt").exists()
        assert (intree_tools / "build.py").exists(), "In-tree tools directory must not be deleted or corrupted"

        # Edge Case 13: Invalid milestone names (.., ., empty)
        log("Testing invalid milestone names reject cleanly without git ref crash...")
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "create", ".."], cwd=str(nested_wt))
        assert rc == 1, f"Expected exit 1 for '..', got {rc}: {out}"
        assert "Invalid worktree name" in out
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "create", "."], cwd=str(nested_wt))
        assert rc == 1, f"Expected exit 1 for '.', got {rc}: {out}"
        assert "Invalid worktree name" in out
        rc, out, _ = run_cmd([AGY_FLASH_BIN, "-w", "..", "invalid test"], cwd=str(nested_wt), env=test_env)
        assert rc == 1, f"Expected exit 1 for agy-flash -w '..', got {rc}: {out}"
        assert "Invalid worktree name" in out

        # Edge Case 14: Self-merge and self-abort guard
        log("Testing self-merge and self-abort guards...")
        rc, out, err = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-self-guard", "--verify", "true", "self guard"],
            cwd=str(nested_wt),
            env=test_env
        )
        assert rc == 0
        self_wt = sandbox / ".worktrees" / "test-self-guard"
        # Try merge from inside self_wt
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "merge", "test-self-guard"], cwd=str(self_wt))
        assert rc == 1, f"Expected exit 1 for self-merge, got {rc}: {out}"
        assert "Cannot merge worktree 'test-self-guard' into itself" in out
        # Try abort from inside self_wt
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "abort", "test-self-guard"], cwd=str(self_wt))
        assert rc == 1, f"Expected exit 1 for self-abort, got {rc}: {out}"
        assert "Cannot abort worktree 'test-self-guard' from within itself" in out
        # Abort properly from nested_wt
        rc, out, _ = run_cmd([AGY_WORKTREE_BIN, "abort", "test-self-guard"], cwd=str(nested_wt))
        assert rc == 0

        # Edge Case 15: claude-audit invoked inside worktree without -w flag
        log("Testing claude-audit directly inside worktree without -w...")
        rc, out, err = run_cmd(
            [AGY_FLASH_BIN, "-w", "test-inside-audit", "--verify", "true", "inside audit test"],
            cwd=str(nested_wt),
            env=committer_env
        )
        assert rc == 0
        inside_wt = sandbox / ".worktrees" / "test-inside-audit"
        # Run claude-audit from inside_wt without -w flag
        rc, out, err = run_cmd([CLAUDE_AUDIT_BIN, "--simulate", "CLAUDE_AUDIT_PASSED"], cwd=str(inside_wt))
        assert rc == 0, f"claude-audit inside worktree failed: {err}\n{out}"
        assert "CLAUDE_AUDIT_PASSED" in out
        run_cmd([AGY_WORKTREE_BIN, "abort", "test-inside-audit"], cwd=str(nested_wt))

        # Edge Case 16: claude-audit excludes untracked internal metadata files
        log("Testing claude-audit untracked internal metadata filtering...")
        clean_wt_dir = sandbox / ".worktrees" / "test-meta-filter"
        run_cmd(["git", "worktree", "add", "-B", "worker/test-meta-filter", str(clean_wt_dir), "HEAD"], cwd=str(sandbox))
        # Add untracked metadata files and .agents
        (clean_wt_dir / ".agy-receipt.json").write_text("{}")
        (clean_wt_dir / ".opus-report.json").write_text("{}")
        (clean_wt_dir / ".agents").mkdir()
        (clean_wt_dir / ".agents" / "notes.txt").write_text("agent scratchpad")
        # claude-audit should report NO_CHANGES because internal files are filtered out
        rc, out, _ = run_cmd([CLAUDE_AUDIT_BIN], cwd=str(clean_wt_dir))
        assert rc == 0, f"Expected 0 for clean worktree with metadata: {out}"
        assert "NO_CHANGES" in out
        run_cmd([AGY_WORKTREE_BIN, "abort", "test-meta-filter"], cwd=str(nested_wt))

        # Edge Case 17: claude-audit cascades to opus-audit on availability condition
        log("Testing claude-audit availability circuit & fallback cascade to opus-audit...")
        # Reset circuit first
        run_cmd([CLAUDE_AUDIT_BIN, "--reset-circuit"])
        # Mock opus-audit binary in test_env
        mock_opus = sandbox / "mock_opus.sh"
        mock_opus.write_text("""#!/usr/bin/env bash
echo "OPUS LOGIC AUDIT"
echo "Verdict: APPROVED"
exit 0
""")
        mock_opus.chmod(0o755)
        opus_test_env = test_env.copy()
        opus_test_env["OPUS_AUDIT_BIN"] = str(mock_opus)
        rc, out, _ = run_cmd(
            [CLAUDE_AUDIT_BIN, "--simulate", "CLAUDE_TEMPORARILY_UNAVAILABLE"],
            cwd=str(nested_wt),
            env=opus_test_env
        )
        assert rc == 0, f"Expected cascade to Opus to exit 0 on APPROVED, got {rc}: {out}"
        assert "Claude Continuity Mode Active" in out
        assert "Automatically cascading to Claude Opus 4.6" in out
        run_cmd([CLAUDE_AUDIT_BIN, "--reset-circuit"])

        log("✅ Fault tolerance and edge cases passed successfully.")
    finally:
        shutil.rmtree(sandbox, ignore_errors=True)

def main():
    print("=" * 60)
    print("STARTING WORKTREE DELEGATION HARNESS VERIFICATION")
    print("=" * 60)
    try:
        test_help_invocations()
        test_git_path_resolutions()
        test_end_to_end_isolated_worktree_lifecycle()
        test_fault_tolerance_and_edge_cases()
        print("=" * 60)
        print("ALL VERIFICATION SUITES PASSED (0 ERRORS)")
        print("=" * 60)
        sys.exit(0)
    except AssertionError as e:
        import traceback
        traceback.print_exc()
        print(f"\n❌ ASSERTION FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"\n❌ UNEXPECTED ERROR: {e}")
        sys.exit(2)

if __name__ == "__main__":
    main()
