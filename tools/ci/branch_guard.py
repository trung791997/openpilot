"""Enforce the Honda-PTM branch flow on pull requests (docs/BRANCH_WORKFLOW.md).

Rulesets can protect a branch, but they cannot say which branch a pull request comes from. This check does:
  - Honda-PTM-Stage takes pull requests only from Honda-PTM-Testing or a Testing snapshot,
  - Honda-PTM-Stable takes pull requests only from Honda-PTM-Stage or a Stage snapshot,
  - Honda-PTM-Testing takes no pull request from Stage, Stable or Clean (changes move one way), and
    no branch named with a tool or agent prefix such as `claude/`,
  - Honda-PTM-Clean is checked by tools/night_star_port/check_clean_branch.py against Honda-PTM-Stable.

A snapshot is a branch named `promote-testing-<sha>` (or `promote-stage-<sha>`) whose tip is that commit, and that
commit is already in Testing (or Stage). It promotes a fixed, tested point while the source branch keeps taking merges.
tools/ci/promote.py cuts one.

Promotion pull requests must come from this repository, not from a fork.

usage: branch_guard.py <base> <head> <head-repo> <repo> [<head-sha>]
"""
import re
import subprocess
import sys

STABLE, STAGE, TESTING, CLEAN = 'Honda-PTM-Stable', 'Honda-PTM-Stage', 'Honda-PTM-Testing', 'Honda-PTM-Clean'
LONG_LIVED = (STABLE, STAGE, TESTING, CLEAN)
PROMOTES_FROM = {STAGE: TESTING, STABLE: STAGE}
SNAPSHOT_PREFIX = {TESTING: 'promote-testing-', STAGE: 'promote-stage-'}
SNAPSHOT_SHA = re.compile(r'[0-9a-f]{10,40}')
AGENT_PREFIXES = ('claude/', 'codex/', 'copilot/', 'cursor/', 'gemini/')


def snapshot_violations(source, head, head_sha, is_ancestor):
  name_sha = head[len(SNAPSHOT_PREFIX[source]):]
  if not SNAPSHOT_SHA.fullmatch(name_sha):
    return [f'snapshot branch {head} must be named {SNAPSHOT_PREFIX[source]}<commit>, with 10 to 40 lowercase hex digits']
  if not head_sha or not head_sha.startswith(name_sha):
    return [f'{head} has moved off {name_sha}: a snapshot never takes new commits. Cut a new snapshot instead.']
  if not is_ancestor(head_sha, source):
    return [f'{name_sha} is not in {source}: a snapshot promotes only what has already merged there']
  return []


def violations(base, head, head_repo, repo, head_sha=None, is_ancestor=None):
  if base in PROMOTES_FROM:
    source = PROMOTES_FROM[base]
    if head_repo == repo and head == source:
      return []
    if head_repo == repo and head.startswith(SNAPSHOT_PREFIX[source]):
      return snapshot_violations(source, head, head_sha, is_ancestor)
    snapshot = f'{SNAPSHOT_PREFIX[source]}<commit>'
    return [f'{base} takes pull requests only from {repo}:{source} or a {snapshot} snapshot, not {head_repo}:{head}. Promote through {TESTING}.']
  if base == TESTING:
    if head in LONG_LIVED and head_repo == repo:
      return [f'{head} does not merge back into {TESTING}: changes move Testing -> Stage -> Stable only. Branch the fix off {TESTING} instead.']
    if head.lower().startswith(AGENT_PREFIXES):
      return [f'branch {head} uses a tool or agent prefix; name it after the work (for example wifi-power-save)']
  return []


def git_is_ancestor(sha, branch):
  # actions/checkout with fetch-depth: 0 leaves every branch under origin/
  return subprocess.run(['git', 'merge-base', '--is-ancestor', sha, f'origin/{branch}']).returncode == 0


def main(argv):
  if len(argv) not in (4, 5):
    sys.exit(__doc__)
  found = violations(*argv, is_ancestor=git_is_ancestor)
  for line in found:
    print(f'::error::{line}')
  print(f'{argv[2]}:{argv[1]} -> {argv[0]}: ' + ('FAIL' if found else 'ok'))
  return 1 if found else 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
