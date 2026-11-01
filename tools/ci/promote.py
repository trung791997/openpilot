#!/usr/bin/env python3
"""Cut a promotion snapshot and open its pull request (docs/BRANCH_WORKFLOW.md, Promoting).

  promote.py testing [<commit>]   Testing -> Stage, from <commit> (default: the tip of Testing)
  promote.py stage [<commit>]     Stage -> Stable, from <commit> (default: the tip of Stage)
  promote.py cleanup              delete promote-* snapshot branches whose pull request is merged or closed

The snapshot branch is `promote-testing-<commit>` (or `promote-stage-<commit>`). It never takes another commit,
so the pull request's checks and approval stay on exactly what was tested, while Testing keeps taking merges.
branch guard checks the name, that the branch has not moved, and that <commit> is already in the source branch.

Needs git and an authenticated gh. --remote names this repo's remote (default: origin).
"""
import argparse
import json
import subprocess
import sys

from branch_guard import PROMOTES_FROM, SNAPSHOT_PREFIX, STABLE, STAGE, TESTING

TARGET = {TESTING: STAGE, STAGE: STABLE}
SOURCES = {'testing': TESTING, 'stage': STAGE}


def run(*cmd, capture=True):
  result = subprocess.run(cmd, check=True, text=True, capture_output=capture)
  return result.stdout.strip() if capture else ''


def repo_slug(remote):
  return run('gh', 'repo', 'view', run('git', 'remote', 'get-url', remote), '--json', 'nameWithOwner', '-q', '.nameWithOwner')


def merged_pull_requests(remote, target, sha):
  # first-parent merges between the target and the snapshot: the pull requests this promotion carries
  log = run('git', 'log', '--first-parent', '--merges', '--format=%s', f'{remote}/{target}..{sha}')
  return [line for line in log.splitlines() if line.startswith('Merge pull request #')]


def promote(source, commit, remote):
  target = TARGET[source]
  assert PROMOTES_FROM[target] == source
  run('git', 'fetch', '--quiet', remote, source, target, capture=False)
  sha = run('git', 'rev-parse', '--verify', f'{commit or f"{remote}/{source}"}^{{commit}}')
  if subprocess.run(['git', 'merge-base', '--is-ancestor', sha, f'{remote}/{source}']).returncode != 0:
    sys.exit(f'{sha[:10]} is not in {remote}/{source}: only merged commits can be promoted')
  if subprocess.run(['git', 'merge-base', '--is-ancestor', sha, f'{remote}/{target}']).returncode == 0:
    sys.exit(f'{sha[:10]} is already in {target}: nothing to promote')

  branch = f'{SNAPSHOT_PREFIX[source]}{sha[:10]}'
  carried = merged_pull_requests(remote, target, sha)
  run('git', 'push', remote, f'{sha}:refs/heads/{branch}', capture=False)

  lines = [f'Promotes `{source}` at {sha[:10]} to `{target}` from the snapshot branch `{branch}`.',
           f'`{source}` can keep taking merges; they wait for the next promotion.', '']
  if carried:
    lines += ['Carries:', *(f'- {line}' for line in carried), '']
  lines += ['The person who opens this does not approve it. Delete the snapshot branch after it merges (`tools/ci/promote.py cleanup`).']
  run('gh', 'pr', 'create', '--repo', repo_slug(remote), '--base', target, '--head', branch,
      '--title', f'Promote {source} {sha[:10]} to {target.removeprefix("Honda-PTM-")}', '--body', '\n'.join(lines),
      capture=False)


def cleanup(remote):
  slug = repo_slug(remote)
  for prefix in SNAPSHOT_PREFIX.values():
    prs = json.loads(run('gh', 'pr', 'list', '--repo', slug, '--state', 'all', '--search', f'head:{prefix}',
                         '--json', 'headRefName,state', '--limit', '100'))
    open_heads = {pr['headRefName'] for pr in prs if pr['state'] == 'OPEN'}
    done = {pr['headRefName'] for pr in prs if pr['state'] != 'OPEN'} - open_heads
    remote_heads = run('git', 'ls-remote', '--heads', remote, f'{prefix}*')
    for line in remote_heads.splitlines():
      branch = line.split('refs/heads/', 1)[1]
      if branch in done:
        run('git', 'push', remote, '--delete', branch, capture=False)


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('source', choices=(*SOURCES, 'cleanup'))
  parser.add_argument('commit', nargs='?')
  parser.add_argument('--remote', default='origin')
  args = parser.parse_args()
  if args.source == 'cleanup':
    cleanup(args.remote)
  else:
    promote(SOURCES[args.source], args.commit, args.remote)


if __name__ == '__main__':
  main()
