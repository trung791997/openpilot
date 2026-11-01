import unittest

from tools.ci.branch_guard import CLEAN, STABLE, STAGE, TESTING, violations

REPO = 'Biscuit-Labs/openpilot'
FORK = 'someone/openpilot'
SHA = '2ac4ca60e2b1d0f6a3c45e7f8a9b0c1d2e3f4a5b'


def in_branch(*members):
  return lambda sha, branch: (sha, branch) in members


class TestBranchGuard(unittest.TestCase):
  def test_promotions_go_one_step_at_a_time(self):
    self.assertEqual(violations(STAGE, TESTING, REPO, REPO), [])
    self.assertEqual(violations(STABLE, STAGE, REPO, REPO), [])
    for base, head in ((STABLE, TESTING), (STABLE, 'wifi-power-save'), (STAGE, 'wifi-power-save'), (STAGE, STABLE)):
      with self.subTest(base=base, head=head):
        self.assertEqual(len(violations(base, head, REPO, REPO)), 1)

  def test_promotion_from_a_fork_with_the_same_name_is_refused(self):
    self.assertEqual(len(violations(STAGE, TESTING, FORK, REPO)), 1)
    self.assertEqual(len(violations(STABLE, STAGE, FORK, REPO)), 1)

  def test_a_snapshot_of_the_source_branch_can_be_promoted(self):
    in_testing = in_branch((SHA, TESTING))
    in_stage = in_branch((SHA, STAGE))
    self.assertEqual(violations(STAGE, f'promote-testing-{SHA[:10]}', REPO, REPO, SHA, in_testing), [])
    self.assertEqual(violations(STAGE, f'promote-testing-{SHA}', REPO, REPO, SHA, in_testing), [])
    self.assertEqual(violations(STABLE, f'promote-stage-{SHA[:10]}', REPO, REPO, SHA, in_stage), [])

  def test_a_snapshot_must_be_its_named_commit_and_already_in_the_source(self):
    in_testing = in_branch((SHA, TESTING))
    moved = 'f' * 40
    cases = (
      (STAGE, f'promote-testing-{SHA[:10]}', SHA, in_branch()),           # not merged into Testing
      (STAGE, f'promote-testing-{SHA[:10]}', moved, in_testing),          # took a commit after it was cut
      (STAGE, f'promote-testing-{SHA[:10]}', None, in_testing),           # no head sha to check
      (STAGE, f'promote-testing-{SHA[:6]}', SHA, in_testing),             # name too short to mean one commit
      (STAGE, f'promote-testing-{SHA[:10].upper()}', SHA, in_testing),
      (STAGE, f'promote-stage-{SHA[:10]}', SHA, in_branch((SHA, STAGE))),  # Stage snapshots go to Stable only
      (STABLE, f'promote-testing-{SHA[:10]}', SHA, in_testing),           # no skipping Stage
    )
    for base, head, head_sha, is_ancestor in cases:
      with self.subTest(base=base, head=head, head_sha=head_sha):
        self.assertEqual(len(violations(base, head, REPO, REPO, head_sha, is_ancestor)), 1)

  def test_a_snapshot_from_a_fork_is_refused(self):
    self.assertEqual(len(violations(STAGE, f'promote-testing-{SHA[:10]}', FORK, REPO, SHA, in_branch((SHA, TESTING)))), 1)

  def test_testing_takes_feature_branches_but_nothing_flows_back(self):
    self.assertEqual(violations(TESTING, 'wifi-power-save', REPO, REPO), [])
    self.assertEqual(violations(TESTING, 'wifi-power-save', FORK, REPO), [])
    for head in (STAGE, STABLE, CLEAN):
      with self.subTest(head=head):
        self.assertEqual(len(violations(TESTING, head, REPO, REPO)), 1)

  def test_agent_prefixed_branches_are_refused_into_testing(self):
    for head in ('claude/project-thread-ja9lwc', 'Codex/fix', 'copilot/x'):
      with self.subTest(head=head):
        self.assertEqual(len(violations(TESTING, head, REPO, REPO)), 1)

  def test_other_bases_are_not_this_checks_business(self):
    self.assertEqual(violations('wifi-power-save', 'claude/x', REPO, REPO), [])
    self.assertEqual(violations(CLEAN, 'clean-sync', REPO, REPO), [])


if __name__ == '__main__':
  unittest.main()
