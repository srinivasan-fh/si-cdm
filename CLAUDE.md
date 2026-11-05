# Notes for Claude working in this repository

## Commits

- The author (and committer) of every commit is the owner:
  `Srinivasan <169058352+srinivasan-fh@users.noreply.github.com>` (set in this clone's git
  config: `git config user.name "Srinivasan"`,
  `git config user.email "169058352+srinivasan-fh@users.noreply.github.com"`).
- Claude is credited as co-author with this trailer, the last lines of every commit message:

  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: <the session link>
  ```
- Push the work branch, then fast-forward `main` when it is an ancestor
  (`git fetch origin main && git merge-base --is-ancestor origin/main HEAD && git push origin HEAD:main`).
