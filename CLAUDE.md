# Notes for Claude working in this repository

## Commits

- Commit as Claude only: the author and committer are `Claude <noreply@anthropic.com>`
  (the repository's git config). Never commit under the owner's name.
- Credit the owner as co-author with this trailer, the last lines of every commit message:

  ```
  Co-authored-by: srinivasan-fh <169058352+srinivasan-fh@users.noreply.github.com>
  Claude-Session: <the session link>
  ```

  No `Co-Authored-By: Claude ...` line: Claude is already the author.
- Push the work branch, then fast-forward `main` when it is an ancestor
  (`git fetch origin main && git merge-base --is-ancestor origin/main HEAD && git push origin HEAD:main`).
