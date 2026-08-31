# STATE_PROTOCOL.md

> **Historical.** This protocol governed a multi-branch development repo.
> COCO 2.0 is now frozen on a single branch — `main` — and a fresh clone
> of it is sufficient to understand the finished project; no feature
> branch is required to discover any code or any state. The document is
> kept because the *reasoning* below is reusable and the failure it
> records is a real one, not because the branch rules still apply.

**How project state was stored, and which branch owned which file.**

This exists because the first attempt got it wrong in a way that is worth
recording: `PROJECT_STATE.md` and `docs/ROADMAP.md` were written on the
C2-M1 feature branch. Both describe *the project*, not that branch, and
putting them there produced two concrete defects:

1. **A fresh agent checking out the trunk saw no state at all.** The
   whole point of the handoff protocol is that clearing the conversation
   is safe. It was not — the state was hiding on a branch nobody had been
   told to check out.
2. **They are singleton mutable snapshots.** Two feature branches that
   both checkpoint will both rewrite the same lines of `PROJECT_STATE.md`
   and conflict on every merge, forever. That is not a merge accident; it
   is the predictable consequence of version-controlling a "current
   value" on parallel branches.

---

## The rule

| File | Owner | Edited on |
|---|---|---|
| `PROJECT_STATE.md` | trunk | **`jazzy-harmonic-port` only** |
| `docs/ROADMAP.md` | trunk | **`jazzy-harmonic-port` only** |
| `docs/STATE_PROTOCOL.md` | trunk | **`jazzy-harmonic-port` only** |
| `docs/SESSION_LOG.md` | shared, **append-only** | any branch |
| `docs/RESULTS.md` | the branch that measured it | any branch |
| `CLAUDE.md` | trunk, but rules may land with the work that proved them | any branch |
| source, tests, launch, rviz | the feature branch | feature branch |

**Feature branches must not edit `PROJECT_STATE.md` or
`docs/ROADMAP.md`.** If a feature branch changes what those files should
say, land the change on the trunk separately — it is two lines of text
and it keeps the merge clean.

`docs/SESSION_LOG.md` is exempt because it is **append-only**. Two
branches appending different entries conflict only at the tail, and the
resolution is always "keep both, in date order". Never rewrite an
existing entry to resolve a conflict.

## Why not a separate long-lived state branch

Considered and rejected. A parallel `state` branch would need merging
into every feature branch to be readable from them, which is strictly
more work than keeping state on the trunk and more likely to go stale.
Trunk is the branch a fresh agent lands on; state belongs where the
agent already is.

## Bootstrapping a fresh session

`CLAUDE.md` is loaded automatically and its first section points at
`PROJECT_STATE.md`. That is the entry point, and it is on the trunk, so
it is visible from a bare clone with no branch knowledge.

`PROJECT_STATE.md` carries a **BRANCH MAP** naming every branch with
unmerged work. That is what makes trunk-only state honest: the trunk may
not *contain* the C2-M1 code, but it always *knows where it is*.

## When work merges

The merge itself does not update `PROJECT_STATE.md` — nothing does
automatically. After merging a feature branch to the trunk:

1. Update the **BRANCH MAP** (drop the merged branch).
2. Update **CURRENT MILESTONE**, **TESTS LAST RUN**, and
   **LAST VERIFIED COMMIT**.
3. Append a `docs/SESSION_LOG.md` entry recording the merge.

## Invariant to check if state ever looks wrong

```bash
git ls-tree --name-only jazzy-harmonic-port PROJECT_STATE.md docs/ROADMAP.md
```

Both must be listed. If either is missing from the trunk, the protocol
is broken again and a fresh agent is flying blind.
