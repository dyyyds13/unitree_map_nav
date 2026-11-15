---
name: project-handoff
description: Save a concise project checkpoint, prepare a handoff to a fresh Codex conversation, or resume work from repository state. Use when the user explicitly asks for checkpoint, handoff, context transfer, new-session recovery, or to avoid repeated context compaction.
---

# Project handoff

Modes: `checkpoint`, `handoff`, `resume`. Infer mode from the explicit user request; if ambiguous, default to `checkpoint` and do not close or reset any conversation.

## Common rules
- The actual working tree, tests, and Git status are the source of truth; documentation may be stale.
- Inspect only relevant files and narrowly scoped diffs. Avoid full transcript export, unbounded file dumps, generated data and logs.
- Do not modify application code as part of a documentation-only handoff.
- Preserve unrelated changes; do not run `git reset`, `git clean`, commit, push, or discard work without permission.
- Separate verified facts, hypotheses, and next steps. Never invent a test result, file path, decision, or Git commit.
- Summaries should describe interfaces and file paths rather than copying source code.

## checkpoint
1. Inspect `git status --short`, current branch and HEAD when Git is available. If not, mark Git metadata unavailable.
2. Inspect the files, diffs, and tests relevant to the current task; do not automatically scan the entire repository.
3. Update `docs/CURRENT_STATE.md`: objective, implemented/verified work, in progress, blockers, tests, and next action. Prefer concise entries; remove superseded status.
4. Update `docs/DECISIONS.md` only for material decisions with rationale/tradeoffs; update `docs/ARCHITECTURE.md` only if architecture actually changed.
5. Report what changed and what was not verified.

## handoff
1. Perform checkpoint steps.
2. Write `docs/HANDOFF.md` as a stand-alone, concise, actionable recovery note (target ~300–700 words, shorter if sufficient). Include:
   - Timestamp/timezone if known; current goal and task boundary.
   - Git branch/HEAD and uncommitted changes; if Git unavailable, state this.
   - Verified completed work, current issue and precise next action.
   - Only relevant file paths, interfaces, commands, and tests/results.
   - Pending decisions, risks, assumptions and explicit do-not-repeat/do-not-do items.
3. Do not copy full chat history, lengthy error logs, generated files or secrets.
4. Output a one-paragraph new-session prompt that points to `AGENTS.md`, `docs/HANDOFF.md`, and `docs/CURRENT_STATE.md`.
5. Do not automatically start a new conversation or imply that the desktop app has transferred state by itself.

## resume
1. Read repository `AGENTS.md` (as provided by Codex), `docs/HANDOFF.md`, and `docs/CURRENT_STATE.md` if available.
2. Inspect `git status --short`, branch/HEAD and the task-relevant files; reconcile contradictory documentation against the working tree and tests.
3. Read architecture/decisions documentation only when needed. Do not re-read the entire chat or repository.
4. Summarize in 5–10 lines: objective, verified state, local changes, blockers, intended next action and validation plan.
5. Continue the indicated task if the request authorizes it; otherwise stop after the summary. Respect any pending approval and do not silently discard work.

## Quality check
- File paths actually exist or are clearly marked as intended.
- Tests marked passed were actually run and results observed.
- No credentials, tokens, personal data, or private logs copied into the handoff.
- No stale claim is presented as fact.
- Docs are compact enough for a fresh session to read without loading unrelated project history.
