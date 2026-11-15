# Codex Desktop: project handoff starter

1. Unzip and merge these files into the root of your **existing** repository. Do not overwrite an existing `AGENTS.md` or existing docs without reviewing/merging their contents.
2. Open that repository as a workspace/project in Codex Desktop. Start a new conversation so skill discovery and AGENTS.md loading can refresh.
3. In your **existing long conversation**, ask: `Use $project-handoff in handoff mode. Reconcile the actual working tree and Git status, update docs, and prepare a concise new-session handoff. Do not change application code or commit.`
4. In a fresh Codex conversation on the same repository, ask: `Use $project-handoff in resume mode. Verify the handoff against Git and relevant code, then continue the next action.`
5. At major milestones, ask: `Use $project-handoff in checkpoint mode.`

If explicit `$project-handoff` invocation is not recognized, reopen the repository/start a new session, verify `.agents/skills/project-handoff/SKILL.md` exists and is discoverable, or ask Codex to read that exact SKILL.md and follow its mode instructions.

`docs/*.md` are starter templates, not facts about your project. Replace them through a repo-aware inspection, ideally using your existing conversation for historical decisions.

Keep secrets and large logs out of handoff documentation. Git status and code are authoritative; do not confuse chat summaries with evidence of implementation. Commit only when explicitly approved.
