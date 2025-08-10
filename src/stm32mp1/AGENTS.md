## Mission
Implement: iwdg initialization

## Branch & Scope
- Branch: `codex_iwdg`
- Code areas to touch: `plat.zig`, `stm32mp157c.zig`
- Out of scope: enything else

## Hard Constraints (no-run environment)
- Do **not** run or install anything (`zig build`, `zig test`, `zig fmt`, package installs).
- Keep edits **text-only** (code, docs, examples, diffs).

## Implementation Requirements
- Instances of objects: instances of peiphery live in plat.zig 
- Documentaion of code: refer to document numbers in the beginning of files and add <page>[<id>] similar to existing refs within the code, mainly for registers/fields/bits

## Repository Map (for orientation only)
- Source: `./`

## Acceptance Criteria
- No edits outside declared scope.
- Provide a **unified diff** and a PR-ready description (below).

## Guardrails / Do Not
- Don’t rename existing public types without explicit note.
- Don’t add dependencies or codegen steps.

## Notes to Codex
- Prefer conservative changes; keep diffs small.
- If ambiguity arises, leave a TODO comment and proceed with the least risky choice.
