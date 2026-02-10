# CLAUDE.md

All text must be written in English.

## General rules

- Never include Claude's signature (e.g., `Co-Authored-By: Claude ...`) in any text, including commit messages.
- Before reading large-token files (images, videos, PDFs, etc.), always ask the user for confirmation first.
- When unused packages or source files (not referenced anywhere in the codebase) are found, proactively suggest their removal — but never delete them without user approval.

## Security

- This repository is **public**. If any API tokens, personal access tokens, secrets, or credentials are found in source code, report them to the user immediately — this is a critical security risk.

## Coding style

### C/C++

- Always separate declarations into header files (`.h`/`.hpp`) and implementations into source files (`.c`/`.cpp`), except for the `main` entry point file.

### Docker

- When writing a Dockerfile, minimize the final image size and build time (e.g., multi-stage builds, layer caching, minimal base images, combining RUN instructions).
- Proactively suggest optimizations whenever opportunities are found.

## Git and CI

### Commit and PR conventions

- Commit messages and PR titles must follow [Conventional Commits](https://www.conventionalcommits.org/) (e.g., `feat:`, `fix:`, `chore:`, `docs:`, `refactor:`, `test:`, `ci:`).
- Commit messages and PR descriptions must include:
  - **Detailed description** of what was changed and why.
  - **How to verify** — steps or commands for reviewers to confirm the change works correctly.
  - **Improvements** — what this change improves.
  - **Limitations** — any known limitations or caveats (if applicable).

### Pre-commit

When editing files, always run pre-commit against all config files matching `.pre-commit-config*` and confirm there are no errors before committing. Run pre-commit with each config file:

```bash
pre-commit run --all-files -c .pre-commit-config.yaml
pre-commit run --all-files -c .pre-commit-config-optional.yaml
pre-commit run --all-files -c .pre-commit-config-ansible.yaml
```

### CI compatibility

When making changes, ensure the modified source code does not break existing GitHub workflows. This means writing code that passes existing CI checks — not modifying the workflows themselves to make them pass.
