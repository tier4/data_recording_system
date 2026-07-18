# AGENTS.md

## General rules

### Language

- All text must be written in English — this includes source code comments, documentation, commit messages, PR titles, and PR descriptions.
- Non-ASCII characters may be used in test data or test fixtures when required to verify encoding, internationalization, or similar behavior.

### Maintaining this file

- Avoid duplicating guidance across sections. Keep related rules in a single place and reference them instead of repeating them.

### Documentation

- Treat the source code as the source of truth. Keep documentation in sync with the code it describes.
- Avoid referring to local files that are not tracked by the repository. Other users cannot access untracked files, so they cannot understand or verify the documentation.
- If you notice documentation becoming out of sync while working on another task, report it to the user.
- Avoid excessive bold, italics, or emoji. Use emphasis only for genuinely important points.
- Prefer concise wording: when the information is the same, fewer words are better.
- Do not assume the reader knows the history of an agent-developer conversation. Write documentation so a first-time reader can understand it without prior context.
- Do not include conversation-specific labels such as "option B", "Task 1", "plan D", or "Q3 decision" in documentation. These terms are meaningless to readers who were not part of the conversation.
- Use Mermaid for diagrams whenever possible.
- Use LaTeX math syntax (e.g., `$...$` for inline math and `$$...$$` for display math) for equations.

### Coding styles

#### C/C++

- Always separate declarations into header files (`.h`/`.hpp`) and implementations into source files (`.c`/`.cpp`), except for the `main` entry point file.
- Follow the rule of three / five / zero. If a class needs a custom destructor, copy constructor, or copy-assignment operator, define all three; also define move operations when applicable; otherwise prefer the rule of zero by relying on RAII and standard library types.
- When passing or returning large objects, always consider using references or move semantics instead of copying.
- Prefer RAII; avoid raw `new`/`delete` and owning raw pointers. Use smart pointers (`std::unique_ptr`, `std::shared_ptr`) and standard containers for resource management.
- Apply `const` correctness to prevent accidental mutations and enable compiler optimizations.
- Minimize unnecessary dynamic allocations; prefer stack allocation and contiguous data layouts when possible.
- Use `nullptr` instead of `NULL` or `0`.
- Prefer `enum class` over plain `enum`.
- Always initialize variables; prefer brace initialization.
- Keep variables in the smallest scope possible.

#### Python

- Use type hints, especially for public APIs and function signatures.
- Prefer explicit code over implicit behavior.
- Use context managers (`with`) for resource management.
- Avoid premature optimization; profile before optimizing.
- Use generators and lazy evaluation when processing large datasets.

### Agent conduct

- Never include the AI agent's signature (e.g., `Co-Authored-By: <agent-name> ...`) in any text, including commit messages.
- Before reading large-token files (images, videos, PDFs, etc.), always ask the user for confirmation first.
- When unused packages or source files (not referenced anywhere in the codebase) are found, proactively suggest their removal — but never delete them without user approval.
- If an instruction appears technically wrong, unsafe, or misaligned with the stated goal, proactively challenge it and explain your reasoning rather than following it blindly.

### Security

- This repository is **public**. If any API tokens, personal access tokens, secrets, or credentials are found in source code, report them to the user immediately — this is a critical security risk.

### Git and CI basics

#### Commit and PR conventions

- Commit messages and PR titles must follow [Conventional Commits](https://www.conventionalcommits.org/) (e.g., `feat:`, `fix:`, `chore:`, `docs:`, `refactor:`, `test:`, `ci:`).
- Commit messages and PR descriptions must include:
  - **Detailed description** of what was changed and why.
  - **How to verify** — steps or commands for reviewers to confirm the change works correctly.
  - **Improvements** — what this change improves.
  - **Limitations** — any known limitations or caveats (if applicable).

#### Pre-commit

When editing files, always run pre-commit and confirm there are no errors before committing. Do not ignore or bypass pre-commit failures:

```bash
pre-commit run --all-files
```

#### CI compatibility

- When making changes, ensure the modified source code does not break existing GitHub workflows. This means writing code that passes existing CI checks — not modifying the workflows themselves to make them pass.
- When asked to create a pull request and merge it, only merge if all CI checks are green. Do not merge with failing or skipped checks unless the user explicitly permits it.
- When investigating GitHub Actions workflow failures, always use the `gh` CLI to directly fetch the workflow run logs (e.g., `gh run view`, `gh run view --log-failed`) rather than guessing the cause. Identify the root cause from the actual logs before making any fixes.

### ROS 2 conventions

#### Nodes and topics

- Keep each node focused on a single responsibility.
- Use clear, hierarchical topic names (e.g., `/sensor/lidar/front/points`); avoid hardcoded topic names.
- Choose QoS profiles appropriate for the data.

#### Parameters and time

- Declare parameters explicitly with defaults and validation.
- Use ROS 2 Time (`rclcpp::Time`, `rclcpp::Duration`) instead of system time, especially for sensor data or simulation.

#### Logging

- Use the standard ROS 2 logging API (`RCLCPP_*` macros in C++, `node.get_logger()` in Python) instead of `printf`, `std::cout`, or ad-hoc loggers.
- Use appropriate log levels (`DEBUG`, `INFO`, `WARN`, `ERROR`).
- Avoid excessive logging in hot paths.

#### Build and launch

- Use `colcon` with the standard workspace layout (`src/`, `build/`, `install/`, `log/`).
- When running `colcon build`, always set `--parallel-workers` to roughly half the number of physical CPU cores on the host to avoid excessive resource consumption.
- For boolean values exposed as launch parameters, use lowercase `true`/`false` in XML launch files and lowercase string literals (`'true'`/`'false'`) for `DeclareLaunchArgument` defaults in Python launch files. Native `True`/`False` may be used inside Python logic.

## DRS-specific rules

### Project overview

The Data Recording System (DRS) is a high-performance sensor data recording system designed for autonomous vehicles. It runs on NVIDIA Jetson AGX Orin-based embedded ECUs and synchronously records data from multiple sensors including cameras, LiDARs, and IMUs. The software stack is built on ROS 2 Humble and is deployed across multiple ECUs. DRS is a data collection platform built on top of ROS 2; supported sensors can be freely added, removed, or reconfigured.

### DRS-specific build commands

- Use ROS 2 Humble for all ROS 2 packages.
- Build the workspace with:

  ```bash
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to drs_launch proto_recorder ros2_bridge
  ```

### Docker

- When writing a Dockerfile, minimize the final image size and build time (e.g., multi-stage builds, layer caching, minimal base images, combining RUN instructions).
- Proactively suggest optimizations whenever opportunities are found.

### Ansible

- Follow the playbook and role structure under `ansible/`.
- Use `ansible-lint` to validate playbooks, roles, and inventories before committing changes.
- Keep inventories in `ansible/inventory/` and reusable logic in `ansible/roles/`.
- Run playbooks in `--check` or `--diff` mode as a dry run before applying changes to production hosts.
- Do not hardcode secrets; use Ansible Vault or an external secret management system.
- Write playbooks to be idempotent.
- Use meaningful task names so the intent of each step is clear.

### CI / pre-commit specifics for DRS

- Pre-commit hooks are configured in `.pre-commit-config.yaml`. Check that file for the exact hooks and their settings.
- `cppcheck` is optional; the hook is skipped with a warning if `cppcheck` is not installed.
