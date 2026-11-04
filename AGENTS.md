# Instructions for AI coding agents

This file guides AI coding assistants (Codex, Copilot, Cursor, Claude Code, Gemini CLI, ...)
working on this repository. Humans should read the
[contributing guidelines](https://github.com/ros-controls/.github/blob/master/CONTRIBUTING.md)
instead; everything below follows from them.

## Contributing guidelines

Follow the ros-controls
[CONTRIBUTING.md](https://github.com/ros-controls/.github/blob/master/CONTRIBUTING.md).
The rules agents most often break:

- **Limited scope.** One PR does one thing. Do not add unrelated fixes or reformat untouched code.
- **New code = new tests.** Every new feature or bug fix comes with tests that exercise it.
- **Target `master`.** Never base work on a distro branch (`humble`, `jazzy`, ...);
  maintainers decide about backports.
- **Do not break API/ABI** unless the task requires it, and say so in the PR if it does.
- **No force pushes** once a PR has been reviewed.

## Build and test

Run from the colcon workspace root, with ROS 2 sourced:

```bash
colcon build --packages-select <package_name>
colcon test --packages-select <package_name>
colcon test-result --verbose
```

A change is not done until the affected packages build and their tests pass.

## Pre-commit

Run from the repository root and fix everything it reports:

```bash
pre-commit run -a
```

It enforces formatting (clang-format, black), linters (cpplint, cppcheck, flake8),
copyright headers, rst style, and spelling. Never bypass it with `--no-verify`.

## Documentation

- Document new or changed public APIs with doc comments.
- User-visible changes go into `doc/release_notes.rst`; breaking changes also into
  `doc/migration.rst`.
- General documentation lives in
  [control.ros.org](https://github.com/ros-controls/control.ros.org).

## Pull requests

Only open a PR when the user asks for it. When you do:

- Open it from the user's fork against `master`.
- Use a descriptive title.
- Fill in the body using the
  [ros-controls PR template](https://github.com/ros-controls/.github/blob/master/.github/pull_request_template.md)
  and keep all of its sections:
  - **Description**: what changed and why, plus `Fixes #<issue>` if there is one.
  - **Is this user-facing behavior change?**: explain it, or leave it empty if not.
  - **Did you use Generative AI?**: always fill this in. Name the tool and model you are
    (e.g. "Claude Code, Claude Opus 5.5") and which parts you generated.
  - **Additional Information**: anything reviewers need to know.
  - **TODOs**: keep the checklist and tick only the items that are actually done.
    For example, tick the tests item only if you ran `colcon test` and `pre-commit run`
    and they passed.
- After opening, check CI and fix any failures.
