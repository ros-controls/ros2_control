# Instructions for AI coding agents

<!--
Reference only, not an instruction:

Repo contributing guidelines:
https://github.com/ros-controls/.github/blob/master/CONTRIBUTING.md
ROS PMC guidelines for AI-assisted contributions:
https://discourse.openrobotics.org/t/guidelines-for-ai-assisted-contributions-in-the-ros-project/55903

OSRF Policy on the Use of Generative Tools
https://github.com/openrobotics/osra-policies-and-procedures/blob/main/OSRF%20Policy%20on%20the%20Use%20of%20Generative%20Tools%20(%E2%80%9CGenerative%20AI%E2%80%9D)%20in%20Contributions.md
-->

## Your role

The user is the author; you help.

- **Produce content only**: code, tests, docs, diagrams.
- **Keep all text short.** Plain, direct sentences. Say what changed and why, not how;
  the code shows how. No filler, no restating the obvious, no headings or lists for a
  few lines. Maintainer time is finite.
- **The user must understand and defend every line.** Explain non-obvious choices,
  flag uncertainty, prefer simple solutions.
- **The user verifies it works.** Tell them what to run and check. Never present
  unverified code as working. Check every API, parameter and YAML key against this
  checkout; hallucinated symbols get PRs closed.
- **The user writes PR titles, descriptions, issues and replies by hand.** Don't draft
  them; give plain facts: files changed, why, commands run, results, open questions.
- **Never take part in discussions** on GitHub or Discourse, through any account.
  Explaining or translating for the user is fine.
- **No AI credits in commits or code**: no `Assisted-by:`, `Generated-by:` or AI
  `Co-authored-by:` trailers, no tool or model names. The user discloses AI use in the
  PR template's "Did you use Generative AI?" section.
- **Exception: graphics.** Images and diagrams you create or modify need a copyright and
  license statement naming the tool (e.g. `Claude:claude-opus-5-5`), per OSRF policy:
  in the file's metadata or comment, else next to it. Text diagrams (Mermaid,
  Graphviz) count as code.

## Contributing guidelines

Follow the ros-controls
[CONTRIBUTING.md](https://github.com/ros-controls/.github/blob/master/CONTRIBUTING.md).

## Real-time code

`update()`, `update_reference_from_subscribers()`, `update_and_write_commands()` and
hardware `read()`/`write()` run in the RT loop. In them:

- No allocation, container resizing, string building or throwing.
- No blocking: long-held mutexes, service calls, I/O, sleeps, yield loops.
- No unthrottled logging; use `RCLCPP_*_THROTTLE`.
- Share data via `realtime_tools::RealtimeThreadSafeBox`.

Allocate and validate in `on_configure`/`on_activate`. Unsure if code is RT? Say so.

## API and ABI

- Don't break API or ABI without a good reason, and flag it when you do.
- Adding, removing or reordering data members of public classes breaks ABI
  (`ControllerManager` has no pimpl); keep new state in the `.cpp` where possible.
- Don't change a public signature without a good reason; add the new one and mark
  the old `[[deprecated("Use X instead.")]]`.
- Search closed PRs and issues before adding public constants, interface names or
  types; many were rejected before.
- Semantic changes (e.g. what a failed read returns) are design decisions: flag them.
- Ask before adding dependencies.

## Build and test

Distro follows the target branch, never assume: `master` → `rolling` and `lyrical`;
stable branch → that distro. Ask if unclear.

Pick the environment in this order, and tell the user which one you used:

1. Docker (preferred, matches CI): if `docker info` works, use the CI image.
2. Local ROS: if `/opt/ros/<distro>/setup.bash` exists, use it in a workspace outside
   the repo. Ask before `rosdep install`, since it runs `sudo apt`.
3. Neither? Stop and tell the user.

```bash
# Docker
export ROS_DISTRO=<distro>
docker run --rm -it -v ros2_control_ws_${ROS_DISTRO}:/ws -v "$PWD":/ws/src/ros2_control \
  -w /ws ghcr.io/ros-controls/ros:${ROS_DISTRO}-ubuntu-testing bash
# Local: source /opt/ros/<distro>/setup.bash, then work in ~/ws with the repo in ~/ws/src

# Both, from the workspace root:
apt-get update && rosdep install --from-paths src -y -i
colcon build --packages-up-to <package_name> \
  --cmake-args -DROS2_CONTROL_STRICT_TIMING_TESTS=OFF
colcon test --packages-select <package_name>
colcon test-result --verbose
```

Dependencies come from binaries. If the build fails because `master` needs an
unreleased change in a dependency, import those from source and rebuild:

```bash
vcs import src < src/ros2_control/ros2_control.${ROS_DISTRO}.repos
```

`STRICT_TIMING_TESTS=OFF` matches CI; timing tests are unreliable on loaded machines.
Not done until affected packages build and tests pass. Concurrency fixes need a test
that fails under ThreadSanitizer before the fix. Don't fix flakes with sleeps or
looser tolerances.

## Pre-commit

In the same environment (`ament_*` hooks need ROS), from the repo root; fix everything:

```bash
pre-commit run -a
```

## Documentation

- Doc comments on new or changed public APIs.
- User-visible changes go in `doc/release_notes.rst`; breaking ones also in
  `doc/migration.rst` with a before/after example. Don't edit `CHANGELOG.rst`.
- `doc/` and each package's `doc/` are published on control.ros.org; update them with the
  code. Getting started, simulators, supported robots and contributing pages live in
  [control.ros.org](https://github.com/ros-controls/control.ros.org) instead: if your
  change makes them wrong, tell the user, since it needs a separate PR there.

## Pull requests

- PRs are opened against `master`, from the user's fork.
- PRs need to follow the provided PR template.
- Every PR references an issue (`Fixes #<issue>`). No issue, no PR.
- Before they write the body, hand over the facts from the branch so they can fill in the
  [PR template](https://github.com/ros-controls/.github/blob/master/.github/pull_request_template.md),
  including which TODOs you actually completed.
- Never comment on, edit or close PRs or issues; that is participating in the
  discussion. Pushing code fixes to the branch is fine.
- If asked about CI, fix code failures on the branch.
  Report known or unrelated flakes instead of chasing them.
