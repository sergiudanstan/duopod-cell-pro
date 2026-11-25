# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-11-25

First tagged release of the DuoPod transfer cell Pro digital twin as published at https://sergiudanstan.github.io/duopod-cell-pro/: the duopod-cell scene extended with rigid-body dynamics, motor and vacuum models, a smart cycle, a scale optimiser, a what-if sweep, a PLC I/O list and an MuJoCo physics replay.

### Added
- `package.json` as the single version source, with `playwright-core` (test tooling only).
- Headless page smoke test (`node tests/smoke.mjs`) run in CI on every push and pull request.
- CI runs the existing logic tests (cell logic checks (kinematics, range, cartons, clearances, dynamics, motor, vacuum, smart cycle, optimiser, what-if sweep, physics replay)), their negative probes, and a check that the committed `index.html` equals a fresh build from `src/`.
- CI, CodeQL, OpenSSF Scorecard and a tag-triggered release workflow with build
  provenance; Dependabot for actions and npm.
- `SECURITY.md`, `CONTRIBUTING.md` and this changelog.

[Unreleased]: https://github.com/sergiudanstan/duopod-cell-pro/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/sergiudanstan/duopod-cell-pro/releases/tag/v0.1.0
