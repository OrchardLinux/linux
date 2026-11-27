# Agent instructions

This is Orchard's kernel for the 32-bit boards (A5 to A6X): upstream Linux
v6.12 with Project Cascadia's work and Orchard's changes on top (branch
`main` has the arm64 boards). It is the source of the kernel Orchard builds. Kernel work
happens here: branch from `main` or `armv7`, commit, and open the pull request
in OrchardLinux/linux. Never add kernel patch files to the orchard repository.

- Commits in upstream style: one topic each, the subsystem's subject prefix
  (`Input: apple_z2: ...`, `arm64: dts: apple: ...`), `Assisted-by: LLM` when an
  LLM helped. Driver changes and device trees go in separate commits.
- Hardware differences belong in match data and the device tree, selected by
  compatible; no machine checks in shared code.
- Build a branch with orchard: `orchard build ... --kernel-tree DIR`.
- What a board needs beside its kernel (`devices/<board>/`, firmware recipe,
  docs) is a pull request in orchard, which takes the new kernel in the same
  pull request (orchard's `kernel/README.md`). The kernel pull request merges
  first.
- Never rewrite `main` or `armv7`.

LLM-assisted work is welcome here, but it stays in Orchard: no patches, pull
requests, issues or comments to Asahi's or Hoolock's trees.
