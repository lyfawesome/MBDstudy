# MBDstudy Agent Guide

This repository is a long-term learning and implementation project for multibody dynamics.

## Project Goals

- Start from physical assumptions and mathematical equations.
- Explain each concept through physical meaning, mathematical meaning, and engineering use.
- Convert each new concept into Python code that can become part of an extensible solver.
- Pair every runnable case with visualization, trend analysis, and a code implementation document.
- Iterate the learning order based on the learner's feedback.

## Documentation Rules

- Theory documents live in `docs/`.
- Every runnable example must have a matching code implementation document.
- Code implementation documents must connect formulas to code objects, array dimensions, functions, classes, and non-obvious Python syntax.
- Inline short formulas should use `$...$` for compatibility with VS Code's built-in Markdown preview.
- Complex formulas, vectors, matrices, derivatives, and constraints should use standalone `$$...$$` blocks.
- Do not wrap mathematical variables or equations in backticks unless discussing literal code.

## Code Rules

- Keep code clear, teachable, and extensible.
- Prefer explicit numerical code that mirrors the equations.
- Avoid abstractions until multiple examples really need the shared structure.
- Examples must be runnable from the repository root.
- Run the relevant example after changing solver code.

## Current Structure

- `docs/`: learning notes, derivations, roadmap, and case implementation notes.
- `mbd/`: reusable Python solver code.
- `examples/`: runnable examples for each lesson.
- `outputs/`: generated plots and result files.

## Recommended Commands

Use the conda environment `mbd-study` when working locally.

Run the first example from the repository root:

```powershell
python -m examples.lesson_01_free_fall
```

Cloud environments should install at least:

```bash
pip install numpy matplotlib
```
