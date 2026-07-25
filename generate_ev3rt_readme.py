#!/usr/bin/env python3
"""
generate_ev3rt_readme.py

Replicates, as a series of small Claude Code calls, everything we did
interactively today for EV3RT's `2026base`: an overview, a class diagram, a
sequence diagram per subsystem, an execution-flow flowchart, all merged into
one README.md.

One call per section instead of one giant prompt -- exactly how we actually
built this in conversation -- because asking for all 7 sequence diagrams in a
single reply made them come out over-simplified. Splitting means each diagram
gets the model's full attention, and the merge itself is just local string
concatenation with fixed headers, not another model call.

Uses the Claude Code CLI (`claude -p`), authenticated with your existing
subscription login (`claude login`) rather than a Console API key, so usage
draws from your plan's usage credits instead of separate API billing.

Usage:
    claude login                    # once, if you haven't already

    python3 generate_ev3rt_readme.py \
        --repo /path/to/EV3RT \
        --source-dir 2026base \
        --out README.md

--source-dir defaults to 2026base but can point at any sibling folder with the
same sample.py + py_etrobo_util/ layout (e.g. 2025base) to analyze a different
year's course without editing this file.

--model is optional and passed straight through to `claude --model` (e.g.
sonnet, opus); leave it unset to use your account's default.
"""

import argparse
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap

SOURCE_DIR = "2026base"

SOURCE_FILES = [
    "sample.py",
    "py_etrobo_util/__init__.py",
    "py_etrobo_util/hint.py",
    "py_etrobo_util/plotter.py",
    "py_etrobo_util/util.py",
    "py_etrobo_util/video.py",
]

# One entry per subsystem sequence diagram. Each gets its own call so the
# model can go into full detail instead of compressing seven diagrams into one
# reply.
SUBSYSTEMS = [
    ("Application Bootstrap & Concurrent Main Loop",
     "how the ETRobo behavior-tree tick loop and the VideoThread camera-"
     "capture loop run concurrently, communicating only through shared "
     "globals (g_video, g_plotter)"),
    ("Sensor-Based Line Tracing",
     "the TraceLine behavior: reading the color sensor directly, low-pass "
     "filtering, PID, adaptive speed / gain scheduling"),
    ("Camera-Based Line Tracing",
     "the TraceLineCam behavior: reading theta/tilt/range-of-edges published "
     "by the background camera thread's Video.process(), and how it never "
     "calls process() itself"),
    ("QR Code Detection & Decoding",
     "the concurrency between the capture thread, Video's internal "
     "background detection worker thread, and the IsQRDecoded behavior, "
     "ending in Hint.resolve()/decrypt()"),
    ("Bottle Catching",
     "IsBottleInsight, then CatchBottle's internal state machine, then "
     "HasCaughtBottle, and how CatchBottle's states use the camera then "
     "switch to gyro-only heading hold"),
    ("Gyro-Based Turning",
     "the shared heading-PID pattern behind SpinAround, SpinAndLocateLine, "
     "and RunByGyro, and how the latter two diverge from SpinAround"),
    ("Junction Detection",
     "the IsJunction behavior's small state machine reading the camera's "
     "range-of-edges signal"),
]

MERMAID_RULES = textwrap.dedent("""\
    CRITICAL Mermaid syntax rules (violating these will silently corrupt the
    diagram in real viewers -- these are hard requirements, not style advice):
      - NEVER put a semicolon `;` inside a node label, message text, or note
        text. Use two separate arrows/lines instead of joining them with `;`.
      - NEVER put a bare `<` or `>` comparison operator inside label/message/
        note text (e.g. writing `x < 5` breaks HTML-aware label parsing).
        Spell comparisons out in words instead (e.g. "below 5"). The ONLY
        exception is the literal `<br/>` tag for line breaks inside a label,
        and Mermaid's own arrow/inheritance syntax (`-->`, `->>`, `<|--`)
        outside of label text.
      - Keep every quotation mark balanced within a single label.
      - Use short snake_UPPER or CamelCase node/participant IDs; put the
        human-readable text inside the label/alias instead.
    """)

SYSTEM_PROMPT = textwrap.dedent("""\
    You are a senior software architect producing developer documentation for
    a LEGO EV3 competition robot's control program (EV3RT / py_trees based).

    You will be given the full contents of several Python source files, then
    asked to produce ONE piece of a larger Markdown document. Read the source
    carefully -- do not guess at behavior you can't see in the code, and do
    not invent classes, methods, or constants that aren't present. Everything
    you need is in the user message below -- do not attempt to read, search,
    or modify any files on disk.

    Respond with ONLY the requested piece, in Markdown, ready to be inserted
    as-is into a larger document. Do not add a top-level title, and do not
    wrap your answer in an outer code fence. Any Mermaid diagram must be in
    its own ```mermaid fenced block.

    """) + MERMAID_RULES


def load_source_bundle(repo_root: pathlib.Path, source_dir: str) -> str:
    """Read all target files and bundle them into one clearly-delimited blob."""
    base = repo_root / source_dir
    parts = []
    for rel in SOURCE_FILES:
        path = base / rel
        display = f"{source_dir}/{rel}"
        if not path.exists():
            print(f"warning: {display} not found under {repo_root}, skipping", file=sys.stderr)
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        parts.append(f"===== FILE: {display} =====\n{text}\n")
    if not parts:
        raise SystemExit(f"No source files found under {base}. Check --repo/--source-dir.")
    return "\n".join(parts)


def overview_prompt(source_dir: str, class_diagram: str, sequence_sections: str,
                     execution_flow: str) -> str:
    return (
        f"Here are the sections already written for a developer README covering "
        f"EV3RT's `{source_dir}` (sample.py, a py_trees behavior tree driving a "
        f"competition robot, built on the py_etrobo_util support library):\n\n"
        f"===== CLASS DIAGRAM SECTION =====\n{class_diagram}\n\n"
        f"===== SEQUENCE DIAGRAM SECTIONS =====\n{sequence_sections}\n\n"
        f"===== EXECUTION FLOW SECTION =====\n{execution_flow}\n\n"
        "Now write the 'Overview' section that will introduce the document as a "
        "whole. Synthesize across the sections above -- don't re-derive it "
        "independently from source code. Explain what the program is, how "
        "sample.py relates to py_etrobo_util, and specifically the two "
        "concurrent loops (behavior-tree tick loop and camera-capture thread "
        "loop) that the sequence diagrams above show communicating only via "
        "shared globals.\n\n"
        "Keep it short: at most two brief paragraphs of prose (roughly 150 "
        "words total), optionally followed by a compact 3-row table linking "
        "to the Class Diagram / Sequence Diagrams / Execution Flow sections "
        "with the question each one answers. This is a short orienting "
        "introduction, not a restatement of what's already fully covered in "
        "the diagrams themselves."
    )


def class_diagram_prompt(bundle: str) -> str:
    return (
        f"{bundle}\n\n"
        "Write the 'Class Diagram' section: 2-3 sentences of prose, then ONE "
        "Mermaid classDiagram covering every class/enum defined across the "
        "given files (both py_etrobo_util's support classes and every "
        "Behaviour subclass plus orchestration classes in sample.py). Show "
        "inheritance from external base classes (py_trees.behaviour.Behaviour, "
        "threading.Thread) as stub classes. Show key attributes and method "
        "signatures, not every private helper. Show relationships: uses "
        "(-->), creates/owns (*--), and runtime/global-driven dependency "
        "(..>) for classes that reach a shared global Video/Plotter instance "
        "rather than owning one via composition."
    )


def sequence_diagram_prompt(bundle: str, title: str, hint: str) -> str:
    return (
        f"{bundle}\n\n"
        f"Write ONE subsystem's sequence diagram: '{title}'. It should show "
        f"{hint}. First 2-4 sentences of prose explaining what it shows, then "
        "ONE Mermaid sequenceDiagram in full detail -- this is the only "
        "diagram in this reply, so do not compress or simplify it to save "
        "space. Use `par`/`and` for genuinely concurrent threads, `loop` for "
        "repeated ticks, `alt`/`opt` for branches, and `Note over` for side "
        "commentary."
    )


def execution_flow_prompt(bundle: str) -> str:
    return (
        f"{bundle}\n\n"
        "Write the 'Execution Flow' section. Find the function that builds "
        "the behavior tree (look for the root Sequence and its "
        "add_children calls) and read it in the exact order those calls "
        "execute -- do not just dump the tree structure, interpret how it "
        "actually runs. First a short intro paragraph, then ONE Mermaid "
        "flowchart: a top-to-bottom chain of phases in actual execution "
        "order, using traditional flowchart node shapes throughout (this "
        "matters -- do not put multiple pieces of logic inside one generic "
        "box). For any Parallel/SuccessOnOne composite with a 'driver' child "
        "(runs forever, e.g. steering/moving) and a 'watcher' child (a "
        "sensor check that actually ends the phase), represent it as a small "
        "subgraph containing exactly two nodes: the driver as a RECTANGLE "
        "node (Mermaid `[\"...\"]` syntax) describing the continuous action, "
        "and the watcher as a DIAMOND/decision node (Mermaid `{\"...\"}` "
        "syntax) phrased as a yes/no question about the sensor condition "
        "that ends the phase. Plain sequential steps (a leaf that just runs "
        "to completion on its own, like a spin-to-heading or a wait timer) "
        "are also rectangles, but standalone, outside any subgraph. Style "
        "the race subgraphs distinctly from plain sequential steps and from "
        "start/end (use a Mermaid classDef for each of the three styles). "
        "Finally add a short 'Reading it' section explaining the legend and "
        "the most structurally interesting phase (e.g. any nested race-"
        "within-a-sequence)."
    )


def call_claude_code(prompt: str, system_prompt: str, model: str,
                      timeout: int, claude_bin: str = "claude") -> str:
    """
    Call the local `claude` CLI (Claude Code) in non-interactive print mode
    (`-p`), authenticated with whatever the user is already logged into via
    `claude login` -- their subscription, not a Console API key.

    The prompt is piped over stdin, which `claude -p` reads and appends to
    the prompt when stdin isn't a TTY; only a short instruction is passed as
    the -p argument itself. The system prompt is written to a temp file and
    passed via --system-prompt-file, replacing Claude Code's default coding-
    agent prompt -- appropriate here since this is a one-shot documentation
    task, not an interactive coding session.
    """
    if shutil.which(claude_bin) is None:
        raise SystemExit(
            f"Could not find '{claude_bin}' on PATH.\n"
            f"Install the Claude Code CLI with: curl -fsSL https://claude.ai/install.sh | bash\n"
            f"Then log in once with: claude login"
        )

    with tempfile.NamedTemporaryFile(
        "w", suffix=".txt", delete=False, encoding="utf-8"
    ) as f:
        f.write(system_prompt)
        system_prompt_path = f.name

    try:
        cmd = [
            claude_bin, "-p",
            "Read the content piped below on stdin and respond as instructed.",
            "--system-prompt-file", system_prompt_path,
        ]
        if model:
            cmd += ["--model", model]
        try:
            result = subprocess.run(
                cmd, input=prompt, capture_output=True, text=True, timeout=timeout,
            )
        except subprocess.TimeoutExpired:
            raise SystemExit(f"`claude` did not finish within {timeout}s.") from None
    finally:
        os.remove(system_prompt_path)

    if result.returncode != 0:
        raise SystemExit(
            f"`claude` exited with status {result.returncode}.\n"
            f"stderr:\n{result.stderr.strip()}\n\n"
            f"If this mentions authentication, run `claude login` first."
        )
    if not result.stdout.strip():
        raise SystemExit(f"`claude` produced no output.\nstderr was:\n{result.stderr.strip()}")
    return result.stdout.strip()


def check_mermaid_blocks(markdown: str) -> list[str]:
    """Flag semicolons and stray '<' inside mermaid fenced blocks."""
    problems = []
    blocks = re.findall(r"```mermaid\n(.*?)```", markdown, re.S)
    for i, block in enumerate(blocks, 1):
        for line in block.splitlines():
            stripped = line.strip()
            if stripped.startswith("classDef") or stripped.startswith("class "):
                continue  # legitimate semicolon-terminated statements
            if ";" in line:
                problems.append(f"diagram {i}: semicolon in line: {line.strip()}")
            if "<" in line and "<br/>" not in line and "<|--" not in line and "<<" not in line:
                problems.append(f"diagram {i}: stray '<' in line: {line.strip()}")
    return problems


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", type=pathlib.Path, required=True,
                     help="Path to the local EV3RT checkout (contains SOURCE_DIR)")
    ap.add_argument("--source-dir", default=SOURCE_DIR,
                     help=f"Subdirectory of --repo to read sample.py and "
                          f"py_etrobo_util/ from (default: {SOURCE_DIR!r})")
    ap.add_argument("--model", default=None,
                     help="Optional value passed to `claude --model` (e.g. "
                          "sonnet, opus); left unset uses your account's default")
    ap.add_argument("--claude-bin", default="claude",
                     help="Path to the Claude Code executable (default: claude)")
    ap.add_argument("--timeout", type=int, default=600,
                     help="Timeout in seconds per section call")
    ap.add_argument("--out", type=pathlib.Path, default=pathlib.Path("README.md"),
                     help="Output path for the generated README")
    args = ap.parse_args()

    bundle = load_source_bundle(args.repo, args.source_dir)

    def generate(label: str, prompt: str) -> str:
        print(f"Generating: {label} ...", file=sys.stderr)
        return call_claude_code(prompt, SYSTEM_PROMPT, args.model, args.timeout, args.claude_bin)

    class_diagram = generate("class diagram", class_diagram_prompt(bundle))

    sequence_sections = []
    for i, (title, hint) in enumerate(SUBSYSTEMS, 1):
        reply = generate(f"sequence diagram {i}/{len(SUBSYSTEMS)}: {title}",
                          sequence_diagram_prompt(bundle, title, hint))
        sequence_sections.append(f"### 3.{i} {title}\n\n{reply}")
    sequence_sections_text = "\n\n".join(sequence_sections)

    execution_flow = generate("execution flow", execution_flow_prompt(bundle))

    # Generated last and fed the sections above, so it's a synthesizing
    # overview of what was actually produced -- not an independent read of
    # the source that ends up restating everything the diagrams already show.
    overview = generate("overview", overview_prompt(
        args.source_dir, class_diagram, sequence_sections_text, execution_flow))

    readme = "\n\n---\n\n".join([
        f"# EV3RT `{args.source_dir}` — Architecture Reference\n\n## 1. Overview\n\n{overview}",
        f"## 2. Class Diagram\n\n{class_diagram}",
        "## 3. Sequence Diagrams by Subsystem\n\n" + "\n\n".join(sequence_sections),
        f"## 4. Execution Flow\n\n{execution_flow}",
    ]) + "\n"

    problems = check_mermaid_blocks(readme)
    if problems:
        print("warning: possible Mermaid syntax issues found:", file=sys.stderr)
        for p in problems:
            print(f"  - {p}", file=sys.stderr)
    else:
        print("Mermaid sanity check passed.", file=sys.stderr)

    args.out.write_text(readme, encoding="utf-8")
    print(f"Wrote {args.out}", file=sys.stderr)


if __name__ == "__main__":
    main()
