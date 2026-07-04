#!/usr/bin/env python3
"""
build_examples.py — turn Jessica's conversation logs into a clean examples file.

Reads the JSONL logs written by jessica_chatbot.py (default ~/jessica_ws/logs/),
pulls out the spoken-feedback entries (good / bad), pairs each with the `turn`
it graded (via the feedback `ref` -> turn `id` link), and emits tidy examples
you can use to improve Jessica:

  * 'good' feedback  -> confirmed input->command examples, ready for few-shot.
  * 'bad'  feedback  -> corrections to review: the input, what she did, and your
                        spoken note, with a blank `corrected_command` to fill in.

Usage:
  python3 build_examples.py                    # write ~/jessica_ws/logs/examples.jsonl
  python3 build_examples.py --prompt           # print good ones as few-shot text
  python3 build_examples.py --label bad        # only corrections to review
  python3 build_examples.py --since 2026-07-01 # only logs on/after this date
  python3 build_examples.py --logs-dir DIR --out FILE
"""
import argparse
import json
import sys
from pathlib import Path

DEFAULT_LOGS = Path.home() / "jessica_ws" / "logs"


def load_entries(logs_dir: Path, since: str | None):
    """Yield every JSON object from jessica_*.jsonl, oldest file first."""
    for f in sorted(logs_dir.glob("jessica_*.jsonl")):
        # Filenames are jessica_YYYY-MM-DD.jsonl — ISO dates sort as strings.
        if since and f.stem.replace("jessica_", "") < since:
            continue
        with open(f, encoding="utf-8") as fh:
            for ln, line in enumerate(fh, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    yield json.loads(line)
                except json.JSONDecodeError:
                    print(f"  (skipped malformed line {f.name}:{ln})", file=sys.stderr)


def build(logs_dir: Path, since: str | None, label_filter: str):
    """Return a list of example dicts built from feedback entries."""
    turns = {}
    feedback = []
    for e in load_entries(logs_dir, since):
        if e.get("type") == "turn":
            turns[e.get("id")] = e
        elif e.get("type") == "feedback":
            feedback.append(e)

    examples = []
    for fb in feedback:
        label = fb.get("label")
        if label_filter != "all" and label != label_filter:
            continue
        # Prefer the linked turn (has reply_spoken); fall back to the
        # self-contained fields the feedback entry already carries.
        turn = turns.get(fb.get("ref"), {})
        inp = turn.get("heard") or fb.get("orig_heard")
        did = {
            "action":     turn.get("action", fb.get("orig_action")),
            "parameters": turn.get("params", fb.get("orig_params", {})),
        }
        ex = {
            "label":         label,
            "input":         inp,
            "model_command": did,                       # what she actually did
            "model_say":     turn.get("reply_spoken"),
            "note":          fb.get("note"),
        }
        if label == "good":
            ex["command"] = did                          # confirmed target
        else:
            ex["corrected_command"] = None               # fill in from `note`
        examples.append(ex)
    return examples


def to_prompt(examples: list) -> str:
    """Render good examples as User/Assistant few-shot pairs; list bad ones."""
    out = []
    good = [e for e in examples if e["label"] == "good"]
    out.append(f"# {len(good)} confirmed examples (paste into SYSTEM_PROMPT few-shot):\n")
    for e in good:
        assistant = {"say": e.get("model_say") or "", "robot_command": e["command"]}
        out.append(f'User: {e["input"]}')
        out.append(f'Assistant: {json.dumps(assistant, ensure_ascii=False)}')
        out.append("")

    bad = [e for e in examples if e["label"] == "bad"]
    if bad:
        out.append(f"\n# {len(bad)} corrections to review "
                   f"(work out the right command from your note):")
        for e in bad:
            out.append(f'- input:    {e["input"]!r}')
            out.append(f'  she did:  {json.dumps(e["model_command"], ensure_ascii=False)}')
            out.append(f'  your note: {e["note"]!r}')
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(
        description="Build a clean examples file from Jessica's conversation logs.")
    ap.add_argument("--logs-dir", type=Path, default=DEFAULT_LOGS,
                    help=f"log directory (default {DEFAULT_LOGS})")
    ap.add_argument("--out", type=Path, default=DEFAULT_LOGS / "examples.jsonl",
                    help="output JSONL file (default <logs-dir>/examples.jsonl)")
    ap.add_argument("--since", help="only include logs on/after YYYY-MM-DD")
    ap.add_argument("--label", choices=["good", "bad", "all"], default="all",
                    help="which feedback to include (default all)")
    ap.add_argument("--prompt", action="store_true",
                    help="print good examples as few-shot text instead of writing JSONL")
    args = ap.parse_args()

    if not args.logs_dir.exists():
        print(f"No logs directory: {args.logs_dir}", file=sys.stderr)
        sys.exit(1)

    examples = build(args.logs_dir, args.since, args.label)
    if not examples:
        print("No feedback entries found. Give Jessica some 'good girl' / "
              "'that was wrong' feedback first.", file=sys.stderr)
        sys.exit(0)

    if args.prompt:
        print(to_prompt(examples))
    else:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            for e in examples:
                f.write(json.dumps(e, ensure_ascii=False) + "\n")
        good = sum(1 for e in examples if e["label"] == "good")
        bad = sum(1 for e in examples if e["label"] == "bad")
        print(f"Wrote {len(examples)} examples ({good} good, {bad} to review) -> {args.out}")


if __name__ == "__main__":
    main()
