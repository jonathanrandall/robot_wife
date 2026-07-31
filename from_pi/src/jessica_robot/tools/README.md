# tools

Offline utilities for reviewing and improving Jessica's behaviour. No ROS dependency — plain Python 3.

## build_examples.py

Scans the conversation logs (`~/jessica_ws/logs/jessica_*.jsonl`) and pairs `feedback` entries with the `turn` entries they reference, producing a clean examples file for prompt tuning or fine-tuning.

```bash
# Print confirmed good examples as ready-to-paste few-shot text
python3 build_examples.py --prompt

# Write all examples to logs/examples.jsonl
python3 build_examples.py

# Only corrections (label=bad) to review
python3 build_examples.py --label bad

# Only logs on or after a date
python3 build_examples.py --since 2026-07-01
```

Options: `--logs-dir DIR`, `--out FILE`, `--since YYYY-MM-DD`, `--label good|bad|all`, `--prompt`.

### Output format (JSONL)

```json
{"label": "good", "input": "Jessica darling, look left",
 "model_command": {"action": "look", "parameters": {"direction": "left"}},
 "model_say": "Looking left, babe.", "note": "Good girl!",
 "command": {"action": "look", "parameters": {"direction": "left"}}}
```

### Workflow

1. Run `--prompt` and paste confirmed good examples into `SYSTEM_PROMPT` in `jessica_chatbot.py`.
2. Run `--label bad`, fill in `corrected_command` for each, add those as few-shot examples too.
3. Accumulate the JSONL as a fine-tuning seed dataset for later QLoRA.
