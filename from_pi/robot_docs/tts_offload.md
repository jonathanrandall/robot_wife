# TTS offload to the PC — design notes (not yet implemented)

Status 2026-07-19: **design agreed, nothing changed in the robot code yet.**
Piper currently runs locally on the Pi inside `text_to_speech()` in
`jessica_chatbot.py`, synthesizing the whole reply before playback. On the
Pi 5 that is the dominant chunk of the listening→talking gap now that
Whisper and Ollama run on the PC GPU (both sub-second there).

## Why offload

- **Latency**: PC synthesis is a fraction of a second vs seconds on the Pi.
- **CPU headroom**: Piper is one of the heaviest CPU spikes on the Pi (which
  already had one CPU-starvation incident: JPEG encoding starving the mic).
- **Better voices later**: the PC can run larger Piper voices (or a different
  TTS engine entirely) that the Pi can't do in real time.
- **Pi 4 viability** (user's point, 2026-07-19): with STT, LLM and TTS all
  on the PC, the Pi is just audio I/O + motors + cameras + display — light
  enough that the whole project could run on a Pi 4, making it more
  accessible to rebuild. (Local-Piper fallback would be slow there, but
  it's only a fallback.)
- **Cheap architecturally**: same pattern as the existing Whisper service —
  one more HTTP endpoint on the PC. Speech only ever happens as a response
  to Ollama, which already needs the PC, so the added dependency is marginal.

## Agreed design (Pi side)

Keep both paths in the package, selected by a **preference with automatic
fallback** — never a hard switch:

- New constant/param in `jessica_chatbot.py` next to `WHISPER_URL`:

  ```python
  TTS_URL = "http://192.168.1.106:8766/synthesize"   # "" = always local Piper
  ```

- `text_to_speech(text, wav_path)` is the **only** seam. New behaviour:
  1. If `TTS_URL` is set: `requests.post(TTS_URL, json={"text": text}, timeout=2.0)`,
     write `response.content` (WAV bytes) to `wav_path`.
  2. On **any** failure (timeout, refused, non-200, empty body): log one
     warning and fall back to local Piper — the robot degrades to "slower
     speech", never to mute.
- Everything downstream is untouched: `_publish_speech_envelope()` and
  `play_wav()` read the same WAV file regardless of where it was made.
- Keep the lazy local Piper load as-is (fallback stays warm-able, and
  `TTS_URL=""` behaves exactly like today).
- The PC service must use the **same voice** as the Pi so she doesn't change
  accent when the fallback kicks in: `en_US-ljspeech-medium`.

Optional later: a `voice` field in the request JSON for per-utterance voice
switching, and a ROS param instead of a constant so it can be flipped at
runtime.

## What needs to be done on the PC

The PC already runs the Whisper service on port 8765; this mirrors it on
port **8766**.

1. Install Piper into whatever Python environment the Whisper service uses:
   `pip install piper-tts` (plus `flask` or `fastapi`/`uvicorn`, matching
   the Whisper service's framework).
2. Download the same voice the Pi uses:
   `en_US-ljspeech-medium.onnx` + its `.json` config
   (from https://huggingface.co/rhasspy/piper-voices — path
   `en/en_US/ljspeech/medium/`).
3. Run an HTTP service on **0.0.0.0:8766** with endpoint `POST /synthesize`:
   - Request: JSON `{"text": "what to say"}`
   - Response: `200` with body = complete WAV file bytes
     (`Content-Type: audio/wav`), i.e. exactly what
     `piper_voice.synthesize_wav()` produces.
   - Errors: any non-200 is fine — the Pi falls back to local Piper.
4. Load the voice **once at startup** (not per request) — this is where the
   speed comes from.
5. Start it alongside the Whisper service (same autostart mechanism).
6. Test from the Pi:

   ```bash
   curl -s -X POST http://192.168.1.106:8766/synthesize \
        -H "Content-Type: application/json" \
        -d '{"text": "Hello Jonny, this is a test."}' -o /tmp/tts_test.wav
   aplay /tmp/tts_test.wav
   ```

## Prompt for Claude running on the PC

> I have a small HTTP service on this PC that serves Whisper transcription
> on port 8765 for my robot (a Raspberry Pi at another IP on my LAN). Find
> it first (check how it's implemented, what framework it uses, and how it
> is started on boot) and then build a matching TTS service next to it:
>
> - Piper TTS (`piper-tts` Python package), voice `en_US-ljspeech-medium`
>   (download the .onnx and .onnx.json from rhasspy/piper-voices on
>   Hugging Face). Load the voice once at startup, not per request.
> - HTTP endpoint: `POST /synthesize` on `0.0.0.0:8766`. Request body is
>   JSON `{"text": "..."}`. Response is the complete WAV file bytes with
>   `Content-Type: audio/wav` — the same output `synthesize_wav()` writes.
>   Return a non-200 status on any error; the caller has a 2 s timeout and
>   falls back to its own local TTS, so never block for long.
> - Use the same framework and the same autostart mechanism as the Whisper
>   service so both come up together after a reboot.
> - When it's running, verify end-to-end with:
>   `curl -s -X POST http://localhost:8766/synthesize -H "Content-Type: application/json" -d '{"text": "test"}' -o /tmp/t.wav`
>   and confirm `/tmp/t.wav` is a valid WAV (`file /tmp/t.wav`), then tell
>   me the exact URL my robot should use.

## Pi-side change checklist (when the PC service exists)

- [ ] Add `TTS_URL` next to `WHISPER_URL` in `jessica_chatbot.py`.
- [ ] Wrap `text_to_speech()` with the try-PC-then-fallback logic.
- [ ] Rebuild `jessica_robot`, test a conversation with the PC service up.
- [ ] Test the fallback: stop the PC service mid-session, confirm she still
      talks (slower) and a single warning is logged.
- [ ] Update `documentation.md` (topics/packages unchanged — just the
      chatbot appendix + PC-side services list).
