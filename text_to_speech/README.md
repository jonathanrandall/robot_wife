# text_to_speech — Laptop-Side Servers

FastAPI servers that run on the GPU laptop and handle speech recognition and text-to-speech synthesis for Jessica. The Pi offloads these tasks over HTTP to keep latency low.

## whisper_server.py — Speech-to-Text (port 8765)

Uses `faster-whisper` with CUDA for fast, accurate transcription.

```bash
pip install fastapi uvicorn faster-whisper
python whisper_server.py
```

Environment variables:
- `WHISPER_MODEL` — model size (default: `small.en`). Options: `tiny.en`, `base.en`, `medium.en`, `large-v3`
- `WHISPER_PORT` — port (default: `8765`)

**API:**
- `POST /transcribe` — body: WAV bytes → `{"text": "transcribed text"}`
- `GET /health` → `{"status": "ok"}`

## tts_server.py — Text-to-Speech (port 8766)

Uses Piper TTS. The voice model (`en_US-ljspeech-medium.onnx`) is bundled in `voices/`.

```bash
pip install fastapi uvicorn piper-tts numpy
python tts_server.py
```

Environment variables:
- `TTS_VOICE_MODEL` — path to `.onnx` voice model (default: bundled ljspeech model)
- `TTS_PORT` — port (default: `8766`)

**API:**
- `POST /synthesize` — body: `{"text": "Hello love"}` → WAV audio bytes
- `GET /health` → `{"status": "ok"}`

## Pointing the Pi at these servers

Update `WHISPER_URL`, `TTS_URL`, and `OLLAMA_URL` at the top of `jessica_chatbot.py` to your PC's LAN IP address.
