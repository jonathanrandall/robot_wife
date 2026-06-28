import io
import os
import time
import wave
import logging

import uvicorn
from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import JSONResponse
from faster_whisper import WhisperModel

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)

MODEL_SIZE = os.environ.get("WHISPER_MODEL", "small.en")
PORT = int(os.environ.get("WHISPER_PORT", 8765))

log.info("Loading Whisper model '%s' (CUDA, float16)...", MODEL_SIZE)
model = WhisperModel(MODEL_SIZE, device="cuda", compute_type="float16")
log.info("Model loaded.")

app = FastAPI()


def _is_silent(wav_bytes: bytes) -> bool:
    """Return True if the WAV contains no frames or zero-length audio."""
    try:
        with wave.open(io.BytesIO(wav_bytes)) as wf:
            return wf.getnframes() == 0
    except Exception:
        return False


@app.post("/transcribe")
async def transcribe(request: Request):
    body = await request.body()
    if not body:
        return JSONResponse({"text": ""})

    if _is_silent(body):
        log.info("Received silent/empty audio, returning empty text.")
        return JSONResponse({"text": ""})

    t0 = time.perf_counter()
    try:
        audio_file = io.BytesIO(body)
        segments, _info = model.transcribe(audio_file, language="en")
        text = " ".join(seg.text.strip() for seg in segments).strip()
    except Exception as exc:
        log.exception("Transcription failed: %s", exc)
        raise HTTPException(status_code=500, detail="Transcription error")

    elapsed = time.perf_counter() - t0
    log.info("Transcribed in %.2fs: %r", elapsed, text)
    return JSONResponse({"text": text})


@app.get("/health")
async def health():
    return JSONResponse({"status": "ok"})


if __name__ == "__main__":
    print(f"\n  Whisper STT server listening on  http://0.0.0.0:{PORT}\n")
    uvicorn.run(app, host="0.0.0.0", port=PORT)
