import io
import logging
import os
import time
import wave

import numpy as np

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import Response
from piper.voice import PiperVoice
from pydantic import BaseModel

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)

PORT = int(os.environ.get("TTS_PORT", 8766))

_here = os.path.dirname(os.path.abspath(__file__))
VOICE_MODEL = os.environ.get(
    "TTS_VOICE_MODEL",
    os.path.join(_here, "voices", "en_US-ljspeech-medium.onnx"),
)

log.info("Loading Piper voice from %s ...", VOICE_MODEL)
voice = PiperVoice.load(VOICE_MODEL)
log.info("Voice loaded.")

app = FastAPI()


class SynthRequest(BaseModel):
    text: str


@app.post("/synthesize")
async def synthesize(req: SynthRequest):
    if not req.text.strip():
        raise HTTPException(status_code=400, detail="text is empty")

    t0 = time.perf_counter()
    try:
        buf = io.BytesIO()
        with wave.open(buf, "wb") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)  # 16-bit
            wav.setframerate(voice.config.sample_rate)
            for chunk in voice.synthesize(req.text):
                pcm = (chunk.audio_float_array * 32767).astype(np.int16)
                wav.writeframes(pcm.tobytes())
        wav_bytes = buf.getvalue()
    except Exception as exc:
        log.exception("Synthesis failed: %s", exc)
        raise HTTPException(status_code=500, detail="Synthesis error")

    elapsed = time.perf_counter() - t0
    log.info("Synthesized %d chars in %.2fs (%d bytes)", len(req.text), elapsed, len(wav_bytes))
    return Response(content=wav_bytes, media_type="audio/wav")


@app.get("/health")
async def health():
    from fastapi.responses import JSONResponse
    return JSONResponse({"status": "ok"})


if __name__ == "__main__":
    print(f"\n  Piper TTS server listening on  http://0.0.0.0:{PORT}\n")
    uvicorn.run(app, host="0.0.0.0", port=PORT)
