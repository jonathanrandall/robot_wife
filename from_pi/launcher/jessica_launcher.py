#!/home/jonny/venvs/jazzy/bin/python
"""Jessica's power-on menu: three big touch buttons on the Waveshare 7".

Runs OUTSIDE ROS — it only spawns/stops `ros2 launch` processes, so it can
come up from systemd before anything else and survive stack restarts.

  ┌──────────────────────────────┐
  │   Start Jessica              │  full robot (hardware+cameras+chatbot)
  │   Chatbot Only               │  mic/speakers/display, no ESP32s
  │   Drive Mode                 │  hardware+cameras, no chatbot (gamepad)
  └──────────────────────────────┘

While a stack is running the launcher RELEASES the display (jessica_display
needs the one-and-only KMS/DRM master) but keeps reading raw touch events
from the WaveShare via evdev: hold a finger anywhere for STOP_HOLD_S seconds
to shut the stack down and get the menu back. No SSH needed for anything.

Small corner buttons (each behind a confirm screen): Exit quits the
launcher; Shutdown Pi powers off safely. Shutdown tries `systemctl
poweroff` first, then `sudo -n shutdown`. For it to work without a
password, install a sudoers rule once:

  echo 'jonny ALL=(root) NOPASSWD: /usr/sbin/shutdown' | \
      sudo tee /etc/sudoers.d/010-jessica-shutdown >/dev/null
  sudo chmod 440 /etc/sudoers.d/010-jessica-shutdown

Logs:  ~/jessica_ws/logs/launcher.log        (this program)
       ~/jessica_ws/logs/launcher_stack.log  (the ros2 launch output,
                                              fresh file per start)
"""

import sys
sys.path.insert(0, "/home/jonny/venvs/jazzy/lib/python3.12/site-packages")
sys.path.append("/usr/lib/python3/dist-packages")   # system pygame (kmsdrm)

import glob
import os
import select
import signal
import socket
import subprocess
import time

# Pick the DRM card with a connected display (card0 on the Pi 5 is the render
# node with no connectors; SDL sometimes tries it first and fails).
os.environ.setdefault("SDL_VIDEODRIVER", "kmsdrm")
for _status in sorted(glob.glob("/sys/class/drm/card*-*/status")):
    with open(_status) as _f:
        if _f.read().strip() == "connected":
            _card = os.path.basename(os.path.dirname(_status)).split("-")[0]
            os.environ.setdefault("SDL_VIDEO_KMSDRM_DEVICE_INDEX", _card[-1])
            break

import pygame
from evdev import InputDevice, ecodes

WS = "/home/jonny/jessica_ws"
LOG_DIR = os.path.join(WS, "logs")
TOUCH_DEV = "/dev/input/by-id/usb-WaveShare_WS170120_220211-event-if00"

STOP_HOLD_S = 3.0        # finger held this long while running = stop stack
SIGINT_GRACE_S = 25.0    # ros2 launch gets this long to shut down cleanly
DRM_RETRY_S = 20.0       # how long to wait for jessica_display to free DRM

LAUNCH_PREFIX = (
    "source /opt/ros/jazzy/setup.bash && "
    f"source {WS}/install/setup.bash && "
    "exec ros2 launch jessica_robot jessica.launch.py "
)

BUTTONS = [
    # (title, subtitle, extra launch args)
    ("Start Jessica", "full robot — chatbot, motors, head, cameras", ""),
    ("Chatbot Only", "voice + display only, no motors/cameras",
     "hardware:=false camera:=false tof:=false"),
    ("Drive Mode", "motors, head + cameras, no chatbot (gamepad)",
     "chatbot:=false"),
]

BG = (5, 5, 15)
BUTTON_HUES = (330, 30, 210)          # pink / orange / blue, one per button
HINT = (140, 140, 160)


def log(msg: str):
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {msg}"
    print(line, flush=True)
    try:
        with open(os.path.join(LOG_DIR, "launcher.log"), "a") as f:
            f.write(line + "\n")
    except OSError:
        pass


def hsv(h, s=1.0, v=1.0):
    c = pygame.Color(0)
    c.hsva = (h % 360, s * 100, v * 100, 100)
    return c


def my_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))          # no packets sent — just routes
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "no network"


# ---------------------------------------------------------------------------
# Display handling — the launcher owns DRM only while the menu is visible
# ---------------------------------------------------------------------------

class Screen:
    def __init__(self):
        self.surf = None
        self.fonts = {}

    def acquire(self, timeout=DRM_RETRY_S):
        """(Re)take the display; retries while the old owner lets go of DRM."""
        deadline = time.time() + timeout
        while True:
            try:
                pygame.display.init()
                pygame.font.init()
                self.surf = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
                pygame.mouse.set_visible(False)
                return
            except pygame.error as e:
                pygame.display.quit()
                if time.time() > deadline:
                    raise
                log(f"display busy ({e}) — retrying...")
                time.sleep(1.0)

    def release(self):
        self.surf = None
        self.fonts = {}
        pygame.display.quit()

    def font(self, size):
        if size not in self.fonts:
            self.fonts[size] = pygame.font.SysFont("dejavusans", size, bold=size > 30)
        return self.fonts[size]

    def text(self, s, size, color, center):
        r = self.font(size).render(s, True, color)
        self.surf.blit(r, r.get_rect(center=center))


def draw_menu(scr: Screen, status: str):
    w, h = scr.surf.get_size()
    scr.surf.fill(BG)
    rects = []
    margin, gap = 40, 20
    top, bottom = 16, 120         # bottom: exit/shutdown row + footer text
    bh = (h - top - bottom - 2 * gap) // 3
    for i, (title, sub, _) in enumerate(BUTTONS):
        rect = pygame.Rect(margin, top + i * (bh + gap), w - 2 * margin, bh)
        base = hsv(BUTTON_HUES[i], 0.85, 0.30)
        edge = hsv(BUTTON_HUES[i], 0.9, 0.9)
        pygame.draw.rect(scr.surf, base, rect, border_radius=18)
        pygame.draw.rect(scr.surf, edge, rect, width=4, border_radius=18)
        scr.text(title, 52, edge, (w // 2, rect.centery - 18))
        scr.text(sub, 24, (200, 200, 210), (w // 2, rect.centery + 30))
        rects.append(rect)
    # deliberately small corner buttons — both lead to a confirm screen
    small = {}
    sw, sh, sy = 220, 52, h - 104
    for key, label, x, hue in (("exit", "Exit", margin, None),
                               ("shutdown", "Shutdown Pi", w - margin - sw, 0)):
        rect = pygame.Rect(x, sy, sw, sh)
        edge = (120, 120, 135) if hue is None else hsv(hue, 0.85, 0.75)
        fill = (18, 18, 28) if hue is None else hsv(hue, 0.85, 0.16)
        pygame.draw.rect(scr.surf, fill, rect, border_radius=12)
        pygame.draw.rect(scr.surf, edge, rect, width=3, border_radius=12)
        scr.text(label, 26, edge, rect.center)
        small[key] = rect
    footer = status or f"tap a button to start  ·  hold {STOP_HOLD_S:.0f}s to stop later"
    scr.text(footer, 20, HINT, (w // 2, h - 40))
    scr.text(f"jessica @ {my_ip()}", 20, HINT, (w // 2, h - 16))
    pygame.display.flip()
    return rects, small


def menu(scr: Screen, status: str):
    """Show the menu until something is tapped.
    Returns a BUTTONS index, or "exit" / "shutdown"."""
    rects, small = draw_menu(scr, status)
    clock = pygame.time.Clock()
    w, h = scr.surf.get_size()
    pygame.event.clear()          # drop touches from before/during the stack run
    while True:
        for ev in pygame.event.get():
            pos = None
            if ev.type == pygame.FINGERDOWN:
                pos = (ev.x * w, ev.y * h)
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                pos = ev.pos
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                raise KeyboardInterrupt
            if pos:
                for i, r in enumerate(rects):
                    if r.collidepoint(pos):
                        return i
                for key, r in small.items():
                    if r.collidepoint(pos):
                        return key
        clock.tick(20)


def confirm(scr: Screen, question: str, yes_label: str, hue: int) -> bool:
    """Full-screen are-you-sure. True on yes, False on back-to-menu."""
    w, h = scr.surf.get_size()
    scr.surf.fill(BG)
    scr.text(question, 46, (230, 230, 240), (w // 2, h // 4))
    yes = pygame.Rect(w // 2 - 260, h // 2 - 20, 520, 90)
    back = pygame.Rect(w // 2 - 260, h // 2 + 100, 520, 90)
    for rect, label, edge in ((yes, yes_label, hsv(hue, 0.9, 0.9)),
                              (back, "Back to menu", (150, 150, 165))):
        pygame.draw.rect(scr.surf, (18, 18, 28), rect, border_radius=14)
        pygame.draw.rect(scr.surf, edge, rect, width=4, border_radius=14)
        scr.text(label, 34, edge, rect.center)
    pygame.display.flip()
    pygame.event.clear()
    clock = pygame.time.Clock()
    while True:
        for ev in pygame.event.get():
            pos = None
            if ev.type == pygame.FINGERDOWN:
                pos = (ev.x * w, ev.y * h)
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                pos = ev.pos
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                return False
            if pos:
                if yes.collidepoint(pos):
                    return True
                if back.collidepoint(pos):
                    return False
        clock.tick(20)


def shutdown_pi(scr: Screen) -> str:
    """Safe power-off. Returns a status line for the menu if it fails."""
    log("shutdown requested from touchscreen")
    w, h = scr.surf.get_size()
    scr.surf.fill(BG)
    scr.text("Shutting down...", 48, (230, 230, 240), (w // 2, h // 2 - 20))
    scr.text("safe to switch off when the screen goes dark", 24, HINT,
             (w // 2, h // 2 + 40))
    pygame.display.flip()
    for cmd in (["systemctl", "poweroff"],
                ["sudo", "-n", "/usr/sbin/shutdown", "-h", "now"]):
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode == 0:
            time.sleep(60)        # power disappears mid-sleep
            return ""
        log(f"{' '.join(cmd)} failed: {(r.stderr or r.stdout).strip()}")
    return "shutdown failed — see logs/launcher.log (sudoers rule needed?)"


# ---------------------------------------------------------------------------
# Stack lifecycle
# ---------------------------------------------------------------------------

def start_stack(extra_args: str) -> subprocess.Popen:
    logfile = open(os.path.join(LOG_DIR, "launcher_stack.log"), "w")
    proc = subprocess.Popen(
        ["bash", "-c", LAUNCH_PREFIX + extra_args],
        stdout=logfile, stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        start_new_session=True,           # own process group -> killpg gets ALL nodes
        cwd=WS,
    )
    logfile.close()                       # child keeps its own handle
    return proc


def stop_stack(proc: subprocess.Popen) -> int:
    """SIGINT the whole group, escalate if needed. Returns exit code."""
    pgid = proc.pid                       # == pgid thanks to start_new_session
    for sig, grace in ((signal.SIGINT, SIGINT_GRACE_S),
                       (signal.SIGTERM, 5.0), (signal.SIGKILL, 3.0)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            break
        deadline = time.time() + grace
        while time.time() < deadline:
            if proc.poll() is not None:
                # leader gone — sweep any stragglers left in the group
                try:
                    os.killpg(pgid, 0)
                except ProcessLookupError:
                    return proc.returncode
                time.sleep(0.5)
            else:
                time.sleep(0.5)
        log(f"stack still up after {sig.name}, escalating")
    proc.poll()
    return proc.returncode if proc.returncode is not None else -1


def watch_stack(proc: subprocess.Popen) -> str:
    """Display-less wait: stack runs until it dies or a long-press stops it."""
    try:
        touch = InputDevice(TOUCH_DEV)
    except OSError as e:
        touch = None
        log(f"touch device unavailable ({e}) — stop only via SSH this session")

    held_since = None
    while True:
        if proc.poll() is not None:
            stop_stack(proc)   # sweep any nodes the dying launch left behind
            return f"stack exited (code {proc.returncode}) — see logs/launcher_stack.log"

        if touch is None:
            time.sleep(1.0)
            continue
        r, _, _ = select.select([touch.fd], [], [], 0.5)
        try:
            if r:
                for ev in touch.read():
                    if ev.type == ecodes.EV_KEY and ev.code in (
                            ecodes.BTN_TOUCH, ecodes.BTN_LEFT):
                        held_since = time.monotonic() if ev.value else None
        except OSError:
            touch = None                  # unplugged mid-run; keep watching proc
            continue
        if held_since and time.monotonic() - held_since >= STOP_HOLD_S:
            log("long-press detected — stopping stack")
            code = stop_stack(proc)
            return f"stopped by long-press (exit {code})"


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    log("=== jessica launcher up ===")
    scr = Screen()
    scr.acquire()
    status = ""
    running = None
    # systemd stop / Ctrl+C: take the stack down with us, never orphan it
    def on_term(_sig, _frm):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)

    try:
        while True:
            choice = menu(scr, status)
            if choice == "exit":
                if confirm(scr, "Exit the launcher?", "Yes — exit", 30):
                    log("exit via touchscreen")
                    break
                status = ""
                continue
            if choice == "shutdown":
                if confirm(scr, "Shut down the Pi?", "Yes — shut down", 0):
                    status = shutdown_pi(scr)
                else:
                    status = ""
                continue
            title, _, args = BUTTONS[choice]
            log(f"button: {title!r}  args: {args!r}")
            w, h = scr.surf.get_size()
            scr.surf.fill(BG)
            scr.text(f"Starting {title}...", 48, hsv(BUTTON_HUES[choice], 0.9, 0.9),
                     (w // 2, h // 2))
            scr.text(f"hold {STOP_HOLD_S:.0f}s anywhere to stop",
                     22, HINT, (w // 2, h // 2 + 60))
            pygame.display.flip()
            time.sleep(1.5)               # let the message be seen
            scr.release()                 # hand DRM to jessica_display

            running = start_stack(args)
            status = watch_stack(running)
            running = None
            log(status)
            scr.acquire()                 # jessica_display may take a moment to die
    except KeyboardInterrupt:
        log("launcher shutting down")
    finally:
        if running and running.poll() is None:
            log("stopping stack before exit")
            stop_stack(running)
        pygame.quit()


if __name__ == "__main__":
    main()
