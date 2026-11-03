#!/usr/bin/env python3
"""End-to-end test: amqtt broker + the sketch running in the host harness +
paho-mqtt client sending commands. Verifies connect, LWT, subscriptions,
command replies, state JSON, keep-alive, and reconnect after a broker restart."""
import asyncio, json, os, subprocess, sys, threading, time
from amqtt.broker import Broker
import paho.mqtt.client as mqtt

PREFIX = "clearcore/leds"
HERE = os.path.dirname(os.path.abspath(__file__))
STOP = os.path.join(HERE, "build", "stop_harness")
HARNESS = os.path.join(HERE, "build", "harness_mqtt")
failures = []

def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    if not cond: failures.append(msg)

# ---------------- broker in a background asyncio loop
loop = asyncio.new_event_loop()
threading.Thread(target=loop.run_forever, daemon=True).start()
def broker_config(port):
    return {"listeners": {"default": {"type": "tcp", "bind": f"127.0.0.1:{port}", "max_connections": 10}},
            "sys_interval": 0, "auth": {"allow-anonymous": True, "plugins": ["auth_anonymous"]},
            "topic-check": {"enabled": False}}
def start_broker(port=1883):
    b = Broker(broker_config(port), loop=loop)
    asyncio.run_coroutine_threadsafe(b.start(), loop).result(10)
    return b
def stop_broker(b):
    asyncio.run_coroutine_threadsafe(b.shutdown(), loop).result(10)

broker = start_broker()
time.sleep(0.5)

# ---------------- observer client
msgs = []          # (topic, payload)
lock = threading.Lock()
def on_message(c, u, m):
    with lock: msgs.append((m.topic, m.payload.decode(errors="replace"), time.time()))
obs = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="observer")
obs.on_message = on_message
obs.connect("127.0.0.1", 1883, 60)
obs.subscribe(PREFIX + "/#")
obs.loop_start()

def wait_for(pred, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        with lock:
            for m in list(msgs):
                if pred(m): return m
        time.sleep(0.05)
    return None
def clear():
    with lock: msgs.clear()
def last_state(timeout=3):
    m = wait_for(lambda m: m[0] == PREFIX + "/state", timeout)
    return json.loads(m[1]) if m else None
def send(cmd, expect_reply=None, timeout=3):
    clear()
    obs.publish(PREFIX + "/cmd", cmd)
    r = wait_for(lambda m: m[0] == PREFIX + "/reply", timeout)
    reply = r[1] if r else None
    if expect_reply is not None:
        check(reply == expect_reply, f"'{cmd}' -> reply {reply!r} (expected {expect_reply!r})")
    return reply

# ---------------- start the sketch
if os.path.exists(STOP): os.remove(STOP)
harness = subprocess.Popen([HARNESS, STOP, "150"], cwd=os.path.join(HERE, "build"),
                           stdout=open(os.path.join(HERE, "build", "harness.log"), "w"), stderr=subprocess.STDOUT)
m = wait_for(lambda m: m[0] == PREFIX + "/status" and m[1] == "online", 10)
check(m is not None, "status 'online' published after connect")
st = last_state(5)
check(st is not None and st.get("effect") == "rainbow", f"boot state is rainbow: {st}")
check(st is not None and st.get("segments") == 105, "state reports 105 segments")

# ---------------- commands
send("chase #ff4000 speed=80 size=12", "ok chase")
st = last_state(); check(st and st["effect"] == "chase" and st["c1"] == "#ff40000000" and st["speed"] == 80.0 and st["size"] == 12, f"chase state {st}")
send("white 3000 180", "ok white")
st = last_state(); check(st and st["effect"] == "white" and st["kelvin"] == 3000, f"white state {st}")
send("brightness 100", "ok brightness 100")
st = last_state(); check(st and st["brightness"] == 100, f"brightness state {st}")
send("alternate red green rate=0.5 size=6", "ok alternate")
st = last_state(); check(st and st["c1"] == "#ff00000000" and st["c2"] == "#00ff000000" and st["rate"] == 0.5 and st["size"] == 6, f"alternate state {st}")
send("bogus 1 2", "error unknown command 'bogus'")
send("chase purple banana", "error don't understand 'banana'")
send("solid 10,20,30,40,50", "ok solid")
st = last_state(); check(st and st["c1"] == "#0a141e2832", f"5-channel colour {st}")
send("police", "ok police")
send("fire 2", "ok fire")
send("sparkle white #000020 rate=40", "ok sparkle")
st = last_state(); check(st and st["c2"] == "#0000200000" and st["rate"] == 40.0, f"sparkle state {st}")
send("speed 55", "ok speed 55")
send("dir rev", "ok dir rev")
st = last_state(); check(st and st["speed"] == 55.0 and st["dir"] == -1, f"tweak state {st}")
send("off", "ok off")
st = last_state(); check(st and st["effect"] == "off", f"off state {st}")
send("on", "ok on sparkle")
st = last_state(); check(st and st["effect"] == "sparkle", f"on restores sparkle {st}")
send("cycle 2", "ok cycle every 2 s")
time.sleep(2.5); clear(); send("status", "ok status")
st = last_state(); check(st and st["cycle"] is True and st["effect"] in ("rainbow", "chase"), f"cycle running {st}")
time.sleep(2.2); clear(); send("status", "ok status")
st2 = last_state(); check(st2 and st2["effect"] != st["effect"], f"cycle advanced {st['effect']} -> {st2['effect']}")
# rendering sanity from the harness dump
time.sleep(1.2)
dump = open(os.path.join(HERE, "build", "frame_dump.txt")).read().strip()
check("fx=" in dump, f"frame dump: {dump[:80]}")
send("solid red", "ok solid"); time.sleep(1.2)
dump = open(os.path.join(HERE, "build", "frame_dump.txt")).read().strip()
check("fx=solid" in dump and "[255,0,0,0,0]" in dump, f"solid red rendered: {dump[:90]}")
st = last_state(); check(st and st["fps"] >= 100, f"fps {st and st.get('fps')}")

# ---------------- keep-alive: 20 s of silence must not drop the session (keepalive 30 s, ping at 15 s)
clear(); time.sleep(20)
check(wait_for(lambda m: m[0] == PREFIX + "/status" and m[1] == "offline", 0.1) is None, "no LWT during 20 s idle (ping keeps session alive)")
send("status", "ok status")
check(last_state() is not None, "still responsive after idle")

# ---------------- broker restart: client must reconnect and re-announce
clear()
stop_broker(broker)
time.sleep(2)
broker = start_broker()
time.sleep(0.5)
obs.loop_stop(); obs.disconnect()
obs = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="observer2"); obs.on_message = on_message
obs.connect("127.0.0.1", 1883, 60); obs.subscribe(PREFIX + "/#"); obs.loop_start()
m = wait_for(lambda m: m[0] == PREFIX + "/status" and m[1] == "online", 20)
check(m is not None, "reconnected and re-announced after broker restart")
send("marquee red white speed=12", "ok marquee")

# ---------------- LWT: kill the sketch without DISCONNECT
clear()
harness.kill(); harness.wait()
m = wait_for(lambda m: m[0] == PREFIX + "/status" and m[1] == "offline", 10)
check(m is not None, "last-will 'offline' delivered after the client vanished")

obs.loop_stop()
stop_broker(broker)
print("RESULT:", "FAILED %d" % len(failures) if failures else "PASSED")
sys.exit(1 if failures else 0)
