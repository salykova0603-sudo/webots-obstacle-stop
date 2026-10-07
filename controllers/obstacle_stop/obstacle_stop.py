"""
Контроллер «obstacle_stop» для Pioneer 3-DX + Lidar (Webots R2023b и новее).

Параметры передаются через поле controllerArgs робота:
    <задержка, с>  <порог, м>  <метка запуска>
Например:  "0.2"  "0.40"  "A2"

Каждый запуск пишет файл run_<метка>.csv в папку контроллера.
"""
from controller import Robot
import math
import csv
import sys

# ---------- Исходные условия ----------
V0 = 0.5                    # начальная скорость, м/с
A = 0.5                     # заданное замедление, м/с²
R = 0.0975                  # радиус колеса Pioneer 3-DX, м
L = 0.03                    # лидар (x=0.20) позади переднего бампера (~0.23) — ПРОВЕРИТЬ В СЦЕНЕ
SECTOR = math.radians(10)   # передний сектор ±10°
COLLISION_EPS = 0.01        # d_body <= 1 см считаем касанием

args = sys.argv[1:]
DELAY = float(args[0]) if len(args) > 0 else 0.0
THR = float(args[1]) if len(args) > 1 else 0.40
TAG = args[2] if len(args) > 2 else f"d{DELAY}_thr{THR}"

# ---------- Устройства ----------
robot = Robot()
ts = int(robot.getBasicTimeStep())
dt = ts / 1000.0

lidar = robot.getDevice("lidar")
lidar.enable(ts)

ml = robot.getDevice("left wheel")
mr = robot.getDevice("right wheel")
for m in (ml, mr):
    m.setPosition(float("inf"))   # режим управления скоростью
    m.setVelocity(0.0)

sl = robot.getDevice("left wheel sensor")
sr = robot.getDevice("right wheel sensor")
sl.enable(ts)
sr.enable(ts)

w_need = V0 / R
w_max = min(ml.getMaxVelocity(), mr.getMaxVelocity())
print(f"[{TAG}] шаг={ts} мс, задержка={DELAY} с, порог={THR} м, "
      f"ω={w_need:.2f} рад/с (макс. {w_max:.2f})")
if w_need > w_max:
    print("ОШИБКА: требуемая скорость колеса превышает допустимую.")

# Индексы лучей переднего сектора (лучи идут слева направо, центр — ось робота)
N = lidar.getHorizontalResolution()
fov = lidar.getFov()
if N > 1:
    front = [i for i in range(N) if abs(fov / 2 - i * fov / (N - 1)) <= SECTOR]
else:
    front = [0]
print(f"Лидар: {N} лучей, обзор {math.degrees(fov):.0f}°, в секторе {len(front)} лучей")


def front_distance():
    """Минимальное расстояние до препятствия от передней границы корпуса, м.
    None — если в секторе нет ни одного действительного измерения."""
    img = lidar.getRangeImage()
    vals = [img[i] for i in front if math.isfinite(img[i])]
    return (min(vals) - L) if vals else None


# Первый шаг — чтобы датчики положения выдали числа, а не NaN
robot.step(ts)
prev_l, prev_r = sl.getValue(), sr.getValue()

state = "MOVE"
v_cmd = V0
t_trigger = None
collision = False
reported = False

f = open(f"run_{TAG}.csv", "w", newline="", encoding="utf-8")
log = csv.writer(f)
log.writerow(["t", "d_body", "v_cmd", "v_enc", "state"])

while robot.step(ts) != -1:
    t = robot.getTime()
    d = front_distance()

    # Оценка скорости по энкодерам
    l, r = sl.getValue(), sr.getValue()
    v_enc = R / 2 * ((l - prev_l) / dt + (r - prev_r) / dt)
    prev_l, prev_r = l, r

    if d is not None and d <= COLLISION_EPS:
        collision = True

    # ---------- Конечный автомат ----------
    if state in ("STOP", "ERROR"):
        v_cmd = 0.0
    elif d is None:
        state, v_cmd = "ERROR", 0.0
        print(f"[{TAG}] t={t:.3f}: нет действительных измерений в секторе — аварийная остановка")
    else:
        if state == "MOVE" and d <= THR:
            t_trigger = t
            state = "WAIT" if DELAY > 0 else "BRAKE"
            print(f"[{TAG}] t={t:.3f}: порог достигнут, d={d:.3f} м")
        if state == "WAIT" and t - t_trigger >= DELAY:
            state = "BRAKE"
            print(f"[{TAG}] t={t:.3f}: начало торможения, d={d:.3f} м")
        if state == "BRAKE":
            v_cmd = max(0.0, v_cmd - A * dt)
            if v_cmd == 0.0 and abs(v_enc) < 1e-3:
                state = "STOP"

    w = v_cmd / R
    ml.setVelocity(w)
    mr.setVelocity(w)

    log.writerow([f"{t:.3f}", "" if d is None else f"{d:.4f}",
                  f"{v_cmd:.4f}", f"{v_enc:.4f}", state])

    if state in ("STOP", "ERROR") and not reported:
        f.flush()
        print(f"[{TAG}] ИТОГ: состояние={state}, расстояние после остановки="
              f"{'—' if d is None else f'{d:.3f}'} м, столкновение={'да' if collision else 'нет'}")
        reported = True

f.close()
