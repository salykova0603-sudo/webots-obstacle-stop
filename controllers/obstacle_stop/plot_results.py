"""
Графики и итоговая таблица по файлам run_*.csv.
Запуск в папке с CSV:  python plot_results.py
Метки запусков: A1, A2, A3 (серия А) и B1, B2, B3 (серия Б).
"""
import csv
import glob
import matplotlib.pyplot as plt


def load(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    t = [float(r["t"]) for r in rows]
    d = [float(r["d_body"]) if r["d_body"] else float("nan") for r in rows]
    vc = [float(r["v_cmd"]) for r in rows]
    ve = [float(r["v_enc"]) for r in rows]
    st = [r["state"] for r in rows]
    return t, d, vc, ve, st


def first_time(t, st, names):
    for ti, s in zip(t, st):
        if s in names:
            return ti
    return None


summary = []
for series in ("A", "B"):
    files = sorted(glob.glob(f"run_{series}*.csv"))
    if not files:
        continue
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    for path in files:
        tag = path[4:-4]
        t, d, vc, ve, st = load(path)
        t_thr = first_time(t, st, ("WAIT", "BRAKE"))
        t_brk = first_time(t, st, ("BRAKE",))
        line, = ax1.plot(t, d, label=tag)
        c = line.get_color()
        ax2.plot(t, vc, color=c, linestyle="--", label=f"{tag} заданная")
        ax2.plot(t, ve, color=c, label=f"{tag} по энкодерам")
        for ax in (ax1, ax2):
            if t_thr is not None:
                ax.axvline(t_thr, color=c, linestyle=":", alpha=0.7)
            if t_brk is not None:
                ax.axvline(t_brk, color=c, linestyle="-.", alpha=0.7)
        d_valid = [x for x in d if x == x]
        d_final = d_valid[-1] if d_valid else float("nan")
        collision = any(x <= 0.01 for x in d_valid)
        summary.append((tag, d_final, "да" if collision else "нет"))

    ax1.set_ylabel("Расстояние до препятствия, м")
    ax1.set_title(f"Серия {series}  ( : — порог,  -. — начало торможения )")
    ax1.grid(True); ax1.legend()
    ax2.set_ylabel("Скорость, м/с"); ax2.set_xlabel("Время, с")
    ax2.grid(True); ax2.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(f"series_{series}.png", dpi=150)
    print(f"Сохранено: series_{series}.png")

print("\nЗапуск | Расстояние после остановки, м | Столкновение")
for tag, d, c in summary:
    print(f"{tag:6} | {d:29.3f} | {c}")
