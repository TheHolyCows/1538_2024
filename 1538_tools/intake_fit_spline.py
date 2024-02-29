import sys

import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

df_ref = pd.read_csv(sys.argv[1], header=None, names=["time", "acceleration", "current"])

acc_avg = df_ref["acceleration"].rolling(5, min_periods=1).mean()

peak_idx = df_ref.loc[:, "acceleration"].idxmax()
peak_time = df_ref.loc[peak_idx, "time"]
peak_acc = df_ref.loc[peak_idx, "acceleration"]
print(f"Peak - i:{peak_idx} t:{peak_time} v:{peak_acc}")

steady_idx = df_ref[(df_ref.index > peak_idx) & (acc_avg.abs() < 0.5)].index.min()
steady_time = df_ref.loc[steady_idx, "time"]
steady_acc = df_ref.loc[steady_idx, "acceleration"]
print(f"Steady - i:{steady_idx} t:{steady_time} v:{steady_acc}")

df = df_ref.loc[:steady_idx]
# df.loc[steady_idx, "acceleration"] = 0

weights = np.ones(df.shape[0])
weights[0] = 100
weights[peak_idx] = 100
weights[steady_idx] = 100

weights_1 = weights[:peak_idx + 1]
fit_1 = np.polynomial.polynomial.polyfit(df.loc[:peak_idx, "time"], df.loc[:peak_idx, "acceleration"], 3, w=weights_1)
fit_1_x = np.linspace(0, peak_time, 500)
fit_1_y = np.array([np.polynomial.polynomial.polyval(x, fit_1) for x in fit_1_x])

weights_2 = weights[peak_idx:]
fit_2 = np.polynomial.polynomial.polyfit(df.loc[peak_idx:, "time"], df.loc[peak_idx:, "acceleration"], 3, w=weights_2)
fit_2_roots = np.polynomial.polynomial.polyroots(fit_2)
fit_2_x = np.linspace(peak_time, np.real(fit_2_roots[0]), 500)
fit_2_y = np.array([np.polynomial.polynomial.polyval(x, fit_2) for x in fit_2_x])

fit_3_x = np.linspace(np.real(fit_2_roots[0]), 3, 500)
fit_3_y = np.zeros(fit_3_x.shape[0])

np.set_printoptions(suppress=True)

print("---")

print(f"INTAKE_STEADY_CURRENT = {df_ref.loc[steady_idx, "current"]};")

print("")

print(f"INTAKE_CURVE_I_A = {fit_1[0]};")
print(f"INTAKE_CURVE_I_B = {fit_1[1]};")
print(f"INTAKE_CURVE_I_C = {fit_1[2]};")
print(f"INTAKE_CURVE_I_D = {fit_1[3]};")
print(f"INTAKE_CURVE_I_END_TIME = {peak_time};")

print("")

print(f"INTAKE_CURVE_II_A = {fit_2[0]};")
print(f"INTAKE_CURVE_II_B = {fit_2[1]};")
print(f"INTAKE_CURVE_II_C = {fit_2[2]};")
print(f"INTAKE_CURVE_II_D = {fit_2[3]};")
print(f"INTAKE_CURVE_II_END_TIME = {np.real(fit_2_roots[0])};")

fig, (ax_ref, ax_fit, ax_both, ax_current) = plt.subplots(4)
fig.tight_layout()

ax_ref.set_title("Data")
ax_ref.scatter(x=df_ref["time"], y=df_ref["acceleration"])

ax_fit.set_title("Fitted Curve")
ax_fit.set_xlim(ax_ref.get_xlim())
ax_fit.set_ylim(ax_ref.get_ylim())
ax_fit.plot(fit_1_x, fit_1_y, c="orange")
ax_fit.plot(fit_2_x, fit_2_y, c="orange")
ax_fit.plot(fit_3_x, fit_3_y, c="orange")

ax_both.set_title("Data & Fitted Curve")
ax_both.set_xlim(ax_ref.get_xlim())
ax_both.set_ylim(ax_ref.get_ylim())
ax_both.scatter(x=df_ref["time"], y=df_ref["acceleration"])
ax_both.plot(fit_1_x, fit_1_y, c="orange")
ax_both.plot(fit_2_x, fit_2_y, c="orange")
ax_both.plot(fit_3_x, fit_3_y, c="orange")

ax_current.set_title("Current")
ax_current.scatter(x=df_ref["time"], y=df_ref["current"])

plt.show()
