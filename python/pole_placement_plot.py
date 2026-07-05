import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

L      = 0.4
g      = 9.81
alpha  = np.sqrt(g / L)
beta   = 1
gamma  = 10
omega0 = 1

fig, ax = plt.subplots(figsize=(10, 7))
ax.set_facecolor('white')
fig.patch.set_facecolor('white')

xlim = (-12, 7)
ylim = (-2.5, 2.5)
ax.set_xlim(xlim)
ax.set_ylim(ylim)

# Highlight stable half-plane (Re < 0)
rect = plt.Rectangle((xlim[0], ylim[0]), abs(xlim[0]), ylim[1] - ylim[0],
                      color='#FFF3CD', alpha=0.7, zorder=0)
ax.add_patch(rect)
ax.text(-11.5, 2.2, 'stable half-plane (Re < 0)', color='#856404', fontsize=9)

# Minimum rate-of-decay line at Re = -beta
ax.axvline(x=-beta, color='#7F77DD', linewidth=1.8,
           linestyle=(0, (8, 4)), zorder=1)
ax.text(-beta - 0.15, 2.2, 'minimum rate-of-decay\nline',
        color='#7F77DD', fontsize=8, ha='right', va='top')

# Axes
ax.axhline(y=0, color='black', linewidth=1.2, zorder=2)
ax.axvline(x=0, color='black', linewidth=1.2, zorder=2)

ax.annotate('', xy=(xlim[1], 0), xytext=(xlim[1] - 0.3, 0),
            arrowprops=dict(arrowstyle='->', color='black', lw=1.2))
ax.annotate('', xy=(0, ylim[1]), xytext=(0, ylim[1] - 0.15),
            arrowprops=dict(arrowstyle='->', color='black', lw=1.2))
ax.text(xlim[1] - 0.1, -0.2, 'Re', fontsize=11, fontweight='bold')
ax.text(0.15, ylim[1] - 0.1, 'Im', fontsize=11, fontweight='bold')

# Migration arrows (open-loop → closed-loop)
mig = dict(arrowstyle='->', color='#555555', lw=1.2, linestyle='dashed')

ax.annotate('', xy=(-alpha, 0), xytext=(alpha, 0),
            arrowprops=dict(arrowstyle='->', color='#555555', lw=1.2,
                            linestyle='dashed', connectionstyle='arc3,rad=-0.4'))
ax.annotate('', xy=(-beta,  0.08),   xytext=(-0.1,  0.08),   arrowprops=mig)
ax.annotate('', xy=(-beta,  omega0), xytext=(-0.1,  omega0), arrowprops=mig)
ax.annotate('', xy=(-beta, -omega0), xytext=(-0.1, -omega0), arrowprops=mig)

# Open-loop poles (initial)
s0 = 160
for x in [alpha, -alpha]:
    ax.scatter(x, 0, marker='s', s=s0, facecolors='#FCEBEB',
               edgecolors='#E24B4A', linewidths=2, zorder=5)

for pt in [(0, 0), (0, omega0), (0, -omega0)]:
    ax.scatter(*pt, marker='o', s=s0, facecolors='#FCEBEB',
               edgecolors='#E24B4A', linewidths=2, zorder=5)

# Closed-loop poles (target)
s1 = 200
ax.scatter(-alpha, 0, marker='D', s=s1, facecolors='#C0DD97',
           edgecolors='#3B6D11', linewidths=2, zorder=6)

for pt in [(-beta, 0), (-beta, omega0), (-beta, -omega0)]:
    ax.scatter(*pt, marker='^', s=s1, facecolors='#C0DD97',
               edgecolors='#3B6D11', linewidths=2, zorder=6)

ax.scatter(-gamma, 0, marker='p', s=s1, facecolors='#C0DD97',
           edgecolors='#3B6D11', linewidths=2, zorder=6)

# Pole labels
kr = dict(color='#A32D2D', fontsize=9, ha='center')
kg = dict(color='#1a5c0a', fontsize=9, fontweight='bold')

ax.text( alpha,        0.35,           '+α\n(unstable)', **kr)
ax.text( 0.25,         0.20,           '0',    color='#A32D2D', fontsize=9)
ax.text( 0.25,         omega0 + 0.2,  '+iω₀', color='#A32D2D', fontsize=9)
ax.text( 0.25,        -omega0 - 0.3,  '-iω₀', color='#A32D2D', fontsize=9)
ax.text(-alpha,        0.35,           '-α (×2)',      **kg, ha='center')
ax.text(-beta - 0.15, -0.35,           '-β',           **kg, ha='right')
ax.text(-gamma,        0.35,           '-γ\n(fast)',   **kg, ha='center')
ax.text(-beta - 0.15,  omega0 + 0.2,  '-β+iω₀',      **kg, ha='right')
ax.text(-beta - 0.15, -omega0 - 0.3,  '-β-iω₀',      **kg, ha='right')

# Legend
legend_elements = [
    Line2D([0], [0], marker='s', color='w', markerfacecolor='#FCEBEB',
           markeredgecolor='#E24B4A', markeredgewidth=2, markersize=9,
           label='Initial poles of P(s) : +α (unstable), -α'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='#FCEBEB',
           markeredgecolor='#E24B4A', markeredgewidth=2, markersize=9,
           label='Imposed poles of S(s) : 0, ±iω₀'),
    Line2D([0], [0], marker='D', color='w', markerfacecolor='#C0DD97',
           markeredgecolor='#3B6D11', markeredgewidth=2, markersize=9,
           label='Closed-loop pole from P(s) : -α (×2)'),
    Line2D([0], [0], marker='^', color='w', markerfacecolor='#C0DD97',
           markeredgecolor='#3B6D11', markeredgewidth=2, markersize=9,
           label='Closed-loop poles from S(s) : -β, -β±iω₀'),
    Line2D([0], [0], marker='p', color='w', markerfacecolor='#C0DD97',
           markeredgecolor='#3B6D11', markeredgewidth=2, markersize=9,
           label='Imposed fast pole : -γ'),
    Line2D([0], [0], color='#7F77DD', linewidth=1.8, linestyle=(0, (8, 4)),
           label='Minimum rate-of-decay line (Re = -β)'),
    Line2D([0], [0], color='#555555', linewidth=1.2, linestyle='dashed',
           label='Open-loop → closed-loop migration'),
]

ax.legend(handles=legend_elements, loc='lower right',
          fontsize=8, framealpha=1, edgecolor='#888888')

ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
for spine in ax.spines.values():
    spine.set_visible(False)

ax.set_title('Complex plane — RST pole placement (inverted pendulum)',
             fontsize=11, pad=12)

plt.tight_layout()
plt.savefig('complex_plane_RST.pdf', dpi=300, bbox_inches='tight')
plt.savefig('complex_plane_RST.png', dpi=300, bbox_inches='tight')
plt.show()
