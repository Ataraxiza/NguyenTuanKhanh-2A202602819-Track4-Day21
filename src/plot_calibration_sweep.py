from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


INPUT = "results/calibration_sweep.csv"
OUTPUT = "results/figures/calibration_sweep.png"


def main() -> None:
    df = pd.read_csv(INPUT, dtype={"frame": str})

    fig, axes = plt.subplots(
        1,
        3,
        figsize=(15, 4.5),
        sharey=True,
    )

    for ax, perturb_type in zip(
        axes,
        ["yaw", "pitch", "roll"],
    ):
        subset = df[df["perturb_type"] == perturb_type]

        for frame, g in subset.groupby("frame"):
            grouped = (
                g.groupby("perturb_value")
                .apply(
                    lambda x: (
                        x["hits"].sum()
                        / x["object_points"].sum()
                    ),
                    include_groups=False,
                )
                .reset_index(name="score")
            )

            ax.plot(
                grouped["perturb_value"],
                100 * grouped["score"],
                marker="o",
                label=f"frame {frame}",
            )

        ax.axhline(
            80,
            color="red",
            linestyle="--",
            linewidth=1,
            label="warning 80%",
        )

        ax.set_title(perturb_type.capitalize())
        ax.set_xlabel(
            "Perturbation (degree)"
        )
        ax.grid(alpha=0.3)

    axes[0].set_ylabel(
        "Alignment score = hit_ratio (%)"
    )

    axes[0].set_ylim(0, 105)
    axes[-1].legend(
        fontsize=8,
        loc="best",
    )

    fig.tight_layout()

    out = Path(OUTPUT)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)

    print(f"-> {out}")


if __name__ == "__main__":
    main()