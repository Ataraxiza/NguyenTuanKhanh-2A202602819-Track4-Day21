# Phần 05: Thí nghiệm chính (CP3)

> **Phút 50–85 của giờ lab.** Bạn chạy thí nghiệm chính của topic với **ít nhất 3 mức** thay đổi, ghi kết quả ra CSV và vẽ ít nhất 1 biểu đồ.
> 
> **Sản phẩm khi kết thúc:** `results/<tên_thí_nghiệm>.csv`, `results/figures/<tên_biểu_đồ>.png`, mục 2 của REPORT có bảng hoặc biểu đồ kèm nhận xét, và đã commit `CP3: ...`.
> 
> Tiêu chí này chiếm **25/100 điểm**.

```
### 5.1. Bốn nguyên tắc của một thí nghiệm tốt

| Nguyên tắc | Làm cụ thể thế nào | Nếu vi phạm |
|---|---|---|
| **Ít nhất 3 mức** | Ví dụ yaw 0°, 1°, 2°, 3°. Luôn có mức 0 (không thay đổi gì) để làm mốc so sánh | Chỉ 2 mức thì không thấy được xu hướng, tối đa mức "Đạt" (15–21 điểm) |
| **Mỗi lần chỉ thay đổi một yếu tố** | Giữ nguyên frame, class, vùng range và seed. Chỉ đổi đúng yếu tố đang khảo sát | Không biết metric thay đổi là do yếu tố nào |
| **Tái lập được** | Mọi phép ngẫu nhiên đều truyền seed cố định. Chạy lại script phải ra đúng cùng số | Giảng viên chạy lại ra số khác: tiêu chí 1.2 tối đa 50% điểm |
| **Metric đo đúng điều claim nói** | Claim nói "điểm rơi ra khỏi vật thể" thì đo tỉ lệ điểm nằm trong box, không đo số điểm trong ảnh | Metric không liên quan tới claim: mức "Yếu" |
```
### Cấu trúc file CSV
Mỗi **dòng** là một lần chạy (một cấu hình), mỗi **cột** là một tham số hoặc một metric:
```
dataset,frame,yaw_deg,n_points,inside_image,object_points,hit_ratio
kitti_mini,000011,0.0,108004,19946,725,0.9945
kitti_mini,000011,0.5,108004,19946,690,0.9188
```
Có cột tham số (`frame`, `yaw_deg`) thì bạn và giảng viên mới biết mỗi con số được đo trong điều kiện nào.

## 5.2. Script mẫu cho topic A: quét góc lệch yaw
Script này đo: khi calibration bị lệch yaw, bao nhiêu % điểm LiDAR thuộc về vật thể còn rơi đúng vào 2D box của vật thể đó trên ảnh.

### Ý tưởng của metric

1. Dùng **calibration gốc** để biết điểm nào thực sự thuộc vật thể: điểm nằm trong 3D box của label.
2. Dùng **calibration đã làm lệch** để chiếu các điểm đó lên ảnh.
3. Đếm tỉ lệ điểm rơi vào trong 2D box của label. Calibration đúng thì tỉ lệ này gần 100%.

"""Topic A: lệch yaw bao nhiêu độ thì điểm LiDAR rơi ra khỏi 2D box của vật thể.

Chạy từ gốc repo:
    python -m src.exp_yaw_sweep --data-root data/kitti_mini --frames 000008 000011 000049
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from starter.datasets import load_frame
from starter.projection import perturb_extrinsic, project_velo_to_image, velo_to_cam

CLASSES = ("Car", "Van", "Pedestrian", "Cyclist")


def points_in_box(points_cam: np.ndarray, obj) -> np.ndarray:
    """Mask (N,) các điểm (đã ở camera frame) nằm trong 3D box của label."""
    h, w, l = obj.dimensions
    c, s = np.cos(obj.rotation_y), np.sin(obj.rotation_y)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    local = (points_cam - obj.location) @ R        # toạ độ của điểm trong hệ trục gắn với box
    return ((np.abs(local[:, 0]) <= l / 2) & (local[:, 1] <= 0) & (local[:, 1] >= -h)
            & (np.abs(local[:, 2]) <= w / 2))


def run_one(fr: dict, yaw_deg: float) -> dict:
    pts = fr["points"][np.isfinite(fr["points"]).all(axis=1)]
    cam_true = velo_to_cam(pts[:, :3], fr["calib"])           # vị trí thật, theo calib gốc
    calib = perturb_extrinsic(fr["calib"], yaw_deg=yaw_deg)   # calib đã bị lệch
    uv, _, mask = project_velo_to_image(pts, calib, fr["image"].shape)
    uv_all = np.full((len(pts), 2), np.nan)
    uv_all[mask] = uv

    obj_pts = hits = 0
    for obj in fr["labels"]:
        if obj.type not in CLASSES:
            continue
        sel = points_in_box(cam_true, obj) & mask
        u, v = uv_all[sel, 0], uv_all[sel, 1]
        x1, y1, x2, y2 = obj.bbox
        hits += int(((u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)).sum())
        obj_pts += int(sel.sum())
    return {"n_points": len(pts), "inside_image": int(mask.sum()), "object_points": obj_pts,
            "hit_ratio": round(hits / obj_pts, 4) if obj_pts else float("nan")}


def main() -> None:
    ap = argparse.ArgumentParser(description="Quét góc lệch yaw, đo % điểm của vật thể nằm trong 2D box")
    ap.add_argument("--data-root", default="data/kitti_mini")
    ap.add_argument("--frames", nargs="+", default=["000008", "000011", "000049"])
    ap.add_argument("--yaw-levels", nargs="+", type=float, default=[0, 0.5, 1, 2, 3])
    ap.add_argument("--out", default="results/yaw_perturb_sweep.csv")
    args = ap.parse_args()

    rows = []
    for frame in args.frames:
        fr = load_frame(args.data_root, frame)
        for yaw in args.yaw_levels:
            row = {"dataset": Path(args.data_root).name, "frame": frame, "yaw_deg": yaw, **run_one(fr, yaw)}
            rows.append(row)
            print(row)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"-> {out} ({len(rows)} dòng)")


if __name__ == "__main__":
    main()

✅ Kỳ vọng
Lệnh in 15 dòng (3 frame × 5 mức), rồi ghi results/yaw_perturb_sweep.csv. Ở mức 0°, hit_ratio chưa đạt 100% vì label do con người vẽ, 2D box và 3D box không khớp tuyệt đối với nhau. Đây là "mức sàn" của metric. Hãy nhắc tới điều này khi đặt ngưỡng phát hiện.

### Script mẫu chỉ là điểm xuất phát
Chạy nguyên script mẫu chỉ cho bạn mức Basic của tiêu chí Benchmark. Để đạt mức Good trở lên, hãy **mở rộng** nó theo ít nhất một hướng, và giải thích được từng dòng code trong buổi vấn đáp:

- Tách `hit_ratio` theo **từng class** hoặc **từng khoảng cách** (0–15 m, 15–30 m, trên 30 m) để kiểm chứng claim "vật hẹp và vật xa bị ảnh hưởng nặng hơn".
- Thêm **pitch**, **roll**, hoặc **dịch chuyển** `t_xyz_m` (2, 5, 10 cm), so sánh với yaw.
- Chạy trên `data/nuscenes_mini_subset` để so sánh với KITTI (bonus B5, xem Phần 07).
- Tìm **ngưỡng phát hiện**: `hit_ratio` dưới bao nhiêu thì nên cảnh báo calibration lệch?
Ghi rõ trong mục 6 của REPORT rằng bạn dùng script mẫu của codelab làm điểm xuất phát, và đã mở rộng những gì.

5.3. Vẽ biểu đồ
Tạo file src/plot_yaw_sweep.py:

"""Vẽ kết quả quét yaw. Chạy từ gốc repo: python -m src.plot_yaw_sweep"""
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("results/yaw_perturb_sweep.csv", dtype={"frame": str})   # giữ "000011", không đổi thành 11

fig, ax = plt.subplots(figsize=(6, 4))
for frame, g in df.groupby("frame"):
    ax.plot(g["yaw_deg"], 100 * g["hit_ratio"], marker="o", label=f"frame {frame}")
ax.set_xlabel("Lệch yaw (độ)")
ax.set_ylabel("% điểm của vật thể nằm trong 2D box")
ax.set_ylim(0, 105)
ax.grid(alpha=0.3)
ax.legend()
fig.tight_layout()

out = Path("results/figures/yaw_sweep.png")
out.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(out, dpi=150)
print(f"-> {out}")

✅ Kỳ vọng
Một biểu đồ tốt phải có: tên trục kèm đơn vị, chú thích (legend) cho từng đường, trục y bắt đầu từ 0 để không phóng đại khác biệt, và mỗi điểm đo có đánh dấu (marker="o") để người xem biết đâu là số đo thật, đâu là phần nối.