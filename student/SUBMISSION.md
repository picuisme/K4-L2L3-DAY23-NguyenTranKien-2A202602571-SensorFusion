# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Trần Kiên
- MSSV: 2A202602571
- Email: nguyentrankien23@gmail.com
- Link repo (fork): https://github.com/picuisme/K4-L2L3-DAY23-NguyenTranKien-2A202602571-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): HEAD của nhánh `main` — hash 40 ký tự nộp trên LMS (file không thể chứa hash của chính commit chứa nó). Lần chạy chấm điểm nằm ở commit `CP5: ...`; code E–H không đổi từ CP4.

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]` (199 frame), `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: `0.9701`, `0.7004`, `519 / 16 / 222`
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `0.1503 m`, `502`, `11.3437 m²`, `0`, `239`, `2.5226`
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `0.1359 m`, `502`, `9.2668 m²`, `0`, `239`, `2.5226`
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: xem bảng và 4 ý ngay dưới.

Số đầy đủ (chép từ `student/artifacts/metrics.json`, không làm tròn tay):

| Chỉ số | LiDAR-only | LiDAR + camera | Chênh (fused − lidar) |
|---|---|---|---|
| `rmse` (m) | 0.1503227094069944 | 0.13586679671553112 | −0.0145 (−9.6 %) |
| `matches` | 502 | 502 | 0 |
| `sum_sq_err` (m²) | 11.34365231565676 | 9.266812797769207 | −2.0768 (−18.3 %) |
| `ghost_track_frames` | 0 | 0 | 0 |
| `missed_gt_frames` | 239 | 239 | 0 |
| `mean_confirmed_tracks` | 2.522613065326633 | 2.522613065326633 | 0 |
| `precision_track = matches/(matches+ghost)` | 1.000 | 1.000 | — |
| `coverage = matches/det_tp` | 502/519 = 0.967 | 0.967 | — |

Đối chiếu ngưỡng RUBRIC 1.2: RMSE hai mode ≤ 0.45 m, `precision_track` ≥ 0.75,
`coverage` ≥ 0.70, và `rmse_fused − rmse_lidar = −0.0145 m ≤ 0.05 m`.

Đọc kết quả:

1. **Số ghép, ghost, miss giống hệt nhau ở hai mode — đúng thiết kế, không phải trùng hợp.**
   Trong `grade_run.log`, cột `confirmed` và `matches` của record `lidar` và `fused`
   bằng nhau ở cả 199 frame. Lý do: tạo/xác nhận/xoá track chỉ do lượt LiDAR quyết
   định (`TrackManager.manage_tracks` thoát ngay khi `sensor.name != "lidar"`), camera
   chỉ sửa `x`, `P`. Vì vậy camera không thể thêm hay bớt track; nó chỉ có thể dịch
   vị trí track.
2. **Khác biệt nằm ở `sum_sq_err`.** Cùng 502 cặp ghép nhưng tổng bình phương sai số
   giảm 11.34 → 9.27 m², nên RMSE giảm 0.150 → 0.136 m. Vì số cặp không đổi, RMSE
   thấp hơn ở đây là cải thiện thật về vị trí chứ không phải do bỏ bớt cặp khó.
   Tính theo frame: trong 195 frame có cặp ghép, fused có `sum_sq_err` nhỏ hơn ở
   137 frame và lớn hơn ở 58 frame — camera không tốt hơn ở mọi frame. Ví dụ frame 4–5
   (track vừa confirmed, vận tốc còn rất bất định) fused kém hơn (0.032 so với 0.011 m²);
   frame 150 fused tốt hơn rõ (0.033 so với 0.076 m²).
3. **`ghost = 0` nhưng `miss = 239` — RMSE đẹp không có nghĩa là thấy hết xe.**
   `valid_gt` tổng là 741; detector có sẵn chỉ bắt được `det_tp = 519` (recall 0.70,
   `det_fn = 222`). Tracker không thể ghép xe mà detector không thấy, nên 222/239 miss
   là của detector. 17 miss còn lại là của tracker: track cần 5 hit LiDAR liên tiếp
   (`score` 1/6 → 5/6 > 0.8) mới confirmed, nên ở frame 0–3 `confirmed = 0`,
   `misses = 2` dù `det_tp = 2`; frame 4 mới có `confirmed = 2`. 16 detection FP không
   thành ghost vì chúng không lặp lại đủ 5 frame để được xác nhận.
4. **Giới hạn của kết quả fused.** Đo camera là tâm hộp 2D ground-truth FRONT cộng nhiễu
   σ = 0.5 px theo `--seed`, không phải output của một camera detector. Nó gần như
   không có bỏ sót, không có báo nhầm, và sai số nhỏ hơn nhiều so với `R` mà EKF giả
   định (σ = 5 px). Mức cải thiện 0.014 m vì thế là cận trên lạc quan cho “camera lý
   tưởng”, không chứng minh một hệ perception độc lập với GT. Phần bonus còn cho thấy
   innovation ngang trung bình lệch −11.9 px ngay cả khi calibration đúng: tâm hộp 2D
   trên ảnh không trùng hình chiếu của tâm hộp 3D, tức mô hình đo `h(x)` có sai lệch hệ
   thống mà `R` đường chéo không mô tả.

Môi trường chạy: Linux x86-64, CPU, Python 3.12, torch 2.5.1+cpu, numpy 2.5.3, scipy 1.18.1.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. **Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?**

   | | LiDAR | Camera |
   |---|---|---|
   | `z` | 3×1 `(x, y, z)` mét — tâm hộp 3D của detector | 2×1 `(u, v)` pixel — tâm hộp 2D |
   | `h(x)` | tuyến tính: `p_s = R·p + t` (LiDAR dùng `T = I`) | phi tuyến: `u = c_i − f_i·y_s/x_s`, `v = c_j − f_j·z_s/x_s` |
   | `H` | 3×6 hằng `[R_3×3 \| 0]` | 2×6 Jacobian tính lại tại `x` mỗi lần (`Sensor.get_H`) |
   | `R` | `diag(0.1², 0.1², 0.1²)` m² | `diag(5², 5²)` px² (`build_camera_measurement`) |

   Cùng một `ekf_update` (`kalman.py`) xử lý cả hai vì nó chỉ dùng `meas.sensor.get_H(x)`,
   `get_hx(x)` và `meas.R`: `γ = z − h(x)`, `S = H P Hᵀ + R`, `K = P Hᵀ S⁻¹`. Khác biệt
   quan trọng về nội dung thông tin: LiDAR quan sát đủ 3 chiều vị trí; camera chỉ cho
   hướng tia (2 ràng buộc), **không quan sát được độ sâu** — một sai số dọc tia hầu như
   không đổi `(u, v)`. Vì `f ≈ 2000 px`, 5 px ở khoảng cách 20 m tương đương ~0.05 m
   theo phương ngang, tức camera chặt hơn LiDAR (0.1 m) theo phương ngang/dọc ảnh nhưng
   mù theo phương dọc tia. Đó là lý do fused giảm sai số mà không thay được LiDAR.
   Cột vận tốc của `H` bằng 0 ở cả hai: vận tốc chỉ được suy ra qua `F` và tương quan
   trong `P`.

2. **Vì sao cần gating Mahalanobis trước khi gán?**
   Gán greedy luôn chọn cặp cost nhỏ nhất còn lại; nếu không có cổng, một track vừa
   mất đo vẫn bị ép nhận detection của xe khác hoặc một FP, kéo trạng thái sai và còn
   được cộng score. Cổng χ² (`chi2_gate`: `d² < chi2.ppf(0.995, dim_meas)` → 12.84 cho
   LiDAR 3D, 10.60 cho camera 2D) biến câu hỏi “gần không?” thành “có phù hợp thống kê
   với độ bất định không?”. `d² = γᵀ S⁻¹ γ` chuẩn hoá theo `S = H P Hᵀ + R`, nên khác
   Euclid ở hai điểm: (a) track mới có `P` lớn (σ vận tốc 50 m/s → sau một predict vị trí
   bất định hàng mét) được phép nhận đo xa hơn, track đã hội tụ thì cổng hẹp lại;
   (b) đơn vị không còn quan trọng — pixel và mét dùng chung một ngưỡng xác suất. Ngoài
   ra `association_cost_matrix` kiểm tra `meas.sensor.in_fov(track.x)` **trước** khi tính
   `d²`: track sau lưng camera có độ sâu ≤ 0, chiếu pinhole sẽ chia cho 0/âm, nên cặp đó
   nhận `inf` mà không gọi `get_hx`/`get_H` (test `test_out_of_fov_gating_precedes_projection`).
   Bằng chứng trên số liệu: 16 detection FP không tạo ra ghost nào, và ở bonus khi lệch
   camera ≥ 1° cổng loại ~96 % đo camera thay vì để chúng kéo lệch track.

3. **Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.**
   **Track-then-fuse.** Không có bước nào gộp point cloud với ảnh trước detection; chỉ có
   một danh sách track, mỗi frame `KF.predict` một lần rồi `associate_and_update(..., lidar_sensor)`,
   sau đó `associate_and_update(..., camera_sensor)` (xem `run_lab.run`). Dấu vết trong
   `student/artifacts/grade_run.log`:
   - `det_tp/det_fp/det_fn/valid_gt` của record `lidar` và `fused` trùng nhau ở mọi frame
     (ví dụ frame 60: cả hai `det_tp=3, det_fp=0, det_fn=0`) → detection không dùng camera;
     nếu fuse-then-track thì detection phải khác giữa hai mode.
   - `confirmed`, `matches`, `ghosts`, `misses` trùng nhau ở cả 199 frame → vòng đời track
     không phụ thuộc camera.
   - Chỉ `sum_sq_err` khác: frame 150 là `0.0761` (lidar) so với `0.0327` (fused) với cùng
     `matches=3` → camera chỉ là lần update thứ hai trên cùng track.

4. **Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?**
   Innovation camera không còn trung bình 0 mà có **bias cùng dấu trên mọi track**, độ lớn
   ≈ `f·δ` (yaw lệch δ → dịch ngang ~35 px/độ với `f ≈ 2000 px`), và `d²` trung bình vượt
   kỳ vọng 2 của χ² 2 bậc tự do. Đo thật trong bonus (bảng ở mục Bonus): lệch 0.25° →
   mean `γ_u` −20.0 px, `d²` 4.5; lệch 0.5° → −26.9 px, `d²` 7.3, và 40 % đo bị cổng loại.
   Hậu quả có hai pha: lệch nhỏ thì đo vẫn qua cổng và EKF “tin” nó, kéo track lệch ngang
   một cách có hệ thống → RMSE fused 0.211 m, **tệ hơn** LiDAR-only 0.150 m; lệch lớn
   (≥ 1°) thì gần như mọi đo bị cổng loại, fused quay về gần LiDAR-only, còn vài đo lọt
   cổng là gán nhầm sang xe khác. Lệch calibration nhỏ vì thế nguy hiểm hơn lệch lớn,
   và cách phát hiện là theo dõi trung bình có dấu của innovation, không phải RMSE.

5. **Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng? Vì sao
   lidar quyết định score/init/delete còn camera chỉ EKF update?**
   Khi `meas_list` rỗng, không còn `meas.sensor` nào để suy ra lượt này là của cảm biến
   nào, nhưng hai lượt có nghĩa hoàn toàn khác: LiDAR không thấy gì là **bằng chứng track
   có thể đã biến mất** — mọi track trong FOV LiDAR phải bị trừ `1/window` rồi xét xoá;
   camera không có đo thì không được làm gì. Vì vậy `associate_and_update` của tôi luôn
   kết thúc bằng `manager.manage_tracks(unassigned_tracks, unassigned_meas, sensor)` kể cả
   khi không có đo (test `test_empty_lidar_frame_scores_then_deletes_exhausted_track`:
   track 1 hit, frame sau rỗng → score 0 → bị xoá). Nếu bỏ lời gọi này ở frame rỗng,
   track sẽ sống mãi khi xe ra khỏi tầm detector và thành ghost.
   LiDAR giữ quyền sinh/tử vì: (a) nó cho vị trí 3D đầy đủ nên khởi tạo được `x`, `P`
   (`init_track_state_from_meas`); một điểm ảnh 2D không xác định độ sâu nên không khởi
   tạo được track; (b) FOV LiDAR ở đây phủ ±90° còn FRONT camera chỉ ~±25°, nếu camera
   được trừ điểm thì xe ở làn bên sẽ bị xoá oan; (c) nếu cả hai cảm biến cùng cộng score,
   một vật được đếm hai lần mỗi frame và confirmed sớm gấp đôi, ngưỡng `window` mất ý nghĩa.
   Trong lab còn một lý do riêng: đo camera lấy từ GT nên nếu nó được tạo track thì kết
   quả là dùng nhãn để tracking.

6. **Điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xoá track.**
   (`track_management.py`, tham số từ `get_tracking_params()`: `window = 6`,
   `confirmed_threshold = 0.8`, `delete_threshold = 0.6`, `max_P = 9 m²`.)
   - **Khởi tạo:** `score = 1/6`, `state = "initialized"`; hit LiDAR đầu tiên sau đó → `"tentative"`.
   - **Xác nhận:** mỗi hit LiDAR `score = min(1, score + 1/6)`; khi `score > 0.8` (so sánh
     chặt) → `"confirmed"`. Từ 1/6 cần 4 hit nữa để đạt 5/6, nên sớm nhất là frame thứ 5 —
     khớp log: frame 0–3 `confirmed = 0`, frame 4 `confirmed = 2`.
   - **Giữ confirmed sau miss:** miss trong FOV LiDAR trừ `1/6` nhưng **không hạ trạng thái**;
     confirmed chỉ kết thúc bằng xoá. Track score 1.0 chịu được 2 miss liên tiếp
     (1.0 → 0.833 → 0.667, vẫn ≥ 0.6); miss thứ 3 (0.5 < 0.6) mới bị xoá. Track ngoài FOV
     LiDAR không bị tính miss (`manage_tracks` kiểm tra `sensor.in_fov`).
   - **Xoá** (các điều kiện là OR, chỉ xét trong lượt LiDAR): `P[0,0] > 9` hoặc `P[1,1] > 9`
     bất kể score (σ vị trí > 3 m: ước lượng không còn dùng được); hoặc confirmed có
     `score < 0.6`; hoặc chưa confirmed có `score ≤ 0`. Một FP đơn lẻ vì thế sống đúng
     1 frame: sinh với 1/6, frame sau miss → 0 → xoá, không bao giờ thành ghost.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- **Đã làm 2/3 mục bonus (trực quan hoá và phân tích calibration); không làm export CVAT.**
  Toàn bộ sinh bởi `student/bonus/bonus_experiments.py`: script chạy detector có sẵn một
  lần, rồi replay tracker với đúng thứ tự của `fusion-run-lab` bằng code E–H trong
  `workspace/`. Trước khi làm thí nghiệm nó `assert` rằng replay cho lại đúng
  `metrics.json` (RMSE khớp tới 1e-9, cùng matches/ghost) cho cả hai mode. Script không
  ghi vào `student/artifacts/` và không sửa `platform/`.

- **Trực quan hoá track và đo (RUBRIC: 2–5 ảnh có chú thích).**
  - `fig1_bev_tracks.png` — BEV cả segment: tâm GT, đo LiDAR và 4 track ID được confirmed.
    Thấy rõ các xe GT ở `y < −9 m` không có detection nào (nguồn của 222 `det_fn`), và cụm
    detection lẻ ở `y ≈ −3 m` không thành track confirmed.
  - `fig2_camera_update_effect.png` — **tác dụng của camera update trên track 10**: 101 frame
    so sánh, sai số 3D so với GT giảm từ RMSE 0.195 m (LiDAR-only) xuống 0.140 m (fused).
    Hai đường trùng nhau tới frame 115 (chưa có camera update), tách ra từ frame 116 khi
    camera bắt đầu ghép được; mỗi update dịch trạng thái trung bình 0.14 m.
  - `fig3_front_camera_projection.jpg` — ảnh FRONT frame 60: đo camera `z`, hình chiếu
    `h(x)` sau LiDAR update và sau camera update. Track 0 có |innovation| 29 px; sau
    camera update hình chiếu trùng với đo.
  - `fig4_calibration_sweep.png` — đồ thị của bảng dưới.

- **Phân tích calibration (7 mức lệch yaw của extrinsic camera).** Tracker được cho một
  extrinsic bị xoay `δ` quanh trục z, còn pixel đo vẫn đến từ camera thật. Số liệu:
  `student/bonus/calibration_sweep.md`, `bonus_results.json`. LiDAR-only RMSE = 0.1503 m;
  cổng χ² 2D = 10.60. Innovation chỉ thống kê trên các update đã qua cổng.

  | yaw lệch (°) | RMSE fused (m) | matches / ghost / miss | đo camera qua cổng / track trong FOV | mean γ_u (px) | mean \|γ_v\| (px) | mean d² |
  |---|---|---|---|---|---|---|
  | 0 | 0.1359 | 502 / 0 / 239 | 509 / 533 (95 %) | −11.9 | 9.5 | 2.25 |
  | 0.1 | 0.1452 | 502 / 0 / 239 | 506 / 534 (95 %) | −15.4 | 9.5 | 3.04 |
  | 0.25 | 0.1732 | 502 / 0 / 239 | 486 / 534 (91 %) | −20.0 | 9.5 | 4.55 |
  | 0.5 | 0.2112 | 502 / 0 / 239 | 322 / 534 (60 %) | −26.9 | 8.2 | 7.28 |
  | 1 | 0.1726 | 502 / 0 / 239 | 20 / 535 (4 %) | −75.0 | 36.6 | 3.51 |
  | 2 | 0.1760 | 502 / 0 / 239 | 19 / 537 (4 %) | −80.0 | 45.4 | 3.92 |
  | 4 | 0.1648 | 502 / 0 / 239 | 15 / 541 (3 %) | −68.5 | 37.9 | 2.45 |

  Nhận xét:
  - **Triệu chứng trên innovation:** `γ_u` lệch có hệ thống theo một hướng và lớn dần gần
    tuyến tính với `δ` trong vùng đo còn qua cổng (−11.9 → −15.4 → −20.0 → −26.9 px);
    `γ_v` gần như không đổi (9.5 px) vì xoay yaw chỉ dịch ảnh theo phương ngang; `d²`
    trung bình tăng 2.25 → 7.28, tiến sát ngưỡng 10.60.
  - **Vì sao gating không chặn lệch nhỏ:** `S = H P Hᵀ + R` gồm cả bất định vị trí của
    track sau LiDAR update (~0.1 m ≈ 10 px ở 20 m) lẫn `R` = 25 px², nên cổng rộng cỡ
    ±35 px. Lệch 0.1–0.5° (≈ 3.5–18 px) nằm trong cổng, đo được chấp nhận và kéo track
    lệch ngang: RMSE tăng 0.136 → 0.211 m, tệ hơn cả không dùng camera. Lệch này chỉ làm
    đổi `sum_sq_err`; matches/ghost/miss không đổi vì camera không đụng vào vòng đời.
  - **Vì sao gating chặn lệch lớn:** từ 1° (≈ 35 px) dịch chuyển vượt cổng, 96 % đo camera
    bị loại và RMSE quay về 0.165–0.176 m. Nó không về hẳn 0.150 m vì 15–20 đo vẫn lọt
    cổng với innovation rất lớn (|γ_u| ~75–100 px) — đây là gán nhầm sang xe khác hoặc
    track mới có `P` lớn; gating giảm thiệt hại chứ không sửa được calibration.
  - **Mức lệch 0° đã có bias −11.9 px**: giới hạn của đo camera mô phỏng (tâm hộp 2D ≠ hình
    chiếu tâm 3D), đã nêu ở phần tóm tắt kết quả.

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude (Anthropic, model Opus 5.5) chạy ở chế độ agent.
- Dùng cho phần nào (hàm, câu hỏi, debug): dùng AI cho **phần lớn bài**. Claude viết code của toàn bộ hàm Part E–H (`kalman.py`, `association.py`, `camera_fusion.py`, `track_management.py`), viết script bonus `student/bonus/bonus_experiments.py`, chạy `pytest` và `fusion-run-lab`, và soạn bản nháp báo cáo này (bảng số liệu, 6 câu trả lời, phần bonus) theo yêu cầu và hướng dẫn của tôi. Các commit do Claude tạo có dòng `Co-Authored-By: Claude`. Không dùng AI để tạo hay sửa số liệu: mọi con số lấy từ `metrics.json`, `grade_run.log` và `student/bonus/*.json` do chính code trong repo sinh ra.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): `pytest student/tests -q` → 128 passed, không `failed`/`xfailed`; chạy `fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0` trên frame 0–198 sau lần sửa code E–H cuối cùng; `python tools/check_submission.py` → `SẴN SÀNG NỘP`; script bonus replay độc lập và cho lại đúng `metrics.json`; đối chiếu công thức trong code với `docs/HUONG_DAN_KY_THUAT.md` (thứ tự predict → AssocL → AssocC, `γ`, `S`, `K`, điều kiện score/xoá) và kiểm tra tay các mốc trong log (frame 4 là frame đầu có `confirmed`, `matches + ghosts == confirmed`, `matches + misses == valid_gt`).

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không sửa
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
