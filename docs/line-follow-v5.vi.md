# HUSTROY LINE_FOLLOW V5

## Bản thử và bằng chứng ngày 26/11/2026

`line-follow-v5-r2` được build và nạp vào ESP32 DevKit tại `/dev/cu.usbserial-120`. Bản này dùng centroid theo trọng số AO và một vòng PD 100 Hz. Thời gian giữa mẫu lấy từ `micros()`. Đạo hàm lọc với hằng số thời gian 45 ms. PWM nền giảm theo độ lệch, tốc độ thay đổi vị trí và độ tin cậy. Xét từng cụm liền nhau để tách giao lộ khỏi vị trí line đơn.

| Thử nghiệm | Kết quả quan sát |
|---|---|
| GPIO và thứ tự | ADC1 `33,32,34,35,36`. Trong lượt quét tay, đỉnh tín hiệu di chuyển S1→S5 và ngược chiều khi kéo về. Có mẫu chồng lấn ở cả 4 cặp kề trong lượt chính. |
| Vùng phủ | 0 khung `00000` từ lúc bắt đầu trên S1 đến khi qua S5 trong lượt 45 giây. Lượt quét cũ có các khoảng `00000` giữa một số cặp. Cần lặp ở độ cao chạy đã đo để xác định khả năng tái hiện. |
| S3 trên trắng | Lượt cũ ghi 571–579 ADC và từng vượt ngưỡng nhị phân 572. Lượt mới giữ nền trắng ổn định cho trung vị khoảng 206. Profile V5 dùng tương phản chuẩn hóa riêng từng mắt. |
| ADC | Cả 5 kênh từng chạm 4095 trên vạch. S2 và S5 bị bão hòa ở đoạn lấy chuẩn đen. Người vận hành xác nhận AO 3,3 V. Chưa ghi điện áp cực đại từng mắt bằng đồng hồ. |
| Timing USB, VM ngắt | 199 nhịp trong 2 giây. Chu kỳ 9.032–10.958 µs, thời gian xử lý tối đa 1.676 µs, năm lượt đọc ADC tối đa 578 µs. `MISSED=0`, `LATE=0`, `TX_DROP=1`. Người vận hành không cấp nguồn cho bánh. |
| Tín hiệu ở lượt USB | V5 tìm được vị trí trong toàn bộ thời gian có mẫu được ghi nhận, kể cả lúc tín hiệu S4 ở dưới ngưỡng nhị phân. Vạch dịch sang S4 trong lượt, nên kết quả này đo khả năng bắt tín hiệu và timing. |
| Xung motor trên giá kê | PWM +60 trong 1 giây, +60 và +100 trong khoảng 15 giây, +50 và +100 trong 5 giây. Firmware ghi lệnh và dừng theo timeout. Cần quan sát trục quay để xác nhận chuyển động. |

Lượt bench 15 giây dùng hai lệnh cùng duty tại giây 0 và 7 với timeout firmware 8 giây mỗi lệnh. Lượt V5 2 giây kết thúc `RUN_TIMEOUT`. Các lượt benchtop có số vòng vật lý bằng 0. Chi tiết gốc nằm trong [thư mục bằng chứng](evidence/line-follow-v5).

### Số liệu đối chiếu

| Bản | PWM nền | Kp / Kd | Kết quả đường chạy | Có tín hiệu | Mất line dài nhất |
|---|---:|---:|---|---:|---:|
| V3 | 60 | 45 / 1,5 | 1 vòng theo quan sát, xe giật và tìm line liên tục | 23,9% | 920 ms |
| V4 | 60 | 45 / 0,5 | Chuyển động tệ hơn V3 theo quan sát, 111/216 khung có tín hiệu chỉ sáng S5 | 59,5% | 1.700 ms |
| V5 r2 | 60 | 34 / 0,45 | Thử trên ESP32 với VM ngắt và các xung thử bàn | 100% thời gian được lấy mẫu trong lượt USB 2 giây | 0 ms trong lượt USB |

V3 và V4 dùng định nghĩa `LOST` khác nhau. Mức 23,9% lấy từ `TRACK_MS=21520/90000`. Mức 59,5% lấy từ `DETECTED_MS=53520/89980`. V5 ghi thời gian `BIN_ZERO_US` và `ZERO_US` riêng, cùng `KNOWN_US` và `UNKNOWN_US`. Ở r2, mỗi nhịp đóng góp tối đa 10 ms vào mẫu trước đó. `UNKNOWN_US` vì thế gồm sai số jitter dương và khoảng trước mẫu đầu, kể cả khi không bỏ nhịp. Tỷ lệ thiếu deadline dùng số nhịp hoàn thành trong slot chia tổng nhịp thực hiện và slot bỏ qua. Tính vòng đua bằng mốc vật lý hoặc video.

Các giới hạn nổi bật của V3/V4 là vị trí nhị phân thay đổi theo bước nguyên pitch, nhịp 20 ms và cú nhảy PWM giữa bám vạch và xoay tìm vạch. Trên V4, 111/216 mẫu hoạt động ở S5 trùng với các đoạn bánh trong được yêu cầu lùi tới -45. Telemetry cũng cho thấy lần mất line dài hơn. S5 có biến thiên nền lớn trong nhiều lượt, nên offset ngang, góc gá và đường đi dây cần xác nhận bằng phép đo độc lập.

### Profile hiệu chuẩn USB

| | S1 | S2 | S3 | S4 | S5 |
|---|---:|---:|---:|---:|---:|
| `WHITE` | 240 | 227 | 206 | 197 | 1186 |
| `BLACK` | 3851 | 4095 | 3671 | 3579 | 4095 |
| `FLOOR` | 360 | 310 | 320 | 315 | 2190 |

`WHITE` là trung vị 353 mẫu cuối của lượt giữ nền trắng. `BLACK` là trung vị nửa giữa của đoạn đầu từng mắt đi qua line trong lượt quét đủ 5 mắt. `FLOOR` làm tròn lên từ mức nền quan sát được trên nền trắng và các đoạn ngoài vùng line của từng mắt. S5 có mức nền lên khoảng 2181 khi line ở gần mắt bên cạnh. Tọa độ `[-2,-1,0,1,2]` giả định khoảng cách đều. Để dùng hình học đo được, lấy tâm trục dọc thân xe làm zero, rồi chia tọa độ từng mắt cho `(x5-x1)/4` trước khi gửi `X=`.

Hai lượt xung 15 giây đều `11111` trước, trong và sau khi quay. Tình trạng này phù hợp với việc không có nền phản xạ ở độ cao làm việc, hoặc với một vấn đề chung về nguồn hoặc cảm biến. Lượt +50 và lượt ngắn +60 cũng có thay đổi nền trước xung. Để tách nhiễu điện, giữ robot và tấm trắng cố định đúng độ cao chạy, lấy các cửa sổ OFF–ON–OFF với tấm trắng giữ nguyên và đo 3V3/VM đồng thời. Hệ số PWM khởi động/duy trì cho mỗi bánh và chiều, dòng pin và độ lệch khi chạy thẳng đang chờ phép đo. Mặc định `FF=0,0,0,0/0,0,0,0` và `TRIM=4` được giữ làm điểm xuất phát. [Pololu QTR](https://www.pololu.com/docs/0J19?section=3) · [Vishay TCRT5000](https://www.vishay.com/docs/83760/tcrt5000.pdf) · [Espressif ADC](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html) · [Hiệu chuẩn ADC ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc/adc_calibration.html)

### Hành vi khi mất line và các thử thách HRC

Khi vừa ở giữa vạch với độ tin cậy cao, V5 cho tiến chậm tối đa 20 ms rồi xoay tại chỗ theo hướng được xác nhận gần nhất. Nếu vạch vẫn mất, V5 thử hướng ngược và dừng khi chạm 300 ms. Mẫu tách cụm hoặc vạch phủ quá rộng cho duty bằng 0 ngay. Nếu tình trạng kéo dài 80 ms, firmware dừng và DISARM với `LINE_AMBIGUOUS`. `11111` được lưu vào log.

Bản thể lệ địa phương ghi vạch rộng 18 ± 2 mm, khoảng đứt 25–75 mm, đảo màu, ngã rẽ/đường cụt, né vật cản và cổng tính giờ theo hướng dẫn BTC. Profile này dành cho bám line đen liên tục. Các thử thách đó cần bố trí và xác nhận riêng. GPIO TRIG/ECHO của HC-SR04P trong mã hiện tại là `-1` và chỉ là chỗ giữ. Chưa có bộ đọc siêu âm đang chạy trong bản nguồn hoặc bài kiểm tra phần cứng cho chức năng này. [Bản thể lệ công khai](https://www.studocu.vn/vn/document/truong-dai-hoc-bach-khoa-ha-noi/de-thi-giua-ki/the-le-cuoc-thi-hrc-2026-hust-robotics-competition/179209321?origin=course-new-1)

## Tái tạo bản V5

Ngắt VM trước khi nạp. Dùng PlatformIO Core 6.2.0 với `espressif32` 8.0.0 và Arduino-ESP32 3.3.12 như lượt thử. Xác định thiết bị Serial hiện tại bằng `pio device list`.

```sh
python3 tests/run_host_tests.py
pio run -e esp32dev
pio run -e esp32dev -t upload --upload-port /dev/cu.usbserial-120
```

Serial 115200. Các cấu hình viết hoa và áp dụng khi dừng. `SAVE` ghi profile V5 vào NVS, tách khỏi key của V6/V7. Chỉ lưu sau khi chốt profile. Ở lần nạp mới, profile bench được dùng để hiển thị và `GO` yêu cầu `CAL=` hoặc profile đã lưu. Đặt line gần giữa và đợi ba mẫu tin cậy sau `ARM`.

```text
MODE LINE_FOLLOW
CAL=240,227,206,197,1186/3851,4095,3671,3579,4095/360,310,320,315,2190
KP=34
KD=0.45
SPD=60
TRIM=4
LIMIT=1500
GET
CAL?
```

`GO` chỉ sau khi người đứng cạnh robot đã xác nhận lượt. `!` dừng và DISARM ngay khi nhận byte. `STOP` dừng, giữ ARM. `DISARM` khóa lại. Nút BOOT bắt đầu đếm ngược sau một lần nhấn–thả khi đang dừng. Nhấn BOOT trong countdown hoặc khi chạy sẽ dừng. `LIMIT=1500` tính bằng ms. `PULSE=1000` giới hạn xung MOTOR_TEST cả hai chiều. `FF=` nhận thứ tự `L+,L-,R+,R-` cho 4 giá trị khởi động, dấu `/`, rồi 4 giá trị duy trì. Dùng `TRACE=1` ở chế độ cảm biến hoặc motor để ghi trace 50 Hz.

```sh
python3 scripts/line_trial.py trace docs/evidence/line-follow-v5/v5-sweep-all.txt.gz
python3 scripts/line_trial.py compare docs/evidence/line-follow-v5/v5-r2-usb-control.txt.gz
```

`capture` dùng Python có `pyserial`, chẳng hạn `~/.platformio/penv/bin/python`. Mỗi file so sánh chứa một lượt, với `LINE_CFG`, `LINE_CAL`, `LINE_STATS` và `LINE_TIMING`. Để thêm kết quả độc lập, `--observations` nhận CSV có cột `log,laps,lap_ms,interventions,evidence`. Để giữ các lượt so sánh tương đương, ghi cùng mốc bắt đầu/kết thúc, loại pin, mặt sàn và người quan sát. Bắt đầu SPD 60, thử ba lượt tương đương trước khi tăng và chốt tốc độ theo ít nhất năm vòng liên tiếp có bằng chứng.

### Rollback

- `codex/line-follow-v4` là mã nguồn đầy đủ từ patch rollback V4 tìm được, dựa trên `5a22c31`. Tạo worktree riêng bằng `git worktree add ../HUSTROY-HRC2026-v4 codex/line-follow-v4` để build mà không chạm vào bản đang làm.
- `codex/line-follow-pre-v5` giữ nguyên mã nguồn local lúc bắt đầu. `.pio/line-follow-rollback/` có patch, binary build V7 và bản đọc vùng application của ESP32 V6 từ `0x10000`. Không đọc NVS của bo. Giữ bản sao thư mục này nếu dọn `.pio`.
- Log V3 gốc được lưu dưới `docs/evidence/line-follow-v5/v3-original-serial.txt`. Chưa tìm thấy snapshot mã nguồn hoặc binary V3 đầy đủ trong Git hay các bản sao tại máy.
