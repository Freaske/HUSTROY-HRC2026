# HUSTROY HRC2026 · Kịch bản test xe

## Chuẩn bị và ghi kết quả

- Chia nhiệm vụ điều khiển, bấm giờ và ghi kết quả. Ghi mã bản nạp, người test, tình trạng pin và mặt sân để đối chiếu các lượt.
- Xác nhận điện áp tín hiệu, ngưỡng và cực tính năm cảm biến, thứ tự trái → phải, chiều bánh và kiểm tra khi rút dây cảm biến. Firmware đọc thô năm ngõ AO trên GPIO 32/33/34/35/36. Chạy T01/T02 trước khi xác nhận `SENSOR_CALIBRATION_CONFIRMED` và `SENSOR_DISCONNECT_BIAS_CONFIRMED` để thử bám line.
- Dùng line rộng 18 ± 2 mm khi dựng bài tập theo thể lệ. R100 và góc nhọn là bài tập của đội. Thể lệ chỉ mô tả các dạng thử thách, còn bố trí sa bàn theo hướng dẫn của BTC.
- Chạy tối thiểu **5 lượt mỗi bài**, theo T01 → T08. Giữ nguyên cấu hình trong mỗi nhóm 5 lượt để so sánh. Nếu đổi cấu hình, giữ các kết quả cũ và thêm 5 lượt mới. Chỉ chuyển bài khi các lượt của cấu hình đang thử đều đạt. Ghi `Bị chặn` cùng nguyên nhân nếu chưa thể thực hiện, rồi thử lại khi đã xử lý.
- Trước từng lượt, ghi `KP`, `KD`, `SPD` từ `GET`. Với T02, ghi thêm lệnh PWM thực tế của bài thử. Bắt đầu bấm giờ từ thao tác kích hoạt, kết thúc tại vạch mốc, khi đã kiểm xong hoặc tại thời điểm xe dừng do lỗi. Ghi thời gian bằng giây, kể cả lượt không đạt đã chạy. Với lượt bị chặn trước khi chạy, để trống thời gian và ghi lý do.
- Ghi `Đạt`, `Không đạt` hoặc `Bị chặn`. Nếu có lỗi, ghi đoạn sân và giá trị `STOP`/`LINE` trong log kèm ảnh, video hoặc file log nếu có. Đưa xe về vị trí xuất phát cho từng lượt.

| Mã | Bài test | Cách làm | Tiêu chí đạt mỗi lượt |
| --- | --- | --- | --- |
| T01 | Cảm biến | Vào `MODE SENSOR_TEST`. Quét line qua từng mắt từ trái sang phải, qua giữa rồi bỏ line. Quan sát `RAW`, `BITS`, `POS` qua USB và kiểm tra mẫu phủ đủ năm mắt. | Từng mắt đổi đúng thứ tự và giữa cho vị trí xấp xỉ 0. Sau hiệu chuẩn, nền trống cho `LINE_LOST` và cả năm mắt active cho `INTERSECTION`. Motor đứng yên. |
| T02 | Động cơ | Kê hai bánh. Qua USB, gửi `MODE MOTOR_TEST`, `ARM`, lần lượt `LEFT 30`, `RIGHT 30`, `BOTH 30`, `STOP`. Gửi `DISARM` trước khi hạ xe. | Đúng bánh, đúng chiều và bánh còn lại đứng yên khi thử đơn. Cả hai bánh dừng khi `STOP`. Ghi PWM thực tế nếu phải đổi mức để thắng ma sát. |
| T03 | Bám line thẳng | Đánh dấu đầu và cuối đoạn thẳng 1 m nếu đủ sân. Đặt xe thẳng tại mốc đầu, `ARM`, đợi line ổn định rồi `GO`. Dừng bấm giờ khi xe qua mốc cuối và gửi `STOP`. | Xe đi hết đoạn, không cần chạm tay và không ra khỏi line. Không có `LINE_LOST`, `INTERSECTION` hoặc dừng ngoài dự kiến. |
| T04 | Cua R100 | Dùng đoạn cua R100 của sa bàn tập, có đoạn dẫn trước và sau cua. Đánh dấu mốc vào và ra, bấm giờ từ `GO` đến mốc ra. | Qua hết cung, bám line đến mốc ra và không cần chạm xe. Nếu có vật cản ngoài line, xe không được làm đổ. Ghi hướng cua. |
| T05 | Góc nhọn | Đánh dấu góc theo sa bàn. Đặt xe trên đoạn dẫn, `GO`, quan sát đường đi của xe và log đến mốc sau góc. | Xe đi đúng nhánh sau góc và đến mốc, không bỏ góc, mất line hoặc cần đặt lại xe. Ghi góc/hướng và vị trí nếu lỗi. |
| T06 | Ngã tư | Thống nhất nhánh hợp lệ trước mỗi loạt. Cho xe tiếp cận từ cùng một nhánh và quan sát đường ra cùng log tại giao điểm. | Xe tự đến mốc trên nhánh hợp lệ. Nếu vào nhánh sai, xe phải tự quay về tuyến và không đi tắt. Dừng ở `INTERSECTION` là không đạt. |
| T07 | Ô đích | Kẻ ô 400 × 300 mm và vạch đối chứng trước ô. Quan sát tín hiệu nhận đích và vị trí toàn xe từ trên xuống khi dừng. | Xe qua vạch đối chứng, nhận đúng ô và tự dừng với toàn bộ hình chiếu nằm trong ô. Dừng do `INTERSECTION`, `LINE_LOST` hoặc hết giờ là không đạt. |
| T08 | Chạy cả vòng | Đặt xe tại START. Bấm giờ từ lúc kích hoạt đến khi xe tự dừng. Sau khi xuất phát, quan sát và ghi kết quả mà không gửi lệnh. | Xe đi đúng tuyến, qua đủ mốc, tự xử lý các đoạn thử thách và dừng với toàn bộ hình chiếu trong ô đích. Hoàn thành trong thời gian đặt cho lượt. |

Khi thi, hoàn tất chỉnh tham số trước khi đặt xe vào START và xuất phát bằng nút theo hiệu lệnh. Bluetooth và Wi‑Fi trong tài liệu này dùng khi tập nội bộ. Thao tác chỉnh trong Pit Stop chỉ thực hiện sau khi trọng tài cho phép. [Thể lệ HRC 2026, mục 3.1, 4 và 6](https://drive.google.com/file/d/1PVf8mOg5K7oMSuhhUea1b_E_kdrq-ePu/view)

## Trạng thái firmware trước buổi test

`LINE_FOLLOW` hiện dừng và DISARM khi thấy `INTERSECTION` hoặc mất line. Logic chọn nhánh và nhận ô đích còn chờ tích hợp. T06/T07/T08 cần ghi nhận đúng lỗi hoặc lý do bị chặn trong bản hiện tại. Mỗi lần `GO` có giới hạn mặc định 5 giây. Khi tích hợp bài chạy dài hơn, đối chiếu giới hạn `LINE_FOLLOW_MAX_RUN_MS` với thời gian thực tế của sa bàn trước T08. Nếu T03–T05 chạm giới hạn, giữ lỗi `RUN_TIMEOUT` trong bảng để phân biệt với lỗi bám line.

## Kiểm tra công cụ chỉnh và log trước khi đặt xe xuống sân

1. Kê bánh, kết nối điện thoại theo [hướng dẫn chỉnh tham số](test-link.vi.md). Gửi `GET`, `KP=20`, `KD=0`, `SPD=50` rồi `GET` để kiểm tra giá trị. Khi gửi `SPD=151` và `KD=nan`, firmware phải báo lỗi và giữ giá trị trước đó.
2. Với phần cứng đã hiệu chuẩn, đặt mẫu line lệch nhẹ ổn định (ví dụ `01100` khi mắt ngoài cùng bên trái là chữ số đầu). Gửi `MODE LINE_FOLLOW`, `ARM`, đợi line ổn định, `GO`. Trong một lần thử, đổi `KP=30` khi e khác 0, kiểm tra chênh lệch `PWM_L`/`PWM_R` đổi trong log. Thử lại với `SPD=60`, xem mức PWM trung bình đổi. Kiểm tra PWM về 0 khi gửi `STOP`.
3. Gửi `SAVE` khi đã dừng, cắt nguồn rồi bật lại. `GET` phải trả lại giá trị đã lưu và xe vẫn ở trạng thái dừng. Thay tham số mà chưa `SAVE`, khởi động lại và kiểm tra giá trị đã lưu vẫn được nạp.
4. Trong một đoạn chạy ngắn, kiểm đếm các dòng `t=` Bluetooth nhận trong khoảng một giây. Mục tiêu là 10–20 dòng/giây khi kết nối ổn định. Lưu đoạn log có `MODE`, `LINE`, `e`, `PWM_L`, `PWM_R` và một lần `STOP`. Với Wi‑Fi, kiểm tra log cập nhật trong Safari và các lệnh trả phản hồi.
