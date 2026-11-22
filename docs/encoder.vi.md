# Encoder bánh xe · Ghi chú khảo sát

Mục tiêu cho xe HRC2026 là đo tốc độ từng bánh và chênh lệch trái/phải. Dùng số đo để kiểm tra việc đi thẳng ở cùng PWM trước khi tích hợp vòng điều khiển tốc độ. PID bám line vẫn lấy sai số từ dãy cảm biến, encoder đo chuyển động quay của bánh.

## Xác minh phần cứng

1. Lấy mã motor/encoder và datasheet. Ghi kiểu ngõ ra (A/B quadrature hoặc một kênh), nguồn nuôi, mức logic, số xung ở trục nào, tỉ số hộp số và màu dây. Đo hoặc xác minh mức logic 3,3 V trước khi nối ESP32. Kiểm tra nhu cầu kéo lên và nối chung GND khi nguồn phù hợp.
2. Đối chiếu `include/config.h`. TB6612 đã dùng GPIO 18/13/14/19/21/22/23 và LED dùng GPIO 2. Năm cảm biến, nút mode và các vị trí đặt trước cần được kiểm tra theo cấu hình dây thực tế. Encoder A/B hai bánh cần bốn ngõ vào. Nếu dùng GPIO 34–39 đang rảnh, kiểm tra kéo mức ngoài vì các chân này không có điện trở kéo trong. Chốt GPIO sau khi chốt dây cảm biến.
3. Dùng bộ đếm xung **PCNT** của ESP32. Hai kênh A/B lệch pha cho biết chiều quay. PCNT kết hợp cạnh của một kênh với trạng thái kênh kia để cộng hoặc trừ. Với đếm ×4, đếm cả hai cạnh của cả hai kênh. Cấu hình lọc glitch theo xung nhanh nhất thực tế và xử lý giới hạn bộ đếm nếu tích lũy liên tục. [Tài liệu PCNT và ví dụ encoder của Espressif](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/pcnt.html)

## Quy trình đo đề xuất

- Kê bánh, đánh dấu vị trí. Quay tay 10 vòng theo chiều tiến rồi 10 vòng chiều lùi, làm cho từng bánh. Ghi tổng xung có dấu, chia độ lớn cho 10 để tính `counts_per_wheel_rev`. Nếu datasheet cho PPR là số chu kỳ một kênh trên trục motor, dự báo số đếm bằng `PPR × 4 × tỉ số truyền` ở chế độ ×4. Làm rõ cách nhà sản xuất định nghĩa PPR/CPR trước khi nhân hệ số.
- Chuẩn hóa dấu để cả hai bánh quay theo chiều tiến đều cho số đếm dương. Khi bánh đứng yên, số đếm phải giữ nguyên. Lặp lại 5 lần mỗi tình huống và ghi xung, chênh so với dự báo và lỗi nếu có.
- Đọc chênh lệch đếm `Δcount` và thời gian thực `Δt` giữa hai lần lấy mẫu. Tính `RPM = 60 × Δcount / (counts_per_wheel_rev × Δt_s)`. Nếu cần quãng đường, dùng `π × đường kính bánh × Δcount / counts_per_wheel_rev`. So sánh RPM trái/phải ở một số mức PWM với bánh kê, sau đó trên sân.
- Khi đã xác nhận chân, chiều và số đếm, bổ sung `RPM_L/R` vào log. Sự khác biệt RPM ở cùng duty là dữ liệu để cân chỉnh hai bánh. Repo hiện chưa xác nhận model encoder hoặc pinout nên bản này chưa khởi tạo encoder.
