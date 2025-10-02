# QoS Setting dengan Image Transport

Node ini berfungsi untuk mengkonversi QoS settings dan mendukung compressed image transport.

## Error yang Diperbaiki:

### Error QoS Syntax:
- **Problem**: `sub_qos.reliable()` dan `sub_qos.best_effort()` tidak bekerja
- **Solution**: Menggunakan `sub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable)`

### Error TransportHints:
- **Problem**: `image_transport::TransportHints` constructor tidak sesuai
- **Solution**: Menggunakan approach sederhana tanpa custom transport hints

### Error dengan image_transport + custom QoS:
- **Problem**: `image_transport` tidak mendukung custom QoS secara langsung
- **Solution**: Membuat versi hybrid yang menggunakan `rclcpp` subscriber/publisher

## File yang Dimodifikasi/Dibuat:

1. **qos_setting.cpp** - Versi basic menggunakan image_transport
2. **qos_setting_advanced.cpp** - Versi advanced dengan image_transport (QoS terbatas)
3. **qos_setting_hybrid.cpp** - Versi hybrid dengan custom QoS + compressed support
4. **qos_setting_launch.py** - Launch file untuk semua versi

## Fitur Masing-masing Versi:

### Basic Version (qos_setting.cpp):
- Menggunakan `image_transport` untuk mendukung compressed images
- Subscribe dari `/camera_image_gray`
- Publish ke `/topic_best_effort`
- QoS default dari image_transport

### Advanced Version (qos_setting_advanced.cpp):
- Parameter yang dapat dikonfigurasi untuk topic names
- Menggunakan image_transport (QoS terbatas)
- Otomatis mendukung raw dan compressed formats

### Hybrid Version (qos_setting_hybrid.cpp) - RECOMMENDED:
- **Full custom QoS support** (Reliable/Best Effort)
- **Manual compressed/raw selection**
- Parameter yang dapat dikonfigurasi:
  - `input_topic`: Topic input (default: "/camera_image_gray")
  - `output_topic`: Topic output (default: "/topic_best_effort")
  - `queue_size`: Ukuran queue (default: 10)
  - `input_reliable`: QoS reliable untuk input (default: true)
  - `output_reliable`: QoS reliable untuk output (default: false)
  - `use_compressed`: true untuk compressed, false untuk raw (default: false)

## Cara Menggunakan:

### Build:
```bash
cd /home/ichbinwil/Documents/Rover_ws
colcon build --packages-select rover_middleware
source install/setup.bash
```

### Run Basic Version:
```bash
ros2 run rover_middleware qos_convert
```

### Run Advanced Version (image_transport, QoS terbatas):
```bash
ros2 run rover_middleware qos_setting_advanced
```

### Run Hybrid Version (RECOMMENDED - full QoS + compressed):
```bash
ros2 run rover_middleware qos_setting_hybrid
```

### Run dengan Parameters - Raw Images:
```bash
ros2 run rover_middleware qos_setting_hybrid --ros-args \
  -p input_topic:="/camera/image_raw" \
  -p output_topic:="/processed/image" \
  -p use_compressed:=false \
  -p input_reliable:=true \
  -p output_reliable:=false
```

### Run dengan Parameters - Compressed Images:
```bash
ros2 run rover_middleware qos_setting_hybrid --ros-args \
  -p input_topic:="/camera/image_raw" \
  -p output_topic:="/processed/image" \
  -p use_compressed:=true \
  -p input_reliable:=true \
  -p output_reliable:=false
```

### Run dengan Launch File:
```bash
ros2 launch rover_middleware qos_setting_launch.py
```

## Compressed Image Support:

Dengan menggunakan `image_transport`, node akan otomatis:
- Subscribe ke topic compressed jika tersedia (`/topic/compressed`)
- Publish dalam format raw dan compressed
- Handle konversi otomatis antara format

## Topic yang Akan Tersedia:

Jika output topic adalah `/topic_best_effort`, maka akan tersedia:
- `/topic_best_effort` (raw image)
- `/topic_best_effort/compressed` (compressed image)
- `/topic_best_effort/compressedDepth` (jika depth image)
- `/topic_best_effort/theora` (jika video stream)

## Monitoring:

Untuk melihat topic yang tersedia:
```bash
ros2 topic list | grep image
```

Untuk melihat info topic:
```bash
ros2 topic info /topic_best_effort
ros2 topic info /topic_best_effort/compressed
```

Untuk monitor bandwidth:
```bash
ros2 topic bw /topic_best_effort
ros2 topic bw /topic_best_effort/compressed
```
