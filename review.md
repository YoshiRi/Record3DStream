# コードレビュー — Record3DStream

対象コミット: `005a337`
レビュー日: 2026-04-25

---

## バグ（要修正）

### B1: `_broadcast_pose` が一度も呼ばれていない — TF ツリーが不完全

**ファイル**: `ros2-driver/ros2_driver/iphone_sensor_node.py:521`

`_broadcast_pose()` が定義されているが `_process_frame()` から呼び出されていない。`frame.transform`（ARKitカメラ姿勢）が完全に未使用。`world → camera_link` TFが一切パブリッシュされず、光学フレームが宙に浮く。RViz2やSLAMとの連携が全て壊れる。

**修正**: `_process_frame` 末尾に追加:
```python
self._broadcast_pose(frame.transform, stamp)
```

---

### B2: マジックバイト "REC3D" が recv 境界をまたぐとバッファを誤って消去する

**ファイル**: `sdk/sdk/client.py:225-229`

バッファ末尾が `b"REC3"` の状態で次の `recv()` が来た場合、`buffer.find(b"REC3D")` は -1 を返し `clear()` が走る。マジック文字列の先頭4バイトが捨てられ、デフォルト同期が壊れる。

**修正**:
```python
elif magic_pos < 0:
    self._buffer = self._buffer[-4:]  # "REC3" がまたいでいる可能性
```

---

## パフォーマンス

### P1: バッファスライスが毎フレーム O(n) のメモリコピーを発生させる

**ファイル**: `sdk/sdk/client.py:238`

```python
self._buffer = self._buffer[packet_size:]
```

`bytearray` スライスは毎回全コピーを生成する。30fps・パケット約2MBで毎秒約60MBの無駄なコピー。

**修正案**: 読み取り位置 `int` を管理し一定間隔でtrimする:
```python
self._buf_pos = 0
# パース後:
self._buf_pos += packet_size
# 定期的にtrim:
if self._buf_pos > 65536:
    del self._buffer[:self._buf_pos]
    self._buf_pos = 0
```

---

### P2: カラー画像の回転（8MB）が受信スレッドをブロックする

**ファイル**: `ros2-driver/ros2_driver/iphone_sensor_node.py:212`

1920×1440×3 ≈ 8MB のカラー回転をcritical pathで実行。デプスは受信スレッドでパブリッシュするため必要だが、カラー回転はheavy workerへ移動できる。

---

### P3: ヘッダーを1パケットごとに2回パースしている

**ファイル**: `sdk/sdk/client.py:222` / `sdk/sdk/protocol.py:228`

`get_packet_size()` と `parse_frame()` がそれぞれ独立して `parse_header()` を内部で呼ぶ。`parse_frame(data, header=None)` のように渡せるようにすると無駄が減る。

---

### P4: ポイントクラウドの構造体配列コピーが冗長

**ファイル**: `ros2-driver/ros2_driver/iphone_sensor_node.py:455-464`

構造体型配列へのフィールド代入はコピーが発生する。`np.empty((n_points, 4), dtype=np.float32)` に直接代入してから `tobytes()` にすると1アロケーションで済む。

---

## 設計

### D1: IMU 共分散がゼロのまま — 「誤差なし」を意味してしまう

**ファイル**: `ros2-driver/ros2_driver/iphone_sensor_node.py:262`

`linear_acceleration_covariance` と `angular_velocity_covariance` が初期値ゼロ（= 「完全に正確」）のまま。SLAMが誤った信頼度を使う。未知の場合は `[0] = -1.0` にすべき:
```python
imu_msg.linear_acceleration_covariance[0] = -1.0
imu_msg.angular_velocity_covariance[0] = -1.0
```

---

### D2: デプスフィルタのパラメータがハードコード

**ファイル**: `ros2-driver/ros2_driver/iphone_sensor_node.py:128-129`

`_temporal_alpha=0.4`, `_temporal_delta=0.04`, `bilateral_d=5` 等が定数。環境に応じた調整ができないため ROS2 パラメータとして公開すべき。

---

### D3: 再接続ロジックがない

**ファイル**: `ros2-driver/ros2_driver/iphone_sensor_node.py:170-173`

WiFi切断後は受信スレッドが終了し、ノード再起動しか手段がない。エクスポネンシャルバックオフによる再接続ループが必要。

---

### D4: LaserScan が未フィルタのデプスを使用

**ファイル**: `ros2-driver/ros2_driver/iphone_sensor_node.py:266-270`

低レイテンシを優先して raw デプスからスキャンを生成しているが、ナビゲーション用途ではノイズが問題になりやすい。`publish_raw_scan` / フィルタ済みスキャンを選べるパラメータが望ましい。

---

## 細かい点

### M1: `struct` がインポートされているが未使用

**ファイル**: `ros2-driver/ros2_driver/iphone_sensor_node.py:11`

`import struct` が残っているが使われていない。

---

### M2: `_frames_dropped` のカウント意味論がずれている

**ファイル**: `sdk/sdk/client.py:190`

空きを作って再投入に成功した場合でも `_frames_dropped` がインクリメントされる。実際には届いているフレームを「ドロップ」と数えている。

---

### M3: `stop()` が未接続時でも "Disconnected" を出力

**ファイル**: `sdk/sdk/client.py:134`

`_running` フラグで分岐させると自然。

---

## 優先度サマリー

| 優先度 | 番号 | 内容 |
|--------|------|------|
| **即修正** | B1 | `_broadcast_pose` 未呼び出し — TF tree が全断 |
| **即修正** | B2 | magic byte 境界バグでデータ消失 |
| 高 | D1 | IMU共分散ゼロ — SLAMが誤った信頼度を仮定 |
| 高 | D3 | 再接続なし — WiFi切断でノード死亡 |
| 中 | P1 | バッファスライスO(n) — CPU無駄 |
| 中 | P2 | カラー回転がcritical path |
| 中 | D2 | フィルタパラメータ非公開 |
| 低 | P3, P4, D4, M1–M3 | その他最適化・細かい修正 |
