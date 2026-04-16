# Vehicle CAN Bridge — 初期導入スコープ（Phase 1）

本ドキュメントは、`config/vehicle_schema.yaml` に定義された共通スキーマ全体のうち、
**初期導入時に有効化するサブセット**とその選定理由を定める。

共通スキーマはあらゆる車両（手動運転／ADAS 搭載／AD 搭載）を包含する広範な定義であるため、
一度に全ドメインを導入すると、対象車両で存在しない信号が `STATUS_INITIAL` で埋まり、
カバレッジ評価・データ利用側の実装が煩雑になる。
そこで本プロジェクトでは段階的に導入する方針とし、Phase 1 のスコープを本ドキュメントで確定する。

---

## 1. 対象車両の前提

- **用途**: 手動運転を前提としたデータ計測車両
- **AD / ADAS 機能**: **非搭載**（あるいは計測対象外）
- **位置情報**: 専用の GNSS / INS デバイスが別トピックで一次ソースを提供
- **主なデータ利用目的**:
  - ドライバー挙動（操作入力）と車両応答（運動状態）の対応関係の解析
  - 車線変更・右左折などのドライバー意図イベントの抽出・セグメンテーション
  - 夜間・トンネル等の環境タグ付け

---

## 2. Phase 1 有効化ドメイン

`schema_domain_names` を以下の **3 ドメイン**に限定する。

| Domain      | Topic                | 役割                         |
| ----------- | -------------------- | ---------------------------- |
| `dynamics`  | `/vehicle/dynamics`  | 車両運動状態（真値）         |
| `operation` | `/vehicle/operation` | ドライバー操作入力・車両応答 |
| `body`      | `/vehicle/body`      | ドライバー意図・環境タグ     |

> `body` ドメインは全信号ではなく、後述の**最小サブセット**のみを `schema.body.signals` に列挙する。

---

## 3. ドメイン別 採否と理由

### DBC カバレッジ凡例

| 記号 | 意味                                     |
| ---- | ---------------------------------------- |
| P    | AStuff PACMod v3 (`as_pacmod.dbc`)       |
| T    | commaai Toyota 2017 (`_toyota_2017.dbc`) |
| P+T  | 両 DBC で対応                            |
| \*   | いずれの DBC にも未収録                  |

### 3.1 採用ドメイン

#### `dynamics` — 走行状態の真値

データセットの**背骨**。軌跡解析・挙動解析・スリップ検出など、
あらゆる下流解析の基準となるため、全信号を採用する。

| 採用信号                            | DBC | 採用理由                                       |
| ----------------------------------- | --- | ---------------------------------------------- |
| `dynamics.speed.longitudinal`       | P+T | 最も基本的な走行速度                           |
| `dynamics.speed.lateral`            | \*  | 横滑り・旋回挙動の解析に必要                   |
| `dynamics.wheel_speed.*`（4 輪）    | P+T | スリップ検出・ABS 挙動解析の基礎               |
| `dynamics.accel.longitudinal`       | P+T | 加減速挙動                                     |
| `dynamics.accel.lateral`            | P+T | 旋回加速度                                     |
| `dynamics.accel.vertical`           | P   | 路面状況・段差イベント検出                     |
| `dynamics.angular_vel.yaw`          | P+T | 旋回率（経路追従解析の基礎）                   |
| `dynamics.angular_vel.pitch / roll` | P   | 車体姿勢変化                                   |
| `dynamics.steering.angle / rate`    | T   | コラムセンサーによる実操舵量（ドライバー入力） |
| `dynamics.steering.torque_driver`   | T   | ハンドル操舵トルク（ドライバー負荷）           |
| `dynamics.steering.torque_eps`      | T   | EPS アシストトルク                             |

#### `operation` — ドライバー操作入力と車両応答

「**なぜその挙動になったか**」の因果関係を説明するために必須。
手動運転が前提のため、`*.command.*` や `*.manual_input` は車両によっては
`STATUS_INITIAL` になるが、DBC カバレッジ情報として意味があるためそのまま採用する。

| 採用信号                                 | DBC | 採用理由                                       |
| ---------------------------------------- | --- | ---------------------------------------------- |
| `operation.steering.command.*`           | P   | 操舵コマンド（DBW 指令値）                     |
| `operation.steering.report.angle`        | P+T | 操舵応答角度                                   |
| `operation.steering.report.torque`       | P   | 操舵応答トルク                                 |
| `operation.steering.manual_input`        | P   | ドライバー手動操舵角                           |
| `operation.throttle.command.position`    | P   | アクセルコマンド                               |
| `operation.throttle.report.position`     | P+T | アクセル応答                                   |
| `operation.throttle.manual_input`        | P   | ドライバー手動アクセル                         |
| `operation.brake.command.position`       | P   | ブレーキコマンド                               |
| `operation.brake.report.position`        | P+T | ブレーキ応答                                   |
| `operation.brake.manual_input`           | P   | ドライバー手動ブレーキ                         |
| `operation.brake.decel_command`          | P   | 減速要求（XBR）                                |
| `operation.brake.pressed`                | T   | ブレーキ踏下状態                               |
| `operation.shift.command / manual_input` | P   | ギアコマンド・手動操作                         |
| `operation.shift.report`                 | P+T | ギア応答                                       |
| `operation.parking_brake.command`        | P   | パーキングブレーキ操作                         |
| `operation.parking_brake.report`         | P   | 駐車／発進区間のセグメント化に有用             |
| `operation.engaged`                      | P   | DBW 制御中フラグ（手動運転車両でも状態確認用） |

#### `body`（最小サブセット） — ドライバー意図・環境タグ

ドライバーの**意図**とシーンの**環境条件**をイベントタグとして付与するための最小限の信号のみ。

| 採用信号                     | DBC | 採用理由                                                       |
| ---------------------------- | --- | -------------------------------------------------------------- |
| `body.lights.turn_signal`    | P+T | 車線変更・右左折イベントの抽出（ドライバー意図の主要シグナル） |
| `body.lights.headlight_mode` | P+T | 夜間／トンネル走行区間のタグ付け                               |
| `body.lights.brake_lights`   | P   | ブレーキランプ実挙動（`operation.brake.pressed` の冗長確認）   |

---

### 3.2 除外ドメイン（Phase 2 以降）

| Domain         | 除外理由                                                                                                                              |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| `system`       | AD / ADAS 非搭載車両では autonomy FSM・safety FSM・override 等の概念が無く、常に固定値（`STATUS_INITIAL` 相当）となり情報価値が無い   |
| `location`     | GNSS / INS 専用デバイスが別トピックで一次ソースを提供しており、CAN 由来の位置情報は更新レート・精度ともに劣るため重複させる意義が薄い |
| `powertrain`   | 診断用途（エンジン・燃料・オイル等）であり、挙動解析の初期スコープには不要                                                            |
| `chassis`      | タイヤ圧・ブレーキ圧・ABS/TCS/ESC 介入などは Phase 2 で必要性を再評価                                                                 |
| `adas`         | 本車両は ADAS 機能を持たないため全項目が `STATUS_INITIAL` となる                                                                      |
| `body`（残り） | door / seatbelt / occupancy / climate / fog・reverse・daytime ランプは走行挙動解析の初期スコープ外                                    |

#### 将来的な単独追加候補

除外ドメイン内でも、以下はコスト対効果が比較的高く、Phase 2 検討候補に挙げておく。

- `location.odometer` — GNSS からは得られない独立情報（DBC にあれば拾う価値あり）
- `chassis.safety.abs_active / tcs_active / esc_active` — 介入イベントタグ付け
- `body.lights.fog_lights_front` / `reverse_lights` — 環境タグ・後退イベント
- `operation.engaged` — （既に Phase 1 に含むが、AD 化する将来に備え `system.autonomy.*` と合わせて再評価）

---

## 4. Phase 1 の設定例

```yaml
vehicle_can_node:
  ros__parameters:
    schema_domain_names:
      - dynamics
      - operation
      - body

    # dynamics / operation はスキーマ全信号をそのまま採用する。
    # （config/vehicle_schema.yaml の該当セクションをそのまま使う）

    # body は最小サブセットに上書き
    schema.body.topic: /vehicle/body
    schema.body.signals:
      - body.lights.turn_signal
      - body.lights.headlight_mode
      - body.lights.brake_lights
```

> 実運用では `config/vehicle_schema.yaml` を直接編集するのではなく、
> 起動時に読み込むパラメータファイルを分けて上書きする構成が望ましい。

---

## 5. DBC カバレッジ評価の指針

- Phase 1 で有効化した信号のうち、**DBC でマッピングされた信号の割合**を「有効カバレッジ」として評価する。
- `STATUS_INITIAL` のまま発行される信号は、カバレッジ分母から除外せず**未カバー**として記録する。
  - これにより、DBC 追補時のターゲットを明確化できる。
- Phase 1 サブセットに対するカバレッジ評価を先行させ、Phase 2 拡張時は新規ドメインのカバレッジを別途評価する。

---

## 6. Phase 進行ロードマップ（参考）

| Phase | 追加ドメイン／信号                                        | 想定トリガ                                         |
| ----- | --------------------------------------------------------- | -------------------------------------------------- |
| 1     | `dynamics` / `operation` / `body`（最小）                 | 初期導入（本ドキュメント）                         |
| 2     | `chassis.safety.*` / `body`（拡張） / `location.odometer` | 介入イベント解析・環境タグ拡張の必要性が発生した時 |
| 3     | `powertrain` / `chassis`（圧力系）                        | 車両診断データの利用要件が発生した時               |
| 4     | `system` / `adas`                                         | AD / ADAS 搭載車両の計測開始時                     |

---

## 関連ドキュメント

- [`abstracted_schema.md`](./abstracted_schema.md) — 共通スキーマ全体の正準信号カタログ
- [`config/vehicle_schema.yaml`](../config/vehicle_schema.yaml) — スキーマ定義本体
