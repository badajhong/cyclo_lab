# NRF 토스트 환경 작업본: 복구·고정 시나리오·검증

Fork: https://github.com/kkkkkmj/cyclo_lab.git
브랜치: `toast-scenarios-batch02`. 이 문서는 공유 준비 설명이며 commit/push 완료를 뜻하지 않습니다.
원본 `README.md`, `README_TOAST.md` 및 기존 로컬 변경은 보존합니다.

전체 작업본은 이 Fork에, 수집 담당자에게 전달할 최소 데이터는
[scenario_delivery/README.md](scenario_delivery/README.md)에 정리합니다.

## 1. 누락된 Isaac Lab datasets 소스 복구

전달본에서 `isaaclab.utils.datasets`가 빠져 import가 실패했습니다.
루트 `.gitignore`의 `datasets` 규칙이 실제 시연 데이터뿐 아니라 같은 이름의
Python package도 제외한 상태였습니다.

복원 출처는 공식 `isaac-sim/IsaacLab`의 **v2.3.0**입니다.

- tag commit: `3c6e67bb5c7ada942a6d1884ab69338f57596f77`
- 로컬 참조: `/home/mj/nrf_humanoid/reference_sources/IsaacLab-v2.3.0`
- 공식 경로: https://github.com/isaac-sim/IsaacLab/tree/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab/isaaclab/utils/datasets
- 복원 위치: `third_party/IsaacLab/source/isaaclab/isaaclab/utils/datasets/`
- 파일: `__init__.py`, `dataset_file_handler_base.py`, `episode_data.py`, `hdf5_dataset_file_handler.py`.

4개 파일은 공식 로컬 원본과 SHA-256이 각각 일치합니다. Isaac Lab 전체 교체나
최신 버전 업그레이드, stub 대체를 하지 않았습니다. VERSION만 같다는 이유로
전달본 전체가 공식 소스와 동일하다고 취급하지 않습니다.

`.gitignore`에서는 기존 `datasets` 규칙을 유지하고, 위 package 경로와 그 안의
`*.py`만 추적 가능하게 예외 처리했습니다. 실제 시연 데이터 제외는 유지합니다.
이 변경은 기존 작업이며 이번 공유 준비에서 다시 수정하지 않았습니다.

## 2. 고정 배치 구현

대상 task는 `Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2`입니다.

- scene 생성 전에 `cfg.scenario_id` 및 선택한 manifest의 최종 pose 적용.
- 거치대 pose 변경에 빵 4개를 함께 변환하며 상대 배치를 보존.
- 원본 Z·scale 유지. 기존 reset이 선택한 초기 상태를 복원.
- 최종 position/quaternion을 저장하므로 seed만으로 재생성하는 방식이 아님.
- 기본 `scenario_id=None` 및 기존 pilot ID는 보존.
- 물리 설정·expert·성공 조건을 완화하지 않음.

batch02는 물체 순서 6종을 17/17/17/17/16/16개로 구성한 100개입니다.
토스터 기준 yaw 150°, 접시 0°, 거치대 -90°에 각각 ±5°를 적용했습니다.
환경 +Y→-Y 슬롯은 +0.40/0/-0.40 m, X 기준은 토스터 .55/접시 .49/거치대 .46 m입니다.
접시 X만 ±.010 m이고, 나머지 각 XY 이동은 ±.015 m입니다.
A/B 각각 50개, 순열별 인원 차이는 최대 1개입니다.
A/B는 수집 담당 구분이며 장비나 방법을 지정하지 않습니다.

geometry 증거를 재사용해 실제 상판 지지 영역, scale/yaw 반영 footprint,
최소 간격, 평면 접근 여유, 양팔/lift 모델 기반 보수적 도달 상한을 검사했습니다.
visual bbox·authored collider bounds는 PhysX cooked collision/실제 접촉 검사와 구분합니다.
기준은 metadata 여유 1 mm, 상판 여유 20 mm, 물체 간격 30 mm, 평면 접근 여유 60 mm입니다.

## 3. batch02 실행 검증 결과와 한계

[검증 요약](scenarios/make_toast_v2_batch02/runtime_review/summary.json),
[담당자별 검증표](scenarios/make_toast_v2_batch02/runtime_review/operator_validation.csv).

- 원본 결과: `logs/scenario_batch02_six.46bMoE/results` 및
  `logs/scenario_batch02_remaining.ucq8XR/results`.
- 두 summary의 manifest SHA-256과 현재 batch02가 일치.
- 정확히 100개 ID, 누락/중복 없음. 모두 exit 0 및 초기 pose·reset 2회 PASS.
- operator_A 50개 / operator_B 50개 모두 배치/reset 검증 완료.
- 초기 위치 최대 오차 약 2.97e-8 m, 기록된 reset 상태의 최대 scalar 차이 0.
- 1,500 PNG 무결성 확인, 전체 ID의 초기/두 정착 head 영상 300장 육안 검토.
  보이는 영역에서 뚜렷한 이탈·전도·빵 낙하 후보는 없었습니다.
  화면 가장자리 잘림과 가려진 접촉면은 미확인입니다.

**실제 조작·IK·충돌 없는 접근 경로·토스트 완주·시연 수집 성공은 미검증**입니다.
기존 원본 expert 1회 검사는 접시 서빙 release 단계에서 실패했으며,
배치/reset PASS를 expert 성공으로 해석하지 않습니다.
정착 중 접시·빵의 하강과 기울어짐이 있고, PASS는 물체가 전혀 움직이지 않는다는 뜻이 아닙니다.

manifest의 `CANDIDATES_NOT_GPU_VERIFIED`/`gpu_verified: 0`은 생성 당시 기록입니다.
데이터를 덮어쓰지 않고 runtime 검증 요약을 manifest 해시로 연결해 상태를 구분했습니다.

## 4. 실행 방법 — 명령 예시이며 공유 준비 중 실행하지 않음

기존 Isaac Sim 5.1.0 / Isaac Lab 2.3.0 Docker 환경과 필요한 asset이 준비되어 있어야 합니다.
아래는 저장소가 `/workspace/cyclo_lab`에 있는 기존 `cyclo_lab` 컨테이너 기준입니다.
GPU 작업·메모리 여유를 먼저 확인하며, 다른 연구 환경이나 컨테이너를 변경할 필요가 없습니다.

### 원본 기본 배치의 expert 진단

시나리오를 지정하지 않으면 원본 배치입니다. 다음은 컨테이너 내부 명령입니다.
시연 데이터는 만들지 않지만 expert를 실행하므로 필요할 때만 별도로 사용합니다.

```bash
cd /workspace/cyclo_lab
run_dir=$(mktemp -d /workspace/cyclo_lab/logs/original_toast_check.XXXXXX)
/isaac-sim/python.sh scripts/imitation_learning/isaaclab_recorder/generate_make_toast_demos.py \
  --check_only --max_attempts 1 --snapshots --headless \
  --report "$run_dir/report.json"
```

### 공유 manifest로 ID 적용 및 reset 확인

[전달물 사용법](scenario_delivery/README.md)의 호스트 명령을 사용합니다.
이 명령은 expert 없이 초기 배치와 reset을 검사합니다. GUI 확인은 `--headless`를 빼면 되며,
계속 켜두는 viewer가 아니라 유한 길이 검사입니다.

별도 코드에서는 `parse_env_cfg()` 후 **`gym.make()` 이전**에 다음 필드를 지정합니다.

```python
cfg.scenario_id = "batch02_toast_000"
cfg.scenario_manifest = "/workspace/cyclo_lab/scenario_delivery/manifest.json"
```

ID 변경은 새 cfg/environment로 수행합니다. 동일 환경에서 `env.reset()`하면 해당 초기 배치로 복원됩니다.
원본 upstream 코드에 데이터 폴더만 복사해서는 자동 적용되지 않습니다.
이 Fork의 `scenarios.py`, 환경 연결 변경 및 검사 도구가 필요합니다.

## 5. 증거 의존성과 재현 제한

고정 manifest 적용과 HTML 미리보기에는 과거 실행 로그가 필요하지 않습니다.
실제 환경 실행에는 원본 환경 의존성과 robot/object/table asset이 필요합니다.

다음 작업은 원본 증거 없이 그대로 재실행/재검토할 수 없습니다.

| 작업 | 별도 필요한 로컬 자료 |
|---|---|
| batch02 generator 재실행 | 기존 세 pilot `report.json`, pinned metadata/robot USD, 추출 `geometry.json` |
| audit 재실행 | 해당 geometry.json, manifest, 로컬 asset 해시 검사용 파일 |
| `validate_static.py` 전체 실행 | batch01, geometry, metadata/robot, preservation 기록이 참조하는 원본 파일들 |
| 과거 100개 runtime 결과 독립 재검토 | 두 실행 폴더의 원본 report/result/summary/log/PNG |
| 보존 해시 검사 | 기록에 열거된 과거 파일들; hash 목록만으로 원본을 복원할 수 없음 |

배포된 요약의 hash/path는 출처 기록이며 원본 파일을 내장하지 않습니다.
정적 코드 단위 테스트와 실제 geometry/시뮬 검증도 구분해야 합니다.

## 6. 공유 파일 목록과 권장 커밋 구성

현재 staged/unstaged 상태는 이번 준비에서 변경하지 않았습니다. 아래는 **향후 구성 권장안**이며,
아직 index 조정·commit·push를 수행하지 않았습니다.

### 권장 커밋 1 — 누락 모듈 복구

- `.gitignore`의 datasets Python 소스 예외 변경.
- 위 `third_party/IsaacLab/.../utils/datasets/` Python 4개 파일.

### 권장 커밋 2 — 시나리오 기능과 고정 배치·검증 요약

기존 base repository 위에 다음 파일을 포함합니다.

- `source/cyclo_lab/cyclo_lab/manager_based/manipulation/make_toast/config/ffw_bg2/`
  - `scenarios.py`, `scenario_pilots.json`, `bimanual_env.py`, `bimanual_env_cfg.py`.
- `scripts/tools/`
  - `generate_make_toast_scenarios.py`, `toast_batch_lib.py`
  - `check_make_toast_scenarios.py`, `run_make_toast_scenario_batch.py`
  - `export_make_toast_geometry.py`, `toast_geometry_support.py`, `audit_make_toast_scenarios.py`
  - `test_make_toast_scenarios.py`, `test_toast_batch.py`, `test_toast_geometry_support.py`
- `scenarios/make_toast_v2_batch01/`: 기존 calibration/비교 배치와 정적 전달물 보존.
- `scenarios/make_toast_v2_batch02/`: manifest·배정·HTML·생성/audit/정적 검증 JSON·
  사용법·검증 도구 및 `runtime_review/`의 JSON/CSV 요약과 해시 목록.
  이 폴더의 README/SHARING은 신규 시나리오 문서이며 루트 기존 README 변경과 구분합니다.

### 권장 커밋 3 — 작업 설명과 최소 전달 패키지

- 새 `README_WORK.md`.
- `scenario_delivery/README.md`, `scenario_delivery/ASSIGNMENTS.md`.
- `scenario_delivery/manifest.json`, `preview.html`: batch02 원본과 byte-identical 복사본.
- `scenario_delivery/assignments.csv`: 원본의 ID·담당자·순서를 그대로 유지한 3열 전달용 CSV.

상대방에게 따로 보내는 것은 **scenario_delivery의 위 5개 파일**입니다.
이 폴더는 원본 구현·asset·과거 로그를 포함하지 않는 배치 데이터 전달물입니다.

### 이번 업로드 대상에서 제외하고 로컬 보존

- 루트 `README.md`, `README_TOAST.md`의 기존 수정: 되돌리거나 삭제하지 않고 업로드 범위에서 제외.
- `scenarios/toaster_yaw_design01/`: 회전 확대 설계는 이번 ±5° batch02 공유 범위 밖.
- `README_QUEST.md`, `scripts/quest_diagnostic/`: 별도 작업이며 이번 범위 밖.
- 대형 USD/다운로드 asset, `logs/**`, 원본 PNG/MP4/HDF5, 학습 및 촉각 연구 자료.
  기존 base repository에 이미 포함된 파일을 삭제하라는 뜻은 아닙니다.
- HTML 미리보기는 자체 포함된 벡터 도면으로, 별도 대형 이미지 업로드가 필요하지 않습니다.

`scenarios/make_toast_v2_batch02/SHARING.md`에는 원본 증거를 별도 아카이브로
전달하는 선택지도 있으나, **현재 GitHub 업로드 대상은 위 목록을 우선**합니다.
원본 로그·이미지를 제외하므로 상대방이 원본 증거를 재검토하려면 별도로 요청해야 합니다.
