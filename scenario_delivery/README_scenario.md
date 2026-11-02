# 토스트 배치 100개 전달물 — batch02

`manifest.json`은 ID별 최종 pose, `assignments.csv`는 담당자 배정입니다.
`preview.html`은 브라우저에서 바로 열 수 있으며 인터넷이나 별도 이미지가 필요 없습니다.

## 배치와 담당 ID

ID는 `batch02_toast_000`~`batch02_toast_099`입니다.
**operator_A/B 각각 50개**이며 `assignments.csv`에서 자신의 담당 ID를 선택합니다.
A/B는 **수집 담당자 구분**으로, 수집 장비·방법의 구분이 아닙니다.

좌우 순서는 환경 +Y→−Y, 즉 로봇 왼쪽→오른쪽입니다.
토스터/접시/거치대의 6가지 순열을 모두 포함합니다:
토-접-거, 토-거-접, 접-토-거, 접-거-토, 거-토-접, 거-접-토.
각 순열은 17/17/17/17/16/16개이며 두 담당자에게 최대 1개 차이로 배분했습니다.

Y 기준 슬롯은 +0.40/0/−0.40 m입니다.

| 물체 | 기준 X | X 변화 | Y 변화 | 기준 yaw 및 변화 |
|---|---:|---:|---:|---|
| 토스터 | 0.550 m | ±0.015 m | 슬롯 주위 ±0.015 m | 150° ±5° |
| 접시 | 0.490 m | ±0.010 m | 슬롯 주위 ±0.015 m | 0° ±5° |
| 빵거치대 | 0.460 m | ±0.015 m | 슬롯 주위 ±0.015 m | −90° ±5° |

환경 +X는 로봇 정면, +Y는 로봇 왼쪽, +Z는 위입니다.
위치는 **m**, quaternion은 **wxyz**이며 위 각도 표는 설명용 도(degree)입니다.
manifest의 최종 quaternion을 그대로 사용합니다. 원본 Z·scale과 거치대에 대한 빵 4개의 상대 배치를 유지합니다.

## 확인된 범위

100개 모두 **초기 배치 적용·reset 2회 복원 검증 완료**입니다.
실제 조작·IK·접근 경로·토스트 완주·시연 수집 성공은 미검증입니다.
영상상 잘리거나 가려진 부분도 미확인입니다.
manifest의 `gpu_verified: 0` 등은 생성 당시 기록을 보존한 값이며,
후속 runtime 검증 결과는 Fork의 `README_WORK.md` 및 batch02 검증 요약에 따로 기록했습니다.

## Fork 코드로 적용하기

**이 폴더는 배치 데이터 전달물입니다. 원본 upstream 코드에 복사하는 것만으로 자동 적용되지 않습니다.**
Fork `https://github.com/kkkkkmj/cyclo_lab.git`의 `toast-scenarios-batch02` 작업본에
시나리오 적용 코드가 포함된 상태를 사용해야 합니다(공유 준비 시점에는 아직 commit/push 전).
기존 Isaac Sim/Isaac Lab 환경과 asset도 필요합니다.

아래는 호스트 저장소가 `/home/mj/nrf_humanoid/cyclo_lab`, 기존 컨테이너가 `cyclo_lab`,
컨테이너 저장소가 `/workspace/cyclo_lab`인 경우입니다. 먼저 GPU 여유를 확인합니다.
이 예시는 **배치/reset 검사**이며 시연 수집 명령이 아닙니다.

```bash
cd /home/mj/nrf_humanoid/cyclo_lab
scenario_id=batch02_toast_000  # assignments.csv의 본인 담당 ID로 변경
run_dir=$(mktemp -d "$PWD/logs/delivery_check.XXXXXX")
docker exec -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/check_make_toast_scenarios.py \
  --scenario_id "$scenario_id" \
  --scenario_manifest /workspace/cyclo_lab/scenario_delivery/manifest.json \
  --headless --snapshots \
  --output_dir "/workspace/cyclo_lab/logs/$(basename "$run_dir")/$scenario_id"
```

GUI로 확인하려면 `--headless`를 생략합니다. 새 출력 폴더에 검사 결과가 기록됩니다.
동일 ID·manifest로 다시 실행하거나 해당 환경에서 reset하면 같은 초기 배치를 사용합니다.
ID 변경 시에는 새 환경을 생성합니다. 전달 폴더를 다른 위치에 두었다면
`--scenario_manifest`를 **컨테이너에서 보이는 실제 manifest 경로**로 바꾸십시오.
