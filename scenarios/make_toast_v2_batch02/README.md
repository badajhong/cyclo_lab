# Toast v2 batch02 — 100개 초기 배치/reset 검증 완료

후보 100개가 동일한 상판·간격·로봇 모델 사전 필터를 통과했습니다.
**저장된 실행 결과에서 100개 초기 pose·reset 2회 복원이 확인됐습니다. 도달 가능성 및 전체 작업 성공은 미확인**입니다.
최종 검토: [요약](runtime_review/summary.json), [담당자별 검증표](runtime_review/operator_validation.csv), [수치 상세](runtime_review/numeric_review.json).
`NOT_EXCLUDED`는 작업 가능 확정이 아닙니다. IK는 실행하지 않았습니다.

## 설정과 증거

- ID: `batch02_toast_000`–`batch02_toast_099`. batch01 ID와 겹치지 않습니다.
- 접시: 기준 X=0.490 m, X ±0.010 m, Y ±0.015 m, 기존 방향 기준 yaw ±5°.
- 토스터 X=0.55 m, 거치대 X=0.46 m; XY ±0.015 m 및 yaw ±5° 유지.
- 세 Y 슬롯: +0.40, 0, −0.40 m. 순서는 환경 +Y→−Y(로봇 왼쪽→오른쪽).
- 원본 Z, scale, 거치대에 대한 빵 4개의 상대 pose 유지. quaternion wxyz, 단위 m.
- 재사용 증거: `logs/scenario_batch_geometry_fix.lKmiuB/geometry.json`.
  해시는 manifest provenance와 audit.json에 기록했습니다. 재추출하지 않았습니다.
- 이전 pilot report는 읽기만 했고 검사 실행은 반복하지 않았습니다.

상판 경계 20 mm, metadata bbox 여유 1 mm, 물체 간격 30 mm,
평면 접근 여유 60 mm, 기존 로봇 모델 상한 및 접시 X>0.419 조건은 유지했습니다.
생성 시 기존 프레임 계획 범위도 유지하면서 실제 추출 상판 coverage를 추가 적용합니다.
현재 100개는 상판 20 mm 여유 이후에도 최소 약 11.986 mm가 남으며,
visual footprint 사이 최소 간격은 약 89.691 mm입니다.
Collision 검사는 authored mesh bound이며 cooked geometry·동적 접촉 검사가 아닙니다.
로봇 모델은 양팔/lift 기반 보수적 도달 상한이며, IK·팔 간 충돌·접근 경로는 미검증입니다.

## 결과와 배정

| +Y→−Y 순열 | 개수 | A | B | 대표 ID |
|---|---:|---:|---:|---|
| 토스터/접시/거치대 | 17 | 9 | 8 | batch02_toast_000 |
| 토스터/거치대/접시 | 17 | 8 | 9 | batch02_toast_017 |
| 접시/토스터/거치대 | 17 | 9 | 8 | batch02_toast_034 |
| 접시/거치대/토스터 | 17 | 8 | 9 | batch02_toast_051 |
| 거치대/토스터/접시 | 16 | 8 | 8 | batch02_toast_068 |
| 거치대/접시/토스터 | 16 | 8 | 8 | batch02_toast_084 |

총 100개 생성 / 생성 중 탈락 0개 / 별도 audit NOT_EXCLUDED 100개.
A 50개, B 50개. pose 중복 및 batch01과의 ID/pose 중복 없음.

- `manifest.json`: 최종 pose·설정·소스 버전·검사 결과. seed만 저장하지 않습니다.
- `assignments.csv`: 고정 배정.
- `preview.html`: 브라우저에서 열기. 검은 실선=추출 상판, 회색 점선=기존 프레임 범위.
- `generation_report.json`, `audit.json`, `static_validation.json`: 정적 결과.
- `preservation_before.json`: batch01·기존 검사 결과·환경 소스의 변경 전 해시.
- `validate_static.py`: 순열/배정, 적용된 pose·높이·scale·빵 상대 배치,
  metadata footprint 및 독립적인 상판 변 거리, 기존 파일 보존 검사. 시뮬레이션 아님.

## 대표 6개 실행 명령 — 이미 완료된 검사, 재실행용 참고

대표 6개와 나머지 94개 검사는 모두 완료됐습니다. 아래 명령을 이번 검토에서 재실행하지 않았습니다.
필요하여 별도로 재검사할 때만 호스트 터미널에서 GPU 메모리와 진행 중인 작업을 먼저 확인하십시오.
여유가 없으면 검증을 보류하고, 기존 GPU 작업을 종료하지 마십시오.

```bash
cd /home/mj/nrf_humanoid/cyclo_lab
nvidia-smi
# GPU 여유 확인 후 아래 실행
run_dir=$(mktemp -d "$PWD/logs/scenario_batch02_six.XXXXXX")
chmod 755 "$run_dir"
container_run="/workspace/cyclo_lab/logs/$(basename "$run_dir")"
docker exec -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/run_make_toast_scenario_batch.py \
  --manifest /workspace/cyclo_lab/scenarios/make_toast_v2_batch02/manifest.json \
  --audit /workspace/cyclo_lab/scenarios/make_toast_v2_batch02/audit.json \
  --ids batch02_toast_000 batch02_toast_017 batch02_toast_034 \
        batch02_toast_051 batch02_toast_068 batch02_toast_084 \
  --output_root "$container_run/results" > "$run_dir/run.log" 2>&1
run_rc=$?
printf '%s\n' "$run_rc" > "$run_dir/exit_code"
echo "$run_dir"
```

기존 runner가 각 ID의 exit code·JSON·로그·스냅샷을 기록하고,
한 ID 실패 후에도 나머지를 계속 검사합니다. 종료코드 0만으로 PASS 처리하지 않습니다.
`LAYOUT_RESET_VERIFIED`는 초기 pose 및 reset 2회 복원 검증이며,
도달 가능성·안정적인 조작·전체 작업 성공과 구분합니다.
검사 결과는 새 logs 폴더에 기록하며 이 manifest의 후보 상태를 자동 변경하지 않습니다.

## ID 적용 및 재시도

batch02에는 **명시적인 manifest 경로가 필요**합니다. 기본 manifest는 기존 batch01을 유지합니다.
`gym.make()` 전에 `cfg.scenario_id='batch02_toast_000'`과
`cfg.scenario_manifest='/workspace/cyclo_lab/scenarios/make_toast_v2_batch02/manifest.json'`을 지정합니다.
동일 환경의 기존 `env.reset()`이 선택한 초기 배치를 복원합니다.
ID를 바꿀 때는 새 cfg/environment를 생성하십시오.

## 재현 명령

기존 전달물을 덮어쓰지 않도록 `--output`은 새 경로여야 합니다.

```bash
python3 -B scripts/tools/generate_make_toast_scenarios.py \
  --profile batch02 --seed 20261102 \
  --evidence logs/scenario_batch_geometry_fix.lKmiuB/geometry.json \
  --output NEW_DIRECTORY
```

기존 generator 기본 profile은 batch01 그대로입니다. batch02는 상판·세 collider bound·
FK 검증 모델 증거가 없으면 생성 전에 거절합니다. 생성 부족 시 조건 완화 없이 확보 수를 보고합니다.
정적 결과 재확인은 `python3 -B scenarios/make_toast_v2_batch02/validate_static.py`입니다.


## 저장 결과 최종 검토

- 대표 6개: `logs/scenario_batch02_six.46bMoE/results`
- 나머지 94개: `logs/scenario_batch02_remaining.ucq8XR/results`
- 두 summary의 manifest SHA-256은 모두
  `edd672dde1619837bf6f065e9293d42c86486eb113cabdf417af5bc028213120`으로 현재 파일과 일치합니다.
- 정확히 100개 ID, 누락/중복 0, 각 ID 종료코드 0.
- 각 report의 초기 pose와 reset 2회 PASS, 초기/정착0/reset0/정착1/reset1 샘플 존재 확인.
- 초기 position 최대 오차 2.9712e-8 m, quaternion 내적 오차 최대 1.606e-7.
  기록된 reset 전체 상태의 초기값 대비 최대 scalar 차이는 0입니다.
- operator_A 50/50, operator_B 50/50가 배치/reset 검증 완료입니다.
  순열별 배정은 위 표와 동일하며 manifest·CSV·±5° 설정을 변경하지 않았습니다.

검사기는 각 reset 전 50 step(현재 25 Hz에서 2초)을 유지하고 빵 하나를 교란한 뒤
다른 seed로 복원합니다. 이 PASS는 로봇/물체 root, robot/toaster joint,
거치대 pose 및 toast-cycle 상태의 초기 복원 범위입니다.
초기값을 로딩하고 reset할 수 있다는 뜻이며 정착 후 물리적 안정성을 자동 인증하지 않습니다.

### 정착 후 기록과 육안 확인

- 토스터·거치대는 위치/방향의 수치 오차 수준 변화만 있습니다.
- 접시는 약 6.286–6.289 mm 하강하고, 최대 수평 이동 0.00342 mm,
  초기 대비 최대 회전 0.0575°로 정착했습니다.
- 빵은 최대 약 13.987 mm 수평 이동, 약 16.816–17.678 mm 하강,
  약 10.99–17.12° 회전했습니다. 네 조각은 기록된 정착 시점에서 Z≥1.07552 m입니다.
  초기 pose가 정착 후에도 그대로 유지된 것은 아닙니다.
- 100개 ID의 초기·두 정착 시점 머리 카메라 300장을 육안 검토했습니다.
  보이는 영역에서 전도·테이블/바닥으로 빵 낙하·뚜렷한 물체 이탈 후보는 발견하지 못했습니다.
- 1,500개 PNG는 모두 파일 무결성 검사(Pillow.verify) 통과. 나머지 1,200장을 모두 육안 검토했다는 뜻은 아닙니다.
  손목 카메라는 020/042/064/079/093의 settled_1 양쪽 10장을 표본 확인했으며 작업대가 보이지 않았습니다.
- 000–016 및 034–050은 우측 거치대/빵, 051–067은 우측 토스터,
  068–083은 좌측 거치대/빵, 084–099는 양쪽 거치대/토스터의
  화면 가장자리 외곽 확인이 제한됩니다(잘림 정도는 프레임별 차이).
  017–033도 우측 접시 가장자리와 접촉면의 완전한 확인은 보장하지 않습니다.
  화면 밖/가림/밑면의 실제 상태는 **미확인**이며 영상 잘림을 물체 이탈로 판정하지 않았습니다.

IK·팔/손 접근 경로·충돌 없는 조작·정착 후 정밀 접촉/장시간 안정성·토스트 완료·
자동 expert 완주·시연 수집 가능성은 이번 검증 범위 밖입니다.
`manifest.json`과 정적 생성 결과의 `gpu_verified: 0`은 생성 당시 기록으로 보존합니다.
현재 runtime 검증 상태는 manifest 해시로 연결되는 `runtime_review/summary.json`을 보십시오.

GitHub 공유 파일과 원본 증거 보관 범위는 [SHARING.md](SHARING.md)에 정리했습니다.
