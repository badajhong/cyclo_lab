# 토스트 v2 고정 배치 후보 100개

**이 전달물은 후보 100개이며, 새 후보의 GPU 배치/reset 검증은 0개입니다.**
실제 사용 가능한 상판 영역과 로봇 모델을 이 세션에서 읽지 못했습니다.
따라서 검증 완료된 작업 가능 시나리오 100개를 확보했다고 해석하면 안 됩니다.
Docker 소켓 접근이 거절되어 재시도하지 않았습니다. 설치·GPU 실행은 하지 않았습니다.
기존 `logs/scenario_pilots.mlY0sM`의 세 pilot PASS를 해시와 함께 재사용했습니다.
기존 pilot의 PASS를 새 후보에게 전파하지 않았습니다.

## 전달 파일

- `manifest.json`: `toast_000`–`toast_099`, 물체 3개+빵 4개의 최종 position/quaternion/scale, 생성 설정, 소스 해시, 검사 범위.
- `assignments.csv`: operator_A 50개 / operator_B 50개. 순열별 두 사람 차이는 최대 1개.
- `preview.html`: 브라우저에서 직접 열 수 있는 전체 평면 비교와 담당자 필터. 외부 서버 불필요.
- `generation_report.json`: 6가지 순열의 요청·확보·시도·탈락 건수.
- `delivery_checks.json`: 전달 시 정적 검사 결과와 관련 소스/파일 해시.

원본 `scenario_id=None` 동작과 `baseline`, `small_offset`, `swapped_candidate`는 보존했습니다.
이 파일을 덮어쓰지 마십시오. 재시도는 동일 manifest와 ID를 사용합니다.
다른 manifest를 만들 때 ID 문자열만 비교하지 말고 manifest 해시도 함께 기록해야 합니다.
담당자 배정은 현재 후보 기준이며, 검사에서 탈락하면 확보 수를 줄여 보고하고 별도 배치를 생성해야 합니다.

## 좌표·크기·배치 근거

환경 원점 기준 미터, quaternion **wxyz**, 수평 회전은 환경 +Z축 yaw입니다.
평면도는 +X가 위, +Y가 왼쪽입니다. 로봇 정면이 +X이고 양의 Y가 로봇의 왼쪽입니다.
좌우 순서는 **+Y → -Y**입니다. 카메라 화면의 좌우와는 다릅니다.

고정 revision `63fcc…`의 전체 값과 각 metadata 해시는 manifest provenance에 있습니다.
`source/cyclo_lab/data/objects/robodojo_make_toast/*/metadata.json`의
`geometry.aligned_bbox.vertices`에 기존 scale, 최종 quaternion, translation을 적용합니다.
metadata의 반올림 오차에 1 mm 여유를 두며, 거치대 그룹에 빵 4개의 footprint도 포함합니다.

| 물체 | scale 적용 후 로컬 bbox 크기 X/Y/Z (m, 약값) | 기존 기준 X → 후보 기준 X |
|---|---|---|
| 토스터 | 0.2267 / 0.1555 / 0.1619 | 0.55 → 0.55 |
| 거치대 | 0.3166 / 0.0836 / 0.0762 | 0.45 → 0.46 |
| 접시 | 0.19425 / 0.19425 / 0.007832 | 0.43 → 0.46 |

실행 report의 테이블 **프레임 외곽**은 X=[0.3190,1.0810],
Y=[-1.3368,1.1368], 상단 Z≈0.99405입니다. 이는 상판 지지면의 확정값이 아닙니다.
현재 후보 생성은 이 외곽에서 20 mm 들어간 영역을 임시 계획 범위로 사용합니다.
외부 USD를 로딩한 뒤 실제 상단 면의 합집합 검사로 이 가정을 대체해야 합니다.

Y 기준점은 +0.40, 0, -0.40 m입니다. 가장 큰 반폭 조합 약 0.16+0.13 m에
두 물체 이동량 30 mm와 접근 여유 60 mm를 고려한 후보 간격입니다.
각 물체의 XY 변화는 기준점 주위 ±15 mm, yaw는 원본 방향 주위 ±5°입니다.
pilot에서 확인된 회전은 ±2°이므로, ±5°는 새 검증이 필요한 설계 범위입니다.
변화량은 확정된 도달 가능 범위가 아닙니다. 원본 Z와 scale은 그대로입니다.
접시 X 이동은 프레임 모서리 여유를 늘리기 위한 것입니다.

물체 footprint 사이 최소 30 mm 및 평면 접근 여유 60 mm를 요구합니다.
60 mm는 expert 소스의 full-jaw opening 106 mm의 절반+7 mm에 근거한 **대리 기준**입니다.
실제 손·팔·접근 경로의 충돌 없는 공간을 보증하지 않습니다.
6순열 요청 수는 17/17/17/17/16/16이며 모두 위 정적 계획 조건 안의 후보를 확보했습니다.
프레임 경계에서 1개 탈락, 기준 완화 없음. 여섯 순열의 물리적 가능성을 확정한 것은 아닙니다.
접시 X≤0.419 후보는 별도 conflict 및 탈락 사유로 표시합니다. 이번 100개에는 없습니다.

## 검사 범위와 한계

| 단계 | 현재 결과 / 구현 |
|---|---|
| 고정 pose·중복·배정·원본 높이/scale·빵 상대 pose | CPU 검사 완료. 변경된 manifest가 빵 상대 pose를 깨뜨리면 적용 전 거절 |
| 회전·scale을 반영한 visual bbox, 간격 | 100개 정적 조건 만족. 실제 mesh 접촉 판정과 다름 |
| 실제 상판 이탈 | **미검증**. USD에서 최상단 수평 convex faces를 추출, 면들의 합집합 밖 영역 제외. 구멍을 convex hull로 덮지 않음 |
| collision geometry | **미검증**. authored collision mesh의 bound를 별도로 검사. 비-mesh collider 경로도 기록. PhysX cooked shape, 움직이는 carriage/lever 및 빵 접촉은 별도 미검증 |
| 로봇 모델 사전 필터 | **미실행**. FFW_BG2 USD 관절 프레임·연결 길이·lift 축/한계로 shoulder 이동선분과 보수적 chain 길이 상한 사용. 홈 FK가 실제 로봇 link pose와 맞아야 사용 |
| IK | **미실행**. 선택 옵션으로 양팔 각각 7관절+lift 제한 내 수치 IK. 파지/삽입/레버/추출/서빙 probe. 목표 orientation과 열린 TCP는 근사치 |
| 새 ID 배치 적용/reset | **미실행**. 기존 pilot 3개 PASS와 별개 |
| 도달 가능성·전체 작업 | **미검증**. 단순 거리나 IK 결과를 reachability PASS로 표시하지 않음 |

모델 상한 밖이면 해당 probe를 현재 고정 base에서 닿을 수 없는 후보로 제외합니다.
상한 안이면 `NOT_EXCLUDED_BY_MODEL_BOUND`일 뿐입니다.
`IK_NOT_FOUND`는 초기값·수치해법·근사 목표의 한계일 수 있으며 물리적 불가능 판정이 아닙니다.
`IK_FOUND`도 팔 간 충돌, 하나의 lift를 공유하는 양팔 협동, 이동 경로를 보장하지 않습니다.
자동 expert 완주 여부는 생성/필터 기준에 포함하지 않았습니다.

`bimanual_mdp.task_done()`의 접시 X>0.419, 초기 접시 pose에서 25 mm 이내,
빵 접시 안착, 원본 standby와 lift 복귀 조건은 그대로입니다.
초기 접시 default pose와 spare bread의 shelf 기준은 선택한 시나리오를 따릅니다.
expert의 고정 팔 역할·waypoint가 새 순열에서 동작한다는 보장은 없습니다.

## 호스트에서 실행할 순서

아래 명령은 **이 세션에서 GPU 실행하지 않았습니다**. 기존 컨테이너를 사용합니다.
먼저 호스트에서 `nvidia-smi`로 작업과 메모리를 확인하십시오.
여유가 없으면 보류하며, 기존 프로세스를 종료하지 마십시오.

```bash
cd /home/mj/nrf_humanoid/cyclo_lab
nvidia-smi
run_dir=$(mktemp -d "$PWD/logs/scenario_batch_geometry.XXXXXX")
chmod 755 "$run_dir"
container_run="/workspace/cyclo_lab/logs/$(basename "$run_dir")"
# 새로운 증거 수집: 기존 pilot reset 검사를 반복하는 명령이 아님.
docker exec -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/export_make_toast_geometry.py --headless \
  --output "$container_run/geometry.json" > "$run_dir/geometry.log" 2>&1
geometry_rc=$?
printf '%s\n' "$geometry_rc" > "$run_dir/geometry.exit_code"
```

종료 코드 0 **및 geometry.json 존재**를 확인한 뒤 같은 호스트 터미널에서:

```bash
python3 scripts/tools/audit_make_toast_scenarios.py \
  --evidence "$run_dir/geometry.json" --output "$run_dir/audit.json"
```

audit는 100개 고정 pose/ID를 바꾸지 않습니다. `REJECTED_BY_FILTER`,
`EVIDENCE_INCOMPLETE`, `NOT_EXCLUDED`를 구별합니다. FK 추출 오류는 geometry JSON에 남습니다.
선택적으로 기존 컨테이너 Python에서 `audit_make_toast_scenarios.py --ik`와
새 output 파일을 사용하면 수치 IK까지 검사합니다(NumPy/SciPy 필요, 여러 초기값으로 시간이 걸릴 수 있음).

처음에는 각 순열에서 1개씩 새 후보 6개만 적용/reset 검사합니다:

```bash
docker exec -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/run_make_toast_scenario_batch.py \
  --audit "$container_run/audit.json" \
  --ids toast_000 toast_017 toast_034 toast_051 toast_068 toast_084 \
  --output_root "$container_run/first_six" > "$run_dir/first_six.log" 2>&1
printf '%s\n' "$?" > "$run_dir/first_six.exit_code"
```

스냅샷·정착 후 pose·reset report를 검토한 뒤, 이미 실행한 6개는 반복하지 않고 나머지:

```bash
mapfile -t remaining_ids < <(python3 - <<'PY'
import json
m=json.load(open('scenarios/make_toast_v2_batch01/manifest.json'))
first={'toast_000','toast_017','toast_034','toast_051','toast_068','toast_084'}
print('\n'.join(s['id'] for s in m['scenarios'] if s['id'] not in first))
PY
)
docker exec -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/run_make_toast_scenario_batch.py \
  --audit "$container_run/audit.json" --ids "${remaining_ids[@]}" \
  --output_root "$container_run/remaining" > "$run_dir/remaining.log" 2>&1
printf '%s\n' "$?" > "$run_dir/remaining.exit_code"
```

audit에서 제외/증거 부족인 ID는 실행하지 않고 `NOT_RUN_AUDIT_BLOCKED`, exit_code=null로 기록합니다.
실행한 ID는 별도 log, result JSON, report, snapshot을 갖습니다.
종료코드 0뿐 아니라 올바른 ID의 report PASS 및 초기/정착/reset 2회 샘플이 모두 있어야
`LAYOUT_RESET_VERIFIED`입니다. 한 ID 오류·timeout 뒤에도 다음 ID로 진행합니다.
전체 종료코드 1은 실패 또는 미실행 ID가 있음을 뜻합니다. summary의 확보 수를 확인하십시오.
검증 결과는 별도 파일이며 원본 manifest의 후보 표시는 조용히 변경되지 않습니다.

## 한 ID를 열고 reset하기

```bash
# 호스트 터미널: GUI 설정은 기존 컨테이너의 DISPLAY/X11 구성을 사용.
docker exec -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/check_make_toast_scenarios.py --scenario_id toast_000 \
  --scenario_manifest /workspace/cyclo_lab/scenarios/make_toast_v2_batch01/manifest.json \
  --steps 250 --snapshots \
  --output_dir /workspace/cyclo_lab/logs/toast_000_gui_NEW
```

output 디렉터리는 매번 새 이름이어야 합니다. `--headless`를 더하면 GUI 없이 실행합니다.
이 도구는 유한 시간 동안 원본 환경을 유지하고 빵 하나를 진단 목적으로 교란한 뒤
`env.reset()` 2회로 원복을 검사합니다. 시연 데이터는 생성하지 않습니다.
정착 후 위치를 기록하지만, 안정 지지/손 접근/작업 성공을 자동 판정하는 도구는 아닙니다.

별도 실행 코드에서는 `gym.make()` **전에** `cfg.scenario_id = 'toast_000'`과
필요하면 `cfg.scenario_manifest = '/workspace/.../manifest.json'`을 지정합니다.
환경 생성자가 cfg의 init_state에 최종 pose를 적용하고, 기존 reset이 이를 복원합니다.
ID를 바꿀 때는 새 cfg/environment를 생성합니다. 로딩된 scene의 cfg를 바꿔 이동하지 않습니다.

## 정적 검사와 재생성

```bash
python3 -B scripts/tools/test_toast_batch.py
python3 -B scripts/tools/test_make_toast_scenarios.py
python3 -B scripts/tools/run_make_toast_scenario_batch.py --static_only
```

마지막 명령은 ID 조회/정적 보고서 출력만 하며 시뮬레이션 PASS가 아닙니다.
생성기는 `scripts/tools/generate_make_toast_scenarios.py --output NEW_DIRECTORY`입니다.
현재 세 pilot report와 pinned metadata/robot USD가 필요합니다. seed만으로 실행하지 않고
생성 시 모든 최종 pose를 저장합니다. 기존 전달 파일 덮어쓰기는 거부합니다.
`--evidence geometry.json [--ik]`로 새 후보 집합 생성도 가능하지만,
현재 고정 ID를 검토할 때는 pose가 유지되는 **audit를 우선 사용**하십시오.
새 배치가 100개 미만이면 summary의 수와 탈락 사유를 보고하며, 기준을 자동 완화하지 않습니다.
