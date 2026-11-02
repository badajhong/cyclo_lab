# GitHub 공유 대상과 증거 보관

이번 작업에서는 파일 목록만 정리했으며 `git add`, commit, push를 수행하지 않았습니다.
기존 작업 변경을 일괄 포함하지 말고 아래 범위를 기준으로 diff를 검토하십시오.

## batch02 전달물 — 저장소에 포함

`scenarios/make_toast_v2_batch02/`의 다음 파일:

- `README.md`, `SHARING.md`: 사용법, 검증 범위와 미확인 사항.
- `manifest.json`, `assignments.csv`, `preview.html`: 고정 pose·배정·평면도. 원본 유지.
- `generation_report.json`, `audit.json`, `static_validation.json`: 생성 당시 정적 증거. 과거 상태값을 덮어쓰지 않음.
- `validate_static.py`, `preservation_before.json`: CPU 확인 방법과 원본 보존 증거.
- `runtime_review/summary.json`: manifest 해시로 연결된 100개 초기 배치/reset 최종 요약.
- `runtime_review/operator_validation.csv`: 각 ID·담당자·초기/reset PASS·exit code·육안 확인 한계.
- `runtime_review/numeric_review.json`: 각 ID의 pose 오차·정착 후 변화량·report 해시.
- `runtime_review/source_sha256.json`: 원본 summary/result/report/log/PNG의 경로와 해시.

`manifest.json` 안의 절대 경로나 `logs/...` 참조는 provenance입니다.
고정 pose의 환경 적용은 그 로그를 필요로 하지 않지만, **재생성/검사 결과 재검토는
아래 원본 증거와 asset이 필요**합니다.

## 실행·생성·정적 검사 코드 — 저장소에 함께 포함

기존 저장소 기준으로 다음 변경/신규 소스를 포함해야 합니다.

- `source/cyclo_lab/cyclo_lab/manager_based/manipulation/make_toast/config/ffw_bg2/`
  - `scenarios.py`: 명시 manifest의 최종 pose 적용 및 bread 상대 pose 보호.
  - `scenario_pilots.json`: 원본 calibration/pilot ID 보존에 필요.
  - `bimanual_env.py`, `bimanual_env_cfg.py`: scene 생성 전 scenario 적용과 설정 필드.
- `scripts/tools/`
  - `generate_make_toast_scenarios.py`, `toast_batch_lib.py`
  - `audit_make_toast_scenarios.py`
  - `check_make_toast_scenarios.py`, `run_make_toast_scenario_batch.py`
  - `export_make_toast_geometry.py`, `toast_geometry_support.py`
  - `test_make_toast_scenarios.py`, `test_toast_batch.py`, `test_toast_geometry_support.py`
- `README_TOAST.md`의 관련 사용법 변경.

기본 scenario 조회는 기존 batch01을 유지하므로 batch01도 보존해 공유하거나,
수신자에게 항상 batch02 `--manifest`/`cfg.scenario_manifest`를 명시하도록 안내하십시오.
`test_toast_batch.py`와 batch02의 보존 검사에는 batch01 및 이전 pilot 증거가 필요합니다.

이 목록은 기존 base repository 위의 변경 목록입니다. 원본 task/robot configuration,
Docker 설정과 dependency 버전 고정, asset manifest/downloader 등은 기존 저장소와 함께 전달해야 합니다.
이전에 복원한 Isaac Lab `utils/datasets`의 Python 4개 파일과 대응 `.gitignore` 수정도
원본 전달본에서 여전히 빠져 있다면 별도 의존성 복구 변경으로 포함해야 합니다.
이번 검토는 그 복구 파일을 수정하지 않았습니다.

## 원본 검증 증거 — 별도 아카이브/Release 첨부 권장

`.gitignore`가 `logs/**`를 제외하므로 위 요약 파일만 커밋하면 원본 report/이미지는 올라가지 않습니다.
독립적으로 결과를 재검토할 수 있도록 다음 폴더를 별도 증거 아카이브로 함께 전달하십시오.
현재 이 작업에서 압축/업로드하지는 않았습니다.

- `logs/scenario_batch02_six.46bMoE/` — 6개 원본 summary/report/result/log/PNG 및 실행 종료코드.
- `logs/scenario_batch02_remaining.ucq8XR/` — 94개 원본 자료.
- `logs/scenario_batch_geometry_fix.lKmiuB/` — 실제 재사용 geometry와 생성 로그/종료코드.
- 재생성까지 재현할 경우 `logs/scenario_pilots.mlY0sM/` — generator가 참조하는 기존 pilot 보고서.

다운로드한 대형 object/robot USD는 일반 코드 변경에 무작정 추가하지 말고,
기존 asset manifest·revision·다운로드 방법과 별도 전달 파일의 해시를 사용하십시오.
이번 범위에 시연 HDF5, 학습 출력, 촉각 연구 자료는 없습니다.
`scenarios/toaster_yaw_design01/`은 별도 회전 설계 작업이며 이번 ±5° batch02 검증 전달과 구분합니다.

## 현재 검증의 정확한 의미

공유 설명은 "100개 고정 배치의 초기 pose·reset 2회 검증 완료"로 적으십시오.
"100개 작업 성공", "100개 collision-free/IK 검증 완료"로 표현하면 안 됩니다.
화면 밖/가려진 접촉면은 미확인이고, 영상상 이상 없음은 자동 성공 판정이 아닙니다.
