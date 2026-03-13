#!/bin/bash
#
# [RT 코어 관리 스크립트]
# 인자로 지정한 CPU 코어만 활성 상태로 만들고, 나머지는 모두 유휴 상태를 허용합니다.
# 인자가 없으면 모든 코어의 유휴 상태를 허용합니다.
# 사용법: sudo ./set-active-cores.sh [CORE_NUM_1] [CORE_NUM_2] ...
# 예시:
#   - sudo ./set-active-cores.sh 1      # CPU 1만 활성, 나머지는 유휴 허용
#   - sudo ./set-active-cores.sh 1 7    # CPU 1, 7은 활성, 나머지는 유휴 허용
#   - sudo ./set-active-cores.sh        # 모든 CPU를 유휴 허용 상태로 복원

# --- 스크립트 시작 ---

# 루트 권한으로 실행되었는지 확인
if [ "$EUID" -ne 0 ]; then
  echo "오류: 이 스크립트는 반드시 루트(sudo) 권한으로 실행해야 합니다."
  exit 1
fi

# 인자로 받은 활성화할 코어 목록을 배열로 저장
ACTIVE_CORES=("$@")

# 인자 존재 여부에 따라 사용자에게 실행할 작업 요약 안내
if [ ${#ACTIVE_CORES[@]} -eq 0 ]; then
    echo "활성화할 코어가 지정되지 않았습니다. 모든 CPU 코어의 유휴 상태를 허용합니다..."
else
    echo "지정된 CPU 코어 [${ACTIVE_CORES[*]}]를 활성 상태로 설정합니다."
    echo "나머지 모든 코어는 유휴 상태를 허용하도록 복원됩니다..."
fi

# 시스템에 존재하는 모든 CPU 코어를 순회
for cpu_path in /sys/devices/system/cpu/cpu[0-9]*; do
    # 경로에서 CPU 번호만 추출
    core_num=$(basename "$cpu_path" | tr -d 'cpu')

    # 현재 코어가 활성화 대상 목록에 있는지 확인하기 위한 플래그
    is_target_core=false
    for target_core in "${ACTIVE_CORES[@]}"; do
        if [[ "$core_num" == "$target_core" ]]; then
            is_target_core=true
            break
        fi
    done

    # 플래그 값에 따라 유휴 상태를 켜거나(0) 끄거나(1) 결정
    if $is_target_core; then
        disable_value=1 # 유휴 상태 끄기 (활성화)
    else
        disable_value=0 # 유휴 상태 켜기 (복원)
    fi

    # 결정된 값을 해당 코어의 모든 C-state(state0 제외)에 적용
    for state_path in ${cpu_path}/cpuidle/state*; do
        state_name=$(basename "$state_path")
        if [[ "$state_name" != "state0" ]] && [[ -f "$state_path/disable" ]]; then
            echo "$disable_value" > "$state_path/disable"
        fi
    done
done

echo "설정 완료."