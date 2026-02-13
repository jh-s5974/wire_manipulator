#!/usr/bin/env python3
import pygame
import time

def main():
    # pygame 초기화
    pygame.init()
    pygame.joystick.init()

    # 연결된 조이스틱 수 확인
    joystick_count = pygame.joystick.get_count()

    if joystick_count == 0:
        print("오류: 컨트롤러가 연결되어 있지 않습니다.")
        return

    # 첫 번째 조이스틱을 사용
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"'{joystick.get_name()}' 컨트롤러가 준비되었습니다.")
    print(f"축 개수: {joystick.get_numaxes()}, 버튼 개수: {joystick.get_numbuttons()}, 햇 스위치 개수: {joystick.get_numhats()}")
    print("입력을 기다리는 중... (종료하려면 Ctrl+C)")

    try:
        while True:
            # 이벤트 큐를 처리
            for event in pygame.event.get():
                # 버튼이 눌렸을 때
                if event.type == pygame.JOYBUTTONDOWN:
                    print(f"버튼 {event.button} 눌림")
                # 버튼을 떼었을 때
                elif event.type == pygame.JOYBUTTONUP:
                    print(f"버튼 {event.button} 떼어짐")
                
                # --- 이 부분이 수정되었습니다 (모든 축 출력) ---
                # 아날로그 스틱/트리거가 움직였을 때
                elif event.type == pygame.JOYAXISMOTION:
                    # 더 이상 특정 축(0, 1)만 필터링하지 않고 모든 축의 정보를 출력
                    print(f"축(Axis) {event.axis}: {event.value:.3f}")

                # --- 이 부분이 추가되었습니다 (십자키 출력) ---
                # 십자키(Hat)가 움직였을 때
                elif event.type == pygame.JOYHATMOTION:
                    # event.value는 (x, y) 튜플 형태로 값을 반환합니다.
                    # x: -1(왼쪽), 0(중앙), 1(오른쪽)
                    # y: -1(아래), 0(중앙), 1(위)
                    print(f"십자키(Hat) {event.hat}: {event.value}")

            # CPU 사용량을 줄이기 위해 잠시 대기
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")
    finally:
        # pygame 종료
        pygame.quit()

if __name__ == "__main__":
    main()