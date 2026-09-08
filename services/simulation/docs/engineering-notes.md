SkyBuddy — 드론 시뮬레이션 환경 (AirSim + ArduPilot SITL)
================================================

[담당 범위]
파란학기제 SkyBuddy 팀 내 시뮬레이션 환경 구축 (이태우).
AirSim(Unreal 기반 시각화) + ArduPilot SITL(비행 컨트롤러)을 MAVLink로 연결해서
실제 기체 없이 드론 미션을 재현하고 KPI 데이터를 수집한다.
drone_link.py가 미들웨어(박병언)와 맞닿는 프로토콜 레이어.

[핵심 파일]
- drone_link.py       MAVLink 프로토콜 레이어 (연결/ARM/이륙/이동/착륙/텔레메트리)
                       미들웨어와 맞닿는 부분
- kpi_mission.py      드론 1대의 미션 + KPI 수집
                       패턴 2종: grid(격자 탐색, 실제 수색용) / perimeter(사각형 순회, 시연용)
- kpi_mission_multi.py 드론 2대 동시 미션. 고도·방향·구역을 나눠 충돌 없이 운용
- start_sim.sh        SITL 2기를 띄우고 GPS 준비까지 대기 + 감쇠 튜닝 자동 적용
- tune_damping.py     자세 루프 감쇠 튜닝 (default / damped / soft)
- test_drone_link.py  단일 드론 호버 테스트 (동작 확인용 최소 스크립트)

[tools/] — 진단용 보조 스크립트 (평소 실행 흐름엔 필요 없음)
- watch_vibration.py        비행 중 자세·진동·수평위치를 관찰만 해서 수치로 요약
- diff_drones.sh            두 드론의 실제 파라미터 차이 비교
- diff_params.sh            과거 정상 시점과 현재 파라미터 비교
- restore_official_params.sh 공식 기본 파라미터(4.3 기준)로 복원
- check_gps.py              GPS 상태 확인
- check_params.py           ArduCopter 파라미터 조회
- check_vibration.py        이륙부터 수행하는 진동 확인 (구버전)
- set_roll_pid.py           자세 PID 개별 조정 실험용

[config_reference/] — 환경 설정 참고용 사본
- airsim_settings_reference.json  → 원래 위치: Documents\AirSim\settings.json
- dot-wslconfig.txt               → 원래 위치: %USERPROFILE%\.wslconfig

[실행 순서 - 드론 2대]
1) Windows PowerShell: settings.json 배포 (2대 설정이 이미 들어가 있으면 생략)
   Copy-Item "\\wsl.localhost\ubuntu-22.04\home\l9302\skybuddy\config_reference\airsim_settings_reference.json" `
             "C:\Users\l9302\OneDrive\Documents\AirSim\settings.json" -Force
2) Blocks.exe 완전 종료 후 재시작
   -> ArduCopter는 external physics라 SITL 켜기 전엔 화면에 드론이 안 보이는 게 정상
3) WSL 터미널:
   bash ~/skybuddy/start_sim.sh
   -> SITL 2기를 띄우고, 둘 다 GPS 잡을 때까지 대기한 뒤 감쇠 튜닝까지 자동 적용
4) "준비 끝" 이 뜨면:
   cd ~/skybuddy && python3 kpi_mission_multi.py

결과: kpi_summary_drone1.csv / kpi_summary_drone2.csv (웨이포인트별 지표)
      kpi_log_drone1.csv   / kpi_log_drone2.csv     (원시 텔레메트리)

[정상 기준값] 2026-09-03 측정
- 웨이포인트 도달: 5/5
- 구간 이동 시간: 7.0~7.3초 (20m 구간)
- 경로 효율: 0.93~1.00
- vibration_z: 0.05 내외  (1.0 넘으면 자세 루프 발진을 의심할 것)
- clipping: 0
- GPS 저하: 없음

[현재 상태]
- 단일 드론: 이륙 → 격자형 웨이포인트 6개 순회 → 착륙까지 안정 동작 확인.
- 비행 중 진동/불안정 이슈가 있었는데, 원인은 파라미터가 아니라 AirSim(Blocks) 프로세스
  자체의 누적된 물리 상태였다. 오래 켜두고 여러 번 충돌/강제종료했다면 SITL만 재시작하지
  말고 Blocks도 완전히 껐다 켜야 한다.
- 다음 단계: 2대 드론 동시 시뮬레이션 (미들웨어 연동 대비)

[중요 - 파라미터는 공식값을 쓸 것]
airsim-quadX.parm 을 임의로 튜닝하지 말 것. 자체적으로 SCHED_LOOP_RATE 를 100 으로
낮춰뒀던 적이 있는데, 이것 때문에 기체가 계속 부들거리고 고도를 못 잡았다.
AirSim이 약 333FPS로 센서를 보내는데 제어 루프만 100Hz라 생긴 문제였고,
공식값 300 으로 되돌리자 즉시 해결됐다 (2026-09-03 확인).
공식 파일: ArduPilot/ardupilot → Tools/autotest/default_params/airsim-quadX.parm
복원 방법: bash ~/skybuddy/restore_official_params.sh
  (공식값으로 덮어쓰고, eeprom.bin 도 지워서 저장된 값이 기본값을 덮지 않게 한다)

[기체 진동(자세 루프 발진) - 해결됨]
2대 구성에서 한쪽 기체만 roll 축이 지속 진동하고 이동이 5배 느려지는 문제가 있었다.
  증상: vibration_z 1.6 (정상 0.05), 웨이포인트 구간 이동 35초 (정상 7초), 경로효율 0.85
두 기체의 파라미터·펌웨어·디렉토리가 완전히 동일함을 diff로 확인했으므로 원인은
설정이 아니라, 그 기체가 AirSim에서 받는 센서 데이터의 지연/지터다. 지연이 크면
같은 게인이라도 위상 여유가 모자라 자세 루프가 발진한다. 특히 D항이 지연에 민감하다.
해결: 자세 루프 게인을 낮추고 필터 차단주파수를 내린다 (ATC_RAT_*_D 0.003 -> 0.0008,
      필터 50Hz -> 15Hz, P/I 0.25 -> 0.135).
      -> vibration_z 0.05, 구간 7초, 경로효율 0.95~1.00 으로 정상화 (2026-09-03 확인)
적용: start_sim.sh 가 두 기체 모두에 자동 적용한다.
      수동 조정이 필요하면 tune_damping.py <포트> <default|damped|soft>

[2대 드론 실행 시 주의]
- settings.json 에 Copter1/Copter2 둘 다 정의돼 있으면, SITL 두 개가 모두 접속해야 한다.
  AirSim은 lock-step 방식이라 한쪽이라도 안 붙어 있으면 Blocks가 그대로 멈춘다 (실측 확인됨).
  드론1이 크래시나서 소켓이 닫혀도 같은 이유로 Blocks가 죽는다.
- 드론2의 SITL은 반드시 별도 디렉토리(~/ardupilot/copter2_instance)에서 실행할 것.
  같은 디렉토리에서 두 개를 돌리면 eeprom/fd 충돌로 "Bad file descriptor" 가 난다.

[계획서 대비 현황] 2026-09-03 기준
개인별 추진일정(이태우) 기준 5~6주차까지 완료. 학사일정보다 앞서 있음.
  1~2주차 AirSim·SITL 환경 기초 구축                      완료
  3~4주차 2대 이상 드론 연동 + MAVLink 상태 데이터 수집    완료
  5~6주차 수색 시나리오·격자 탐색·구역 분할·자동 복귀      완료
  7~8주차 미들웨어 연동 및 드론별 명령 실행 흐름 검증      다음 차례

아직 안 된 것 (중요도 순):
 1) 미들웨어로 데이터를 "전달"하는 경로가 없다. 지금은 CSV 저장까지만.
    박병언 미들웨어가 붙을 API/스트림 인터페이스가 필요하다.
 2) 지휘 방향이 반대다. 현재는 kpi_mission.py 가 경로를 스스로 만들고 실행한다.
    최종 형태는 LLM -> 미들웨어 -> 시뮬 실행이므로, 시뮬은 "명령을 받아 수행하는 쪽"이
    되어야 한다. 7~8주차에 구조를 뒤집어야 한다.
 3) run_mission() 이 연결·이륙·순회·귀환·착륙·CSV 저장을 한 함수에 담고 있어
    "웨이포인트 하나만 실행" 같은 외부 요청을 처리할 수 없다. 명령 단위로 분리 필요.
 4) "산악" 지형이 없다. Blocks는 평지 + 블록 몇 개. 계획서는 지형까지 요구한다.
 5) 탐색 구역 커버리지 지표가 없다. 경로 효율만 있고 "구역의 몇 %를 훑었는가"가 없어
    중복 탐색 감소를 증명할 수 없다. 계획서의 핵심 평가항목이다.
 6) 장애 상황 재현(배터리 부족·통신 지연·드론 이탈) 없음. 9~10주차 항목이며
    미들웨어 팀의 Fallback 검증이 여기에 의존한다.

================================================================================
[이기종 프로토콜 - DDS 드론] 2026-09-07 달성
================================================================================

MAVLink 를 한 번도 쓰지 않고 ROS 2(DDS) 만으로 이륙 -> 웨이포인트 이동 -> 착륙
전 구간을 수행하는 데 성공했다. 이로써 "서로 다른 프로토콜을 쓰는 드론 2대를
미들웨어가 통합한다" 는 주장이 로그로 뒷받침된다.

[왜 DDS 인가]
- 처음엔 PX4 를 이기종 상대로 쓰려 했으나, PX4 도 MAVLink 를 쓰므로
  "프로토콜이 다르다" 는 주장이 성립하지 않는다.
- ISO/IEC 4005(UAAN)는 물리·데이터링크 계층 무선 규격이라 소프트웨어
  시뮬레이션으로 구현할 수 없고 공개 SDK 도 없다.
- ArduPilot 4.5+ 의 AP_DDS 는 네이티브 XRCE-DDS 클라이언트라, 같은 기체를
  MAVLink 없이 ROS 2 인터페이스로만 조종할 수 있다. 이게 유일하게 현실적인 선택이었다.
- 계층 구분에 유의할 것: MAVLink 는 메시지 사전을 포함한 응용계층 프로토콜이고
  DDS 는 응용~전송 사이의 pub/sub 미들웨어다. 따라서 비교 단위는
  "MAVLink" vs "ROS 2 인터페이스(DDS + 메시지 정의)" 로 잡아야 정확하다.

[구성]
- ~/ardupilot    ArduCopter 4.3  : MAVLink 드론용 (기존, 건드리지 않음)
- ~/ardupilot45  ArduCopter 4.7.1: DDS 드론용 (태그 checkout, detached HEAD)
- ~/ardu_ws      ROS 2 Humble 워크스페이스 (ardupilot_msgs)
- ~/Micro-XRCE-DDS-Gen  ardupilot 포크 v4.7.0

[실행]
  0) Windows 에서 Blocks.exe 실행
  1) bash ~/skybuddy/start_dds_sim.sh
     -> Agent + SITL 기동, GPS 준비 대기, DDS 초기화·서비스 목록까지 자동 확인
  2) 스크립트가 출력하는 ros2 명령을 새 터미널에서 실행

[검증된 인터페이스] 4.7.1 기준 - 토픽 18개 / 서비스 6개
  /ap/prearm_check            std_srvs/srv/Trigger
  /ap/mode_switch             ardupilot_msgs/srv/ModeSwitch     (GUIDED=4, LAND=9)
  /ap/arm_motors              ardupilot_msgs/srv/ArmMotors
  /ap/experimental/takeoff    ardupilot_msgs/srv/Takeoff
  /ap/get_parameters          rcl_interfaces/srv/GetParameters
  /ap/set_parameters          rcl_interfaces/srv/SetParameters
  /ap/cmd_gps_pose            ardupilot_msgs/msg/GlobalPosition (구독)
  /ap/geopose/filtered        geographic_msgs/msg/GeoPoseStamped (발행, ~28Hz)

[MAVLink 와의 실측 차이 - 이행서에 쓸 근거]
  - 갱신 주기: DDS ~28Hz  vs  MAVLink ~4Hz
  - 고도 기준: DDS geopose 는 AMSL(582.29m), MAVLink relative_alt 는 AGL. 원점 583m.
  - 진동 지표: DDS 에는 vibration/clipping 토픽이 없다. 프로토콜 간 실제 기능 격차.

================================================================================
[DDS 함정 5가지 - 전부 조용히 실패한다. 반드시 읽을 것]
================================================================================
에러 메시지 없이 그냥 동작만 안 하는 유형이라, 모르면 원인을 찾을 수 없다.

1) 웨이포인트에 frame_id 가 없으면 무시된다  ★가장 악질★
   AP_DDS_ExternalControl.cpp 의 핸들러 첫 줄이
     if (strcmp(cmd_pos.header.frame_id, MAP_FRAME) == 0)
   이고 MAP_FRAME 은 "map" 이다. frame_id 가 비어 있으면 조건에 걸리지 않고
   그대로 return false 로 빠진다. 로그도 안 남는다.
   -> 반드시 "{header: {frame_id: 'map'}, ...}" 를 붙일 것.
   (실측: frame_id 없이 243회 발행했으나 기체가 미동도 하지 않았음)

2) 위치 명령만으로는 이륙하지 않는다
   ArduCopter 의 GUIDED 위치제어 루프는 매 주기 is_disarmed_or_landed() 를 먼저
   검사한다. 착륙 상태(land_complete)면 목표를 받아도 지상 대기 처리로 빠져
   스로틀을 올리지 않는다. MAVLink 에서 NAV_TAKEOFF 를 따로 보내야 했던 것과 같다.
   -> /ap/experimental/takeoff 서비스를 써야 한다. 4.5 에는 이 서비스가 없다.

3) ros2 topic echo 가 아무것도 안 보여준다
   ArduPilot 은 BEST_EFFORT 로 발행하는데 echo 는 기본이 reliable 이라 매칭이 안 된다.
   -> --qos-reliability best_effort 를 붙일 것.

4) 4.7 에서 waf 플래그가 --enable-dds -> --enable-DDS 로 바뀌었다
   소문자는 에러 없이 무시된다. 빌드는 성공하는데 DDS 가 빠진 바이너리가 나온다.
   -> 판별법: configure 로그에 "Checking for program 'microxrceddsgen'" 줄이 있어야 한다.
      실행 후에는 SITL 로그의 "DDS: Init complete" 와 param show DDS* 로 확인.

5) 코드 생성기 버전이 ArduPilot 버전과 맞아야 한다
   구버전 microxrceddsgen 은 IDL 안의 상수를 생성하지 않아서
   'GlobalPosition' has not been declared / 'Status' has not been declared 로 빌드 실패.
   -> ArduPilot 4.7+ 는 ardupilot 포크의 v4.7.0 을 쓸 것 (eProsima 원본 아님).
      bash ~/skybuddy/fix_xrce_gen.sh

부가:
  - 4.7 부터 Agent 의 dds_xrce_profile.xml 이 없어졌다. -r 옵션 붙이면 에러난다.
  - ARMING_CHECK 가 ARMING_SKIPCHK 로 바뀌었고 의미가 반대다("건너뛸 검사").
    4.5 의 ARMING_CHECK 0 은 4.7 에서 ARMING_SKIPCHK -1 로 자동 변환된다.
  - sim_vehicle.py 에는 -N (--no-rebuild) 를 붙여 직접 빌드한 바이너리를 지킬 것.

[관련 스크립트]
- start_dds_sim.sh          DDS 드론 환경 기동 (Agent + SITL + 확인)
- upgrade_ardupilot_dds.sh  ArduPilot 을 최신 안정 태그로 올리고 DDS 포함 재빌드
- fix_xrce_gen.sh           microxrceddsgen 을 ArduPilot 버전에 맞춰 교체 + 재빌드

================================================================================
[이기종 2대 동시 미션 - 달성] 2026-09-07
================================================================================

MAVLink 드론 1대와 DDS 드론 1대가 같은 AirSim 씬에서 동시에 각자 임무를 수행하고
각자 KPI 를 수집하는 데 성공했다. 프로젝트의 목표 구성이다.

[구조]
  kpi_mission.py (미션·KPI 로직 - 프로토콜을 모른다)
         |
   DroneInterface  (drone_interface.py)
         |
    +----+----+
    |         |
 MavlinkDrone  Ros2Drone
 4.3/MAVLink   4.7.1/AP_DDS
   -I1 14561     -I2 ROS2 /ap

kpi_mission.py 에서 프로토콜을 아는 곳은 make_drone() 팩토리 한 곳뿐이다.
미션·도달판정·KPI 계산·CSV 저장 로직은 두 프로토콜에 대해 완전히 동일한 코드가 돈다.
verify_adapters.py 가 이걸 자동 검사한다 (run_mission 본문에 프로토콜 단어가 있으면 실패).

[실행]
  0) PowerShell: 2기체 settings.json 배포 후 Blocks 완전 재시작
       Copy-Item "...\config_reference\airsim_settings_reference.json" `
                 "$env:USERPROFILE\OneDrive\Documents\AirSim\settings.json" -Force
  1) bash ~/skybuddy/start_hetero_sim.sh
  2) 새 터미널:
       source /opt/ros/humble/setup.bash
       source ~/ardu_ws/install/setup.bash
       cd ~/skybuddy && python3 kpi_mission_multi.py
     (kpi_mission_multi.py 의 DRONES = HETEROGENEOUS 확인)

[측정 결과] 20x20m 정사각형 5개 웨이포인트, 두 대 동시 비행
                        drone1 (MAVLink)      drone2 (DDS)
  펌웨어                ArduCopter 4.3        ArduCopter 4.7.1
  순항 고도             30m                   50m
  웨이포인트 도달       5/5                   5/5
  구간 시간             7.0~7.3초             4.2~4.7초
  경로 효율             0.93~1.00 (평균 .957) 0.73~0.92 (평균 .863)
  총 소요               112.4초               86.5초
  총 이동거리           84.0m                 97.5m
  vibration_z 최대      0.046~0.057           (지표 없음)
  clipping              0                     (지표 없음)
  Main loop slow        0회                   0회

[결과 해석 - 이행서에 쓸 때 주의]
  속도·효율 차이를 프로토콜 탓으로 쓰면 안 된다. 두 기체는 펌웨어 버전과 튜닝이
  다르다 (drone1 은 tune_damping.py damped 적용, drone2 는 기본값). 같은 정사각형을
  도는데 drone2 가 16% 더 긴 거리를 난 것은 빠른 만큼 오버슛했기 때문이며,
  이는 WPNAV/ATC 파라미터 차이지 프로토콜 차이가 아니다.

  프로토콜에서 실제로 오는 차이는 이 세 가지다:
    - 갱신 주기      DDS 약 28Hz  vs  MAVLink 약 4Hz
    - 제공 지표      DDS 에는 진동/클리핑 토픽이 없다 (MAVLink 전용 지표)
    - 고도 기준      DDS geopose 는 AMSL, MAVLink relative_alt 는 홈 기준 AGL
  이행서에는 이쪽을 근거로 쓸 것.

  CPU 우려는 실측으로 해소됐다. SITL 2기 + AirSim + Agent + ROS 2 를 동시에 돌려도
  양쪽 다 Main loop slow 0회였다.

[이기종 구성에서 걸렸던 문제 4가지]
 1) lock-step 데드락
    settings.json 에 2기체가 정의돼 있으면 SITL 둘 다 붙기 전까지 Blocks 가
    시뮬레이션 시계를 진행시키지 않는다. 따라서 drone1 의 GPS 준비를 기다린 뒤
    drone2 를 띄우는 순서로 짜면 영원히 멈춘다.
    -> 반드시 둘 다 띄운 다음에 함께 기다릴 것.
 2) MAVProxy 가 즉시 종료
    nohup ... < /dev/null 로 띄우면 대화형 콘솔이 stdin EOF 를 받고 바로 죽는다.
    로그에 "SIM_VEHICLE: MAVProxy exited" 만 남는다.
    -> sim_vehicle.py 에 -m "--daemon" 을 붙일 것.
 3) NAV_TAKEOFF 가 result=4 로 거부
    ArduCopter 의 do_user_takeoff() 는 모터 스풀이 THROTTLE_UNLIMITED 가 되기 전엔
    거부한다. 시동 직후 1초 대기로는 부족했다 (CPU 경합 시 더 길어짐).
    -> drone_link.takeoff() 가 20초 동안 재시도하도록 수정.
 4) 새 인스턴스에서 시동 거부 ("Vehicle is Not Armable")
    새로 만든 copter2_instance 는 eeprom 이 비어 기본값으로 뜨고, AirSim GPS 가
    prearm 을 통과하지 못한다. 게다가 한 번 기본값으로 부팅하면 그 값이 eeprom 에
    저장돼서 --add-param-file 이 무시된다.
    -> config_reference/dds_drone.parm (ARMING_SKIPCHK -1) 을 넘기고,
       기동 시 eeprom.bin 을 지운다. 둘 다 start_hetero_sim.sh 가 처리한다.

[관련 파일]
- drone_interface.py     어댑터 계약 + 공통 텔레메트리 스키마
- ros2_drone.py          AP_DDS 어댑터
- verify_adapters.py     계약 준수 정적 검증 (시뮬 없이 실행)
- start_hetero_sim.sh    이기종 2대 기동
- config_reference/dds_drone.parm  DDS 드론 전용 파라미터

[남은 일]
- 두 기체의 튜닝을 맞춰서 다시 측정. 그래야 속도·효율 비교가 의미를 갖는다.
  drone2 에도 damped 프리셋을 적용하되, MAVLink 를 쓰지 않으려면
  /ap/set_parameters (ros2 param set) 로 넣는 게 프로토콜 순수성 면에서 낫다.
- 산악 지형(LandscapeMountains, C:\SkyBuddy_Mountains) 으로 교체.
  고도 기준 주의: coordinate_frame 6 은 홈 기준 상대고도라 지형이 올라가면
  산에 부딪힌다. FRAME_GLOBAL_TERRAIN_ALT(11) 가 SITL 에서 동작하는지 확인 필요.
- 탐색 구역 커버리지 지표 (계획서 핵심 평가항목인데 아직 없다).

================================================================================

[Git]
- master:           안정 동작 확인된 지점. 문제 생기면 git checkout master로 복귀.
- multi-drone-dev:  현재 개발 브랜치.
