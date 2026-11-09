# AWS 서버 배포 (AirSim과 SITL을 서로 다른 인스턴스에서 실행)

[로컬 전체 파이프라인](local-full-pipeline.md)을 AWS 인스턴스 두 대로 옮기는 절차입니다.
로컬에서는 Windows와 WSL2가 한 PC 안에 있었지만, 서버에서는 AirSim(Windows)과 나머지 전부(Linux)가
서로 다른 인스턴스에 있습니다. **바뀌는 것은 AirSim과 SITL 사이의 UDP 연결뿐**이고, 미들웨어
설정(`drones.real.example.json`)과 나머지 실행 절차는 그대로입니다.

## 0. 구성

서울 리전, 같은 AZ, 같은 서브넷.

| 역할 | 인스턴스 | 올라가는 것 |
|---|---|---|
| Windows (GPU) | g6.2xlarge Windows Server, L4 24GB, 8 vCPU/32GB | AirSim (Blocks) |
| Linux (CPU) | c7i.2xlarge Ubuntu 22.04, 8 vCPU/16GB | SITL 2대, DDS Agent, ROS 2 Humble, 미들웨어, 오케스트레이터 |

```text
[Windows 인스턴스]                         [Linux 인스턴스]
 AirSim Copter1 ── UDP 9013 (센서) ──────▶ SITL 1 (-I1, ArduCopter 4.3) ── UDP 14561 ── MavlinkDrone ─┐
               ◀── UDP 9012 (모터) ──────                                                              │
 AirSim Copter2 ── UDP 9023 (센서) ──────▶ SITL 2 (-I2, ArduCopter 4.7.1) ── DDS /ap (Agent 2019) ── Ros2Drone ─┤
               ◀── UDP 9022 (모터) ──────                                                              │
                                                            미들웨어 MCP 서버 (127.0.0.1:9000) ◀──────┘
                                                                     ▲ HTTP /mcp
                                                            오케스트레이터 (127.0.0.1:8001)
```

인스턴스 사이를 오가는 것은 위 UDP 포트 4개뿐입니다. 14561, 2019, 9000, 8001은 모두 Linux 안의
`127.0.0.1`이므로 외부에 열지 않습니다.

## 1. AWS 준비

### 1-1. 보안 그룹

두 인스턴스에 같은 보안 그룹 `skybuddy-sg`를 붙이고, 인스턴스 사이 규칙은 **소스를 그 보안 그룹
자신**으로 둡니다. IP를 적지 않아도 되고 다른 곳에서는 접근할 수 없습니다.

| 방향 | 프로토콜/포트 | 소스 | 용도 |
|---|---|---|---|
| 인바운드 | UDP 9012, 9022 | `skybuddy-sg` | SITL → AirSim 모터 출력 (Windows가 받음) |
| 인바운드 | UDP 9013, 9023 | `skybuddy-sg` | AirSim → SITL 센서 데이터 (Linux가 받음) |
| 인바운드 | TCP 22 | 팀원 IP/32 | Linux SSH |
| 인바운드 | TCP 3389 (또는 DCV 8443) | 팀원 IP/32 | Windows 원격 데스크톱 |

- 9000(미들웨어), 8001(오케스트레이터)은 **열지 않습니다.** 미들웨어 MCP 서버에는 인증이 없습니다.
  오케스트레이터 Swagger(`/docs`)를 내 PC 브라우저로 보고 싶으면 SSH 포트 포워딩을 씁니다.

  ```bash
  ssh -i <키>.pem -L 8001:127.0.0.1:8001 ubuntu@<Linux 퍼블릭 IP>
  # 내 PC 브라우저에서 http://127.0.0.1:8001/docs
  ```

- 학교나 집 IP가 바뀌면 22, 3389 규칙의 소스를 다시 고칩니다.

### 1-2. IP

- **프라이빗 IP**는 인스턴스를 중지했다가 다시 시작해도 바뀌지 않습니다. 아래 설정에는 프라이빗
  IP만 씁니다.
- 퍼블릭 IP는 중지/시작할 때마다 바뀝니다. 접속 주소를 고정하려면 Elastic IP를 붙입니다(인스턴스가
  꺼져 있는 동안에도 요금이 붙을 수 있으니 필요할 때만).

| 기록 | 값 |
|---|---|
| Windows 프라이빗 IP | `<WINDOWS_PRIVATE_IP>` |
| Linux 프라이빗 IP | `<LINUX_PRIVATE_IP>` |

### 1-3. 비용

g6.2xlarge Windows가 가장 비쌉니다(Linux 기준 시간당 $1.20 + Windows 라이선스). 실험이 없을 때는
**두 인스턴스 모두 중지**합니다. 중지 중에는 EBS 스토리지 요금만 나갑니다.

## 2. Linux 인스턴스 구성

로컬 랩탑(시뮬레이션 담당)과 **같은 버전**을 맞추는 것이 목표입니다. 버전 표는
`services/simulation/INTERFACE.md` 0-1절, 빌드 함정은 `services/simulation/docs/engineering-notes.md`에
있습니다. 아래는 순서와 확인 방법입니다.

### 2-1. 설치 순서

| 순서 | 항목 | 확인 방법 |
|---|---|---|
| 1 | ROS 2 Humble (공식 apt 설치), `python3-colcon-common-extensions` | `source /opt/ros/humble/setup.bash && ros2 --help` |
| 2 | ArduPilot 4.3 → `~/ardupilot` (MAVLink 드론) | 로컬과 같은 태그. 로컬에서 `git -C ~/ardupilot describe --tags`로 확인 |
| 3 | ArduPilot 4.7.1 → `~/ardupilot45` (DDS 드론), **`--enable-DDS`** (대문자) | configure 로그에 `Checking for program 'microxrceddsgen'` 줄 |
| 4 | Micro-XRCE-DDS-Gen **ardupilot 포크 `v4.7.0`** → `~/Micro-XRCE-DDS-Gen` | 3번 빌드가 `'GlobalPosition' has not been declared` 없이 끝남 |
| 5 | micro-XRCE-DDS Agent (`MicroXRCEAgent`) | `which MicroXRCEAgent` |
| 6 | `~/ardu_ws` (ROS 2 워크스페이스, `ardupilot_msgs` colcon 빌드) | `source ~/ardu_ws/install/setup.bash && ros2 interface show ardupilot_msgs/srv/Takeoff` |
| 7 | 저장소 clone, 미들웨어 의존성 (시스템 Python 3.10.12) | 아래 2-2 |
| 8 | 오케스트레이터 가상환경 (`mcp` 2.x) | 아래 2-2 |
| 9 | `~/skybuddy` 보조 파일 | 아래 2-3 |

- ArduPilot의 `Tools/environment_install/install-prereqs-ubuntu.sh -y` 실행 후 새 셸을 엽니다.
- 2, 3번은 c7i.2xlarge(8 vCPU)에서 각각 수십 분 걸립니다. 동시에 돌리지 않습니다.
- sim_vehicle.py에는 `-N`(재빌드 안 함)이 붙어 있으므로 빌드가 먼저 끝나 있어야 합니다.

### 2-2. 저장소와 Python 환경

```bash
git clone https://github.com/bang0930/Skybuddy.git ~/Skybuddy
cd ~/Skybuddy

# 미들웨어: 시스템 Python (rclpy가 3.10에서만 동작, 가상환경 금지)
python3 --version   # Python 3.10.12
python3 -m pip install --user -r services/middleware/requirements.txt -r services/simulation/requirements.txt
python3 -c "import fastmcp, mcp, shapely, pymavlink; print('ok')"

# 오케스트레이터: 별도 가상환경
sudo apt install -y python3.10-venv
python3 -m venv services/orchestrator/venv
services/orchestrator/venv/bin/pip install -r services/orchestrator/requirements.txt
services/orchestrator/venv/bin/python -c "import importlib.metadata as m; print(m.version('mcp'))"   # 2.x
```

### 2-3. `~/skybuddy` 보조 파일

`start_hetero_sim.sh`는 시뮬레이션 담당 랩탑의 배치를 기준으로 아래 경로를 읽습니다. 스크립트를
고치지 않고 서버에서 같은 배치를 만듭니다.

```bash
mkdir -p ~/skybuddy/config_reference ~/skybuddy/logs
cp ~/Skybuddy/services/simulation/scripts/start_hetero_sim.sh ~/skybuddy/
cp ~/Skybuddy/services/simulation/scripts/tune_damping.py     ~/skybuddy/
cp ~/Skybuddy/services/simulation/config/dds_drone.parm      ~/skybuddy/config_reference/
```

- 스크립트 1단계의 `settings.json` 확인은 Windows 경로(`/mnt/c/...`)를 읽지 못해 `[건너뜀]`으로
  지나갑니다. 정상입니다. 서버에서는 3절에서 직접 확인합니다.
- 4절의 `--sim-address` 수정을 반영한 스크립트를 복사합니다.

## 3. Windows 인스턴스 구성

1. **NVIDIA 드라이버**: g6(L4)용 드라이버를 AWS 문서의 G6 안내에 따라 설치합니다. 장치 관리자에서
   L4가 보이고 `nvidia-smi`가 동작해야 합니다.
2. **AirSim 패키지**: 로컬에서 쓰던 Blocks 패키지 빌드를 그대로 복사합니다.
3. **settings.json**: `docs/development/aws/airsim_settings.aws.json`의 `<...>` 두 곳을 실제 IP로
   바꿔 `%USERPROFILE%\Documents\AirSim\settings.json`에 둡니다. (로컬은 OneDrive 경로였지만 서버에는
   OneDrive가 없습니다.)
4. **Windows 방화벽**: 보안 그룹과 별개로 Windows 방화벽도 막습니다. 관리자 PowerShell:

   ```powershell
   New-NetFirewallRule -DisplayName "SkyBuddy SITL to AirSim" -Direction Inbound `
     -Protocol UDP -LocalPort 9012,9022 -RemoteAddress <LINUX_PRIVATE_IP> -Action Allow
   ```

5. **화면 세션**: AirSim은 렌더링이 돌아야 시뮬레이션 시계가 진행됩니다. RDP 창을 최소화하거나 연결을
   끊으면 멈출 수 있습니다. 실험 중에는 원격 데스크톱 창을 열어 둡니다. 영상 촬영까지 고려하면
   NICE DCV(EC2에서 무료)도 검토합니다. **이 항목은 서버에서 아직 확인하지 않았습니다.**

## 4. AirSim ↔ SITL 연결

로컬과 서버의 차이는 이 절뿐입니다.

| 어디 | 로컬 (WSL2) | 서버 |
|---|---|---|
| `settings.json` `UdpIp` (AirSim이 센서 데이터를 보낼 곳) | `127.0.0.1` | `<LINUX_PRIVATE_IP>` |
| `settings.json` `LocalHostIp` (AirSim이 모터 출력을 받을 주소) | `127.0.0.1` | `<WINDOWS_PRIVATE_IP>` |
| `sim_vehicle.py` (SITL이 모터 출력을 보낼 곳) | 옵션 없음 (기본 127.0.0.1) | `--sim-address=<WINDOWS_PRIVATE_IP>` |

포트는 인스턴스 번호를 따릅니다(`-I1` → 9012/9013, `-I2` → 9022/9023). 바꾸지 않습니다.

### 4-1. `start_hetero_sim.sh` 수정 (시뮬레이션 담당과 합의 후)

`sim_vehicle.py` 두 줄에 `--sim-address`를 넣습니다. 기본값을 `127.0.0.1`로 두면 로컬 실행은 그대로
동작합니다.

```bash
AIRSIM_HOST="${AIRSIM_HOST:-127.0.0.1}"   # 서버: export AIRSIM_HOST=<WINDOWS_PRIVATE_IP>

nohup "$AP43/Tools/autotest/sim_vehicle.py" -v ArduCopter -f airsim-copter -I1 \
      --sim-address="$AIRSIM_HOST" \
      -m "--daemon" > "$LOG1" 2>&1 < /dev/null &

nohup "$AP47/Tools/autotest/sim_vehicle.py" -v ArduCopter -f airsim-copter -I2 -N \
      --sim-address="$AIRSIM_HOST" \
      --add-param-file="$HOME/skybuddy/config_reference/dds_drone.parm" \
      -m "--daemon" > "$LOG2" 2>&1 < /dev/null &
```

### 4-2. 연결 확인

AirSim을 먼저 켠 상태에서:

```bash
# Linux: AirSim이 보내는 센서 패킷이 들어오는지 (Windows IP에서 오는 9013, 9023)
sudo tcpdump -ni any -c 10 'udp and (port 9013 or port 9023)'
```

```powershell
# Windows: AirSim이 9012, 9022를 Windows 프라이빗 IP에서 열고 있는지
Get-NetUDPEndpoint -LocalPort 9012,9022
```

SITL을 띄운 뒤 두 SITL 로그에 `EKF3 IMU0 is using GPS`가 나오면 양방향이 정상입니다.

## 5. 실행 순서

기동 순서는 로컬과 같습니다: **AirSim → Agent → SITL 2대 → 미들웨어 → 오케스트레이터.**

1. Windows: Blocks 실행.
2. Linux 터미널 1: `export AIRSIM_HOST=<WINDOWS_PRIVATE_IP> && bash ~/skybuddy/start_hetero_sim.sh`
   - lock-step이라 SITL 2대가 모두 붙기 전에는 AirSim 시계가 진행되지 않습니다. 스크립트가 둘 다
     띄운 뒤 함께 기다리므로 순서를 바꾸지 않습니다.
3. Linux 이후 단계는 [로컬 전체 파이프라인](local-full-pipeline.md)의 **0, 3~6단계를 그대로** 따릅니다.
   - 0단계(FakeDrone 리허설)를 서버에서 먼저 한 번 해서 Python 환경부터 확인합니다.
   - 오케스트레이터는 "WSL에서 실행할 때" 명령을 씁니다(`127.0.0.1` 주소 그대로).
   - `GOOGLE_API_KEY`는 터미널에서만 지정합니다. 저장소나 셸 설정 파일에 적지 않습니다.
4. SSH가 끊겨도 서버가 내려가지 않게 미들웨어와 오케스트레이터는 `tmux` 안에서 실행합니다.

   ```bash
   tmux new -s skybuddy     # 창 나누기: Ctrl+b %   다시 붙기: tmux attach -t skybuddy
   ```

## 6. 서버 첫 실행에서 확인할 것

결과는 [검증 범위와 지원 범위](../contracts/verification-status.md)의 실제 SITL 칸에 반영합니다.

| 확인 | 기준 (로컬 2026-10-14 결과) | 관련 항목 |
|---|---|---|
| lock-step 지연으로 시뮬레이션이 느려지지 않는지 | takeoff 12.8~20.6초, goto 3.9~7.5초, land 32.6~60.1초 | C2~C4 |
| SITL 부하 | 두 SITL 로그에 `Main loop slow` 0회 | |
| `check_mvp_pipeline.py --url` 통과 (LLM 없이 미들웨어 → SITL) | `[6] this run's tasks succeeded`, 종료 코드 0 | F7 |
| 오케스트레이터 `/command`로 LLM → 미들웨어 → SITL | 계획 수락 후 두 드론 `task_finished` succeeded | F9 |
| 클라이언트 연결이 끊겨도 비행 계속, 재접속 후 같은 임무 상태 | | F8 |
| Ctrl+C 시 `server_stopping` 기록 | | F10 |

## 7. 문제 해결

| 증상 | 원인과 조치 |
|---|---|
| SITL 로그에 GPS 준비 신호가 3분 안에 없음 | 센서 패킷이 안 들어옴. 4-2의 tcpdump로 9013/9023 수신 확인. 없으면 `UdpIp`, 보안 그룹 인바운드 9013/9023 확인 |
| 센서는 들어오는데 AirSim 기체가 움직이지 않음 | 모터 출력이 Windows에 안 감. `--sim-address` 누락, `LocalHostIp`, 보안 그룹 9012/9022, Windows 방화벽 확인 |
| AirSim이 멈춘 채로 진행되지 않음 | lock-step. SITL 둘 중 하나가 안 붙었거나 화면 세션이 멈춤. 두 SITL 로그와 RDP 창 확인 |
| settings.json을 고쳤는데 반영 안 됨 | AirSim은 시작할 때만 읽음. Blocks를 완전히 껐다 켬 |
| 나머지 (`rclpy` import 실패, 포트 9000 사용 중, `DRONE_DISCONNECTED` 등) | [로컬 전체 파이프라인](local-full-pipeline.md#문제-해결)과 같음 |
