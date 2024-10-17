# StrideSim

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.0.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-1.0.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](https://opensource.org/license/mit)

## 개요

StrideSim은 Isaac Lab을 기반으로 한 프로젝트입니다. 이 저장소는 Isaac Lab의 핵심 저장소 외부에서 독립적인 환경에서 개발할 수 있도록 설계되었습니다.

## 설치

1. Isaac Sim 4.0.0 설치: [설치 가이드](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html) 참조

2. [Isaac Lab v1.0.0](https://github.com/isaac-sim/IsaacLab/tree/v1.0.0) 설치: [설치 가이드](https://isaac-sim.github.io/IsaacLab/source/setup/installation/index.html) 참조


## 꿀팁

1. 환경변수 설정

```bash
# Isaac Sim root directory
export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/isaac-sim-4.0.0"
# Isaac Sim python executable
alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
# Isaac Sim app
alias ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"
```

## 사용법

1. 강화학습 라이브러리 설치

```bash
cd rl
python -m pip install -e .
```

2. 강화학습 단독 실행

```bash
python rl/train.py --task Template-Isaac-Velocity-Rough-Anymal-D-v0
```

3. StrideSim 실행

3-1. 프로그램 실행

   ```bash
   # 환경변수 설정 후
   ISAACSIM
   ```

3-2. 확장 프로그램 설정

3-2-1. window -> extension 창으로 이동
3-2-2. 삼지창 버튼을 눌러 확장 프로그램 경로 삽입 (본 프로젝트의 exts까지 넣으면 된다.)
![alt text](Asset/image.png)

3-2-3. 좌측에 시뮬레이션 버튼을 클릭 (이때, AUTOLOAD를 활성화하면 편하다.)

3-3. 확장 프로그램 실행
![alt text](Asset/image-1.png)
이제 Isaac Examples 탭에 StrideSim_AnymalD 탭이 나오는 것을 확인할 수 있다.

![alt text](Asset/image-2.png)
버튼을 누르면 위와 같은 장면을 볼수 있고, anymalD를 부르는 것부터해서 학습 및 병렬 실행도 가능하다.

## 코드 포맷팅

pre-commit 훅을 사용하여 코드 포맷팅을 자동화합니다.

pre-commit 설치:

```bash
pip install pre-commit
```

pre-commit 실행:

```bash
pre-commit run --all-files
```

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.
