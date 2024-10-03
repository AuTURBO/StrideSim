# StrideSim

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.0.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-1.0.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](https://opensource.org/license/mit)

## 개요

StrideSim은 Isaac Lab을 기반으로 한 프로젝트입니다. 이 저장소는 Isaac Lab의 핵심 저장소 외부에서 독립적인 환경에서 개발할 수 있도록 설계되었습니다.

## 설치

1. Isaac Sim 설치: [설치 가이드](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html) 참조

2. Isaac Lab 설치: [설치 가이드](https://isaac-sim.github.io/IsaacLab/source/setup/installation/index.html) 참조

3. 라이브러리 설치:

   ```bash
   cd exts/StrideSim
   python -m pip install -e .
   ```

## 사용법

1. 강화학습 라이브러리 설치 (Optional)

```bash
cd rl
python -m pip install -e .
```

2. 강화학습 라이브러리 실행 - Train
```bash
cd StrideSim/rl
python train.py --task=<task_name> --headless
```

Train Tasks list
- Anymal-D(Rough): `Isaac-Velocity-Rough-Anymal-D-v0`
- Anymal-D(Flat): `Isaac-Velocity-Rough-Anymal-D-v0`
- Go2(Rough): `Isaac-Velocity-Rough-Unitree-Go2-v0`
- Go2(Flat): `Isaac-Velocity-Flat-Unitree-Go2-v0`

3. 강화학습 라이브러리 실행 - Play
```bash
cd StrideSim/rl
python play.py --task=<task_name>
```
Play Tasks list
- Anymal-D(Rough): `Isaac-Velocity-Rough-Anymal-D-Play-v0`
- Anymal-D(Flat): `Isaac-Velocity-Rough-Anymal-D-Play-v0`
- Go2(Rough): `Isaac-Velocity-Rough-Unitree-Go2-Play-v0`
- Go2(Flat): `Isaac-Velocity-Flat-Unitree-Go2-Play-v0`


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
