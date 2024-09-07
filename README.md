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

TODO

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
