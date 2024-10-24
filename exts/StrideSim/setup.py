from setuptools import setup, find_packages
import os

# 현재 setup.py 파일의 디렉토리
current_dir = os.path.dirname(__file__)

# README.md 파일의 절대 경로
readme_path = os.path.abspath(os.path.join(current_dir, '..', '..', 'README.md'))

# README.md 파일이 존재하는지 확인
if os.path.exists(readme_path):
    with open(readme_path, 'r', encoding='utf-8') as fh:
        long_description = fh.read()
else:
    long_description = "A description of your StrideSim package"

setup(
    name="StrideSim",
    version="0.1",
    packages=find_packages(where="StrideSim"),  # StrideSim 디렉토리에서 패키지를 찾습니다.
    package_dir={"": "StrideSim"},              # StrideSim 디렉토리를 패키지 루트로 지정합니다.
    install_requires=[
        "numpy",
        "torch",
        # 필요한 다른 의존성을 여기에 추가하세요
    ],
    author="Your Name",
    author_email="your.email@example.com",
    description="A description of your StrideSim package",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/StrideSim",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
