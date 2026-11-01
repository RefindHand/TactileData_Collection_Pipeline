
import os
import subprocess
import ipaddress

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
ENV_PATH = PROJECT_ROOT / ".env"


def load_env_file() -> None:
    """프로젝트 루트의 .env 파일을 읽어 환경변수로 등록한다."""

    if not ENV_PATH.is_file():
        return

    for line in ENV_PATH.read_text(encoding="utf-8").splitlines():
        line = line.strip()

        if not line or line.startswith("#"):
            continue

        key, separator, value = line.partition("=")

        if not separator:
            continue

        key = key.strip()
        value = value.strip().strip('"').strip("'")

        os.environ.setdefault(key, value)


def detect_wsl_ip() -> str:
    """Windows에서 WSL의 IPv4 주소를 자동 탐지한다."""

    result = subprocess.run(
        ["wsl.exe", "hostname", "-I"],
        capture_output=True,
        text=True,
        check=True,
        timeout=5
    )

    for candidate in result.stdout.split():
        try:
            address = ipaddress.ip_address(candidate)

            if isinstance(address, ipaddress.IPv4Address):
                return str(address)

        except ValueError:
            continue

    raise RuntimeError("WSL IPv4 주소를 찾을 수 없습니다.")


def get_network_config() -> tuple[str, int]:
    load_env_file()

    ip = os.getenv("WSL_IP")

    if not ip:
        ip = detect_wsl_ip()

    port = int(os.getenv("UDP_PORT", "5005"))

    if not 1 <= port <= 65535:
        raise ValueError("UDP_PORT는 1~65535 범위여야 합니다.")

    return ip, port
