#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

restore_library() {
  local arch="$1"
  local expected_sha256="$2"
  local library_dir="${REPO_ROOT}/third_party/unitree_sdk2_native/lib/${arch}"
  local archive="${library_dir}/libunitree_sdk2.a.tar.gz"
  local library="${library_dir}/libunitree_sdk2.a"
  local actual_sha256=""

  if [[ -f "${library}" ]]; then
    actual_sha256="$(sha256sum "${library}" | awk '{print $1}')"
    if [[ "${actual_sha256}" == "${expected_sha256}" ]]; then
      printf 'Unitree library already restored: %s\n' "${library}"
      return
    fi
    printf 'Existing library checksum mismatch; restoring: %s\n' "${library}" >&2
  fi

  if [[ ! -f "${archive}" ]]; then
    printf 'Missing vendor archive: %s\n' "${archive}" >&2
    return 1
  fi

  tar -xzf "${archive}" -C "${library_dir}"
  actual_sha256="$(sha256sum "${library}" | awk '{print $1}')"
  if [[ "${actual_sha256}" != "${expected_sha256}" ]]; then
    printf 'Checksum verification failed: %s\n' "${library}" >&2
    return 1
  fi

  printf 'Restored and verified: %s\n' "${library}"
}

restore_library aarch64 d721872acb95bab68ccd7d4856046db17747c40915e5e0490bdd7812ed7356ca
restore_library x86_64 ba018518707f511ddd47558ba4d5609e968c30998d3c52abe2e650ba4aab229d
