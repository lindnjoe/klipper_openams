# OpenAMS Moonraker helpers
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""Patch generic database methods onto AFC_moonraker.

AFC's ``AFC_moonraker`` already has ``update_afc_stats`` / ``get_afc_stats``
(hard-coded to the ``afc_stats`` namespace) and ``remove_database_entry``
(generic).  We add the matching generic write/read so OpenAMS can persist
its own status without duplicating HTTP transport.
"""

from __future__ import annotations

import json
import types
from typing import Any, Dict, Optional
from urllib.request import Request


def _write_database_entry(self, namespace: str, key: str, value: Any):
    """Write an arbitrary key/value to a Moonraker database namespace."""
    payload = json.dumps({
        "request_method": "POST",
        "namespace": namespace,
        "key": key,
        "value": value,
    }).encode()
    req = Request(
        self.database_url, payload,
        headers={"Content-Type": "application/json"},
    )
    return self._get_results(req)


def _read_database_entry(self, namespace: str, key: str) -> Optional[Dict[str, Any]]:
    """Read a key from a Moonraker database namespace."""
    url = self.database_url + f"?namespace={namespace}&key={key}"
    return self._get_results(url, print_error=False)


def ensure_generic_db_methods(moonraker) -> None:
    """Patch ``write_database_entry`` / ``read_database_entry`` onto *moonraker*.

    Safe to call multiple times — skips patching if the methods already exist
    (e.g. if AFC adds them upstream later).
    """
    if not hasattr(moonraker, "write_database_entry"):
        moonraker.write_database_entry = types.MethodType(
            _write_database_entry, moonraker,
        )
    if not hasattr(moonraker, "read_database_entry"):
        moonraker.read_database_entry = types.MethodType(
            _read_database_entry, moonraker,
        )
