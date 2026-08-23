"""Storage management for the Pi's M.2 SSD.

Owns free-space reporting, the startup self-test, and the WIPE command.
The CSV and video writers only create files; deletion happens here,
across all streams.
"""

import os
import shutil

BYTES_PER_GB = 1024 ** 3

# Below this, telemetry raises LOW_SSD.
LOW_SSD_GB = 20

# Below this before LO, the experiment does not have room for the full
# experiment phase (about 60 GB of raw video) plus safe margin.
LOW_SSD_FOR_FLIGHT_GB = 100


class StorageManager:

    def __init__(self, root: str) -> None:
        self._root = root
        self._free_bytes = 0
        self._read_failed = False
        self.refresh()
        self._path_ok = False
        self._writable = False
        self._space_ok = False
        self._wipe_errors = 0


    def refresh(self) -> None:
        """Read free space. Called from the main loop at ~1 Hz, never
        from the poll reply path: this is a filesystem syscall and can
        block on a loaded SSD."""
        try:
            self._free_bytes = shutil.disk_usage(self._root).free
            self._read_failed = False
        except OSError:
            self._read_failed = True


    def free_gb(self) -> int:
        """Cached free space in whole GB, rounded down, clamped to 254.

        This feeds a <B field in STATUS (messages.encode_status), so it
        must fit in a byte. 255 (0xFF) is reserved as the ESP32's sentinel
        for an unreachable Pi; only the ESP32 may set it, so the Pi caps
        at 254. Unreachable with the 128 GB flight SSD - only triggers on
        a larger bench disk.
        """
        return min(254, self._free_bytes // BYTES_PER_GB)


    def self_test(self) -> bool:
        """Startup verification: path present, writable, and enough space
        for the full experiment phase.

        Records individual failure flags rather than a single result, so
        the ground can distinguish an unmounted SSD from a full one.
        """
        self._path_ok = os.path.isdir(self._root)
        if not self._path_ok:
            self._writable = False
            self._space_ok = False
            return False

        self._writable = self._test_write()
        self.refresh()
        self._space_ok = self.free_gb() >= LOW_SSD_FOR_FLIGHT_GB

        return self._path_ok and self._writable and self._space_ok


    def _test_write(self) -> bool:
        """Actually write, sync and delete a file.

        os.access() consults permission bits, which can disagree with
        reality on a read-only mount or a full disk. The only reliable
        test is performing the operation.
        """
        path = os.path.join(self._root, ".write_test")
        try:
            with open(path, "wb") as f:
                f.write(b"\x00")
                f.flush()
                os.fsync(f.fileno())
            os.remove(path)
            return True
        except OSError:
            return False


    def wipe(self) -> int:
        """Delete stored data. Returns bytes freed.

        Deletes everything except the file currently being written in
        each directory and the uplink command log. Pre-LO imagery is
        expendable by design: the experiment data begins at SOE.

        A file still open by a writer must not be unlinked - on Linux
        that frees no space and loses the data the writer is producing.
        """
        freed = 0
        for dirpath, _dirnames, filenames in os.walk(self._root):
            if not filenames:
                continue

            keep = self._newest_group(filenames)
            for name in sorted(filenames):
                if name in keep or name.startswith("uplink_commands"):
                    continue
                path = os.path.join(dirpath, name)
                try:
                    size = os.path.getsize(path)
                    os.remove(path)
                    freed += size
                except OSError:
                    self._wipe_errors += 1

        self.refresh()
        return freed


    @staticmethod
    def _newest_group(filenames: list[str]) -> set[str]:
        """The newest file, plus any sidecar sharing its stem.

        Filenames encode a zero-padded timestamp, so lexical order is
        chronological.
        """
        newest = max(filenames)
        stem = os.path.splitext(newest)[0]
        return {n for n in filenames if os.path.splitext(n)[0] == stem}