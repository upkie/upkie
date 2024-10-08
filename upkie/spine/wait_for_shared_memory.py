import logging
import time
from multiprocessing import resource_tracker
from multiprocessing.shared_memory import SharedMemory


def wait_for_shared_memory(
    shm_name: str,
    retries: int,
) -> SharedMemory:
    r"""!
    Connect to the spine shared memory.

    \param shm_name Name of the shared memory object.
    \param retries Number of times to try opening the shared-memory file.
    \throw SpineError If the spine did not respond after the prescribed number
        of trials.
    """
    # Remove leading slash if present, as SharedMemory will prepend it
    # See https://github.com/upkie/upkie/issues/375
    shm_name = shm_name.lstrip("/")

    for trial in range(retries):
        if trial > 0:
            logging.info(
                f"Waiting for spine /{shm_name} to start "
                f"(trial {trial} / {retries})..."
            )
            time.sleep(1.0)
        try:
            shared_memory = SharedMemory(shm_name, size=0, create=False)
            # Why we unregister: https://github.com/upkie/upkie/issues/376
            # Upstream issue: https://github.com/python/cpython/issues/82300
            resource_tracker.unregister(shared_memory._name, "shared_memory")
            return shared_memory
        except FileNotFoundError:
            pass
    raise SpineError(
        f"spine /{shm_name} did not respond after {retries} attempts"
    )
