# Examples

Wheeled balancing is our entry-level

- [`wheeled_balancing.py`](wheeled_balancing.py): smallest example, it balances Upkie with a proportional wheel controller.
- [`wheeled_balancing_with_logging.py`](wheeled_balancing_with_logging.py): adds logging of actions and observations using [`asynchronous I/O`](https://docs.python.org/3/library/asyncio.html).
- [`wheeled_balancing_cpu_isolation.py`](wheeled_balancing_cpu_isolation.py): on the real robot, the previous example sometimes warns that the "rate limiter is late", as its process runs on the same CPU core as all others. This examples fixes this by isolating the process to its own core.
