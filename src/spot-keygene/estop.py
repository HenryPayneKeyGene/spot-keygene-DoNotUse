from bosdyn.client.estop import EstopEndpoint, EstopKeepAlive


class EStop:
    def __init__(self, client, timeout_sec, name=None):
        self.endpoint = EstopEndpoint(client, name, timeout_sec)
        self.endpoint.force_simple_setup()

        self.keep_alive = EstopKeepAlive(self.endpoint)

        self.keep_alive.allow()

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_value, traceback):
        self.keep_alive.shutdown()

    def stop(self):
        self.keep_alive.stop()

    def allow(self):
        self.keep_alive.allow()

    def settle_then_cut(self):
        self.keep_alive.settle_then_cut()
