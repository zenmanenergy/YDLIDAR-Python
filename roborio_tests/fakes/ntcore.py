"""Stand-in for pyntcore with the 2026 method names. Values live in STORE, keyed by full path."""
STORE = {}
TIMES = {}
PUBLISHED = {}


class _Atomic:
    def __init__(self, time, value):
        self.time = time
        self.serverTime = time
        self.value = value


class _Sub:
    def __init__(self, path, default):
        self.path, self.default = path, default

    def get(self):
        return STORE.get(self.path, self.default)

    def getAtomic(self):
        return _Atomic(TIMES.get(self.path, 0), self.get())


class _Pub:
    def __init__(self, path):
        self.path = path

    def set(self, v):
        PUBLISHED[self.path] = v


class _Topic:
    def __init__(self, path):
        self.path = path

    def subscribe(self, default):
        return _Sub(self.path, default)

    def publish(self):
        return _Pub(self.path)


class NetworkTable:
    def __init__(self, path):
        self.path = path

    def _topic(self, name):
        return _Topic(self.path + "/" + name)

    getIntegerTopic = getBooleanTopic = getDoubleTopic = getDoubleArrayTopic = _topic


class NetworkTableInstance:
    _default = None

    @classmethod
    def getDefault(cls):
        if cls._default is None:
            cls._default = cls()
        return cls._default

    def getTable(self, key):
        return NetworkTable("/" + key)

    def getStructArrayTopic(self, name, type_):
        return _Topic(name)
